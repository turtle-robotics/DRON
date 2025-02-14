using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using System;
using System.Runtime.InteropServices;
using System.Linq;

public class Stereovision : MonoBehaviour
{
    [DllImport ("DisparityFiltering", EntryPoint = "processDisparity")]
    private static extern int processDisparity(float[] pointCloudData, int height, int width, int[] leftImage, int[] rightImage);
    [DllImport ("DisparityFiltering", EntryPoint = "getDirectionVectors")]
    private static extern int getDirectionVectors(float[] dirVecs, int height, int width, int[] image);

    public int everyOther = 1;
    public GameObject meshRegion;
    public Material meshMaterial;

    private GameObject obj;
    private MeshRegionRenderer meshRegionRenderer;
    bool after = false;

    private Image leftImage;
    private Image rightImage;
    private Image thermalImage;
    private DroneInstance droneInstance;
    private Hash hash;
    private Hash checkingHash;
    private Vector3[] pointCloud;
    private float[] pointCloudHeat;
    public LayerMask layerMask;

    // Start is called before the first frame update
    void Start()
    {
        obj = GameObject.Instantiate(meshRegion, Vector3.zero, Quaternion.identity);
        meshRegionRenderer = obj.GetComponent<MeshRegionRenderer>();
        droneInstance = transform.parent.GetComponent<DroneInstance>();
        layerMask = LayerMask.GetMask("PointCloud");
    }

    // Update is called once per frame
    void Update()
    {
        if (after) {
            after = false;
        }
        if (Input.GetKeyUp("space") && leftImage.data != null && rightImage.data != null) {
            // Process disparity and reconstruct 3D mesh when spacebar is pressed
            int height = leftImage.height;
            int width = leftImage.width;

            float[] pointCloudData = new float[height * width * 3];
            Debug.Log(processDisparity(pointCloudData, height, width, leftImage.data, rightImage.data)); // Returns point cloud data

            // Organize floats into Vector3
            Vector3[] pointCloud = new Vector3[height * width / (everyOther + 1)];
            int index = 0;
            int count = 0;
            for (int i = 0; i < pointCloud.Length; i++) {
                if (everyOther != 0) {
                    if (index != everyOther) {
                        index++;
                        continue;
                    } else {
                        index = 0;
                    }
                }
                Vector3 point = new Vector3(pointCloudData[i * 3], pointCloudData[i * 3 + 1], pointCloudData[i * 3 + 2]) / 1000;
                if (float.IsNaN(point.x) || float.IsNaN(point.y) || float.IsNaN(point.z) || float.IsInfinity(point.x) || float.IsInfinity(point.y) || float.IsInfinity(point.z)) {
                    pointCloud[count++] = Vector3.zero;
                } else {
                    pointCloud[count++] = transform.TransformPoint(point);
                }
            }

            Reconstruct3D(pointCloud);
            after = true;
        }
    }

    void Reconstruct3D(Vector3[] pointCloud) {
        Debug.Log("reconstructing");

        pointCloud = JoinClouds(pointCloud);

        // https://docs.opencv.org/4.x/dd/d53/tutorial_py_depthmap.html
        var gradient = new Gradient();
        List<Vector3> vertices = new List<Vector3>();
        List<int> triangles = new List<int>();

        // Blend color from red at 0% to blue at 100%
        var colors = new GradientColorKey[3];
        colors[0] = new GradientColorKey(Color.blue, 0.0f);
        colors[1] = new GradientColorKey(Color.green, 0.5f);
        colors[2] = new GradientColorKey(Color.red, 1.0f);

        // Blend alpha from opaque at 0% to transparent at 100%
        var alphas = new GradientAlphaKey[3];
        alphas[0] = new GradientAlphaKey(1.0f, 0.0f);
        alphas[1] = new GradientAlphaKey(1.0f, 0.0f);
        alphas[2] = new GradientAlphaKey(1.0f, 0.0f);

        gradient.SetKeys(colors, alphas);

        float maxDepth = 0f;
        for (int i = 0; i < pointCloud.Length; i++) {
            //int disparity = data[i]; // measured in pixels

            float depth = pointCloud[i].x;
            /*if (disparity != 0) {
                depth /= disparity;
            }*/

            if (depth > maxDepth) maxDepth = depth;
        }

        for (int i = 0; i < pointCloud.Length; i++) {
            /*int disparity = data[i * width + j]; // measured in pixels

            if (disparity == 0) {
                continue;
            }

            float depth = (cameraSep * focalLength / disparity) * 0.001f; // mm to m*/
            Vector3[] cubeVerts;
            int[] cubeTris;

            //Vector3 worldPos = new Vector3(j/100f, (height-i-1)/100f, depth);
            Vector3 worldPos = pointCloud[i];

            MeshGenerator.GenerateCubeMesh(worldPos, 0.01f * (everyOther + 1f), vertices.Count, out cubeVerts, out cubeTris);
            vertices.AddRange(cubeVerts);
            triangles.AddRange(cubeTris);
        }

        Mesh mesh = new Mesh();
        mesh.indexFormat = IndexFormat.UInt32;
        mesh.vertices = vertices.ToArray();
        mesh.triangles = triangles.ToArray();
        
        // create new colors array where the colors will be created.
        Color[] meshColors = new Color[vertices.Count];

        for (int i = 0; i < vertices.Count; i++) {
            meshColors[i] = gradient.Evaluate(1 - vertices[i].x / maxDepth);
        }
        // assign the array of colors to the Mesh.
        mesh.colors = meshColors;

        Debug.Log(pointCloud.Length);

        this.pointCloud = pointCloud;
        meshRegionRenderer.UpdateMesh(mesh, meshMaterial);
    }

    private Vector3[] JoinClouds(Vector3[] newCloud) {
        if (hash == null) {
            hash = new Hash(1, 1000000);
            checkingHash = new Hash(1, 1000000);
            hash.Create(newCloud);
            
            pointCloudHeat = new float[newCloud.Length];

            return newCloud;
        } else {
            // Compare new cloud and existing cloud points. Whichever has higher concentration in grid point will remain
            hash.Create(pointCloud);
            checkingHash.Create(newCloud);
            
            List<Vector3> joinedCloud = new List<Vector3>();
            List<float> joinedHeat = new List<float>();

            for (int h = 0; h < hash.cellCount.Length - 1; h++) { // This checks through every grid cell
                if (hash.cellCount[h] > checkingHash.cellCount[h]) { // Find which has the higher density
                    // Original is higher density

                    int start = hash.cellStart[h];
                    int end = hash.cellStart[h + 1];

                    for (int i = start; i < end; i++) {
                        joinedCloud.Add(pointCloud[hash.cellEntries[i]]);
                        joinedHeat.Add(pointCloudHeat[hash.cellEntries[i]]);
                    }

                } else {
                    // New is higher density or equal

                    int start = checkingHash.cellStart[h];
                    int end = checkingHash.cellStart[h + 1];

                    for (int i = start; i < end; i++) {
                        joinedCloud.Add(newCloud[checkingHash.cellEntries[i]]);
                        joinedHeat.Add(0);
                    }
                }
            }

            pointCloudHeat = joinedHeat.ToArray();
            return joinedCloud.ToArray();
        }
    }

    private void ProjectThermal() {
        int height = thermalImage.height;
        int width = thermalImage.width;

        float[] dirVecData = new float[height * width * 3];
        Debug.Log(getDirectionVectors(dirVecData, height, width, thermalImage.data)); // Returns the thermal image's direction vectors ungrouped

        // Organize floats into Vector3's
        Vector3[] dirVecs = new Vector3[height * width];
        for (int i = 0; i < height; i++) {
            dirVecs[i] = transform.TransformDirection(new Vector3(dirVecData[i * 3], dirVecData[i * 3 + 1], dirVecData[i * 3 + 2]));
        }

        for (int i = 0; i < dirVecs.Length; i++) {
            RaycastHit hit;
        // Does the ray intersect any objects excluding the player layer
        if (Physics.Raycast(transform.position, dirVecs[i], out hit, 1000, layerMask)) { 
            Debug.DrawRay(transform.position, dirVecs[i] * hit.distance, Color.yellow); 
            Debug.Log("Did Hit"); 
        } else { 
            //Debug.DrawRay(transform.position, transform.TransformDirection(Vector3.forward) * 1000, Color.white); 
            //Debug.Log("Did not Hit"); 
        }
        }
    }

    public void SetLeftImage(int height, int width, string encoding, byte[] data) {
        leftImage = new Image(height, width, encoding, data);
    }

    public void SetRightImage(int height, int width, string encoding, byte[] data) {
        rightImage = new Image(height, width, encoding, data);
    }

    public void SetThermalImage(int height, int width, string encoding, byte[] data) {
        thermalImage = new Image(height, width, encoding, data);
    }
}

public struct Image {
    public int height;
    public int width;
    public string encoding;
    public int[] data;

    public Image(int height, int width, string encoding, byte[] byteData) {
        this.height = height;
        this.width = width;
        this.encoding = encoding;
        data = new int[height * width];
        for (int i = 0; i < data.Length; i++) {
            data[i] = (int) byteData[i];
        }
    }
}

class Hash {
    public float spacing;
    public int tableSize;
    public int[] cellStart;
    public int[] cellCount;
    public int[] cellEntries;
    public int[] queryIds;
    public int querySize;
    public int maxNumObjects;
    public int[] firstAdjId;
    public int[] adjIds;

    public Hash(float spacing, int maxNumObjects) {
        this.spacing = spacing;
        tableSize = 5 * maxNumObjects;
        cellStart = new int[tableSize + 1];
        cellCount = new int[tableSize + 1];
        cellEntries = new int[maxNumObjects];
        queryIds = new int[maxNumObjects];
        querySize = 0;

        this.maxNumObjects = maxNumObjects;
        firstAdjId = new int[maxNumObjects + 1];
        adjIds = new int[10 * maxNumObjects];
    }

    private int HashCoords(int xi, int yi, int zi) {
        int h = (xi * 92837111) | (yi * 689287499) | (zi * 283923481);
        return Mathf.Abs(h) % tableSize;
    }

    private int IntCoord(float coord) {
        return Mathf.FloorToInt(coord / spacing);
    }

    private int HashPos(Vector3 pos) {
        return HashCoords(IntCoord(pos.x), IntCoord(pos.y), IntCoord(pos.z));
    }

    public void Create(Vector3[] pos) {
        int numObjects = Mathf.Min(pos.Length, cellEntries.Length);

        // Determine cell sizes
        /*for (int i = 0; i < cellStart.Length; i++) {
            cellStart[i] = 0;
        }*/
        for (int i = 0; i < cellCount.Length; i++) {
            cellCount[i] = 0;
        }
        for (int i = 0; i < cellEntries.Length; i++) {
            cellEntries[i] = 0;
        }

        // Determine cell starts
        int start = 0;
        for (int i = 0; i < tableSize; i++) {
            start += cellCount[i];
            cellCount[i] = start;
        }
        cellCount[tableSize] = start;

        // Count
        for (int i = 0; i < numObjects; i++) {
            int h = HashPos(pos[i]);
            cellCount[h]++;
        }
        Array.Copy(cellCount, cellStart, cellStart.Length);

        // Partial Sums
        for (int i = 1; i < cellStart.Length; i++) {
            cellStart[i] += cellStart[i-1];
        }

        // Fill in object ids
        for (int i = 0; i < numObjects; i++) {
            int h = HashPos(pos[i]);
            cellStart[h]--;
            cellEntries[cellStart[h]] = i;
        }
    }

    // Find points within maxDist of pos
    public void Query(Vector3 pos, float maxDist) {
        int x0 = IntCoord(pos.x - maxDist);
        int y0 = IntCoord(pos.y - maxDist);
        int z0 = IntCoord(pos.z - maxDist);

        int x1 = IntCoord(pos.x + maxDist);
        int y1 = IntCoord(pos.y + maxDist);
        int z1 = IntCoord(pos.z + maxDist);

        querySize = 0;

        for (int xi = x0; xi < x1; xi++) {
            for (int yi = y0; yi < y1; yi++) {
                for (int zi = z0; zi < z1; zi++) {
                    int h = HashCoords(xi, yi, zi);
                    int start = cellStart[h];
                    int end = cellStart[h + 1];

                    for (int i = start; i < end; i++) {
                        queryIds[querySize] = cellEntries[i];
                        querySize++;
                    }
                }
            }
        }
    }

    public void QueryAll(Vector3[] pos, float maxDist) {
        int num = 0;
        float maxDist2 = maxDist * maxDist;

        for (int i = 0; i < maxNumObjects; i++) {
            int id0 = i;
            firstAdjId[id0] = num;
            Query(pos[id0], maxDist);

            for (int j = 0; j < querySize; j++) {
                int id1 = queryIds[j];
                if (id1 > id0)
                    continue;

                float dist2 = (pos[id0] - pos[id1]).sqrMagnitude;
                if (dist2 > maxDist2)
                    continue;
                
                if (num >= adjIds.Length) {
                    int[] newIds = new int[2 * num];
                    System.Array.Copy(adjIds, newIds, adjIds.Length);

                    adjIds = newIds;
                }
                adjIds[num++] = id1;
            }
        }
    }
}