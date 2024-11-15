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

    public int everyOther = 1;
    public GameObject meshRegion;
    public Material meshMaterial;

    private GameObject obj;
    private MeshRegionRenderer meshRegionRenderer;
    bool after = false;

    private Image leftImage;
    private Image rightImage;
    private DroneInstance droneInstance;

    // Start is called before the first frame update
    void Start()
    {
        obj = GameObject.Instantiate(meshRegion, Vector3.zero, Quaternion.identity);
        meshRegionRenderer = obj.GetComponent<MeshRegionRenderer>();
        droneInstance = transform.parent.GetComponent<DroneInstance>();
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
            Debug.Log(height);

            float[] pointCloudData = new float[height * width * 3];
            Debug.Log(processDisparity(pointCloudData, height, width, leftImage.data, rightImage.data)); // Returns point cloud data

            // Organize floats into Vector3 image
            Vector3[,] pointCloud = new Vector3[height,width];
            for (int i = 0; i < height; i++) {
                for (int j = 0; j < width; j++) {
                    //Vector3 point = transform.TransformPoint(new Vector3(pointCloudData[i * (width * 3) + j * 3], pointCloudData[i * (width * 3) + j * 3 + 1], pointCloudData[i * (width * 3) + j * 3 + 2]) / 1000);
                    Vector3 point = new Vector3(pointCloudData[i * (width * 3) + j * 3], pointCloudData[i * (width * 3) + j * 3 + 1], pointCloudData[i * (width * 3) + j * 3 + 2]) / 1000;
                    if (float.IsNaN(point.x) || float.IsNaN(point.y) || float.IsNaN(point.z) || float.IsInfinity(point.x) || float.IsInfinity(point.y) || float.IsInfinity(point.z)) {
                        pointCloud[i,j] = Vector3.zero;
                    } else {
                        pointCloud[i,j] = point;
                    }
                }
            }

            Debug.Log(pointCloud[0,0]);
            Debug.Log(pointCloud[200,400]);

            Reconstruct3D(pointCloud, height, width);
            after = true;
        }
    }

    void Reconstruct3D(Vector3[,] pointCloud, int height, int width) {
        Debug.Log("reconstructing");

        // https://docs.opencv.org/4.x/dd/d53/tutorial_py_depthmap.html
        var gradient = new Gradient();
        List<Vector3> vertices = new List<Vector3>();
        List<int> triangles = new List<int>();

        // Blend color from red at 0% to blue at 100%
        var colors = new GradientColorKey[3];
        colors[0] = new GradientColorKey(Color.blue, 0.0f);
        colors[1] = new GradientColorKey(Color.green, 0.8f);
        colors[2] = new GradientColorKey(Color.red, 1.0f);

        // Blend alpha from opaque at 0% to transparent at 100%
        var alphas = new GradientAlphaKey[3];
        alphas[0] = new GradientAlphaKey(1.0f, 0.0f);
        alphas[1] = new GradientAlphaKey(1.0f, 0.0f);
        alphas[2] = new GradientAlphaKey(1.0f, 0.0f);

        gradient.SetKeys(colors, alphas);

        int index = 0;

        /*float maxDepth = 0f;
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                if (everyOther != 0) {
                    if (index != everyOther) {
                        index++;
                        continue;
                    } else {
                        index = 0;
                    }
                }
                int disparity = data[i * width + j]; // measured in pixels

                float depth = cameraSep * focalLength * 0.001f;
                if (disparity != 0) {
                    depth /= disparity;
                }

                if (depth > maxDepth) maxDepth = depth;
            }
        }*/

        index = 0;
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                if (everyOther != 0) {
                    if (index != everyOther) {
                        index++;
                        continue;
                    } else {
                        index = 0;
                    }
                }
                /*int disparity = data[i * width + j]; // measured in pixels

                if (disparity == 0) {
                    continue;
                }

                float depth = (cameraSep * focalLength / disparity) * 0.001f; // mm to m*/
                Vector3[] cubeVerts;
                int[] cubeTris;

                //Vector3 worldPos = new Vector3(j/100f, (height-i-1)/100f, depth);
                Vector3 worldPos = pointCloud[i,j];

                MeshGenerator.GenerateCubeMesh(worldPos, 0.01f * (everyOther + 1f), vertices.Count, out cubeVerts, out cubeTris);
                vertices.AddRange(cubeVerts);
                triangles.AddRange(cubeTris);
            }
        }

        Mesh mesh = new Mesh();
        mesh.indexFormat = IndexFormat.UInt32;
        mesh.vertices = vertices.ToArray();
        mesh.triangles = triangles.ToArray();
        
        // create new colors array where the colors will be created.
        Color[] meshColors = new Color[vertices.Count];

        /*for (int i = 0; i < vertices.Count; i++) {
            meshColors[i] = gradient.Evaluate(1 - vertices[i].z / maxDepth);
        }*/
        // assign the array of colors to the Mesh.
        mesh.colors = meshColors;

        meshRegionRenderer.UpdateMesh(mesh, meshMaterial, height, width, everyOther);
    }

    public void SetLeftImage(int height, int width, string encoding, byte[] data) {
        leftImage = new Image(height, width, encoding, data);
    }

    public void SetRightImage(int height, int width, string encoding, byte[] data) {
        rightImage = new Image(height, width, encoding, data);
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