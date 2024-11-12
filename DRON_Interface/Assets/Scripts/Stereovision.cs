using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using System;
using System.Runtime.InteropServices;
using System.Linq;

public class Stereovision : MonoBehaviour
{
    [DllImport ("DisparityFiltering", EntryPoint = "processDisparity")] // nm -D libDisparityFiltering.so to find the decorated name
    private static extern int processDisparity(int[] outData, int height, int width, int[] leftImage, int[] rightImage);

    [SerializeField]
    private int focalLength = 4000; // pixels
    [SerializeField]
    private int cameraSep = 10; // mm

    public int everyOther = 25;
    public GameObject meshRegion;
    public Material meshMaterial;

    private GameObject obj;
    private MeshRegionRenderer meshRegionRenderer;
    int[] data = new int[446464];
    bool after = false;

    private Image leftImage;
    private Image rightImage;

    // Start is called before the first frame update
    void Start()
    {
        obj = GameObject.Instantiate(meshRegion, Vector3.zero, Quaternion.identity);
        meshRegionRenderer = obj.GetComponent<MeshRegionRenderer>();
    }

    // Update is called once per frame
    void Update()
    {
        if (after) {
            after = false;
        }
        if (Input.GetKeyUp("space") && leftImage.data != null && rightImage.data != null) {
            // Process disparity and reconstruct 3D mesh when spacebar is pressed
            data = new int[leftImage.height * leftImage.width];
            Debug.Log(processDisparity(data, leftImage.height, leftImage.width, leftImage.data, rightImage.data));
            Reconstruct3D(data, leftImage.height, leftImage.width);
            after = true;
        }
    }

    void Reconstruct3D(int[] data, int height, int width) {
        print("reconstructing");

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

        float maxDepth = 0f;
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
        }

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
                int disparity = data[i * width + j]; // measured in pixels

                if (disparity == 0) {
                    continue;
                }

                float depth = (cameraSep * focalLength / disparity) * 0.001f; // mm to m
                Vector3[] cubeVerts;
                int[] cubeTris;

                Vector3 worldPos = new Vector3(j/100f, (height-i-1)/100f, depth);

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

        for (int i = 0; i < vertices.Count; i++) {
            meshColors[i] = gradient.Evaluate(1 - vertices[i].z / maxDepth);
        }
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