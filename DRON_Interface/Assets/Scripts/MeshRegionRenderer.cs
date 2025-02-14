using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer), typeof(MeshCollider))]
public class MeshRegionRenderer : MonoBehaviour
{
    float gridSize = 0.5f;
    int maxConcentrationPerCubicMeter = 1000;
    int maxConcentration;
    Dictionary<string, int> concentrations = new Dictionary<string, int>();
    Dictionary<string, List<Vector3>> verticesGridRef = new Dictionary<string, List<Vector3>>();

    private MeshFilter meshFilter;
    private MeshRenderer meshRenderer;
    private MeshCollider meshCollider;

    public Shader paintShader;

    RenderTexture splatMap;
    Material unpaintedMaterial,paintMaterial;

    // Start is called before the first frame update
    void Start()
    {
        maxConcentration = (int)(maxConcentrationPerCubicMeter / (gridSize * gridSize * gridSize));
        meshFilter = GetComponent<MeshFilter>();
        meshRenderer = GetComponent<MeshRenderer>();
        meshCollider = GetComponent<MeshCollider>();

        VertexPaint vertexPaint = GetComponent<VertexPaint>();
        vertexPaint.cam = Camera.main;

        // Mesh Painting Prep
        //paintMaterial = new Material(paintShader);
        //paintMaterial.SetVector("_Color", Color.red);

        unpaintedMaterial = meshRenderer.material;
        splatMap = new RenderTexture(1024, 1024, 0, RenderTextureFormat.ARGBFloat);
        unpaintedMaterial.mainTexture = splatMap;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void UpdateMesh(Mesh newMesh, Material mat) {
        if (meshFilter == null) {
            meshFilter = GetComponent<MeshFilter>();
        }
        if (meshRenderer == null) {
            meshRenderer = GetComponent<MeshRenderer>();
        }

        meshFilter.mesh = newMesh;
        meshRenderer.material = mat;
        meshCollider.sharedMesh = newMesh;
    }

    public void SplatMesh(Vector3 pos) {
        // https://stackoverflow.com/questions/60552364/draw-onto-an-objects-texture-based-on-a-raycast-hit-position-in-world-space
        pos = transform.InverseTransformPoint(pos);
        
        paintMaterial.SetVector("_Coordinate", new Vector4(pos.x, pos.y, pos.z, 0));
        RenderTexture temp = RenderTexture.GetTemporary(splatMap.width, splatMap.height, 0, RenderTextureFormat.ARGBFloat);

        // Cache the old target so that we can reset it later
        RenderTexture previousRT = RenderTexture.active;
        RenderTexture.active = temp;

        Material mat = paintMaterial;
        Mesh mesh = meshFilter.mesh;
        mat.SetTexture("_MainTex", splatMap);
        mat.SetPass(0); // This tells the renderer to use pass 0 from this material
        Graphics.DrawMeshNow(mesh, Vector3.zero, Quaternion.identity);

        RenderTexture.active = previousRT;
        Graphics.Blit(temp, splatMap);

        RenderTexture.ReleaseTemporary(temp);
    }
}
