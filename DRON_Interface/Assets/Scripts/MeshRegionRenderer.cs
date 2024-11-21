using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class MeshRegionRenderer : MonoBehaviour
{
    float gridSize = 0.5f;
    int maxConcentrationPerCubicMeter = 1000;
    int maxConcentration;
    Dictionary<string, int> concentrations = new Dictionary<string, int>();
    Dictionary<string, List<Vector3>> verticesGridRef = new Dictionary<string, List<Vector3>>();

    private MeshFilter meshFilter;
    private MeshRenderer meshRenderer;


    public Shader paintShader;

    RenderTexture splatMap;
    Material unpaintedMaterial,paintMaterial;

    // Start is called before the first frame update
    void Start()
    {
        maxConcentration = (int)(maxConcentrationPerCubicMeter / (gridSize * gridSize * gridSize));
        meshFilter = GetComponent<MeshFilter>();
        meshRenderer = GetComponent<MeshRenderer>();

        // Mesh Painting Prep
        paintMaterial = new Material(paintShader);
        paintMaterial.SetVector("_Color", Color.red);

        unpaintedMaterial = meshRenderer.material;
        splatMap = new RenderTexture(1024, 1024, 0, RenderTextureFormat.ARGBFloat);
        unpaintedMaterial.mainTexture = splatMap;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void UpdateMesh(Mesh newMesh, Material mat, int height, int width, int everyOther) {
        if (meshFilter == null) {
            meshFilter = GetComponent<MeshFilter>();
        }
        if (meshRenderer == null) {
            meshRenderer = GetComponent<MeshRenderer>();
        }


        /*Vector3[] newVertices = newMesh.vertices;
        int[] newTriangles = newMesh.triangles;

        Dictionary<string, int> newConcentrations = new Dictionary<string, int>();
        Dictionary<string, List<Vector3>> newVerticesGridRef = new Dictionary<string, List<Vector3>>();

        // REDO to be cube-based

        // Generates concentrations of the new mesh
        for (int i = 0; i < newVertices.Length; i++) {
            Vector3Int gridPos = Vector3Int.FloorToInt(newVertices[0] / gridSize);
            
            string key = gridPos.x + "_" + gridPos.y + "_" + gridPos.z;
            //Debug.Log(key);

            if (newConcentrations.ContainsKey(key)) {
                newConcentrations[key]++;
                newVerticesGridRef[key].Add(newVertices[i]);
            } else {
                newConcentrations.Add(key, 1);
                List<Vector3> gridVertices = new List<Vector3>();
                gridVertices.Add(newVertices[i]);
                newVerticesGridRef.Add(key, gridVertices);
            }
        }

        List<Vector3> vertices;
        List<int> triangles;

        if (meshFilter.mesh == null) {
            vertices = new List<Vector3>();
            triangles = new List<int>();
        } else {
            //vertices = new List<Vector3>(meshFilter.mesh.vertices);
            //triangles = new List<int>(meshFilter.mesh.triangles);
            vertices = new List<Vector3>();
            triangles = new List<int>();
        }

        foreach (var item in newConcentrations) {
            if (!concentrations.ContainsKey(item.Key)) { // Grid position doesn't previously exist
                concentrations.Add(item.Key, item.Value);
                verticesGridRef.Add(item.Key, newVerticesGridRef[item.Key]);

                vertices.AddRange(newVerticesGridRef[item.Key]);
            } else {
                if (concentrations[item.Key] < item.Value) {
                    concentrations[item.Key] = item.Value;
                    verticesGridRef[item.Key] = newVerticesGridRef[item.Key];

                    vertices.AddRange(newVerticesGridRef[item.Key]);
                } else {
                    vertices.AddRange(verticesGridRef[item.Key]);
                }
            }
        }*/

        //Mesh mesh = 

        meshFilter.mesh = newMesh;
        meshRenderer.material = mat;
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
