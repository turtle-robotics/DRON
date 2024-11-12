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

    // Start is called before the first frame update
    void Start()
    {
        maxConcentration = (int)(maxConcentrationPerCubicMeter / (gridSize * gridSize * gridSize));
        meshFilter = GetComponent<MeshFilter>();
        meshRenderer = GetComponent<MeshRenderer>();
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
}
