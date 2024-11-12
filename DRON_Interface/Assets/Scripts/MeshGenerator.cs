using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class MeshGenerator
{
    public static Mesh GeneratePlanarMesh(List<List<Vector3>> vertices) {
        Mesh mesh = new Mesh();

        mesh.name = "MeshRegion";
        //mesh.vertices = vertices;
        
        //int[] triangles = new int[(vertices.Length / 2 - 1) * 6];

        /*for (int i = 0; i < vertices.Length / 2 - 1; i++) {
            triangles[i*6] = i*2;
            triangles[i*6+1] = i*2+3;
            triangles[i*6+2] = i*2+2;

            triangles[i*6+3] = i*2+1;
            triangles[i*6+4] = i*2+3;
            triangles[i*6+5] = i*2;
        }*/

        List<Vector3> vertList = new List<Vector3>();
        List<int> triList = new List<int>();

        var emptyPoint = Vector3.one;

        int[] vertsPerRow = new int[vertices.Count];

        for (int i = 0; i < vertices.Count; i++) {
            for (int j = 0; j < vertices[i].Count; j++) {
                if (vertices[i][j] != emptyPoint) { // Point is real
                    vertList.Add(vertices[i][j]);
                    vertsPerRow[i]++;
                }
                //vertsPerRow[i]++;
            }
        }

        int index = 0;
        for (int i = 0; i < vertices.Count - 1; i++) {
            int nextRowOffset = vertsPerRow[i];
            for (int j = 0; j < vertices[i].Count - 1; j++) {

                if (vertices[i][j] != emptyPoint && vertices[i][j+1] != emptyPoint && vertices[i+1][j] != emptyPoint && vertices[i+1][j+1] != emptyPoint) {
                    // Standard Square

                    // Upper-left Triangle
                    triList.Add(index);
                    triList.Add(index + 1);
                    triList.Add(index + nextRowOffset);

                    // Lower-right Triangle
                    triList.Add(index + 1);
                    triList.Add(index + 1 + nextRowOffset);
                    triList.Add(index + nextRowOffset);
                } else if (vertices[i][j] != emptyPoint && vertices[i][j+1] != emptyPoint && vertices[i+1][j] == emptyPoint && vertices[i+1][j+1] != emptyPoint) {
                    // Upper-right triangle
                    Debug.Log("Upper-right");
                    triList.Add(index);
                    triList.Add(index + 1);
                    triList.Add(index + nextRowOffset);
                } else if (vertices[i][j] == emptyPoint && vertices[i][j+1] != emptyPoint && vertices[i+1][j] != emptyPoint && vertices[i+1][j+1] != emptyPoint) {
                    // Lower-right triangle
                    Debug.Log("Lower-right");
                    
                    triList.Add(index + 1);
                    triList.Add(index + 1 + nextRowOffset);
                    triList.Add(index + nextRowOffset);
                }
                

                if (vertices[i][j] != emptyPoint) index++;
            }
            
            index++;
        }

        mesh.vertices = vertList.ToArray();
        mesh.triangles = triList.ToArray();

        return mesh;
    }

    public static void GenerateCubeMesh(Vector3 offset, float scale, int triOffset, out Vector3[] vertices, out int[] triangles) {

        vertices = new Vector3[]{ new Vector3(0.5f, 0.5f, 0.5f) * scale + offset, new Vector3(-0.5f, 0.5f, 0.5f) * scale + offset, new Vector3(-0.5f, 0.5f, -0.5f) * scale + offset, new Vector3(0.5f, 0.5f, -0.5f) * scale + offset,
                          new Vector3(0.5f, -0.5f, 0.5f) * scale + offset, new Vector3(-0.5f, -0.5f, 0.5f) * scale + offset, new Vector3(-0.5f, -0.5f, -0.5f) * scale + offset, new Vector3(0.5f, -0.5f, -0.5f) * scale + offset };
        triangles = new int[]{ 0 + triOffset, 1 + triOffset, 2 + triOffset, 0 + triOffset, 2 + triOffset, 3 + triOffset, 4 + triOffset, 6 + triOffset, 5 + triOffset, 4 + triOffset, 7 + triOffset, 6 + triOffset, 0 + triOffset, 3 + triOffset, 7 + triOffset, 0 + triOffset, 7 + triOffset, 4 + triOffset, 2 + triOffset, 1 + triOffset, 5 + triOffset, 2 + triOffset, 5 + triOffset, 6 + triOffset, 1 + triOffset, 0 + triOffset, 4 + triOffset, 1 + triOffset, 4 + triOffset, 5 + triOffset, 3 + triOffset, 2 + triOffset, 6 + triOffset, 3 + triOffset, 6 + triOffset, 7 + triOffset };
    }
}
