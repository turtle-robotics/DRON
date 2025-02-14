using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//[RequireComponent(MeshFilter)]
public class VertexPaint : MonoBehaviour
{
    public Camera cam;
    public Shader paintShader;

    RenderTexture splatMap;
    Material snowMaterial,drawMaterial;

    RaycastHit hit;

    MeshFilter meshFilter;

    private void Awake()
    {
        Application.targetFrameRate = 200;
    }
    void Start()
    {
        drawMaterial = new Material(paintShader);
        drawMaterial.SetVector("_Color", Color.red);

        snowMaterial = GetComponent<MeshRenderer>().material;
        splatMap = new RenderTexture(1000, 1000, 0, RenderTextureFormat.ARGBFloat);
        snowMaterial.mainTexture = splatMap;

        meshFilter = GetComponent<MeshFilter>();
    }
    void Update()
    {
        if (Input.GetMouseButton(0))
        {
            if(Physics.Raycast(cam.ScreenPointToRay(Input.mousePosition),out hit))
            {
                Debug.Log("Hit");
                drawMaterial.SetVector("_Coordinate", transform.InverseTransformPoint(new Vector4(hit.point.x, hit.point.y, hit.point.z, 0)));
                
                RenderTexture temp = RenderTexture.GetTemporary(splatMap.width, splatMap.height, 0, RenderTextureFormat.ARGBFloat);

                // Cache the old target so that we can reset it later
                RenderTexture previousRT = RenderTexture.active;
                RenderTexture.active = temp;

                Material mat = drawMaterial;
                Mesh mesh = meshFilter.mesh;
                mat.SetTexture("_MainTex", splatMap);
                mat.SetPass(0); // This tells the renderer to use pass 0 from this material
                Graphics.DrawMeshNow(mesh, Vector3.zero, Quaternion.identity);

                // Remember to reset the render target
                RenderTexture.active = previousRT;
                Graphics.Blit(temp, splatMap);

                RenderTexture.ReleaseTemporary(temp);
            }
        }
    }
}
