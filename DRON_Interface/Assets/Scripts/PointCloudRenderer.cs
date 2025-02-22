using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.VFX;

public class PointCloudRenderer : MonoBehaviour
{
    Texture2D texColor;
    Texture2D texPosScale;
    VisualEffect vfx;
    public uint resolution = 2048;

    public float particleSize = 0.1f;
    public float boundsSize = 10f;
    bool toUpdate = false;
    uint particleCount = 0;

    public struct Point {
        public float size;
        public Color color;
        public Vector3 position;
    }

    private void Start() {
        vfx = GetComponent<VisualEffect>();

        Vector3[] positions = new Vector3[(int)resolution * (int)resolution];
        Color[] colors = new Color[(int)resolution * (int)resolution];

        for (int x = 0; x < (int)resolution; x++) {
            for (int y = 0; y < (int)resolution; y++) {
                positions[x + y * (int)resolution] = new Vector3(Random.value * boundsSize, Random.value * boundsSize, Random.value * boundsSize);
                colors[x + y * (int)resolution] = new Color(Random.value , Random.value, Random.value, 1);
            }
        }

        SetParticles(positions, colors);
    }

    void Update() {
        if (toUpdate) {
            toUpdate = false;

            vfx.Reinit();
            vfx.SetUInt(Shader.PropertyToID("ParticleCount"), particleCount);
            vfx.SetTexture(Shader.PropertyToID("TexColor"), texColor);
            vfx.SetTexture(Shader.PropertyToID("TexPosScale"), texPosScale);
            vfx.SetUInt(Shader.PropertyToID("Resolution"), resolution);
            vfx.SetFloat(Shader.PropertyToID("BoundsSize"), boundsSize);

        }
    }

    public void SetParticles(Vector3[] positions, Color[] colors) {
        texColor = new Texture2D(positions.Length > (int)resolution ? (int)resolution : positions.Length, Mathf.Clamp(positions.Length / (int)resolution, 1, (int)resolution), TextureFormat.RGBAFloat, false);
        texPosScale = new Texture2D(positions.Length > (int)resolution ? (int)resolution : positions.Length, Mathf.Clamp(positions.Length / (int)resolution, 1, (int)resolution), TextureFormat.RGBAFloat, false);
        int texWidth = texColor.width;
        int texHeight = texColor.height;

        for (int y = 0; y < texHeight; y++) {
            for (int x = 0; x < texWidth; x++) {
                int index = x + y * texWidth;
                texColor.SetPixel(x, y, colors[index]);
                var data = new Color(positions[index].x, positions[index].y, positions[index].z, particleSize);
                texPosScale.SetPixel(x, y, data);
            }
        }

        texColor.Apply();
        texPosScale.Apply();
        particleCount = (uint)positions.Length;
        toUpdate = true;

    }
}
