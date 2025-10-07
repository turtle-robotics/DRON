Shader "Lit/ThermalPaint"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
        _Coordinate("Coordinate",Vector)=(0,0,0,0)
        _Color("Paint Color",Color)=(1,1,1,1)
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
                float3 localPos : TEXCOORD1;
            };

            sampler2D _MainTex;
            float4 _MainTex_ST;
            fixed4 _Coordinate,_Color;

            v2f vert (appdata v)
            {
                v2f o;

                float2 uv = v.uv.xy;

                // https://docs.unity3d.com/Manual/SL-PlatformDifferences.html

                if (_ProjectionParams.x < 0) {
                    uv.y = 1 - uv.y;
                }

                // Convert from 0,1 to -1,1, for the blit
                o.vertex = float4(2 * (uv - 0.5), 0, 1);

                // We still need UVs to draw the base texture
                o.uv = TRANSFORM_TEX(v.uv, _MainTex);

                // Let's do the calculations in local space instead!
                o.localPos = v.vertex.xyz;

                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                // sample the texture
                fixed4 col = tex2D(_MainTex, i.uv);
                float draw =pow(saturate(1-distance(i.localPos,_Coordinate.xyz)),100);
                fixed4 drawcol = _Color * (draw * 1);
                return saturate(col + drawcol);
            }
            ENDCG
        }
    }
}