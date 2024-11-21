Shader "Unlit/ThermalPaint"
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
                float3 worldPos : TEXCOORD1;
            };

            sampler2D _MainTex;
            float4 _MainTex_ST;
            fixed4 _Coordinate,_Color;

            v2f vert (appdata v)
            {
                v2f o;

                float2 uv = v.texcoord.xy;

                // https://docs.unity3d.com/Manual/SL-PlatformDifferences.html

                if (_ProjectionParams.x < 0) {
                    uv.y = 1 - uv.y;
                }
                
                o.vertex = float4(2 * (uv - 0.5), 0, 1);
                o.uv = TRANSFORM_TEX(v.uv, _MainTex);
                o.localPos = v.vertex.xyz;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                // sample the texture
                fixed4 col = tex2D(_MainTex, i.uv);
                
                float draw = pow(saturate(1-distance(i.worldPos,_Coordinate.xyz)),100);
                fixed4 drawcol = _Color * (draw * 1);
                return saturate(col + drawcol);
            }
            ENDCG
        }
    }
}
