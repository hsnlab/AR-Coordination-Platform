// Upgrade NOTE: replaced 'mul(UNITY_MATRIX_MVP,*)' with 'UnityObjectToClipPos(*)'

Shader "Custom/UI/YUVRender"
{
	Properties
	{
		_MainTex("_MainTex", 2D) = "white" {}
		_UTex("_UTex", 2D) = "white" {}
		_VTex("_VTex", 2D) = "white" {}
	}
	SubShader
	{
		// No culling or depth
		Cull Off ZWrite Off ZTest Always


		Tags
		{
			"RenderType" = "Opaque"
		}

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
			};

			v2f vert(appdata v)
			{
				v2f o;
				o.vertex = UnityObjectToClipPos(v.vertex);
				o.uv = v.uv;
				return o;
			}

			sampler2D _MainTex;
			sampler2D _UTex;
			sampler2D _VTex;

			fixed4 frag(v2f i) : SV_Target
			{
				float xc = (floor(i.uv.x * 1280) + 0.5)/1280;
				float yc = (floor(i.uv.y*960) + 0.5) /960;
				float4 col = tex2D(_MainTex, i.uv);
				float4 u_t = tex2D(_UTex, float2(xc,yc) );
				float4 v_t = tex2D(_VTex, float2(xc, yc));

				//float y = 1.1643 * (col.a - 0.0625);
				float y = 1.1643*(col.a - 0.0625);
				float u = (u_t.a) - 0.5;
				float v = (v_t.a)- 0.5;

				float r = y + 1.596 * v;
				float g = y - 0.391  * u - 0.813 * v;
				float b = y + 2.018  * u;
				//float r = (y + 1.403 * v);
				//float g = (y - 0.344  * u - 0.714 * v) ;
				//float b = (y + 1.770  * u) ;

				col.rgba = float4(r, g, b, 1.f);
				return col;
			}
			ENDCG
		}
	}
}
