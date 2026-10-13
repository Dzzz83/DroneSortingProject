Shader "Hidden/DroneDataset/VisiblePackageId"
{
    SubShader
    {
        Tags { "RenderPipeline" = "HDRenderPipeline" }
        Pass
        {
            ZWrite Off
            ZTest Always
            Cull Back
            Blend Off
            HLSLPROGRAM
            #pragma target 4.5
            #pragma vertex PackageVertex
            #pragma fragment PackageFragment
            #include "Packages/com.unity.render-pipelines.high-definition/Runtime/RenderPipeline/RenderPass/CustomPass/CustomPassCommon.hlsl"
            float4 _DatasetPackageColor;
            struct PackageAttributes { float3 positionOS : POSITION; };
            struct PackageVaryings { float4 positionCS : SV_POSITION; };
            PackageVaryings PackageVertex(PackageAttributes input)
            {
                PackageVaryings output;
                output.positionCS = TransformWorldToHClip(TransformObjectToWorld(input.positionOS));
                return output;
            }
            float4 PackageFragment(PackageVaryings input) : SV_Target
            {
                float sceneDepth = LoadCameraDepth(input.positionCS.xy);
                float packageMetres = LinearEyeDepth(input.positionCS.z, _ZBufferParams);
                float sceneMetres = LinearEyeDepth(sceneDepth, _ZBufferParams);
                // Compare against the actual opaque scene: hidden packages must not receive labels.
                // A 1 mm allowance absorbs floating-point depth differences between the two draws.
                clip(sceneMetres + 0.001 - packageMetres);
                return _DatasetPackageColor;
            }
            ENDHLSL
        }
    }
    Fallback Off
}
