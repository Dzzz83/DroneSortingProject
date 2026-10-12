Shader "DroneSensors/MetricDepth"
{
    SubShader
    {
        Tags { "RenderPipeline" = "HDRenderPipeline" }
        Pass
        {
            ZWrite Off
            ZTest Always
            Cull Off
            Blend Off

            HLSLPROGRAM
            #pragma target 4.5
            #pragma vertex Vert
            #pragma fragment MetricDepth
            #include "Packages/com.unity.render-pipelines.high-definition/Runtime/RenderPipeline/RenderPass/CustomPass/CustomPassCommon.hlsl"

            float MetricDepth(Varyings input) : SV_Target
            {
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(input);
                float deviceDepth = LoadCameraDepth(input.positionCS.xy);
                // Sky/no geometry becomes invalid, rather than a fictitious far surface.
                if (deviceDepth == UNITY_RAW_FAR_CLIP_VALUE)
                    return 0.0;
                return LinearEyeDepth(deviceDepth, _ZBufferParams);
            }
            ENDHLSL
        }
    }
    Fallback Off
}
