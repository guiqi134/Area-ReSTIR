/***************************************************************************
 # Copyright (c) 2015-22, NVIDIA CORPORATION. All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/
import Scene.Scene;
import Utils.Attributes;
import Utils.Debug.PixelDebug;
import Utils.Sampling.TinyUniformSampleGenerator;
import EvalContext;
import Lights;
import Reservoir;
import FinalSample;
import Params;
import PixelAreaSampleData;
import Resampling;

/** Evaluates reservoirs for final shading and stores the result in the finalSamples buffer.
*/
struct EvaluateFinalSamples
{
    // Static configuration.
    static const bool kUseVisibility = USE_VISIBILITY;
    static const bool kReuseVisibility = REUSE_VISIBILITY;
    static const bool kUnbiased = UNBIASED;
    static const uint kMaxHistoryLength = MAX_HISTORY_LENGTH;
    static const float3 kRelativeLumiance = float3(0.2126f, 0.7152f, 0.0722f);
    static const float3x3 kColorConfidenceScales = {
        // Which color channel integration (c_i) on for which color reservoir (c_ij)
        // c_rr, c_rg, c_rb  |  c_gr, c_gg, c_gb  |  c_br, c_bg, c_bb
        float3(1.0, 0.2, 0.2), float3(0.2, 1.0, 0.2), float3(0.2, 0.2, 1.0)
        // float3(0.8, 0.1, 0.1), float3(0.1, 0.8, 0.1), float3(0.1, 0.1, 0.8)
    };

    static const DebugOutput kDebugOutput = DebugOutput(DEBUG_OUTPUT);

    // Resources.
    RWStructuredBuffer<Reservoir> reservoirs;
    RWStructuredBuffer<EvalContext> resEvalContext;
    RWStructuredBuffer<CameraData> prevCameraData;
    RWStructuredBuffer<FinalSample> finalSamples;
    RWStructuredBuffer<PackedHitInfo> finalPrimaryHits;
    RWTexture2D<float4> debugOutput;

    Lights lights;
    uint2 frameDim; ///< Frame dimension (pixels).
    uint frameIndex;
    uint numRestirPasses;
    PixelAreaSampleData pixelAreaSampleData;

    uint getPixelIndex(uint2 pixel) { return pixel.y * frameDim.x + pixel.x; }

    float3 evalPathIncidentRadiance(Reservoir reservoir, EvalContext evalContext, uint pixelIdxInReservoirBuffer, inout FinalSample finalSample)
    {
        if (reservoir.pathSample == 1)
        {
            finalSample.Li = reservoir.weight * evalContext.emission;
        }
        else if (reservoir.pathSample == 2 && reservoir.isLightSampleValid())
        {
            // Evaluate light sample.
            EvaluatedLightSample evaluatedLightSample = lights.evalLightSample(reservoir.lightSample, evalContext.pos, false);

            // Evaluate visibility.
            float visibility = 1.f;
            if (kUseVisibility) visibility = evalContext.evalVisibility(evaluatedLightSample, false);
            print("kUseVisibility = ", kUseVisibility);

            if (visibility > 0.f)
            {
                finalSample.dir = evaluatedLightSample.dir;
                finalSample.distance = evaluatedLightSample.distance;
                finalSample.Li = reservoir.weight * evaluatedLightSample.geomFactor * lights.evalEmission(reservoir.lightSample, -finalSample.dir);
            }
        }

        return finalSample.Li;
    }

    void process(uint2 pixel)
    {
        if (any(pixel >= frameDim)) return;

        // printSetPixel(pixel);
        print("EvaluateFinalSamples() called --------------------------------", 3);

        uint pixelIndex = getPixelIndex(pixel);
        TinyUniformSampleGenerator sg = TinyUniformSampleGenerator(pixel, frameIndex);

        // If seperate RGB, we need to store 3 seperate final samples
        for (uint i = 0; i < numRestirPasses; i++)
        {
            FinalSample finalSample = {};
            uint pixelIdxInReservoirBuffer = getPixelIndex(frameDim, pixel, i);

            // Get evaluation context.
            EvalContext evalContext = resEvalContext[pixelIdxInReservoirBuffer];

            // Load reservoir.
            Reservoir reservoir = reservoirs[pixelIdxInReservoirBuffer];

            // Write weighted incident radiance = r.W * Li * G
            evalPathIncidentRadiance(reservoir, evalContext, pixelIdxInReservoirBuffer, finalSample);

            finalSample.misWeight = 1.0f / numRestirPasses;

            // No primary hit
            if (reservoir.M == 0) evalContext.hit = {};

            // Pixel uv sample should not be affected by invalid light sample
            finalSample.pixelSampleUV = reservoir.pixelSampleUV;
            finalSample.lensSampelUV = reservoir.lensSampleUV;
            finalSample.pathSample = reservoir.pathSample;

            // Write final sample.
            finalSamples[pixelIdxInReservoirBuffer] = finalSample;

            // Write this reservoir's final evaluation context
            resEvalContext[pixelIdxInReservoirBuffer] = evalContext;

            // Write the final primary hits
            finalPrimaryHits[pixelIdxInReservoirBuffer] = evalContext.hit.getData();
        }

        // Write this frame's camera data
        prevCameraData[0] = gScene.camera.data;
    }
};

cbuffer CB
{
    uint resampleResourceType;
    EvaluateFinalSamples gEvaluateFinalSamples;
}

[numthreads(16, 16, 1)]
void main(uint3 dispatchThreadId : SV_DispatchThreadID)
{
    gEvaluateFinalSamples.process(dispatchThreadId.xy);
}
