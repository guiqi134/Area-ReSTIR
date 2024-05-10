/***************************************************************************
 # Copyright (c) 2015-22, NVIDIA CORPORATION. All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/
import Utils.Attributes;
import Utils.Sampling.TinyUniformSampleGenerator;
import Utils.Debug.PixelDebug;
import SurfaceData;
import EvalContext;
import Lights;
import Resampling;
import Params;
import PixelAreaSampleData;

/** Performs temporal resampling by combining the reservoir from the initial
    resampling pass with a reservoir from the last frame obtained through
    reprojection.
*/
struct TemporalResampling
{
    // Static configuration.
    static const uint kMaxHistoryLength = MAX_HISTORY_LENGTH;
    static const bool kUnbiased = UNBIASED;
    static const bool kUsePrevFrameSceneData = USE_PREV_FRAME_SCENE_DATA;
    static const DebugOutput kDebugOutput = DebugOutput(DEBUG_OUTPUT);

    Texture2D<float2> motionVectors;
    RWStructuredBuffer<Reservoir> reservoirs;
    StructuredBuffer<Reservoir> prevReservoirs;
    RWStructuredBuffer<EvalContext> resEvalContext;
    StructuredBuffer<EvalContext> prevResEvalContext;
    StructuredBuffer<EvalContext> pixelCenterEvalContext;
    StructuredBuffer<EvalContext> prevPixelCenterEvalContext;
    StructuredBuffer<CameraData> prevCameraData;
    RWTexture2D<float4> debugOutput;
    StructuredBuffer<float> temporalMISPDFs;
    bool optimizeShift2RIS;

    Lights lights;

    uint2 frameDim;
    uint frameIndex;

    float normalThreshold;
    float depthThreshold;
    bool useMFactor;

    // Area ReSTIR variables
    PixelAreaSampleData pixelAreaSampleData;
    uint resampleEmissionMode;
    uint temporalReuseMode;
    uint shiftMappingModeRIS2;
    float filterRadius;
    float filterAlpha;
    float filterNorm;
    uint restirPassIdx;

    uint getPixelIndex(uint2 pixel) { return pixel.y * frameDim.x + pixel.x; }

    bool isValidPixel(int2 pixel) { return all(pixel >= 0 && pixel < frameDim); }

    void process(uint2 pixel)
    {
        if (any(pixel >= frameDim)) return;

        printSetPixel(pixel);
        print("Temporal resampling ------------------------------------------------", 1);

        const uint pixelIndex = getPixelIndex(pixel);

        // Compute pixel index in reservoir buffer
        uint pixelIdxInReservoirBuffer = getPixelIndex(frameDim, pixel, restirPassIdx);

        uint sampleNum = frameIndex + 8 * restirPassIdx;
        TinyUniformSampleGenerator sg = TinyUniformSampleGenerator(pixel, sampleNum);

        // Back-project this pixel to previous frame.
        const int2 prevPixel = pixel + motionVectors[pixel] * frameDim + (sampleNext2D(sg) * 1.f - 0.f);
        if (!isValidPixel(prevPixel)) return;
        const uint prevPixelIndex = getPixelIndex(prevPixel);
        uint prevPixelIdxInReservoirBuffer = getPixelIndex(frameDim, prevPixel, restirPassIdx);

        // Load reservoirs.
        Reservoir currentReservoir = reservoirs[pixelIdxInReservoirBuffer];
        Reservoir prevReservoir = prevReservoirs[prevPixelIdxInReservoirBuffer];
        prevReservoir.M = min(prevReservoir.M, currentReservoir.M * kMaxHistoryLength);

        // Load shading data
        EvalContext currentContext = resEvalContext[pixelIdxInReservoirBuffer];
        EvalContext prevContext = prevResEvalContext[prevPixelIdxInReservoirBuffer];

        // If don't resample length-1 path, just return background hit
        if (false && !currentContext.isSurfaceValid() && resampleEmissionMode == uint(ResampleEmissionMode::None))
        {
            reservoirs[pixelIdxInReservoirBuffer] = currentReservoir;
            resEvalContext[pixelIdxInReservoirBuffer] = currentContext;
            return;
        }

        // Point (baseline) ReSTIR
        if (!pixelAreaSampleData.reuseSubpixelSample && !pixelAreaSampleData.reuseLensSample)
        {
            // Setup resampling.
            RisState risState = RisState::createEmpty(currentReservoir);

            const LoadedLightSample currentLightSample = lights.loadLightSample(currentReservoir.lightSample, false);
            float currentTargetPdf = currentReservoir.targetPdf;
            float confidenceWeightSum = prevReservoir.M + currentReservoir.M;

            // First resample empty risState with neighbor cancdidate sample
            temporalResamplePointReservoirPairwiseMIS(risState, currentReservoir, currentContext, prevReservoir, prevContext, lights, useMFactor,
                confidenceWeightSum, 1.0f, kUsePrevFrameSceneData, sg, kUnbiased);

            // Then resample updated risState with current sample
            streamingResampleFinalizeMis(risState, currentReservoir, currentTargetPdf, sg);
            risState.weight = risState.targetPdf > 0.f ? risState.weightSum / risState.targetPdf : 0.f;

            // Save reservoir.
            reservoirs[pixelIdxInReservoirBuffer] = risState.toReservoir();
            resEvalContext[pixelIdxInReservoirBuffer] = currentContext;
        }
        // Pixel area sampling part (subpixel or lens reservoir)
        else
        {
            // Setup resampling.
            RisState risState = RisState::createEmpty(currentReservoir);
            EvalContext selectedSampleAtCurrContext = currentContext;

            const LoadedLightSample currentLightSample = lights.loadLightSample(currentReservoir.lightSample, false);
            float confidenceWeightSum = prevReservoir.M + currentReservoir.M;

            resampleAreaReservoirPairwiseMIS( risState, currentReservoir, currentContext, prevReservoir, prevContext, prevPixelCenterEvalContext[prevPixelIndex], frameDim, pixel, prevPixel,
                lights, useMFactor, confidenceWeightSum, 1.0f, kUsePrevFrameSceneData, true, shiftMappingModeRIS2, sg, selectedSampleAtCurrContext, kUnbiased, prevCameraData[0] );

            // Start canonical resampling
            if (streamingResampleFinalizeMis(risState, currentReservoir, currentReservoir.targetPdf, sg))
            {
                selectedSampleAtCurrContext = currentContext;
            }

            risState.weight = risState.targetPdf > 0.f ? risState.weightSum / risState.targetPdf : 0.f;

            // Save reservoir.
            reservoirs[pixelIdxInReservoirBuffer] = risState.toReservoir();

            // Store shading data
            resEvalContext[pixelIdxInReservoirBuffer] = selectedSampleAtCurrContext;
        }

        // Write debug output.
        if (kDebugOutput == DebugOutput::TemporalReuse) debugOutput[pixel] = float4(0.f, 1.f, 0.f, 0.f);
    }
};

cbuffer CB
{
    uint resampleResourceType;
    TemporalResampling gTemporalResampling;
}

[numthreads(16, 4, 1)]
void main(uint3 dispatchThreadId : SV_DispatchThreadID)
{
    gTemporalResampling.process(dispatchThreadId.xy);
}
