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
import Utils.Sampling.TinyUniformSampleGenerator;
import Utils.Debug.PixelDebug;
import SurfaceData;
import EvalContext;
import Lights;
import Resampling;
import Params;
import PixelAreaSampleData;

/** Performs spatial resampling by combining the output reservoir from the
    temporal resampling pass with one or multiple reservoirs from neighbor
    pixels.
*/
struct SpatialResampling
{
    // Static configuration.
    static const uint kNeighborOffsetCount = NEIGHBOR_OFFSET_COUNT;
    static const uint kNeighborOffsetMask = kNeighborOffsetCount - 1;

    static bool kUnbiased = UNBIASED;

    static const DebugOutput kDebugOutput = DebugOutput(DEBUG_OUTPUT);

    // static const bool kUseNeighborCheck = true;

    // Resources.
    Texture2D<uint> normalDepth;
    RWStructuredBuffer<Reservoir> reservoirs;
    StructuredBuffer<Reservoir> prevReservoirs;
    RWStructuredBuffer<EvalContext> resEvalContext;
    StructuredBuffer<EvalContext> prevResEvalContext; // overload for current
    StructuredBuffer<EvalContext> pixelCenterEvalContext; // overload for current
    Texture1D<float2> neighborOffsets;
    RWTexture2D<float4> debugOutput;

    Lights lights;
    PixelAreaSampleData pixelAreaSampleData;

    uint2 frameDim;         ///< Current frame dimensions.
    uint frameIndex;        ///< Current frame index.
    float normalThreshold;
    float depthThreshold;
    bool useMFactor;
    uint neighborCount;     ///< Number of neighbor reservoirs to resample.
    float gatherRadius;     ///< Maximum radius to gather neighbors from.

    uint shiftMappingMode;
    float randomReplaySampleWeight;
    uint misSampleSelection;
    bool rejectNeighborPixelForNormalDepth;
    bool rejectNeighborPixelForHitType;
    uint resampleEmissionMode;
    uint restirPassIdx;
    uint spatialPassIdx;

    uint getPixelIndex(uint2 pixel) { return pixel.y * frameDim.x + pixel.x; }

    bool isValidPixel(int2 pixel) { return all(pixel >= 0 && pixel < frameDim); }

    void process(uint2 pixel)
    {
        if (any(pixel >= frameDim)) return;

        // printSetPixel(pixel);
        print("Spatial Resampling -----------------------------------------------------------------", 2);

        if (kDebugOutput == DebugOutput::SpatialReuse) debugOutput[pixel] = float4(1.f, 0.f, 0.f, 0.f);

        const uint pixelIndex = getPixelIndex(pixel);

        // Compute pixel index in reservoir buffer
        uint pixelIdxInReservoirBuffer = getPixelIndex(frameDim, pixel, restirPassIdx);
        uint sampleNum = (32 * frameIndex + spatialPassIdx) + 8 * restirPassIdx;
        TinyUniformSampleGenerator sg = TinyUniformSampleGenerator(pixel, sampleNum);

        // Setup evaluation context for current/resampled reservoir
        const Reservoir currentReservoir = prevReservoirs[pixelIdxInReservoirBuffer];
        EvalContext currEvalContext = prevResEvalContext[pixelIdxInReservoirBuffer];
        EvalContext currCenterContext = pixelCenterEvalContext[pixelIndex];

        // Point ReSITR part
        if (!pixelAreaSampleData.reuseSubpixelSample && !pixelAreaSampleData.reuseLensSample)
        {
            // Setup resampling.
            RisState risState = RisState::createEmpty(currentReservoir);
            const uint startIndex = sampleNext1D(sg) * kNeighborOffsetCount;
            uint validCount = 0;
            uint validNeighborCache = 0;

            // Compute confidence weight sum
            float confidenceWeightSum = currentReservoir.M;
            for (uint i = 0; i < neighborCount; i++)
            {
                // Get neighbor pixel
                uint neighborIndex = (startIndex + i) & kNeighborOffsetMask;
                int2 neighborPixel = pixel + int2(neighborOffsets[neighborIndex] * gatherRadius);
                if (!isValidPixel(neighborPixel)) continue;
                uint neighborPixelIndex = getPixelIndex(neighborPixel);
                uint neighborPixelIndexInReservoir = getPixelIndex(frameDim, neighborPixel, restirPassIdx);

                // Check for valid neighbor
                const EvalContext neighborCenterContext = pixelCenterEvalContext[neighborPixelIndex];
                if (rejectNeighborPixelForNormalDepth && !currEvalContext.isValidNeighbor( float4(neighborCenterContext.N, neighborCenterContext.depth), normalThreshold, depthThreshold )) continue;

                // Check for valid neighbor based on hit type
                if (rejectNeighborPixelForHitType && currCenterContext.isSurfaceValid() != neighborCenterContext.isSurfaceValid()) continue;

                // Get neighbor reservoir
                const Reservoir neighborReservoir = prevReservoirs[neighborPixelIndexInReservoir];
                confidenceWeightSum += neighborReservoir.M;

                validNeighborCache |= (1 << i);
            }
            risState.canonicalWeight = confidenceWeightSum > 0.0f ? float(currentReservoir.M) / confidenceWeightSum : 0.0f; // for defensive pairwise MIS

            // Start resampling
            for (uint i = 0; i < neighborCount; ++i)
            {
                // Skip invalid neighbors.
                if (!(validNeighborCache & (1 << i))) continue;

                // Get neighbor pixel.
                uint neighborIndex = (startIndex + i) & kNeighborOffsetMask;
                int2 neighborPixel = pixel + int2(neighborOffsets[neighborIndex] * gatherRadius);
                if (!isValidPixel(neighborPixel)) continue;
                uint neighborPixelIndex = getPixelIndex(neighborPixel);
                uint neighborPixelIndexInReservoir = getPixelIndex(frameDim, neighborPixel, restirPassIdx);

                print("neighborPixelIndex = ", neighborPixelIndex);
                print("neighborPixelIndexInReservoir = ", neighborPixelIndexInReservoir);

                const Reservoir neighborReservoir = prevReservoirs[neighborPixelIndexInReservoir];
                const EvalContext neighborContext = prevResEvalContext[neighborPixelIndexInReservoir];

                spatialResamplePointReservoirPairwiseMIS(risState, currentReservoir, currEvalContext, neighborReservoir, neighborContext, lights, useMFactor, confidenceWeightSum, sg, kUnbiased);
            }

            streamingResampleFinalizeMis(risState, currentReservoir, currentReservoir.targetPdf, sg);
            risState.M = currentReservoir.M;
            risState.weight = risState.targetPdf > 0.f ? risState.weightSum / risState.targetPdf : 0.f;

            // Save reservoir.
            reservoirs[pixelIdxInReservoirBuffer] = risState.toReservoir();

            // Save context
            resEvalContext[pixelIdxInReservoirBuffer] = currEvalContext;

            // Write debug output.
            if (kDebugOutput == DebugOutput::SpatialReuse)
                debugOutput[pixel] = float4(lerp(float3(1.f, 0.f, 0.f), float3(0.f, 1.f, 0.f), float(validCount) / neighborCount), 0.f);
        }
        // Area ReSTIR part
        else
        {
            // Setup resampling.
            RisState risState = RisState::createEmpty(currentReservoir);
            EvalContext selectedSampleAtCurrContext = currEvalContext;
            const uint startIndex = sampleNext1D(sg) * kNeighborOffsetCount;

            // Decide how many samples should use
            uint randomReplaySampleCount = uint(randomReplaySampleWeight * neighborCount);

            // Another way is to compute CoC using current center pixel's g-buffer data
            if (misSampleSelection == uint(SpatialSampleSelectionInMisShift::SelectUniqueSample_CenterPixelCoC))
            {
                float randomReplayWeightCoC = computeConfidenceWeightForTwoShifts(frameDim, pixelCenterEvalContext[pixelIndex], gScene.camera.data); // center context is the same for RGB
                randomReplaySampleCount = uint(randomReplayWeightCoC * neighborCount);
            }

            // Compute confidence weight sum
            uint validNeighborCache = 0;
            uint validNeighborCount = 0;
            float confidenceWeightSum = currentReservoir.M;
            for (uint i = 0; i < neighborCount; i++)
            {
                // Get neighbor pixel
                uint neighborIndex = (startIndex + i) & kNeighborOffsetMask;
                int2 neighborPixel = pixel + int2(neighborOffsets[neighborIndex] * gatherRadius);
                if (!isValidPixel(neighborPixel)) continue;
                uint neighborPixelIndex = getPixelIndex(neighborPixel);
                uint neighborPixelIndexInReservoir = getPixelIndex(frameDim, neighborPixel, restirPassIdx);

                // Check for valid neighbor baesd on normal and depth
                const EvalContext neighborCenterContext = pixelCenterEvalContext[neighborPixelIndex];
                if (rejectNeighborPixelForNormalDepth && !currCenterContext.isValidNeighbor( float4(neighborCenterContext.N, neighborCenterContext.depth), normalThreshold, depthThreshold )) continue;

                // Check for valid neighbor based on hit type
                if (rejectNeighborPixelForHitType && currCenterContext.isSurfaceValid() != neighborCenterContext.isSurfaceValid()) continue;

                // Get neighbor reservoir
                const Reservoir neighborReservoir = prevReservoirs[neighborPixelIndexInReservoir];
                confidenceWeightSum += neighborReservoir.M;

                validNeighborCache |= (1 << i);
                validNeighborCount++;
            }
            risState.canonicalWeight = confidenceWeightSum > 0.0f ? float(currentReservoir.M) / confidenceWeightSum : 0.0f; // for defensive pairwise MIS

            // Resample neighbor samples
            for (uint i = 0; i < neighborCount; ++i)
            {
                // Skip invalid neighbors.
                if (!(validNeighborCache & (1 << i))) continue;

                // Get neighbor pixel.
                uint neighborIndex = (startIndex + i) & kNeighborOffsetMask;
                int2 neighborPixel = pixel + int2(neighborOffsets[neighborIndex] * gatherRadius);
                if (!isValidPixel(neighborPixel)) continue;
                uint neighborPixelIndex = getPixelIndex(neighborPixel);
                uint neighborPixelIndexInReservoir = getPixelIndex(frameDim, neighborPixel, restirPassIdx);

                const Reservoir neighborReservoir = prevReservoirs[neighborPixelIndexInReservoir];
                EvalContext neighborContext = prevResEvalContext[neighborPixelIndexInReservoir];

                if (shiftMappingMode != uint(ShiftMappingModeInReusing::MIS) || misSampleSelection == uint(SpatialSampleSelectionInMisShift::DuplicateSample))
                {
                    resampleAreaReservoirPairwiseMIS( risState, currentReservoir, currEvalContext, neighborReservoir, neighborContext, pixelCenterEvalContext[neighborPixelIndex], frameDim,
                        pixel, neighborPixel, lights, useMFactor, confidenceWeightSum, 1.0f, true, false, shiftMappingMode, sg, selectedSampleAtCurrContext, kUnbiased );
                }
                else
                {
                    uint neighborPixelShiftMode;
                    if (misSampleSelection == uint(SpatialSampleSelectionInMisShift::SelectUniqueSample_NeighborPixelCoC))
                    {
                        float randomReplayWeightCoC = computeConfidenceWeightForTwoShifts( frameDim, pixelCenterEvalContext[neighborPixelIndex], gScene.camera.data );
                        neighborPixelShiftMode = uint( sampleNext1D(sg) < randomReplayWeightCoC ? ShiftMappingModeInReusing::OnlyRandomReplay : ShiftMappingModeInReusing::OnlyReconnection );
                    }
                    else
                    {
                        neighborPixelShiftMode = uint( i < randomReplaySampleCount ? ShiftMappingModeInReusing::OnlyRandomReplay : ShiftMappingModeInReusing::OnlyReconnection );
                    }

                    resampleAreaReservoirPairwiseMIS( risState, currentReservoir, currEvalContext, neighborReservoir, neighborContext, pixelCenterEvalContext[neighborPixelIndex], frameDim,
                        pixel, neighborPixel, lights, useMFactor, confidenceWeightSum, 1.0f, true, false, neighborPixelShiftMode, sg, selectedSampleAtCurrContext, kUnbiased );
                }
            }

            if (streamingResampleFinalizeMis(risState, currentReservoir, currentReservoir.targetPdf, sg))
            {
                selectedSampleAtCurrContext = currEvalContext;
            }

            risState.M = currentReservoir.M;
            risState.weight = risState.targetPdf > 0.f ? risState.weightSum / risState.targetPdf : 0.f;
            print("risState.weight = ", risState.weight);

            // Save reservoir.
            reservoirs[pixelIdxInReservoirBuffer] = risState.toReservoir();

            // Save shading data
            resEvalContext[pixelIdxInReservoirBuffer] = selectedSampleAtCurrContext;

            // Write debug output.
            if (kDebugOutput == DebugOutput::SpatialReuse)
                debugOutput[pixel] = float4(lerp(float3(1.f, 0.f, 0.f), float3(0.f, 1.f, 0.f), float(validNeighborCount) / neighborCount), 0.f);
        }
    }
};

cbuffer CB
{
    uint resampleResourceType;
    SpatialResampling gSpatialResampling;
}

[numthreads(16, 16, 1)]
void main(uint3 dispatchThreadId : SV_DispatchThreadID)
{
    gSpatialResampling.process(dispatchThreadId.xy);
}
