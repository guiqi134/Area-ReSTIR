/***************************************************************************
 # Copyright (c) 2015-22, NVIDIA CORPORATION. All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/
#include "Utils/Math/MathConstants.slangh"
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

import Scene.RaytracingInline;
import LoadShadingData;

/** Performs initial resampling from light samples in the light tiles.
*/
struct InitialResampling
{
    // Static configuration.
    static const uint kLightTileCount = LIGHT_TILE_COUNT;
    static const uint kLightTileSize = LIGHT_TILE_SIZE;

    static const uint kScreenTileSize = SCREEN_TILE_SIZE;
    static const uint kInitialLightSampleCount = INITIAL_LIGHT_SAMPLE_COUNT;
    static const uint kInitialBRDFSampleCount = INITIAL_BRDF_SAMPLE_COUNT;
    static const uint kInitialPathSampleCount = INITIAL_PATH_SAMPLE_COUNT;
    static const float kBRDFSampleRatio = float(kInitialBRDFSampleCount) / float(kInitialLightSampleCount + kInitialBRDFSampleCount);
    // static const bool kSpecularOnly = false;

    static const bool kCheckVisibility = CHECK_VISIBILITY;

    static const DebugOutput kDebugOutput = DebugOutput(DEBUG_OUTPUT);

    // Resources.
    [root] StructuredBuffer<SurfaceData> surfaceData;
    [root] StructuredBuffer<SurfaceData> centerSurfaceData; // for pixel center
    Texture2D<uint> normalDepth;
    Texture2D<float3> viewDir;
    StructuredBuffer<LoadedLightSample> lightTileData;
    RWStructuredBuffer<Reservoir> reservoirs;
    RWStructuredBuffer<EvalContext> resEvalContext;
    RWStructuredBuffer<EvalContext> pixelCenterEvalContext;
    RWTexture2D<float4> debugOutput;

    // General variables
    Lights lights;
    uint2 frameDim;
    uint frameIndex;
    float brdfCutoff;

    // Area ReSTIR variables
    PixelAreaSampleData pixelAreaSampleData;
    uint resampleEmissionMode;
    float filterRadius;
    float filterAlpha;
    float filterNorm;
    uint restirPassIdx;

    enum class SamplingTechnique
    {
        Light, BRDF
    };

    uint getPixelIndex(uint2 pixel)
    {
        return pixel.y * frameDim.x + pixel.x; // which pixel?
    }

    /** Heuristic to determine a max visibility ray length from a PDF wrt. solid angle.
        \param[in] pdf PDF wrt. solid angle.
    */
    float maxDistanceFromPdf(float pdf)
    {
        return brdfCutoff > 0.f ? sqrt((1.f / brdfCutoff - 1.f) * pdf) : Lights::kRayTMax;
    }

    float evalBalanceHeuristicMIS(const EvalContext evalContext, const LightSample lightSample, const EvaluatedLightSample evaluatedLightSample, SamplingTechnique technique)
    {
        float misWeight = 1.0f;
        float p_light = evaluatedLightSample.pdf;

        if (kInitialBRDFSampleCount > 0)
        {
            float p_BRDF = evalContext.evalBsdfPDF(evaluatedLightSample.dir, false) * evaluatedLightSample.geomFactor; // change from solid angle to area measure
            float maxDistance = maxDistanceFromPdf(p_BRDF);
            if (lightSample.getKind() == LightKind::Emissive && evaluatedLightSample.distance > maxDistance) p_BRDF = 0.f;
            if (lightSample.getKind() == LightKind::Analytic) p_BRDF = 0.f;

            misWeight = (technique == SamplingTechnique::Light ? p_light : p_BRDF) / (kInitialLightSampleCount * p_light + kInitialBRDFSampleCount * p_BRDF);
        }
        else
        {
            misWeight = p_light / (kInitialLightSampleCount * p_light);
        }

        return misWeight;
    }

    void process(uint2 pixel)
    {
        if (any(pixel >= frameDim)) return;

        printSetPixel(pixel);
        print("Initial Resampling DI ------------------------------------------------------", 0);

        if (kDebugOutput == DebugOutput::InitialWeight) debugOutput[pixel] = float4(0.f);

        uint pixelIndex = getPixelIndex(pixel);

        // Setup evaluation context.
        SurfaceData sd = surfaceData[pixelIndex];
        float2 subPixelUV = pixelAreaSampleData.subPixelUV[pixel];
        float2 lensUV = pixelAreaSampleData.lensUV[pixel];
        EvalContext evalContext = EvalContext::create(pixel, frameDim, sd, viewDir[pixel]);

        // Determine screen tile.
        // Each screen tile uses light samples from a random light tile for improved coherency.
        uint2 screenTile = pixel / kScreenTileSize; // Song: what's the difference between this and kScreenTileDim?

        // Determine light tile to use for this screen tile.
        //  -> pixels in the same screen tile will select the same light tile to sample
        uint sampleNum = frameIndex + 8 * restirPassIdx;
        TinyUniformSampleGenerator tileSg = TinyUniformSampleGenerator(screenTile, sampleNum);
        uint tileIndex = min(uint(sampleNext1D(tileSg) * kLightTileCount), kLightTileCount - 1);
        uint lightTileOffset = tileIndex * kLightTileSize;

        // Determine offset and stride to stratify samples within the light tile.
        //  -> Default stride = 4, so it will sample every 4-th sample in the light tile
        //  -> Offset will be a random between [0, stride]
        TinyUniformSampleGenerator sg = TinyUniformSampleGenerator(pixel, sampleNum);
        uint stride = (kLightTileSize + kInitialLightSampleCount - 1) / kInitialLightSampleCount;
        uint offset = min(uint(sampleNext1D(sg) * stride), stride - 1);

        // Background hit no need to do any resampling, it path sample will be fixed to 1
        if (!evalContext.isSurfaceValid())
        {
            print("Background hit!", 1);

            uint pixelIdxInReservoirBuffer = getPixelIndex(frameDim, pixel, restirPassIdx);

            Reservoir reservoir = Reservoir::createEmpty(subPixelUV, lensUV, 1u);
            reservoir.M = 1;
            reservoir.targetPdf = evalContext.evalTargetFunction( {}, lights, false, false, sg, 1);
            reservoir.weight = 1.0f;
            resEvalContext[pixelIdxInReservoirBuffer] = evalContext;
            reservoirs[pixelIdxInReservoirBuffer] = reservoir;

            return;
        }

        // Setup resampling. If we can reach here, this pixel will not hit background
        RisState risState = RisState::createEmpty(subPixelUV, lensUV, 2u);

        // Resample light samples from the light tile (only if we hit the valid surface).
        if (evalContext.isSurfaceValid())
        {
            for (uint i = 0; i < kInitialLightSampleCount; ++i)
            {
                // Pick light sample and evaluate it.
                uint index = lightTileOffset + (offset + i * stride) % kLightTileSize;
                const LoadedLightSample loadedLightSample = lightTileData[index];
                const EvaluatedLightSample evaluatedLightSample = loadedLightSample.eval(evalContext.pos);
                float p = evaluatedLightSample.pdf;
                float misWeight = evalBalanceHeuristicMIS(evalContext, loadedLightSample.lightSample, evaluatedLightSample, SamplingTechnique::Light);
                float pHat = evalContext.evalTargetFunction(loadedLightSample, lights, false, false, sg, 2u);
                streamingInitialResampleMIS(risState, loadedLightSample.lightSample, pHat, p, misWeight, sg);
            }

            // Resample BRDF samples.
            //  -> we need to sample a direction from BRDF and then trace the ray from the surface to see whether this is
            //     going to hit a light
            if (kInitialBRDFSampleCount > 0)
            {
                for (uint i = 0; i < kInitialBRDFSampleCount; ++i)
                {
                    LightSample lightSample = LightSample::createInvalid();
                    float3 dir;
                    float pdf;
                    if (evalContext.sampleBSDF(false, sg, dir, pdf))
                    {
                        float brdfPdf = evalContext.evalBsdfPDF(dir, false);
                        float maxDistance = maxDistanceFromPdf(brdfPdf);
                        const Ray ray = Ray(evalContext.pos, dir, 0.f, maxDistance + 0.01f);
                        HitInfo hit;
                        if (Lights::kUseEmissiveLights && evalContext.traceRay(ray, hit, false))
                        {
                            // Check if we hit a triangle mesh (other geometry types are not supported). TODO: add support for curve
                            if (hit.getType() == HitType::Triangle)
                            {
                                const TriangleHit triangleHit = hit.getTriangleHit();

                                uint triangleIndex = gScene.lightCollection.getTriangleIndex(triangleHit.instanceID, triangleHit.primitiveIndex);
                                if (triangleIndex != LightCollection::kInvalidIndex) // && any(gScene.lightCollection.getAverageRadiance(triangleIndex) > 0.f))
                                {
                                    // Create a light sample from the emissive triangle hit.
                                    lightSample = LightSample::createEmissive(triangleIndex, triangleHit.barycentrics);
                                    // If MIS cutoff is used, we need to evaluate the sample and make sure it actually could have been
                                    // generated by the area sampling technique. This is due to numerical precision.
                                    if (brdfCutoff > 0.f)
                                    {
                                        const EvaluatedLightSample evaluatedLightSample = lights.evalLightSample(lightSample, evalContext.pos, false);
                                        float brdfPdf = evalContext.evalBsdfPDF(evaluatedLightSample.dir, false);
                                        float maxDistance = maxDistanceFromPdf(brdfPdf);
                                        if (evaluatedLightSample.distance > maxDistance) lightSample = LightSample::createInvalid();
                                    }
                                }
                            }
                        }
                        else if (Lights::kUseEnvLight && !lightSample.isValid())
                        {
                            lightSample = LightSample::createEnv(dir);
                        }
                    }

                    if (!lightSample.isValid())
                    {
                        risState.M += 1.f;
                        continue;
                    }

                    // Resample BRDF light sample.
                    const EvaluatedLightSample evaluatedLightSample = lights.evalLightSample(lightSample, evalContext.pos, false);
                    const LoadedLightSample loadedLightSample = lights.loadLightSample(lightSample, false);
                    float p = evalContext.evalBsdfPDF(evaluatedLightSample.dir, false) * evaluatedLightSample.geomFactor;
                    float misWeight = evalBalanceHeuristicMIS(evalContext, lightSample, evaluatedLightSample, SamplingTechnique::BRDF);
                    float pHat = evalContext.evalTargetFunction(loadedLightSample, lights, false, false, sg, 2u);
                    streamingInitialResampleMIS(risState, lightSample, pHat, p, misWeight, sg);
                }
            }
        }

        // Resample path sample
        if (resampleEmissionMode == uint(ResampleEmissionMode::OneMergedReservoir))
        {
            float pHat = evalContext.evalTargetFunction({}, lights, false, false, sg, 1);
            streamingInitialResampleMIS(risState, 1u, pHat, 1.0f, 1.0f, sg);
        }

        // Final process for each reservoir
        uint pixelIdxInReservoirBuffer = getPixelIndex(frameDim, pixel, restirPassIdx);

        // Check selected sample visibility.
        if (kCheckVisibility && risState.lightSample.isValid() && risState.pathSample == 2)
        {
            const EvaluatedLightSample evaluatedLightSample = lights.evalLightSample(risState.lightSample, evalContext.pos, false);

            // Even if the light sample is occluded, we still need to keep the subpixel sample
            if (evalContext.evalVisibility(evaluatedLightSample, false) == 0.f)
                risState = RisState::createEmpty(risState.pixelSampleUV, risState.lensSampleUV, 2u);
        }

        // Instead of storing the weight sum and weight individually, we precompute the term that would be multiplied with the final sample evaluation.
        //  -> this is the reservoir weight
        risState.weight = risState.targetPdf > 0.f ? risState.weightSum / risState.targetPdf : 0.f;
        risState.M = 1.f;

        // compute filter weight
        if (filterAlpha > 0.f)
            risState.weight *= gScene.camera.getFilterWeight(subPixelUV, filterRadius, filterAlpha, filterNorm) * 4 * filterRadius * filterRadius;

        // Save reservoir.
        reservoirs[pixelIdxInReservoirBuffer] = risState.toReservoir();

        // Save the shading context (same for all RGB)
        resEvalContext[pixelIdxInReservoirBuffer] = evalContext;

        // Create center evaluation context
        Ray centerRay = gScene.camera.computeRayPinholeSubPixelRandom(pixel, frameDim, float2(0.5f));
        SurfaceData centerSd = centerSurfaceData[pixelIndex];
        pixelCenterEvalContext[pixelIndex] = EvalContext::create(pixel, frameDim, centerSd, -centerRay.dir);
    }
};

cbuffer CB
{
    uint resampleResourceType;
    InitialResampling gInitialResampling;
}

[numthreads(16, 16, 1)]
void main(uint3 dispatchThreadId : SV_DispatchThreadID)
{
    gInitialResampling.process(dispatchThreadId.xy);
}
