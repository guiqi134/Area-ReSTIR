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
import Utils.Debug.PixelDebug;
import Utils.Color.ColorHelpers;
import SurfaceData;
import FinalSample;

#ifndef USE_PREV_FRAME_SCENE_DATA
#define USE_PREV_FRAME_SCENE_DATA 0
#endif

/** Public interface to the ReSTIR sampler.
*/
struct AreaReSTIR
{
    // Common resources.
    RWStructuredBuffer<SurfaceData> surfaceData;
    RWStructuredBuffer<SurfaceData> centerSurfaceData;
    RWTexture2D<uint> normalDepth;

    // ReSTIR DI resources.
    StructuredBuffer<FinalSample> finalSamplesForDI;
    StructuredBuffer<PackedHitInfo> finalPrimaryHitsForDI;
    StructuredBuffer<FinalSample> finalSamplesForEmission;
    StructuredBuffer<PackedHitInfo> finalPrimaryHitsForEmission;

    // Common parameters.
    uint2 frameDim;
    bool reuseLensSample;
    bool reuseSubPixelSample;
    uint resampleEmissionMode;
    uint reservoirsPerPixel;
    uint numRestirPasses;

    /** Get linear index for one pixel.
        \param[in] pixel Pixel coordinate.
        \return Returns linear index.
    */
    uint getPixelIndex(uint2 pixel) { return pixel.y * frameDim.x + pixel.x; }

    uint getPixelIndex(uint2 frameDim, uint2 pixel, uint passIdx)
    {
        uint PixelIdxOnScreen = pixel.y * frameDim.x + pixel.x;
        uint totalScreenPixels = frameDim.x * frameDim.y;

        return totalScreenPixels * passIdx +    // which pass?
               PixelIdxOnScreen;                // which pixel on screen?
    }

    /** Set the per-pixel surface data required for screen-space ReSTIR.
        \param[in] pixel Pixel coordinate.
        \param[in] pos Position in world-space.
        \param[in] depth Depth from camera.
        \param[in] normal Shading normal in world-space.
        \param[in] faceNormal Face normal in world-space.
        \param[in] diffuseWeight Diffuse lobe weight.
        \param[in] specularWeight Specular lobe weight.
        \param[in] specularRoughness Specular roughness (linear).
    */
    void setSurfaceData(uint2 pixel, HitInfo hit, float3 pos, float3 normal, float3 faceNormal, float3 diffuseAlbedo, float3 specularAlbedo, float specularRoughness,
        float3 emission, MaterialType type, uint lobeTypes = 0, bool isCenter = false)
    {
        // Store pixel data.
        SurfaceData sd;
        sd.pos = pos;
        sd.depth = distance(gScene.camera.getPosition(), pos);
        sd.normal = normal;
        sd.faceNormal = faceNormal;
        sd.diffuseWeight = luminance(diffuseAlbedo);
        sd.specularWeight = luminance(specularAlbedo);
        sd.diffuseAlbedo = diffuseAlbedo;
        sd.specularAlbedo = specularAlbedo;
        sd.specularRoughness = specularRoughness;
        sd.emission = emission;
        sd.materialType = type;
        sd.lobeTypes = lobeTypes;
        sd.hit = hit;

        if (isCenter)
        {
            centerSurfaceData[getPixelIndex(pixel)] = sd;
        }
        else
        {
            surfaceData[getPixelIndex(pixel)] = sd;

            // Store compact normal depth information for spatial reuse.
            normalDepth[pixel] = packNormalDepth(float4(normal, sd.depth));
        }
    }

    void setSurfaceDataForEnvMap(uint2 pixel, float3 color, bool isCenter)
    {
        SurfaceData sd = SurfaceData::createInvalid();
        sd.emission = color;

        if (isCenter)
            centerSurfaceData[getPixelIndex(pixel)] = sd;
        else
            surfaceData[getPixelIndex(pixel)] = sd;
    }

    /** Mark pixel as containing no valid shading data.
        \param[in] pixel Pixel coordinate.
    */
    void setInvalidSurfaceData(uint2 pixel)
    {
        surfaceData[getPixelIndex(pixel)] = SurfaceData::createInvalid();
    }

    void setInvalidCenterSurfaceData(uint2 pixel)
    {
        centerSurfaceData[getPixelIndex(pixel)] = SurfaceData::createInvalid();
    }

    /** Get the final sample for the given pixel computed using ReSTIR DI.
        \param[in] pixel Pixel coordinate.
        \param[out] dir Direction to light in world-space.
        \param[out] distance Distance to light.
        \param[out] Li Weighted incident radiance.
        \return Returns true if a valid sample was returned.
    */
    bool getFinalLightSample(uint2 pixel, uint passIdx, out float3 dir, out float distance, out float3 Li)
    {
        // uint pixelIndexInFinalSample = frameDim.x * frameDim.y * reservoirIdx + getPixelIndex(pixel);
        uint pixelIndexInFinalSample = getPixelIndex(frameDim, pixel, passIdx);
        FinalSample finalSample = finalSamplesForDI[pixelIndexInFinalSample];
        dir = finalSample.dir;
        distance = finalSample.distance;
        Li = finalSample.Li;
        return distance > 0.f;
    }

    float2 getFinalSubPixelSample(uint2 pixel, uint passIdx)
    {
        uint pixelIndexInFinalSample = getPixelIndex(frameDim, pixel, passIdx);
        FinalSample finalSample = finalSamplesForDI[pixelIndexInFinalSample];
        return finalSample.pixelSampleUV;
    }

    float2 getFinalLensSample(uint2 pixel, uint passIdx)
    {
        uint pixelIndexInFinalSample = getPixelIndex(frameDim, pixel, passIdx);
        FinalSample finalSample = finalSamplesForDI[pixelIndexInFinalSample];
        return finalSample.lensSampelUV;
    }

    uint getFinalPathSample(uint2 pixel, uint passIdx)
    {
        uint pixelIndexInFinalSample = getPixelIndex(frameDim, pixel, passIdx);
        FinalSample finalSample = finalSamplesForDI[pixelIndexInFinalSample];
        return finalSample.pathSample;
    }

    float3 getFinalSampleRadiance(uint2 pixel, uint passIdx)
    {
        float3 radiance = float3(0);
        uint pixelIndexInFinalSample = getPixelIndex(frameDim, pixel, passIdx);
        FinalSample finalSample = finalSamplesForDI[pixelIndexInFinalSample];
        return finalSample.Li;
    }

    float3 getFinalSampleMisWeight(uint2 pixel, uint passIdx)
    {
        uint pixelIndexInFinalSample = getPixelIndex(frameDim, pixel, passIdx);
        FinalSample finalSample = finalSamplesForDI[pixelIndexInFinalSample];
        return finalSample.misWeight;
    }
};
