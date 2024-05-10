/***************************************************************************
 # Copyright (c) 2015-22, NVIDIA CORPORATION. All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/
import Utils.Math.PackedFormats;
import Scene.HitInfo;

// Use compact packing. Changing this requires restarting the renderer.
#define USE_COMPACT_FINAL_SAMPLE 0

#if USE_COMPACT_FINAL_SAMPLE
typedef uint4 PackedFinalSample;
#else
struct PackedFinalSample
{
    float3 dir;
    float distance;
    float3 Li;
    float2 pixelSampleUV;
    float2 lensSampleUV;
    uint pathSample;
    float3 misWeight;
};
#endif

/** Represents a final sample (light, subpixel uv, ...) used for shading pixels.
*/
struct FinalSample
{
    float3 dir;     ///< Direction to the sampled light in world-space.
    float distance; ///< Distance to the sampled light.
    float3 Li;      ///< Weighted incident radiance.
    float2 pixelSampleUV;
    float2 lensSampelUV;
    uint pathSample;
    float3 misWeight; // MIS weight for RGB channel

    static FinalSample createFromPacked(const PackedFinalSample packed)
    {
        FinalSample finalSample;
        finalSample.unpack(packed);
        return finalSample;
    }

    PackedFinalSample pack()
    {
        PackedFinalSample packed;
#if USE_COMPACT_FINAL_SAMPLE
        packed.x = encodeNormal2x16(dir);
        packed.y = asuint(distance);
        packed.z = encodeLogLuvHDR(Li);
        packed.w = 0;
#else
        packed.dir = dir;
        packed.distance = distance;
        packed.Li = Li;
        packed.pixelSampleUV = pixelSampleUV;
        packed.lensSampleUV = lensSampelUV;
        packed.pathSample = pathSample;
        packed.misWeight = misWeight;
#endif
        return packed;
    }

    [mutating] void unpack(const PackedFinalSample packed)
    {
#if USE_COMPACT_FINAL_SAMPLE
        dir = decodeNormal2x16(packed.x);
        distance = asfloat(packed.y);
        Li = decodeLogLuvHDR(packed.z);
#else
        dir = packed.dir;
        distance = packed.distance;
        Li = packed.Li;
        pixelSampleUV = packed.pixelSampleUV;
        lensSampelUV = packed.lensSampleUV;
        pathSample = packed.pathSample;
        misWeight = packed.misWeight;
#endif
    }
};
