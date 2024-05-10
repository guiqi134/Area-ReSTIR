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

#define USE_PACKED_LOCAL_EMISSIVE_TRIANGLE 0

struct PackedLocalEmissiveTriangle
{
    uint4 posNormal;
    uint4 edgesRadiance;
};

/** Compact representation of an emissive triangle.
    This is used in place of the scene data for more efficient
    sampling and evaluation of emissive triangles.
*/
struct LocalEmissiveTriangle
{
    float3 pos;     ///< Position in world-space (i.e. p0).
    float3 normal;  ///< Normal in world-space.
    float3 edge0;   ///< First edge in world-space (i.e. p1 - p0).
    float3 edge1;   ///< Second edge in world-space (i.e. p2 - p0).
    float radiance; ///< Average radiance luminance.
    bool useLightProfile;

    static LocalEmissiveTriangle createFromPacked(const PackedLocalEmissiveTriangle packed)
    {
        LocalEmissiveTriangle tri;
        tri.unpack(packed);
        return tri;
    }

    float3 getPosition(const float2 uv)
    {
        return pos + edge0 * uv.x + edge1 * uv.y;
    }

    float getArea()
    {
        return 0.5f * length(cross(edge0, edge1));
    }

    PackedLocalEmissiveTriangle pack()
    {
        PackedLocalEmissiveTriangle packed;

        packed.posNormal.xyz = asuint(pos);
        packed.posNormal.w = encodeNormal2x8(normal);

        packed.edgesRadiance.xyz = f32tof16(edge0) | (f32tof16(edge1) << 16);
        packed.edgesRadiance.w = asuint(radiance);
        // DEMO21: Pack useLightProfile into sign-bit of radiance (which is always positive).
        if (useLightProfile) packed.edgesRadiance.w |= 0x80000000;

        return packed;
    }

    [mutating] void unpack(const PackedLocalEmissiveTriangle packed)
    {
        pos = asfloat(packed.posNormal.xyz);
        normal = decodeNormal2x8(packed.posNormal.w);

        edge0 = f16tof32(packed.edgesRadiance.xyz);
        edge1 = f16tof32(packed.edgesRadiance.xyz >> 16);
        radiance = asfloat(packed.edgesRadiance.w & 0x7fffffff);
        useLightProfile = (packed.edgesRadiance.w & 0x80000000) != 0;
    }
};
