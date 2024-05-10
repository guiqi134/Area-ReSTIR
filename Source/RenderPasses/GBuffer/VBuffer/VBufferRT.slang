/***************************************************************************
 # Copyright (c) 2015-23, NVIDIA CORPORATION. All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  * Neither the name of NVIDIA CORPORATION nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/
__exported import Scene.Shading;
__exported import Utils.Timing.GpuTimer;
__exported import Utils.Math.Ray;
import Utils.Math.MathHelpers;
import Utils.Sampling.SampleGenerator;
import Utils.Debug.PixelDebug;

RWTexture2D<PackedHitInfo> gVBuffer;
RWTexture2D<PackedHitInfo> gVBufferCenter;
RWTexture2D<float>  gDepth;
RWTexture2D<float4>  gLinearDepth;
RWTexture2D<float2> gMotionVector;
RWTexture2D<float4> gViewW;
RWTexture2D<uint> gTime;
RWTexture2D<float> gMask;
RWTexture2D<float2> gSubPixelUV;
RWTexture2D<float2> gLensUV;
RWTexture2D<float2> gDebugMotion;

#define is_valid(name) (is_valid_##name != 0)

#if !defined(COMPUTE_DEPTH_OF_FIELD) || !defined(USE_ALPHA_TEST) || !defined(RAY_FLAGS)
#error "Not all defines are set!"
#endif

void print4x4(float4x4 mat, String msg)
{
    for (uint i = 0; i < 4; i++)
    {
        print(msg, mat[i]);
    }
}

float persProjToView(float depthNDC, float near, float far)
{
    return (near * far) / (depthNDC * (far - near) - far);
}

struct VBufferRT
{
    static const bool kComputeDepthOfField = COMPUTE_DEPTH_OF_FIELD;
    static const bool kUseAlphaTest = USE_ALPHA_TEST;
    static const uint kRayFlags = RAY_FLAGS;

    uint2 frameDim;
    uint frameCount;
    uint subPixelRandom;
    float areaScaler;
    bool isSceneAnimated; // TODO: need to find a way to check if the global timer is paused
    bool clampMotionVec;
    float mvecClampThreshold;
    bool sameSubpixelRandomForAllPixels;

    float2 computeCenterMotionVector(const HitInfo centerHit, const Ray centerRay)
    {
        VertexData prevVertexData = loadVertexData(centerHit, centerRay.origin, centerRay.dir, true);
        VertexData currVertexData = loadVertexData(centerHit, centerRay.origin, centerRay.dir, false);
        float4 prevPosH = mul(gScene.camera.data.prevViewProjMatNoJitter, float4(prevVertexData.posW, 1.f));
        float4 currPosH = mul(gScene.camera.data.viewProjMatNoJitter, float4(currVertexData.posW, 1.0f));
        float4 currPosNdc = currPosH / currPosH.w;
        float2 currPosScreen = currPosNdc.xy * float2(0.5f, -0.5f) + 0.5f;
        float4 prevPosNdc = prevPosH / prevPosH.w;
        float2 prevPosScreen = prevPosNdc.xy * float2(0.5f, -0.5f) + 0.5f;
        float2 newMotionVector = prevPosScreen - currPosScreen;

        print("prevPosScreen = ", prevPosScreen);
        print("currPosScreen = ", currPosScreen);

        return newMotionVector;
    }

    Ray generateRay(uint2 pixel)
    {
        float2 jitterOnScreen = float2(-gScene.camera.data.jitterX, gScene.camera.data.jitterY) * frameDim; // [-0.5, 0.5]

        // No anti-aliasling
        if (subPixelRandom == 0 && all(jitterOnScreen == float2(0)))
        {
            gSubPixelUV[pixel] = float2(0.5f);

            if (kComputeDepthOfField)
            {
                SampleGenerator sg = SampleGenerator(pixel, frameCount);
                float2 lensUV = sampleNext2D(sg);
                gLensUV[pixel] = lensUV;
                return gScene.camera.computeRayThinlens(pixel, frameDim, lensUV);
            }
            else
            {
                gLensUV[pixel] = float2(0);
                return gScene.camera.computeRayPinhole(pixel, frameDim);
            }
        }
        // Has anti-aliasing
        else
        {
            // Compute subpixel UV in [0, 1)
            SampleGenerator sg = SampleGenerator(pixel, frameCount);
            SampleGenerator sgFixedPixel = SampleGenerator(uint2(1), frameCount);
            SampleGenerator sgForSubpixelUV = sameSubpixelRandomForAllPixels ? sgFixedPixel : sg;
            float2 subPixelUV = subPixelRandom != 0 ? sampleNext2D(sgForSubpixelUV) : float2(0.5f);
            subPixelUV += any(jitterOnScreen != 0) ? jitterOnScreen + 0.5f : float2(0);
            subPixelUV = float2(0.5f) + areaScaler * (subPixelUV - 0.5f);
            gSubPixelUV[pixel] = subPixelUV;

            if (kComputeDepthOfField)
            {
                float2 lensUV = sampleNext2D(sg);
                gLensUV[pixel] = lensUV;
                return gScene.camera.computeRayThinlensSubPixelRandom(pixel, frameDim, subPixelUV, lensUV);
            }
            else
            {
                gLensUV[pixel] = float2(0);
                return gScene.camera.computeRayPinholeSubPixelRandom(pixel, frameDim, subPixelUV);
            }
        }
    }

    VertexData loadVertexData(const HitInfo hit, float3 rayOrigin, float3 rayDir, bool isPrevFrame)
    {
        VertexData v = {};
        // Get previous world position for current hit for computing motion vector
        if (hit.getType() == HitType::Triangle)
        {
            const TriangleHit triangleHit = hit.getTriangleHit();
            v = gScene.getVertexData(triangleHit, isPrevFrame);
        }
        else if (hit.getType() == HitType::DisplacedTriangle)
        {
            const DisplacedTriangleHit displacedTriangleHit = hit.getDisplacedTriangleHit();
            v = gScene.getVertexData(displacedTriangleHit, -rayDir, isPrevFrame);
        }
        else if (hit.getType() == HitType::Curve)
        {
            const CurveHit curveHit = hit.getCurveHit();
            v = gScene.getVertexDataFromCurve(curveHit, isPrevFrame);
        }
        else if (hit.getType() == HitType::SDFGrid)
        {
            const SDFGridHit sdfGridHit = hit.getSDFGridHit();
            v = gScene.getVertexDataFromSDFGrid(sdfGridHit, rayOrigin, rayDir);
        }

        return v;
    }

    void writeHit(uint2 pixel, float3 rayOrigin, float3 rayDir, const HitInfo hit)
    {
        gVBuffer[pixel] = hit.getData();

        VertexData v = loadVertexData(hit, rayOrigin, rayDir, false);
        float depth = 1.f;

        // Compte depth and motion vector
        if (hit.getType() == HitType::Triangle || hit.getType() == HitType::DisplacedTriangle || hit.getType() == HitType::Curve || hit.getType() == HitType::SDFGrid)
        {
            // Compute depth similar to raster (NDC).
            float4 curPosH = mul(gScene.camera.data.viewProjMatNoJitter, float4(v.posW, 1.f));
            depth = curPosH.z / curPosH.w;
        }

        if (is_valid(gDepth)) gDepth[pixel] = depth;
        if (is_valid(gLinearDepth)) gLinearDepth[pixel] = float4(float3(abs(persProjToView(depth, gScene.camera.data.nearZ, gScene.camera.data.farZ)) / 30), 1.0f);
        if (is_valid(gMask)) gMask[pixel] = 1.0f;
    }

    void writeMiss(uint2 pixel, float3 rayOrigin, float3 rayDir)
    {
        gVBuffer[pixel] = {};

        if (is_valid(gDepth)) gDepth[pixel] = 1.f;
        if (is_valid(gDepth)) gLinearDepth[pixel] = float4(1, 1, 1, 0);
        if (is_valid(gMask)) gMask[pixel] = 0.0f;
    }

    void writeAux(uint2 pixel, const Ray ray)
    {
        // Write view direction.
        if (is_valid(gViewW)) gViewW[pixel] = float4(-ray.dir, 0.f);
    }

    void beginTime(inout GpuTimer timer)
    {
        if (is_valid(gTime)) timer.start();
    }

    void endTime(uint2 pixel, inout GpuTimer timer)
    {
        if (is_valid(gTime)) gTime[pixel] = timer.getElapsed();
    }
};
