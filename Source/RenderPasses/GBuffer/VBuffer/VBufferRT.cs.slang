/***************************************************************************
 # Copyright (c) 2015-21, NVIDIA CORPORATION. All rights reserved.
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
import Scene.RaytracingInline;
import VBufferRT;
import Utils.Debug.PixelDebug;
import Utils.Math.MathHelpers;

// import LoadShadingData;

ConstantBuffer<VBufferRT> gVBufferRT;

[numthreads(16, 16, 1)]
void main(uint3 dispatchThreadId : SV_DispatchThreadID)
{
    uint2 pixel = dispatchThreadId.xy;
    if (any(pixel >= gVBufferRT.frameDim)) return;

    printSetPixel(pixel);

    GpuTimer timer;
    gVBufferRT.beginTime(timer);

    // Generate the primary ray from the camera using selected method
    const Ray ray = gVBufferRT.generateRay(pixel);

    // Trace primary ray
    SceneRayQuery<VBufferRT::kUseAlphaTest> sceneRayQuery;
    HitInfo hit;
    float hitT;
    if (sceneRayQuery.traceRay(ray, hit, hitT, VBufferRT::kRayFlags, 0xff, false))
    {
        gVBufferRT.writeHit(pixel, ray.origin, ray.dir, hit);
    }
    else
    {
        gVBufferRT.writeMiss(pixel, ray.origin, ray.dir);
    }
    gVBufferRT.writeAux(pixel, ray);

    // Also compute the pixel center vbuffer (different from subpixel sample's vbuffer) + motion vector
    if (gVBufferRT.subPixelRandom != 0 || gVBufferRT.kComputeDepthOfField || gScene.camera.data.jitterX != 0.0f || gScene.camera.data.jitterY != 0.0f)
    {
        // Trace new ray at the pixel center
        Ray centerRay = gScene.camera.computeRayPinholeSubPixelRandom(pixel, gVBufferRT.frameDim, float2(0.5f));

        // Write center hit
        HitInfo centerHit;
        float centerHitT;
        if (sceneRayQuery.traceRay(centerRay, centerHit, centerHitT, VBufferRT::kRayFlags, 0xff, false))
        {
            gVBufferCenter[pixel] = centerHit.getData();
            gMotionVector[pixel] = gVBufferRT.computeCenterMotionVector(centerHit, centerRay);
        }
        else
        {
            gVBufferCenter[pixel] = {};
            gMotionVector[pixel] = {};
        }
    }
    else
    {
        // No jittering AND No subpixel random -> no need to trace new center ray
        if (hit.isValid())
        {
            gVBufferCenter[pixel] = hit.getData();
            gMotionVector[pixel] = gVBufferRT.computeCenterMotionVector(hit, ray);
        }
        else
        {
            gVBufferCenter[pixel] = {};
            gMotionVector[pixel] = {};
        }
    }

    // gDebugMotion[pixel] = gMotionVector[pixel] * gVBufferRT.frameDim * 1000;

    if (gVBufferRT.clampMotionVec)
    {
        float2 screenMotion = gMotionVector[pixel] * float2(gVBufferRT.frameDim);
        gMotionVector[pixel].x = abs(screenMotion.x) <= gVBufferRT.mvecClampThreshold ? 0.0f : gMotionVector[pixel].x;
        gMotionVector[pixel].y = abs(screenMotion.y) <= gVBufferRT.mvecClampThreshold ? 0.0f : gMotionVector[pixel].y;
    }

    gVBufferRT.endTime(pixel, timer);
}
