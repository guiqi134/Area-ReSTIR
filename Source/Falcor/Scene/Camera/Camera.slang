/***************************************************************************
 # Copyright (c) 2015-22, NVIDIA CORPORATION. All rights reserved.
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
import Scene.Camera.CameraData;
__exported import Utils.Math.Ray;
import Utils.Math.MathHelpers;

struct Camera
{
    CameraData data;

    float3 getPosition() { return data.posW; }
    float4x4 getViewProj() { return data.viewProjMat; }
    float4x4 getViewMat() { return data.viewMat; }
    float4x4 getProjMat() { return data.projMat; }

    /** Computes a camera ray for a given pixel assuming a pinhole camera model.
        The camera jitter is taken into account to compute the sample position on the image plane.
        \param[in] pixel Pixel coordinates with origin in top-left.
        \param[in] frameDim Image plane dimensions in pixels.
        \param[in] applyJitter true if jitter should be applied else false.
        \return Returns the camera ray.
    */
    Ray computeRayPinhole(uint2 pixel, uint2 frameDim, bool applyJitter = true)
    {
        Ray ray;

        // Compute the normalized ray direction assuming a pinhole camera.
        ray.origin = data.posW;
        ray.dir = normalize(computeNonNormalizedRayDirPinhole(pixel, frameDim, applyJitter));

        float invCos = 1.f / dot(normalize(data.cameraW), ray.dir);
        ray.tMin = data.nearZ * invCos;
        ray.tMax = data.farZ * invCos;

        return ray;
    }

    /** Computes the primary ray's direction, non-normalized assuming pinhole camera model.
        The camera jitter is taken into account to compute the sample position on the image plane.
        \param[in] pixel Pixel coordinates with origin in top-left.
        \param[in] frameDim Image plane dimensions in pixels.
        \param[in] applyJitter True if jitter should be applied else false.
        \return Returns the non-normalized ray direction
    */
    float3 computeNonNormalizedRayDirPinhole(uint2 pixel, uint2 frameDim, bool applyJitter = true)
    {
        // Compute sample position in screen space in [0,1] with origin at the top-left corner.
        // The camera jitter offsets the sample by +-0.5 pixels from the pixel center.
        float2 p = (pixel + float2(0.5f, 0.5f)) / frameDim;
        if (applyJitter) p += float2(-data.jitterX, data.jitterY);
        float2 ndc = float2(2, -2) * p + float2(-1, 1);

        // Compute the non-normalized ray direction assuming a pinhole camera.
        return ndc.x * data.cameraU + ndc.y * data.cameraV + data.cameraW;
    }

    float Gaussian(float d, float radius = 2.f, float alpha = 2.f) {
        return max((float)0, float(exp(-alpha * d * d) - exp(-alpha * radius * radius)));
    }
    float getFilterWeight(float2 u, float radius, float alpha, float normfactor)
    {
        // https:www.wolframalpha.com/input?i=integrate+%28exp%28-a*x*x%29-+exp%28-a*r*r%29%29*%28exp%28-a*y*y%29+-+exp%28-a*r*r%29%29%2C+x+from+-r+to+r%2C+y+from+-r+to+r
        // return 1.f; // box
        // PBRT4 gaussian filter
        float h = Gaussian(u.x - 0.5, radius, alpha) * Gaussian(u.y - 0.5, radius, alpha);
        return h / normfactor; // 1.4295; // https://www.wolframalpha.com/input?i=integrate+%28exp%28-2*x*x%29-+exp%28-2*2*2%29%29*%28exp%28-2*y*y%29+-+exp%28-2*2*2%29%29%2C+x+from+-1+to+1%2C+y+from+-1+to+1
        // https://www.wolframalpha.com/input?i=%28-4+%2B+e%5E8+sqrt%282+%CF%80%29+erf%28sqrt%282%29%29%29%5E2%2F%284+e%5E16%29&assumption=%22ClashPrefs%22+-%3E+%7B%22Math%22%7D
    }

    // Sub-pixel offset
    Ray computeRayPinholeSubPixelRandom(uint2 pixel, uint2 frameDim, float2 u)
    {
        Ray ray;

        // Compute the normalized ray direction assuming a pinhole camera.
        ray.origin = data.posW;

        // Compute ray direction with sub-pixel offset
        float2 p = float2(pixel + u) / frameDim;
        float2 ndc = float2(2, -2) * p + float2(-1, 1);
        ray.dir = normalize(ndc.x * data.cameraU + ndc.y * data.cameraV + data.cameraW);

        float invCos = 1.f / dot(normalize(data.cameraW), ray.dir);
        ray.tMin = data.nearZ * invCos;
        ray.tMax = data.farZ * invCos;

        return ray;
    }

    /** Computes a camera ray for a given pixel assuming a thin-lens camera model.
        The camera jitter is taken into account to compute the sample position on the image plane.
        \param[in] pixel Pixel coordinates with origin in top-left.
        \param[in] frameDim Image plane dimensions in pixels.
        \param[in] u Uniform 2D sample.
        \return Returns the camera ray.
    */
    Ray computeRayThinlens(uint2 pixel, uint2 frameDim, float2 u)
    {
        Ray ray;

        // Sample position in screen space in [0,1] with origin at the top-left corner.
        // The camera jitter offsets the sample by +-0.5 pixels from the pixel center.
        float2 p = (pixel + float2(0.5f, 0.5f)) / frameDim + float2(-data.jitterX, data.jitterY);
        float2 ndc = float2(2, -2) * p + float2(-1, 1);

        // Compute the normalized ray direction assuming a thin-lens camera.
        ray.origin = data.posW;
        ray.dir = ndc.x * data.cameraU + ndc.y * data.cameraV + data.cameraW;
        float2 apertureSample = sample_disk(u); // Sample lies in the unit disk [-1,1]^2
        float3 rayTarget = ray.origin + ray.dir;
        ray.origin += data.apertureRadius * (apertureSample.x * normalize(data.cameraU) + apertureSample.y * normalize(data.cameraV));
        ray.dir = normalize(rayTarget - ray.origin);

        float invCos = 1.f / dot(normalize(data.cameraW), ray.dir);
        ray.tMin = data.nearZ * invCos;
        ray.tMax = data.farZ * invCos;

        return ray;
    }

    // u0 is subpixel uv and u1 is lens uv
    Ray computeRayThinlensSubPixelRandom(float2 pixel, uint2 frameDim, float2 u0, float2 u1)
    {
        Ray ray;

        // Sample position "p" in screen space in [0,1] with origin at the top-left corner.
        float2 p = float2(pixel + u0) / frameDim;
        float2 ndc = float2(2, -2) * p + float2(-1, 1);

        // Compute the normalized ray direction assuming a thin-lens camera.
        ray.origin = data.posW;
        ray.dir = ndc.x * data.cameraU + ndc.y * data.cameraV + data.cameraW;
        float2 apertureSample = sample_disk(u1); // Sample lies in the unit disk [-1,1]^2
        float3 rayTarget = ray.origin + ray.dir;
        ray.origin += data.apertureRadius * (apertureSample.x * normalize(data.cameraU) + apertureSample.y * normalize(data.cameraV));
        ray.dir = normalize(rayTarget - ray.origin);

        float invCos = 1.f / dot(normalize(data.cameraW), ray.dir);
        ray.tMin = data.nearZ * invCos;
        ray.tMax = data.farZ * invCos;

        return ray;
    }
};
