import Scene.Scene;
import Scene.RaytracingInline;
__exported import Scene.Camera.CameraData;
import Utils.Color.ColorHelpers;
import Utils.Debug.PixelDebug;
import Utils.Math.MathHelpers;
import EvalContext;
import LoadShadingData;
import Params;
import Reservoir;

// Some utility funtions shared between shaders
uint getPixelIndex(uint2 frameDim, uint2 pixel, uint passIdx)
{
    uint PixelIdxOnScreen = pixel.y * frameDim.x + pixel.x;
    uint totalScreenPixels = frameDim.x * frameDim.y;

    return totalScreenPixels * passIdx +      // which pass?
           PixelIdxOnScreen;                  // which pixel on screen?
}

// Utilities for Area ReSTIR resampling
struct PixelAreaSampleData
{
    bool reuseLensSample;
    bool reuseSubpixelSample;

    Texture2D<float2> subPixelUV;
    Texture2D<float2> prevSubPixelUV;
    Texture2D<float2> lensUV;
    Texture2D<float2> prevLensUV;
}

struct JacobianEvalData
{
    float lensToPixelDist;
    float pixelToRcHitDist;
    float3 rayDir;
    float2 lensUV;

    void printData()
    {
        print("lensToPixelDist = ", lensToPixelDist);
        print("pixelToRcHitDist = ", pixelToRcHitDist);
        print("rayDir = ", rayDir);
        print("lensUV = ", lensUV);
    }
}

Ray computePrevRayThinlensSubPixelRandom(float2 pixel, uint2 frameDim, float2 u0, float2 u1, CameraData prevData)
{
    Ray ray;

    // Compute ray direction with sub-pixel offset
    float2 p = float2(pixel + u0) / frameDim;
    float2 ndc = float2(2, -2) * p + float2(-1, 1);
    ray.dir = ndc.x * prevData.cameraU + ndc.y * prevData.cameraV + prevData.cameraW;

    ray.origin = prevData.posW;
    float2 apertureSample = sample_disk(u1); // Sample lies in the unit disk [-1,1]^2
    float3 rayTarget = ray.origin + ray.dir;
    ray.origin += prevData.apertureRadius * (apertureSample.x * normalize(prevData.cameraU) + apertureSample.y * normalize(prevData.cameraV));
    ray.dir = normalize(rayTarget - ray.origin);

    float invCos = 1.f / dot(normalize(prevData.cameraW), ray.dir);
    ray.tMin = prevData.nearZ * invCos;
    ray.tMax = prevData.farZ * invCos;

    return ray;
}

// Retrace the primary ray for reused sample at different pixel (domain)
EvalContext retracePrimaryRay( float2 pixel, uint2 frameDim, float2 subPixelSampleUV, float2 lensSampleUV, const bool isPrevFrame, const CameraData prevData = {})
{
    EvalContext evalContext = {};

    // Note: it seems there will be a strange precison error when using previous camera data to compute ray direction
    bool cameraChanged = any(prevData.posW != gScene.camera.data.posW) || any(prevData.cameraU != gScene.camera.data.cameraU) ||
        any(prevData.cameraV != gScene.camera.data.cameraV) || any(prevData.cameraW != gScene.camera.data.cameraW);

    // If we are retracing in previous frame, we first need to generate the ray using previous camera data
    Ray ray;
    if (isPrevFrame && cameraChanged)
    {
        ray = computePrevRayThinlensSubPixelRandom(pixel, frameDim, subPixelSampleUV, lensSampleUV, prevData);
    }
    else
    {
        ray = gScene.camera.computeRayThinlensSubPixelRandom(pixel, frameDim, subPixelSampleUV, lensSampleUV);
    }

    SceneRayQuery<EvalContext::kUseAlphaTest> sceneRayQuery;
    HitInfo hit;
    float hitT;
    if (sceneRayQuery.traceRay(ray, hit, hitT, RAY_FLAG_NONE, 0xff, isPrevFrame))
    {
        evalContext = EvalContext::createFromHit(hit, ray.origin, ray.dir, isPrevFrame);
    }
    else
    {
        evalContext.emission = gScene.envMap.eval(ray.dir); // background hit need to evaluate environment map
        evalContext.depth = -1.0f;
        evalContext.surfaceValid = false;
        evalContext.V = -ray.dir;
    }

    return evalContext;
}

