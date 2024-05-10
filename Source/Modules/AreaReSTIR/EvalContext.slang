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
import Scene.RaytracingInline;
__exported import Scene.HitInfo;
import Utils.Math.MathHelpers;
import Utils.Color.ColorHelpers;
import Utils.Debug.PixelDebug;
import Rendering.Materials.IsotropicGGX;
import Rendering.Materials.Fresnel;
import Lights;
__exported import SurfaceData;
import LoadShadingData;
import Params;
import Reservoir;

#define EnableDeltaBSDF 0

/** Context for evaluating ReSTIR samples.
*/
struct EvalContext
{
    // Static configuration.
    static const bool kUseAlphaTest = USE_ALPHA_TEST;
    static const float kMinCosTheta = 1e-6f;

    float3 pos;
    float depth;
    float3 N;
    float3 V;
    float NdotV;
    float diffuseWeight;
    float specularWeight;
    float diffuseSpecularMix;
    float3 diffuseAlbedo;
    float3 specularAlbedo;
    float ggxAlpha;
    float3 emission;
    MaterialType materialType;
    uint lobeTypes;
    HitInfo hit;

    bool surfaceValid;

    bool hasLobe(LobeType lobeType) { return (lobeTypes & (uint)lobeType) != 0; }

    static EvalContext create(uint2 pixel, uint2 frameDim, SurfaceData surfaceData, float3 viewDir)
    {
        EvalContext evalContext = {};
        evalContext.surfaceValid = surfaceData.isSurfaceValid();
        evalContext.V = viewDir;
        evalContext.depth = surfaceData.depth;
        evalContext.emission = surfaceData.emission;
        evalContext.materialType = surfaceData.materialType;

        if (evalContext.surfaceValid)
        {
            evalContext.pos = surfaceData.pos;
            evalContext.N = surfaceData.normal;
            evalContext.NdotV = saturate(dot(evalContext.N, evalContext.V));
            evalContext.diffuseWeight = surfaceData.diffuseWeight;
            evalContext.specularWeight = surfaceData.specularWeight; // for hair material, it's zero
            float weightSum = surfaceData.diffuseWeight + surfaceData.specularWeight;
            evalContext.diffuseSpecularMix = weightSum > 1e-7f ? (surfaceData.diffuseWeight / weightSum) : 1.f;
            evalContext.diffuseAlbedo = surfaceData.diffuseAlbedo;
            evalContext.specularAlbedo = surfaceData.specularAlbedo;
            float roughness = surfaceData.specularRoughness;
            evalContext.ggxAlpha = roughness * roughness;
#if EnableDeltaBSDF
            if (roughness < 0.08f) evalContext.ggxAlpha = 0.0f;
#else
            // Clamp alpha to avoid numerical instability when evaluating the GGX lobe. This is the same threshold as used in the legacy Falcor BRDF.
            evalContext.ggxAlpha = max(0.0064f, roughness * roughness);
#endif
            evalContext.lobeTypes = surfaceData.lobeTypes;
            evalContext.hit = surfaceData.hit;
        }

        return evalContext;
    }

    static EvalContext createFromHit(HitInfo hit, float3 rayOrigin, float3 rayDir, bool isPrevFrame)
    {
        EvalContext evalContext = {};

        let lod = ExplicitLodTextureSampler(0.f);
        ShadingData sd = loadShadingData(hit, rayOrigin, rayDir, true, lod, isPrevFrame);

        // Create BSDF instance and query its properties.
        let hints = getMaterialInstanceHints(hit, true /* primary hit */);
        let mi = gScene.materials.getMaterialInstance(sd, lod, hints);
        let bsdfProperties = mi.getProperties(sd);

        // Check for BSDF lobesTypes that ReSTIR can sample.
        uint lobesTypes = mi.getLobeTypes(sd);
        bool validSurface = (lobesTypes & (uint)LobeType::NonDeltaReflection) != 0 && hit.getType() != HitType::None;

        if (validSurface)
        {
            evalContext.pos = sd.computeRayOrigin();
            evalContext.N = bsdfProperties.guideNormal; // = shadingFrame.N = sd.N
            evalContext.V = -rayDir;
            evalContext.NdotV = saturate(dot(evalContext.N, evalContext.V));
            evalContext.diffuseWeight = luminance(bsdfProperties.diffuseReflectionAlbedo);
            evalContext.specularWeight = luminance(bsdfProperties.specularReflectance);
            float weightSum = evalContext.diffuseWeight + evalContext.specularWeight;
            evalContext.diffuseSpecularMix = weightSum > 1e-7f ? (evalContext.diffuseWeight / weightSum) : 1.f;
            evalContext.diffuseAlbedo = bsdfProperties.diffuseReflectionAlbedo;
            evalContext.specularAlbedo = bsdfProperties.specularReflectionAlbedo;
            // Clamp roughness to avoid numerical instability when evaluating the GGX lobe.
            // This is the same threshold as used in the legacy Falcor BRDF.
            float roughness = bsdfProperties.roughness;
            evalContext.ggxAlpha = roughness * roughness;
#if EnableDeltaBSDF
            if (roughness < 0.08) evalContext.ggxAlpha = 0.0f;
#else
            evalContext.ggxAlpha = max(0.0064f, roughness * roughness);
#endif
            evalContext.depth = distance(rayOrigin, sd.posW);
            evalContext.emission = bsdfProperties.emission;
            evalContext.materialType = sd.mtl.getMaterialType();
            evalContext.lobeTypes = lobesTypes;
            evalContext.hit = hit;
            evalContext.surfaceValid = validSurface;
        }
        else
        {
            evalContext.emission = gScene.envMap.eval(rayDir); // background hit need to evaluate environment map
            evalContext.depth = -1.0f;
            evalContext.surfaceValid = false;
            evalContext.V = -rayDir;
        }

        return evalContext;
    }

    void printContext()
    {
        print("Context.isSurfaceValid = ", surfaceValid);
        print("Context.emission = ", emission);
        print("Context.depth = ", depth);
        print("Context.V = ", V);
        print("Context.pos = ", pos);
        print("Context.N = ", N);
        print("Context.diffuseWeight = ", diffuseWeight);
        print("Context.specularWeight = ", specularWeight);
        print("Context.diffuseSpecularMix = ", diffuseSpecularMix);
        print("Context.ggxAlpha = ", ggxAlpha);

        TriangleHit triHit = hit.getTriangleHit();
        uint emissiveTriIndex = gScene.lightCollection.getTriangleIndex(triHit.instanceID, triHit.primitiveIndex);
        print("emissiveTriIndex = ", emissiveTriIndex);
    }

    void printContextAsUint()
    {
        print("Context.V = ", asuint(V));
        print("Context.pos = ", asuint(pos));
        print("Context.N = ", asuint(N));
        print("Context.diffuseWeight = ", asuint(diffuseWeight));
        print("Context.specularWeight = ", asuint(specularWeight));
        print("Context.diffuseSpecularMix = ", asuint(diffuseSpecularMix));
        print("Context.ggxAlpha = ", asuint(ggxAlpha));
    }


    bool isSurfaceValid()
    {
        return surfaceValid;
    }

    /** Compare depth and normal between this and another pixel and determine if other sample is valid for reuse.
        \param[in] otherNormalDepth Normal and depth of other pixel.
        \param[in] normalThreshold Normal cosine threshold.
        \param[in] depthThreshold Relative depth threshold.
        \return Returns true if other sample should be reused.
    */
    bool isValidNeighbor(float4 otherNormalDepth, float normalThreshold, float depthThreshold)
    {
        // print("dot(N, otherNormalDepth.xyz) = ", dot(N, otherNormalDepth.xyz));
        // return
        //     dot(N, otherNormalDepth.xyz) >= normalThreshold &&
        //     abs((depth - otherNormalDepth.w)) <= depthThreshold * depth;
        return dot(N, otherNormalDepth.xyz) >= normalThreshold;
    }

    /** Compare depth and normal between this and another pixel and determine if other sample is valid for reuse.
        \param[in] otherContext Other evaluation context.
        \param[in] normalThreshold Normal cosine threshold.
        \param[in] depthThreshold Relative depth threshold.
        \return Returns true if other sample should be reused.
    */
    bool isValidNeighbor(const EvalContext otherContext, float normalThreshold, float depthThreshold)
    {
        // return true || isValidNeighbor(float4(otherContext.N, otherContext.depth), normalThreshold, depthThreshold);
        return isValidNeighbor(float4(otherContext.N, otherContext.depth), normalThreshold, depthThreshold);
    }

    /** Transform direction from world- to local-space.
        \param[in] w Direction in world-space.
        \return Returns direction in local-space.
    */
    float3 toLocal(float3 w)
    {
        float3 B = perp_stark(N);
        float3 T = cross(B, N);
        return float3(dot(B, w), dot(T, w), dot(N, w));
    }

    /** Transform direction from local- to world-space.
        \param[in] w Direction in local-space.
        \return Returns direction in world-space.
    */
    float3 toGlobal(float3 w)
    {
        float3 B = perp_stark(N);
        float3 T = cross(B, N);
        return B * w.x + T * w.y + N * w.z;
    }

    IMaterialInstance getBsdfEvalDataFromHit(bool isPrevFrame, out ShadingData sd)
    {
        // Create material instance
        let lod = ExplicitLodTextureSampler(0.f);
        sd = loadShadingData(hit, gScene.camera.data.posW, -V, true, lod, isPrevFrame); // rayOrigin is only used for SDF hit
        let hints = getMaterialInstanceHints(hit, true /* primary hit */);
        return gScene.materials.getMaterialInstance(sd, lod, hints);
    }

    /** Sample the standard BSDF (Lambert diffuse + Mircofacet specular).
        \param[in] specularOnly Use specular lobe only.
        \param[in] xi Uniform random numbers.
        \param[out] dir Sampled direction in world-space.
        \return Returns true if valid sample was generated.
    */
    bool sampleStandardBSDF(float3 xi, out float3 dir, out float pdf)
    {
        dir = {};
        pdf = 0;

        if (xi.x < diffuseSpecularMix) // diffuse
        {
            dir = toGlobal(sample_cosine_hemisphere_concentric(xi.yz, pdf));
        }
        else // specular
        {
            // float3 h = sampleGGX_VNDF(ggxAlpha, toLocal(V), xi.yz, pdf); // use this to match with Falcor side
            float3 h = sampleGGX_NDF(ggxAlpha, xi.yz, pdf);
            dir = reflect(-V, toGlobal(h));
        }

        return dot(N, dir) >= kMinCosTheta;
    }

    bool sampleBSDF<S : ISampleGenerator>(bool isPrevFrame, inout S sg, out float3 dir, out float pdf)
    {
        bool validSample = false;

        if (materialType == MaterialType::Standard)
        {
            validSample = sampleStandardBSDF(sampleNext3D(sg), dir, pdf);
        }
        else if (materialType == MaterialType::Hair)
        {
            ShadingData sd = {};
            BSDFSample bsdfSample = {};
            let mi = getBsdfEvalDataFromHit(isPrevFrame, sd);
            validSample = mi.sample(sd, sg, bsdfSample, true);
            dir = bsdfSample.wo;
            pdf = bsdfSample.pdf;
        }

        return validSample;
    }

    float evalStandardBsdfPDF(float3 L)
    {
        float NdotL = saturate(dot(N, L));
        if (min(NdotV, NdotL) < kMinCosTheta) return float(0.f);
        float diffusePdf = NdotL * M_1_PI;

        float specularPdf = 0.0f;
        if (hasLobe(LobeType::SpecularReflection))
        {
            float3 H = normalize(toLocal(V + L));
            specularPdf = evalPdfGGX_NDF(ggxAlpha, H.z) / (4.0f * saturate(dot(H, toLocal(V))));
        }

        return lerp(specularPdf, diffusePdf, diffuseSpecularMix);
    }

    // wo is the scattering direction. TODO: also maybe only account for reflection events.
    float evalBsdfPDF(float3 wo, bool isPrevFrame)
    {
        float pdf = 0.0f;

        if (materialType == MaterialType::Standard)
        {
            pdf = evalStandardBsdfPDF(wo);
        }
        else if (materialType == MaterialType::Hair)
        {
            ShadingData sd = {};
            let mi = getBsdfEvalDataFromHit(isPrevFrame, sd);
            pdf = mi.evalPdf(sd, wo, true);
        }

        return pdf;
    }

    // TODO: use Falcor's built-in eval() function
    // This is a copy of StandardBSDF eval() function
    float3 evalStandardBSDF(float3 L)
    {
        // When diffuse and specular weight is all 0.0 (surface is dark), it will use diffuse lobe as pdf
        float NdotL = saturate(dot(N, L));
        float3 H = normalize(V + L);
        float NdotH = saturate(dot(N, H));
        float LdotH = saturate(dot(L, H));

        if (min(NdotV, NdotL) < kMinCosTheta) return float(0.f);
        float3 diffuse = diffuseAlbedo * NdotL * M_1_PI;

        float3 specular = 0.0f;
        if (hasLobe(LobeType::SpecularReflection))
        {
            float D = evalNdfGGX(ggxAlpha, NdotH);
            float G = evalMaskingSmithGGXSeparable(ggxAlpha, NdotV, NdotL);
            float3 F = evalFresnelSchlick(specularAlbedo, 1.0f, LdotH);
            specular = max(0.f, D * G * F / (4.f * NdotV));
        }

        return diffuse + specular;
    }

    // TODO: Built-in eval() function will consider both reflection and transmission. But ReSTIR DI only handles reflection, do we need to set transmission pdf to 0 when creating it.
    // wo is the direction to light
    float3 evalBSDF<S : ISampleGenerator>(float3 wo, bool isPrevFrame , inout S sg)
    {
        float3 brdf = float3(0);

        if (materialType == MaterialType::Standard)
        {
            brdf = evalStandardBSDF(wo);
        }
        else if (materialType == MaterialType::Hair)
        {
            ShadingData sd = {};
            let mi = getBsdfEvalDataFromHit(isPrevFrame, sd);
            brdf = mi.eval(sd, wo, sg);
        }

        return brdf;
    }

    /** Evaluate the target function (target pdf) of a reservoir sample.
        \param[in] loadedLightSample Loaded light sample.
        \param[in] withVisibility Evaluate visibility term.
        \return Returns the target function value.
    */
    float evalTargetFunction<S : ISampleGenerator>(const LoadedLightSample loadedLightSample, const Lights lights, bool isPrevFrame, const bool withVisibility, inout S sg,
        uint pathSample = 2)
    {
        if (!isSurfaceValid() && pathSample == 2) return 0.f;

        float pHat = 0.0f;

        if (pathSample == 1)
        {
            // pHat for length-1 path
            pHat = luminance(emission);
        }
        else if (pathSample == 2 && isSurfaceValid())
        {
            // pHat for length-2 path
            const EvaluatedLightSample evaluatedLightSample = loadedLightSample.eval(pos);
            float3 lightRadiance = lights.evalEmission(loadedLightSample.lightSample, loadedLightSample.posOrDir);
            if (any(evaluatedLightSample.dir != 0.0f))
            {
                float3 brdf = evalBSDF(evaluatedLightSample.dir, isPrevFrame, sg);
                float brdfScalerWeight = luminance(brdf);
                float3 weight = max(0, lightRadiance * evaluatedLightSample.geomFactor * brdf);
                float scalerWeight = max(0.f, evaluatedLightSample.emission * evaluatedLightSample.geomFactor * brdfScalerWeight);

                float visibility = 1.0f;
                if (withVisibility) visibility = evalVisibility(evaluatedLightSample, isPrevFrame);
                pHat = scalerWeight * visibility;
            }
        }

        return pHat;
    }

    /** Evaluate the visibility of a light sample.
        \param[in] evaluatedLightSample Evaluated light sample.
        \return Returns the visibility, 1.f if visible, 0.f if occluded.
    */
    float evalVisibility(const EvaluatedLightSample evaluatedLightSample, bool isPrevFrame)
    {
        const Ray ray = Ray(pos, evaluatedLightSample.dir, 0.f, max(0.f, evaluatedLightSample.distance - Lights::kRayEpsilon));
        // const Ray ray = Ray(pos, evaluatedLightSample.dir, 0.f, max(0.f, evaluatedLightSample.distance));
        SceneRayQuery<kUseAlphaTest> sceneRayQuery;
        return sceneRayQuery.traceVisibilityRay(ray, RAY_FLAG_NONE, 0xff, isPrevFrame) ? 1.f : 0.f;
    }

    /** Evaluate the visibility of a light sample.
        \param[in] loadedLightSample Loaded light sample.
        \return Returns the visibility, 1.f if visible, 0.f if occluded.
    */
    float evalVisibility(const LoadedLightSample loadedLightSample, bool isPrevFrame)
    {
        return evalVisibility(loadedLightSample.eval(pos), isPrevFrame);
    }

    /** Trace a ray and report the hit point.
        \param[in] ray Ray.
        \param[out] hit Hit info.
        \return Returns true on hit.
    */
    bool traceRay(const Ray ray, out HitInfo hit, bool isPrevFrame)
    {
        SceneRayQuery<kUseAlphaTest> sceneRayQuery;
        float hitT;
        return sceneRayQuery.traceRay(ray, hit, hitT, RAY_FLAG_NONE, 0xff, isPrevFrame);
    }
};
