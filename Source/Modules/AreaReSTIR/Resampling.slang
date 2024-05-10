/***************************************************************************
 # Copyright (c) 2015-22, NVIDIA CORPORATION. All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/
__exported import Reservoir;
__exported import EvalContext;
__exported import Utils.Sampling.SampleGeneratorInterface;
import Scene.Scene;
import Scene.Camera.CameraData;
import Scene.RaytracingInline;
import Utils.Math.MathHelpers;
import Utils.Debug.PixelDebug;
import Utils.Geometry.GeometryHelpers;
import Lights;
import PixelAreaSampleData;
import LoadShadingData;
import Params;

// static const bool kUseAlphaTest = USE_ALPHA_TEST;
static const float kFloatEpsilon = 0.0001f;

cbuffer SharedResamplingCB
{
    bool useRandomThreshold;
    bool reuseThresholdOffset;
    bool scaleTwoShiftsWeightForMIS;
    bool betterScaleFuntionForMIS;
    uint scalingFunctionIndex;
}

/** Resampled importance sampling state.
*/
struct RisState
{
    LightSample lightSample;    ///< Selected light sample.
    float2 pixelSampleUV;
    float2 lensSampleUV;
    uint pathSample;
    float weightSum;            ///< Sum of weights seen so far.
    float M;                    ///< Number of samples seen so far (float required for pairwise MIS).
    float weight;               ///< UCW weight
    float targetPdf;
    float canonicalWeight; ///< Canonical sample weight factor (required for pairwise MIS).

    /** Create an empty RIS state.
     */
    static RisState createEmpty(float2 subPixelUV = float2(0.5f), float2 lensUV = float2(0), uint pathSample = 2u)
    {
        RisState state = {};
        state.lightSample = LightSample::createInvalid();
        state.pixelSampleUV = subPixelUV;
        state.lensSampleUV = lensUV;
        state.pathSample = pathSample;
        return state;
    }

    static RisState createEmpty(const Reservoir reservoir)
    {
        RisState state = {};
        state.lightSample = LightSample::createInvalid();
        state.pixelSampleUV = reservoir.pixelSampleUV;
        state.lensSampleUV = reservoir.lensSampleUV;
        state.pathSample = reservoir.pathSample;
        return state;
    }

    /** Create a reservoir from the current RIS state.
        An empty reservoir is returned if the weight is infinite or NaN.
        \return Returns a reservoir representing the current RIS state.
    */
    Reservoir toReservoir()
    {
        Reservoir reservoir;
        reservoir.lightSample = lightSample;
        reservoir.weight = weight;
        reservoir.M = uint(M);
        reservoir.pixelSampleUV = pixelSampleUV;
        reservoir.lensSampleUV = lensSampleUV;
        reservoir.pathSample = pathSample;
        reservoir.targetPdf = targetPdf;
        if (isinf(reservoir.weight) || isnan(reservoir.weight)) reservoir = Reservoir::createEmpty(pixelSampleUV, lensSampleUV, pathSample);
        return reservoir;
    }
};


/** Resample a light sample.
    This implements Algorithm 2 from the paper.
    \param[in,out] state RIS state.
    \param[in] lightSample Light sample to resample.
    \param[in] targetPdf Pdf of the sample in the target distribution (p-hat).
    \param[in] sourcePdf Pdf of the sample in the source distribution (p).
    \param[in,out] sg Sample generator.
    \return Returns true if new sample was selected.
*/
bool streamingInitialResampleMIS<S : ISampleGenerator>(inout RisState state, const LightSample lightSample, float targetPdf, float sourcePdf, float misWeight, float pHat_withoutAlbedo,
    inout S sg)
{
    float sampleWeight = misWeight * targetPdf / sourcePdf;

    state.weightSum += sampleWeight;
    state.M += 1.f;

    bool selectSample = sampleNext1D(sg) * state.weightSum < sampleWeight;

    if (selectSample)
    {
        state.lightSample = lightSample;
        state.targetPdf = targetPdf;
    }

    return selectSample;
}

// No pHat_withoutAlbedo
bool streamingInitialResampleMIS<S : ISampleGenerator>( inout RisState state, const LightSample lightSample, float targetPdf, float sourcePdf, float misWeight, inout S sg )
{
    float sampleWeight = misWeight * targetPdf / sourcePdf;

    state.weightSum += sampleWeight;
    state.M += 1.f;

    bool selectSample = sampleNext1D(sg) * state.weightSum < sampleWeight;

    if (selectSample)
    {
        state.lightSample = lightSample;
        state.targetPdf = targetPdf;
    }

    return selectSample;
}

// Resample a path sample
bool streamingInitialResampleMIS<S : ISampleGenerator>(inout RisState state, uint pathSample, float targetPdf, float sourcePdf, float misWeight, inout S sg)
{
    float sampleWeight = misWeight * targetPdf / sourcePdf;
    state.weightSum += sampleWeight;
    state.M += 1.f;
    bool selectSample = sampleNext1D(sg) * state.weightSum < sampleWeight;

    if (selectSample)
    {
        state.pathSample = pathSample;
        state.targetPdf = targetPdf;
    }

    return selectSample;
}

// For RGB reservoirs: resample with a reservoir using balanced heuristic
bool streamingResampleStepMIS<S : ISampleGenerator>(inout RisState state, const Reservoir reservoir, inout S sg, float misWeight)
{
    float sampleWeight = misWeight * reservoir.targetPdf * reservoir.weight * 1.0f;

    state.weightSum += sampleWeight;
    state.M += reservoir.M;

    print("sampleWeight = ", sampleWeight);
    print("reservoir UCW = ", reservoir.weight);

    bool selectSample = sampleNext1D(sg) * state.weightSum < sampleWeight;

    if (selectSample)
    {
        state.lightSample = reservoir.lightSample;
        state.targetPdf = reservoir.targetPdf;
        state.pixelSampleUV = reservoir.pixelSampleUV;
        state.lensSampleUV = reservoir.lensSampleUV;
        state.pathSample = reservoir.pathSample;
    }

    return selectSample;
}

// --------------------------------------------
// New resampling parts
// --------------------------------------------

// Song: I suppose this is used for debiasing? Where it is referred in the paper?
float mFactor(float q0, float q1)
{
    return q0 == 0.f ? 1.f : clamp(pow(min(q1 / q0, 1.f), 8.f), 0.f, 1.f);
}

// out_J = |inverse(T_i'(y))| -> jacobian from target to source domain
float pairwiseMIS_nonDefensiveNonCanonical(float cSum, float c0, float pi, float c1, float pc, float out_J = 1.0f)
{
    float pHatFrom_i = pi * out_J;
    return pc == 0.0f ? 0.0f : c0 * pHatFrom_i / ((cSum - c1) * pHatFrom_i + c1 * pc);
}

float pairwiseMIS_nonDefensiveCanonical(float cSum, float c0, float pi, float c1, float pc, float out_J = 1.0f)
{
    float w = c1 * pc;
    float pHatFrom_i = pi * out_J;
    return pc == 0.0f ? 0.0f : (c0 / (cSum - c1)) * w / ((cSum - c1) * pHatFrom_i + w);
    // return pc == 0.0f ? 0.0f : (c0 / cSum) * w / ((cSum - c1) * pi + w);
}

// Note: when using defensive version, canonical MIS weight should start with c1 / cSum
float pairwiseMIS_defensiveNonCanonical(float cSum, float c0, float pi, float c1, float pc, float out_J = 1.0f)
{
    float pHatFrom_i = pi * out_J;
    float w = (cSum - c1) * pHatFrom_i;
    return pc == 0.0f ? 0.0f : (c0 / cSum) * w / (w + c1 * pc);
}

float pairwiseMIS_defensiveCanonical(float cSum, float c0, float pi, float c1, float pc, float out_J = 1.0f)
{
    float pHatFrom_i = pi * out_J;
    float w = c1 * pc;
    return pc == 0.0f ? 0.0f : (c0 / cSum) * w / ((cSum - c1) * pHatFrom_i + w);
}

// For origin point reservoir temporal resampling
bool temporalResamplePointReservoirPairwiseMIS<S : ISampleGenerator>(
    inout RisState state,
    const Reservoir canonicalReservoir, const EvalContext canonicalContext, const Reservoir candidateReservoir, const EvalContext candidateContext,
    const Lights lights, const bool useMFactor, const float confidenceWeightSum, const float intersectAreaSize,
    const bool usePrevSceneData, inout S sg, const bool withVisibility = false)
{
    float canonicalTargetPdf = canonicalReservoir.targetPdf;
    float candidateTargetPdf = candidateReservoir.targetPdf;

    // Candidate sample evaluated at current
    const LoadedLightSample candidateLightSampleAtCurr = lights.loadLightSample(candidateReservoir.lightSample, false);
    float candidateTargetPdfAtOther = canonicalContext.evalTargetFunction( candidateLightSampleAtCurr, lights, false, withVisibility, sg, 2u);

    // Canonical sample evaluated at previous
    const LoadedLightSample canonicalLightSampleAtPrev = lights.loadLightSample(canonicalReservoir.lightSample, usePrevSceneData);
    float canonicalTargetPdfAtOther = candidateContext.evalTargetFunction( canonicalLightSampleAtPrev, lights, usePrevSceneData, withVisibility, sg, 2u);

    // Compute MIS weights
    float candidateConfidenceWeight = candidateReservoir.M * intersectAreaSize;
    float m0 = pairwiseMIS_nonDefensiveNonCanonical(confidenceWeightSum, candidateConfidenceWeight, candidateTargetPdf,
        canonicalReservoir.M, candidateTargetPdfAtOther);
    float m1 = pairwiseMIS_nonDefensiveCanonical(confidenceWeightSum, candidateConfidenceWeight, canonicalTargetPdfAtOther,
        canonicalReservoir.M, canonicalTargetPdf);

    // Candidate resampling weight
    float sampleWeight = candidateTargetPdfAtOther * candidateReservoir.weight * m0;
    float mScaler = min(mFactor(candidateTargetPdf, candidateTargetPdfAtOther), mFactor(canonicalTargetPdfAtOther, canonicalTargetPdf));
    state.M += candidateConfidenceWeight * (useMFactor ? mScaler : 1.0f);
    state.weightSum += sampleWeight;
    state.canonicalWeight += m1; // MIS weight for canonical reservoir

    bool selectSample = sampleNext1D(sg) * state.weightSum < sampleWeight;

    if (selectSample)
    {
        state.lightSample = candidateReservoir.lightSample;
        state.targetPdf = candidateTargetPdfAtOther;
    }

    return selectSample;
}

// For origin point reservoir spatial resampling
bool spatialResamplePointReservoirPairwiseMIS<S : ISampleGenerator>(
    inout RisState state,
    const Reservoir canonicalReservoir, const EvalContext canonicalContext, const Reservoir candidateReservoir, const EvalContext candidateContext,
    const Lights lights, const bool useMFactor, const float confidenceWeightSum, inout S sg,
    const bool withVisibility = false)
{
    float canonicalTargetPdf = canonicalReservoir.targetPdf;
    float candidateTargetPdf = candidateReservoir.targetPdf;

    const LoadedLightSample candidateLightSample = lights.loadLightSample(candidateReservoir.lightSample, false);
    const LoadedLightSample canonicalLightSample = lights.loadLightSample(canonicalReservoir.lightSample, false);

    float candidateTargetPdfAtOther = canonicalContext.evalTargetFunction( candidateLightSample, lights, false, withVisibility, sg, 2u); // Pc(Xi)
    float canonicalTargetPdfAtOther = candidateContext.evalTargetFunction( canonicalLightSample, lights, false, withVisibility, sg, 2u); // Pi(Xc)

    float canonicalTargetPdf2 = canonicalContext.evalTargetFunction( canonicalLightSample, lights, false, withVisibility, sg, 2u);
    float candidateTargetPdf2 = candidateContext.evalTargetFunction( candidateLightSample, lights, false, withVisibility, sg, 2u);

    // Compute MIS weights
    float candidateConfidenceWeight = candidateReservoir.M;
    float m0 = pairwiseMIS_defensiveNonCanonical(confidenceWeightSum, candidateConfidenceWeight, candidateTargetPdf, canonicalReservoir.M, candidateTargetPdfAtOther);
    float m1 = pairwiseMIS_defensiveCanonical(confidenceWeightSum, candidateConfidenceWeight, canonicalTargetPdfAtOther, canonicalReservoir.M, canonicalTargetPdf);

    // Candidate resampling weight
    float sampleWeight = candidateTargetPdfAtOther * candidateReservoir.weight * m0;
    float mScaler = min(mFactor(candidateTargetPdf, candidateTargetPdfAtOther), mFactor(canonicalTargetPdfAtOther, canonicalTargetPdf));
    state.M += candidateConfidenceWeight * (useMFactor ? mScaler : 1.0f);
    state.weightSum += sampleWeight;
    state.canonicalWeight += m1; // MIS weight for canonical reservoir

    bool selectSample = sampleNext1D(sg) * state.weightSum < sampleWeight;

    if (selectSample)
    {
        state.lightSample = candidateReservoir.lightSample;
        state.targetPdf = candidateTargetPdfAtOther;
    }

    return selectSample;
}

// -----------------------------------------
//  Area ReSTIR resampling part
// -----------------------------------------

// Shift path using random replay
void evalPhatAndJacobianForRandomReplay<S : ISampleGenerator>( uint2 frameDim, Reservoir reservoir, EvalContext baseContext, const Lights lights, const bool withVisibility, float2 shiftedPixel,
    CameraData shiftedCameraData, bool isShiftedToPrevFrame, inout S sg, inout float pHatFromShiftedDomain, inout float jacobian, inout float2 shiftedLensSampleUV, inout EvalContext shiftedEvalContext )
{
    shiftedEvalContext = retracePrimaryRay(shiftedPixel, frameDim, reservoir.pixelSampleUV, reservoir.lensSampleUV, isShiftedToPrevFrame, shiftedCameraData);

    const LoadedLightSample shiftedLightSample = lights.loadLightSample(reservoir.lightSample, isShiftedToPrevFrame);
    pHatFromShiftedDomain = shiftedEvalContext.evalTargetFunction(shiftedLightSample, lights, isShiftedToPrevFrame, withVisibility, sg, reservoir.pathSample);
    jacobian = 1.0f;
    shiftedLensSampleUV = reservoir.lensSampleUV;

    if (baseContext.isSurfaceValid() != shiftedEvalContext.isSurfaceValid())
        pHatFromShiftedDomain = 0;
}

// Compute the Jacobian determinant from source to target domain T'(x)
float evalJacobianForReconnection( float d0, float d1, float3 dir, float3 n0, float3 n1, float d0_shifted, float d1_shifted, float3 dir_shifted, float3 n0_shifted, float3 n1_shifted )
{
    float det_partial1_denom = dot(dir_shifted, n0_shifted) * d1_shifted * d1_shifted;
    float det_partial1 = det_partial1_denom == 0.0f ? 0.0f : (d0_shifted * d0_shifted * dot(dir_shifted, n1_shifted)) / det_partial1_denom ;
    float det_partial2_denom = dot(dir, n1) * d0 * d0;
    float det_partial2 = det_partial2_denom == 0.0f ? 0.0 : (d1 * d1 * dot(dir, n0)) / det_partial2_denom;

    return det_partial1 * det_partial2;
}

float evalJacobianForReconnection(JacobianEvalData baseData, float3 baseCameraNormalW, float3 baseRcNormalW, JacobianEvalData shiftedData, float3 shiftedCameraNormalW, float3 shiftedRcNormalW)
{
    return evalJacobianForReconnection( baseData.lensToPixelDist, baseData.pixelToRcHitDist, baseData.rayDir, baseCameraNormalW, baseRcNormalW,
        shiftedData.lensToPixelDist, shiftedData.pixelToRcHitDist, shiftedData.rayDir, shiftedCameraNormalW, shiftedRcNormalW );
}

// Note, the computation is in *world space*
float3 rayPlaneIntersection(float3 rayOrigin, float3 rayDir, float3 planeOrigin, float3 planeNormal)
{
    float t = abs(dot(rayOrigin - planeOrigin, planeNormal) / dot(rayDir, planeNormal));
    return rayOrigin + t * rayDir;
}

float computeCircleOfConfusionOnScreen(float2 frameDim, float3 primaryHitPosW, CameraData cameraData)
{
    // Pick two neighbor pixels on screen and project to focal plane in world space
    float2 p1 = (float2(uint2(frameDim / 2)) + float2(0.5f, 0.5f)) / frameDim;
    float2 p2 = (float2(uint2(frameDim / 2 + 1)) + float2(0.5f, 0.5f)) / frameDim;
    float2 ndc1 = float2(2, -2) * p1 + float2(-1, 1);
    float2 ndc2 = float2(2, -2) * p2 + float2(-1, 1);
    float3 p1PosW = cameraData.posW + ndc1.x * cameraData.cameraU + ndc1.y * cameraData.cameraV + cameraData.cameraW;
    float3 p2PosW = cameraData.posW + ndc2.x * cameraData.cameraU + ndc2.y * cameraData.cameraV + cameraData.cameraW;
    float onePixeldistW = distance(p1PosW, p2PosW);

    // Compute lens diameter on focal plane
    float dLens = cameraData.apertureRadius * 2.0f;
    float3 primaryHitPosCamera = mul(cameraData.viewMat, float4(primaryHitPosW, 1.0f)).xyz;
    float dFocal = dLens * abs(abs(primaryHitPosCamera.z) - cameraData.focalDistance) / abs(primaryHitPosCamera.z);

    // Scale the diameter to screen space ([-1, 1] -> [0, 1])
    return dFocal / onePixeldistW;
}

JacobianEvalData computeJacobianEvalDataAtBase(uint2 frameDim, EvalContext rcEvalContext, Reservoir reservoir, CameraData originCameraData, float2 originPixel)
{
    JacobianEvalData jEvalData = { 0, 0, normalize(-rcEvalContext.V), reservoir.lensSampleUV };

    if (!rcEvalContext.isSurfaceValid()) return jEvalData;

    float3 rcNormalW = rcEvalContext.N; // n1
    float3 rcHitOffsetedPosW = rcEvalContext.pos;

    // Compute origin lens sample, distance and ray direction
    float4x4 cameraViewMat = originCameraData.viewMat;
    float3 cameraPosW = originCameraData.posW;
    float3 cameraNormal = normalize(originCameraData.cameraW); // n0
    float2 apertureSample = sample_disk(reservoir.lensSampleUV);
    float3 lensSamplePosW = cameraPosW + originCameraData.apertureRadius * (apertureSample.x * normalize(originCameraData.cameraU) + apertureSample.y * normalize(originCameraData.cameraV));

    float2 pixelSamplePosScreen = (originPixel + reservoir.pixelSampleUV) / frameDim;
    float2 pixelSamplePosNDC = float2(2, -2) * pixelSamplePosScreen + float2(-1, 1);
    float3 pixelSamplePosW = cameraPosW + originCameraData.cameraW + (pixelSamplePosNDC.x * originCameraData.cameraU + pixelSamplePosNDC.y * originCameraData.cameraV);

    float3 rayDir = normalize(-rcEvalContext.V);                          // omega_0
    float lensToPixelDist = length(pixelSamplePosW - lensSamplePosW);     // d0
    float pixelToRcHitDist = length(rcHitOffsetedPosW - pixelSamplePosW); // d1
    float lensToRcHitDist = length(rcHitOffsetedPosW - lensSamplePosW);   // Note, d0 + d1 != lensToRcHitDist if rcHit is in front of focal plane

    jEvalData = { lensToPixelDist, pixelToRcHitDist, rayDir, reservoir.lensSampleUV };

    return jEvalData;
}

// Primary hit reconnection shift
void evalPhatAndJacobianForReconnection<S : ISampleGenerator>( uint2 frameDim, EvalContext rcEvalContext, Reservoir reservoir, const Lights lights, const bool withVisibility, float2 originPixel,
    CameraData originCameraData, float2 shiftedPixel, CameraData shiftedCameraData, bool isShiftedToPrevFrame, inout S sg, inout float pHatFromShiftedDomain,
    inout float jacobian, inout float2 shiftedLensSampleUV, inout EvalContext shiftedEvalContext, inout JacobianEvalData jEvalDataShifted, const bool onlyEvalJacobian = false)
{
    // Default values if early out by return
    shiftedLensSampleUV = uint2(0);
    shiftedEvalContext = rcEvalContext;
    shiftedLensSampleUV = float2(0);
    jEvalDataShifted = { 0, 0, normalize(-rcEvalContext.V), float2(0) };
    pHatFromShiftedDomain = 0;
    jacobian = 1.0f;

    // Reconnection shift cannot reuse background hit
    if (!rcEvalContext.isSurfaceValid())
    {
        print("invalid surface hit", 1);
        return;
    }

    // Compute data for jacobian evaluation at base path
    JacobianEvalData jEvalDataBase = computeJacobianEvalDataAtBase(frameDim, rcEvalContext, reservoir, originCameraData, originPixel);

    // Compute shifted primary hit, lens sample, distance and ray direction
    float4x4 shiftedCameraViewMat = shiftedCameraData.viewMat;
    float3 shiftedCameraPosW = shiftedCameraData.posW;
    float3 shiftedCameraNormal = normalize(shiftedCameraData.cameraW); // n0'
    float2 shiftedPixelSamplePosScreen = (shiftedPixel + reservoir.pixelSampleUV) / frameDim;
    float2 shiftedPixelSamplePosNDC = float2(2, -2) * shiftedPixelSamplePosScreen + float2(-1, 1);
    float3 shiftedPixelSamplePosW = shiftedCameraPosW + shiftedCameraData.cameraW + (shiftedPixelSamplePosNDC.x * shiftedCameraData.cameraU + shiftedPixelSamplePosNDC.y * shiftedCameraData.cameraV);
    float3 shiftedPixelSamplePosShiftedCamera = mul(shiftedCameraViewMat, float4(shiftedPixelSamplePosW, 1.0f)).xyz;

    // With object motion, we need to recompute the position at reconnection hit.
    //  -> but without ray direction, we can only compute the unoffseted position first
    uint materialID;
    VertexData shiftedRcHitVertexData = loadVertexData(rcEvalContext.hit, float3(0), float3(0), materialID, isShiftedToPrevFrame);
    float3 shiftedRcHitPosW = shiftedRcHitVertexData.posW;

    // Check if reconnection hit is in front or behind the focal plane -> to ensure precision, we transform primary hit to camera space
    float3 shiftedRcHitPosShiftedCamera = mul(shiftedCameraViewMat, float4(shiftedRcHitPosW, 1.0f)).xyz;
    float distanceToShiftedFocalPlane = abs(shiftedRcHitPosShiftedCamera.z) - shiftedCameraData.focalDistance;
    float3 shiftedRayDir = distanceToShiftedFocalPlane > 0.0f ? normalize(shiftedRcHitPosW - shiftedPixelSamplePosW) : normalize(shiftedPixelSamplePosW - shiftedRcHitPosW); // omega_0'

    // Note: it seems we need to use *unoffseted* reconnection hit for *ray direction*,
    // so no need to recompute shifted ray direction with *offseted* reconnection hit position.

    let lod = ExplicitLodTextureSampler(0.f);
    ShadingData shiftedSd = loadShadingData(rcEvalContext.hit, float3(0), shiftedRayDir, true, lod, isShiftedToPrevFrame);
    float3 shiftedRcHitOffsetedPosW = shiftedSd.computeRayOrigin();

    // Perform a simple ray-plane intersection to find shifted sample on lens
    float3 shiftedLensSamplePosW = rayPlaneIntersection(shiftedRcHitPosW, -shiftedRayDir, shiftedCameraPosW, shiftedCameraNormal);
    float shiftedPixelToLensSampleDist = length(shiftedLensSamplePosW - shiftedPixelSamplePosW);     // d0'
    float rcHitToShiftedPixelSampleDist = length(shiftedPixelSamplePosW - shiftedRcHitOffsetedPosW); // d1'

    // Transform to camera space to compute lens sample uv
    // Note, there are some precision issue causes the shiftedLensSamplePosCamera.z not equal to 0, do we need to fix this?
    float4 shiftedLensSamplePosShiftedCamera = mul(shiftedCameraViewMat, float4(shiftedLensSamplePosW, 1.0f));
    shiftedLensSamplePosShiftedCamera = shiftedCameraData.apertureRadius > 0.0f ? shiftedLensSamplePosShiftedCamera / shiftedCameraData.apertureRadius : 0.0f;
    shiftedLensSampleUV = inverse_sample_disk(shiftedLensSamplePosShiftedCamera.xy);

    // Create new context at shifted path
    shiftedEvalContext = EvalContext::createFromHit(rcEvalContext.hit, shiftedLensSamplePosW, shiftedRayDir, isShiftedToPrevFrame);
    const LoadedLightSample shiftedLightSample = lights.loadLightSample(reservoir.lightSample, isShiftedToPrevFrame);
    const EvaluatedLightSample shiftedEvaluatedLightSample = shiftedLightSample.eval(shiftedEvalContext.pos);

    // Compute jacobian from base to shifted path
    jacobian = evalJacobianForReconnection( jEvalDataBase.lensToPixelDist, jEvalDataBase.pixelToRcHitDist, jEvalDataBase.rayDir, normalize(originCameraData.cameraW), rcEvalContext.N,
        shiftedPixelToLensSampleDist, rcHitToShiftedPixelSampleDist, shiftedRayDir, shiftedCameraNormal, shiftedEvalContext.N ); // Note, n1_shifted should be recompute if object moves
    jEvalDataShifted = { shiftedPixelToLensSampleDist, rcHitToShiftedPixelSampleDist, shiftedRayDir, shiftedLensSampleUV };
    if (onlyEvalJacobian) return;

    // Shifted primary hit should not on the focal plane
    if (abs(distanceToShiftedFocalPlane) == 0.0f) return;

    // Handle special case: static and aperture = 0
    if (distance(originPixel, shiftedPixel) <= kFloatEpsilon && shiftedCameraData.apertureRadius == 0)
    {
        print("Handle special case", 1);
        pHatFromShiftedDomain = reservoir.targetPdf;
        jacobian = 1.0f;
        shiftedEvalContext = rcEvalContext;
        return;
    }

    // Check if the shift is outside valid lens
    if (length(shiftedLensSamplePosW - shiftedCameraData.posW) > shiftedCameraData.apertureRadius)
    {
        print("shift outside valid lens", 1);
        return;
    }

    // Check if the shifted path can be sampled by BRDF
    if (shiftedEvalContext.evalBsdfPDF(shiftedEvaluatedLightSample.dir, isShiftedToPrevFrame) <= 0.0001f)
    {
        print("p(BRDF) too small", 1);
        return;
    }

    // Check if the visibilty ray is occluded
    const Ray primaryShadowRay = Ray(shiftedLensSamplePosW, shiftedRayDir, 0.0f, max(0, length(shiftedRcHitOffsetedPosW - shiftedLensSamplePosW) * 0.9999f));
    SceneRayQuery<kUseAlphaTest> sceneRayQuery;
    if (!sceneRayQuery.traceVisibilityRay(primaryShadowRay, RAY_FLAG_NONE, 0xff, isShiftedToPrevFrame))
    {
        print("shifted ray is occluded", 1);
        return;
    }

    // Evaluate pHat from target (canonical) domain
    pHatFromShiftedDomain = shiftedEvalContext.evalTargetFunction(shiftedLightSample, lights, isShiftedToPrevFrame, withVisibility, sg, reservoir.pathSample); // Pc(Y_i)
}

// Find the shift mapping to use from base to offset path
uint findShiftMapping(uint shiftMappingMode, uint2 frameDim, const float3 basePrimaryHitPosW, const CameraData baseCameraData, uint passIdx)
{
    uint selectedShiftMapping = uint(-1);
    if (shiftMappingMode == uint(ShiftMappingModeInReusing::OnlyRandomReplay))
    {
        selectedShiftMapping = uint(ShiftMapping::RandomReplay);
    }
    else if (shiftMappingMode == uint(ShiftMappingModeInReusing::OnlyReconnection))
    {
        selectedShiftMapping = uint(ShiftMapping::PrimaryHitReconnection);
    }
    else if (shiftMappingMode == uint(ShiftMappingModeInReusing::MIS))
    {
        selectedShiftMapping = passIdx == 0 ? uint(ShiftMapping::RandomReplay) : uint(ShiftMapping::PrimaryHitReconnection);
    }

    return selectedShiftMapping;
}

void evalPhatAndJacobian<S : ISampleGenerator>( uint selectedShiftMapping, uint2 frameDim, EvalContext baseEvalContext, Reservoir baseReservoir, const Lights lights, const bool withVisibility,
    float2 basePixel, CameraData baseCameraData, float2 shiftedPixel, CameraData shiftedCameraData, bool isShiftedToPrevFrame, inout S sg, inout float pHatFromShiftedDomain, inout float jacobian,
    inout float2 shiftedLensSampleUV, inout EvalContext shiftedEvalContext )
{
    if (selectedShiftMapping == uint(ShiftMapping::RandomReplay))
    {
        evalPhatAndJacobianForRandomReplay( frameDim, baseReservoir, baseEvalContext, lights, withVisibility, shiftedPixel, shiftedCameraData, isShiftedToPrevFrame, sg,
            pHatFromShiftedDomain, jacobian, shiftedLensSampleUV, shiftedEvalContext );
    }
    else if (selectedShiftMapping == uint(ShiftMapping::PrimaryHitReconnection))
    {
        // Reuse candidate primary hit x1 and subpixel uv to canonical domain
        JacobianEvalData dummyData = {};
        evalPhatAndJacobianForReconnection( frameDim, baseEvalContext, baseReservoir, lights, withVisibility, basePixel, baseCameraData, shiftedPixel, shiftedCameraData, isShiftedToPrevFrame,
            sg, pHatFromShiftedDomain, jacobian, shiftedLensSampleUV, shiftedEvalContext, dummyData );
    }
}

inline float fitted_function(float x)
{
    const float3 coefficents = float3(8.96701974, -8.27136859, 0.15271796);
    return coefficents.x / (x - coefficents.y) + coefficents.z;
}

inline float fitted_function1(float x)
{
    const float3 coefficents = float3(17.9394832, -21.38280069, 0.1);
    return coefficents.x / (x - coefficents.y) + coefficents.z;
}

inline float fitted_function2(float x)
{
    const float3 coefficents = float3(6.24787234, -5.61499532, 0.2);
    return coefficents.x / (x - coefficents.y) + coefficents.z;
}

inline float fitted_function3(float x)
{
    const float3 coefficents = float3(2.39413454, -1.73174287, 0.3);
    return coefficents.x / (x - coefficents.y) + coefficents.z;
}

float computeConfidenceWeightForTwoShifts(float2 frameDim, EvalContext primaryHitEvalContext, CameraData cameraData)
{
    float confidenceWeightScaler = 0.5;

    if (scaleTwoShiftsWeightForMIS)
    {
        if (!primaryHitEvalContext.isSurfaceValid()) return 1.0f;

        // Note: use sample's primay hit to compute CoC here will cause bias, so we need to use pixel center's hit
        float coc = computeCircleOfConfusionOnScreen(frameDim, primaryHitEvalContext.pos, cameraData);

        // Scale the random replay confidence weight
        if (betterScaleFuntionForMIS)
        {
            if (scalingFunctionIndex == 0)
            {
                confidenceWeightScaler = coc <= 2.0f ? 1.0f : fitted_function1(coc);
            }
            else if (scalingFunctionIndex == 1)
            {
                confidenceWeightScaler = coc <= 2.0f ? 1.0f : fitted_function2(coc);
            }
            else if (scalingFunctionIndex == 2)
            {
                confidenceWeightScaler = coc <= 2.0f ? 1.0f : fitted_function3(coc);
            }
            else if (scalingFunctionIndex == 3)
            {
                confidenceWeightScaler = coc <= 2.0f ? 1.0f : fitted_function(coc);
                confidenceWeightScaler = clamp(confidenceWeightScaler, 0.2f, 1.0f);
            }

        }
        else
        {
            float4 cocMax = float4(2.0f, 8.0f, 32.0f, 500.0f);
            if (coc <= cocMax.x) confidenceWeightScaler = 1.0f; // -> 1.0
            else if (coc <= cocMax.y) confidenceWeightScaler = 1.0f - 0.3f * ((coc - cocMax.x) / (cocMax.y - cocMax.x)); // -> [1.0, 0.7]
            else if (coc <= cocMax.z) confidenceWeightScaler = 0.7f - 0.4f * ((coc - cocMax.y) / (cocMax.z - cocMax.y)); // -> (0.7, 0.3]
            else confidenceWeightScaler = max(0.2f, 0.3f - 0.1 * ((coc - cocMax.z) / (cocMax.w - cocMax.z)));
        }
    }

    return confidenceWeightScaler;
}

// Temporal or spatial resampling using primary hit reconnection shift mapping
void resampleAreaReservoirPairwiseMIS<S : ISampleGenerator>(
    inout RisState state,
    const Reservoir canonicalReservoir, const EvalContext canonicalContext, const Reservoir candidateReservoir, const EvalContext candidateContext,
    const EvalContext candidatePixelCenterContext,
    const uint2 frameDim, const uint2 canonicalPixel, const float2 candidatePixel,
    const Lights lights, const bool useMFactor, const float confidenceWeightSum, const float candidateBilinearWeight,
    const bool usePrevSceneData, bool isTemporalReusing, uint shiftMappingMode,
    inout S sg, inout EvalContext contextFromCanonicalForSelectedSample,
    const bool withVisibility = true, const CameraData prevCameraData = {})
{
    float pHatFromCanonical = canonicalReservoir.targetPdf;
    float pHatCandidate = candidateReservoir.targetPdf;
    CameraData currCameraData = gScene.camera.data;
    CameraData candidateCameraData = isTemporalReusing ? prevCameraData : currCameraData;

    // Scale candidate confidence weight for MIS shift mode
    float confidenceWeightScaler = 1.0f;
    uint numPasses = 1u;
    if (shiftMappingMode == uint(ShiftMappingModeInReusing::MIS))
    {
        numPasses = 2u;
        confidenceWeightScaler = computeConfidenceWeightForTwoShifts(frameDim, candidatePixelCenterContext, candidateCameraData);
    }

    for (uint pass = 0; pass < numPasses; pass++)
    {
        confidenceWeightScaler = pass == 0 ? confidenceWeightScaler : 1.0 - confidenceWeightScaler;
        if (confidenceWeightScaler == 0.0f) continue;

        // Shift candidate path to canonical domain
        float pHatFromCanonicalForCandidateSample = 0.0f;
        float inJacobianForCandidateSample = 1.0f;       // |T'(X_i))|
        float2 shiftedCandidateLensSampleUV = float2(0); // s't'
        EvalContext contextFromCanonicalForCandidateSample = {};

        uint selectedShiftMapping = findShiftMapping(shiftMappingMode, frameDim, candidateContext.pos, candidateCameraData, pass);

        evalPhatAndJacobian( selectedShiftMapping, frameDim, candidateContext, candidateReservoir, lights, withVisibility, candidatePixel, candidateCameraData, canonicalPixel,
            currCameraData, false, sg, pHatFromCanonicalForCandidateSample, inJacobianForCandidateSample, shiftedCandidateLensSampleUV, contextFromCanonicalForCandidateSample );

        // For MIS weight, we also need to shift canonical path to candidate domain
        float pHatFromCandidateForCanonicalSample = 0.0f; // p_j(y_c)
        float outJacobianForCanonicalSample = 1.0f; // |inverse(T(y_c))|, Q: for this Jacobian, is it really the outgoing Jacobian, not the in Jaocbian as previous?
        float2 dummyLensSampleUV = float2(0);
        EvalContext dummyEvalContext = {};

        selectedShiftMapping = findShiftMapping(shiftMappingMode, frameDim, canonicalContext.pos, currCameraData, pass);
        evalPhatAndJacobian( selectedShiftMapping, frameDim, canonicalContext, canonicalReservoir, lights, withVisibility, canonicalPixel, currCameraData, candidatePixel,
            candidateCameraData, isTemporalReusing ? true : false, sg, pHatFromCandidateForCanonicalSample, outJacobianForCanonicalSample, dummyLensSampleUV, dummyEvalContext);

        // Compute pairwise MIS weights
        float confidenceWeightFromCandidate = confidenceWeightScaler * candidateReservoir.M * candidateBilinearWeight;
        float m0_nonDefensive = pairwiseMIS_nonDefensiveNonCanonical( confidenceWeightSum, confidenceWeightFromCandidate, pHatCandidate, canonicalReservoir.M, pHatFromCanonicalForCandidateSample,
            1.0 / inJacobianForCandidateSample );
        float m0_defensive = pairwiseMIS_defensiveNonCanonical( confidenceWeightSum, confidenceWeightFromCandidate, pHatCandidate, canonicalReservoir.M, pHatFromCanonicalForCandidateSample,
            1.0 / inJacobianForCandidateSample );
        float m1_nonDefensive = pairwiseMIS_nonDefensiveCanonical( confidenceWeightSum, confidenceWeightFromCandidate, pHatFromCandidateForCanonicalSample, canonicalReservoir.M, pHatFromCanonical,
            outJacobianForCanonicalSample );
        float m1_defensive = pairwiseMIS_defensiveCanonical( confidenceWeightSum, confidenceWeightFromCandidate, pHatFromCandidateForCanonicalSample, canonicalReservoir.M, pHatFromCanonical,
            outJacobianForCanonicalSample );
        float m0 = isTemporalReusing ? m0_nonDefensive : m0_defensive;
        float m1 = isTemporalReusing ? m1_nonDefensive : m1_defensive;

        // Candidate resampling weight
        float sampleWeight = m0 * pHatFromCanonicalForCandidateSample * candidateReservoir.weight * inJacobianForCandidateSample; // m_i * pHat_c * W_Xi * in_J
        state.M += confidenceWeightFromCandidate;
        state.weightSum += sampleWeight;
        state.canonicalWeight += m1; // MIS weight for canonical reservoir

        bool selectSample = candidateBilinearWeight > 0 ? sampleNext1D(sg) * state.weightSum < sampleWeight : false;
        if (selectSample)
        {
            state.targetPdf = pHatFromCanonicalForCandidateSample;
            state.lightSample = candidateReservoir.lightSample;
            state.pixelSampleUV = candidateReservoir.pixelSampleUV;
            state.pathSample = candidateReservoir.pathSample;
            state.lensSampleUV = shiftedCandidateLensSampleUV;
            contextFromCanonicalForSelectedSample = contextFromCanonicalForCandidateSample;
        }
    }
}

// Resample the canonical reservoir in pairwise MIS -> jacobian is just one (identical shift)
bool streamingResampleFinalizeMis<S : ISampleGenerator>( inout RisState state, const Reservoir canonicalReservoir, float canonicalTargetPdf, inout S sg)
{
    float sampleWeight = state.canonicalWeight * canonicalTargetPdf * canonicalReservoir.weight * 1.0f;

    state.M += canonicalReservoir.M;
    state.weightSum += sampleWeight;

    bool selectSample = sampleNext1D(sg) * state.weightSum < sampleWeight;
    if (selectSample)
    {
        state.targetPdf = canonicalTargetPdf;
        state.lightSample = canonicalReservoir.lightSample;
        state.pixelSampleUV = canonicalReservoir.pixelSampleUV;
        state.lensSampleUV = canonicalReservoir.lensSampleUV;
        state.pathSample = canonicalReservoir.pathSample;
    }

    return selectSample;
}

