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
import Scene.Camera.CameraData;
import Utils.Attributes;
import Utils.Sampling.TinyUniformSampleGenerator;
import Utils.Debug.PixelDebug;
import SurfaceData;
import EvalContext;
import Lights;
import Resampling;
import Params;
import PixelAreaSampleData;
import Utils.Math.BitTricks;

/** Performs temporal resampling by combining the reservoir from the initial
    resampling pass with a reservoir from the last frame obtained through
    reprojection.
*/
struct TemporalResampling
{
    // Static configuration.
    static const uint kMaxHistoryLength = MAX_HISTORY_LENGTH;

    static const bool kUnbiased = UNBIASED;
    static const bool kUsePrevFrameSceneData = USE_PREV_FRAME_SCENE_DATA;
    static const float kFloatCompareEplison = 1e-6;

    static const DebugOutput kDebugOutput = DebugOutput(DEBUG_OUTPUT);

    static const uint2 kPrevNeighborOffsets[4] = {
        uint2(0, 0), uint2(1, 0), uint2(0, 1), uint2(1, 1)
    };
    static const uint2 kPrevNeighborOffsetsVertical[4] = {
        uint2(0, 0), uint2(0, 1), uint2(1, 0), uint2(1, 1)
    };
    static const uint kPrevNeighborCount = 4;
    static const float2 reprojectedPixelSize = float2(1);

    Texture2D<float2> motionVectors;
    RWStructuredBuffer<Reservoir> reservoirs;
    StructuredBuffer<Reservoir> prevReservoirs;
    RWStructuredBuffer<EvalContext> resEvalContext;
    StructuredBuffer<EvalContext> prevResEvalContext;
    StructuredBuffer<EvalContext> pixelCenterEvalContext;
    StructuredBuffer<EvalContext> prevPixelCenterEvalContext;
    StructuredBuffer<CameraData> prevCameraData;
    StructuredBuffer<float> temporalMISPDFs;
    StructuredBuffer<EvalContext> temporalMISEvalContexts;
    StructuredBuffer<JacobianEvalData> temporalMISJacobianData;
    StructuredBuffer<float3> temporalMISPrimHitNormals;
    StructuredBuffer<PackedHitInfo> temporalMISPrimaryHits;

    RWTexture2D<float4> debugOutput;

    Lights lights;

    uint2 frameDim;
    uint frameIndex;

    float normalThreshold;
    float depthThreshold;
    bool useMFactor;
    uint temporalReuseMode; // 0: integer, 1: float unbiased, 2: float biased
    uint shiftMappingModeRIS1;
    uint shiftMappingModeRIS2;
    bool optimizeShift2RIS;
    uint shiftsPerPixel;
    uint resampleEmissionMode;
    uint restirPassIdx;

    float filterRadius;
    float filterAlpha;
    float filterNorm;

    PixelAreaSampleData pixelAreaSampleData;

    // -----------------------------------------
    // Helper funtions
    // -----------------------------------------

    uint getPixelIndex(uint2 pixel) { return pixel.y * frameDim.x + pixel.x; }

    bool isValidPixel(int2 pixel) { return all(pixel >= 0 && pixel < frameDim); }
    bool isValidPixel(float2 pixel) { return all(pixel >= 0.0f && pixel < float2(frameDim)); }

    /**
     * Check if previous pixel sample inside the reprojected range
     * @param[in] prevNeighborOffset pixel offset from the offset array, this gives which one of the four pixel it is
     * @param[in] pixelSampleUV pixel sample in [0, 1)
     * @param[in] reprojectedMinPointFrac reprojected range min point from 2x2 top-left point
     */
    bool sampleInTargetRange(uint2 prevNeighborOffset, float2 pixelSampleUV, float2 reprojectedMinPointFrac)
    {
        // Try to use fractional value to avoid precision loss.
        // Make the basis origin from 2x2 top-left point to center -> two points' range will convert from [0, 2) to [-1, 1)
        float2 pixelSamplePos = prevNeighborOffset - float2(1) + pixelSampleUV;
        float2 minPointPos = reprojectedMinPointFrac - float2(1);
        float2 maxPointPos = minPointPos + float2(1);

        return all(pixelSamplePos >= minPointPos) && all(pixelSamplePos < maxPointPos);
        // return all(pixelSamplePos - minPointPos >= -kFloatCompareEplison) && all(pixelSamplePos - maxPointPos < kFloatCompareEplison);
    }
    bool sampleInTargetRange(uint2 prevNeighborOffset, float2 pixelSampleUV, float2 reprojectedMinPointFrac, float squareRadius)
    {
        float2 pixelSamplePos = prevNeighborOffset - float2(1) + pixelSampleUV;
        float2 squareCenterCoord = reprojectedMinPointFrac - float2(0.5);
        return isSampleInSquare_Use2x2CenterAsOrigin(pixelSamplePos, squareCenterCoord, squareRadius);
    }

    // for non-box filter

    bool isSampleInSquare_Use2x2CenterAsOrigin(float2 sampleCoord, float2 squareCenterCoord, float squareRadius)
    {
        return all(sampleCoord >= squareCenterCoord - squareRadius && sampleCoord < squareCenterCoord + squareRadius);
    }

    float2 computeSquareLocalCoord(float2 sampleCoord, float2 squareCenterCoord, float squareRadius)
    {
        float2 p = sampleCoord - squareCenterCoord;
        if (!all(p >= -squareRadius && p < squareRadius)) return 1e10f;
        else return p;
    }

    // This the same if we compute the weight using bilinear interpolation on four pixels centers
    void computeBilinearWeights(inout float4 weights, float2 reprojectedMinPointFrac, bool onlyVerticalMotion = false)
    {
        // Compute the overlapping area of 4 previous neighbor pixel with reprojected pixel
        float x0 = reprojectedMinPointFrac.x;
        float y0 = reprojectedMinPointFrac.y;

        weights[0] = (1.0f - x0) * (1.0f - y0);                                                       // Top-left
        weights[1] = (reprojectedPixelSize.x - (1.0f - x0)) * (1.0f - y0);                            // Top-right
        weights[2] = (1.0f - x0) * (reprojectedPixelSize.y - (1.0f - y0));                            // Bottom-left
        weights[3] = (reprojectedPixelSize.x - (1.0f - x0)) * (reprojectedPixelSize.y - (1.0f - y0)); // Bottom-right

        if (onlyVerticalMotion)
        {
            float temp = weights[2];
            weights[2] = weights[1];
            weights[1] = temp;
        }
    }

    // This is for temporal resampling previous neighbor with float motion, using balanced heurstic
    bool streamingResampleStepMis<S : ISampleGenerator>( inout RisState state, const Reservoir reservoir, inout S sg, const float bilinearWeight = 1.0f, const float misWeight = 1.0f,
        const float jacobianSourceToTarget = 1.0f )
    {
        // m_i * (bilinearWeight) * p_Hat * W_i * Jacobian (=1.0f)
        float sampleWeight = misWeight * reservoir.targetPdf * reservoir.weight * jacobianSourceToTarget; // TODO: for bilinear resampling, we need to

        state.weightSum += sampleWeight;
        state.M += reservoir.M * bilinearWeight;

        bool selectSample = bilinearWeight > 0 ? sampleNext1D(sg) * state.weightSum < sampleWeight : false;
        if (selectSample)
        {
            state.targetPdf = reservoir.targetPdf;
            state.lightSample = reservoir.lightSample;
            state.pixelSampleUV = reservoir.pixelSampleUV;
            state.lensSampleUV = reservoir.lensSampleUV;
            state.pathSample = reservoir.pathSample;
        }

        return selectSample;
    }

    // -----------------------------------------
    // Temporal Resampling functions
    // -----------------------------------------

    void process_2RIS(uint2 pixel)
    {
        if (any(pixel >= frameDim)) return;

        // printSetPixel(pixel);

        print("temporal reusing float motion 2RIS ---------------------", 1);

        const uint pixelIndex = getPixelIndex(pixel);
        uint pixelIdxInReservoirBuffer = getPixelIndex(frameDim, pixel, restirPassIdx);
        TinyUniformSampleGenerator sg = TinyUniformSampleGenerator(pixel, frameIndex);

        // Get current frame pixel and its reservoir data
        Reservoir currentReservoir = reservoirs[pixelIdxInReservoirBuffer];
        EvalContext currentContext = resEvalContext[pixelIdxInReservoirBuffer];

        // Get camera data between two frames
        CameraData currCameraData = gScene.camera.data;
        CameraData previousCameraData = prevCameraData[0];
        float3 prevCameraNormalW = normalize(previousCameraData.cameraW);
        uint numPassesForFirstRIS = shiftMappingModeRIS1 == uint(ShiftMappingModeInReusing::MIS) ? 2u : 1u;

        // Reproject current pixel to previous frame
        float2 screenMotion = motionVectors[pixel] * float2(frameDim);
        uint2 reprojectedMinPointInt = pixel + int2(floor(screenMotion));
        float2 reprojectedMinPointFrac = screenMotion - floor(screenMotion);
        if (!isValidPixel(reprojectedMinPointInt)) return;

        // Without motion, no need to use all four previous pixels.
        //  -> also if there is only x/y motion, only need to resample two pixels
        uint prevNeighborCount = screenMotion.x != 0 ? (screenMotion.y != 0 ? kPrevNeighborCount : 2u) : (screenMotion.y != 0 ? 2u : 1u);
        bool onlyVerticalMotion = screenMotion.x == 0 && screenMotion.y != 0 ? true : false;
        uint2 prevNeighborOffsets[4] = onlyVerticalMotion ? kPrevNeighborOffsetsVertical : kPrevNeighborOffsets;

        // Compute the bilinear weight of 4 previous neighbor pixel with reprojected pixel
        float4 bilinearWeights;
        computeBilinearWeights(bilinearWeights, reprojectedMinPointFrac, onlyVerticalMotion);

        // If only DoF is enabled, we don't have range for subpixel sample, it's always at the center. So, we need to use reprojected *point* to decide
        // which pixel as the target domain
        bool onlyReuseDoF = !pixelAreaSampleData.reuseSubpixelSample && pixelAreaSampleData.reuseLensSample;
        uint2 targetPrevPixel = reprojectedMinPointInt + uint2(reprojectedMinPointFrac + float2(0.5f));
        uint2 targetPrevPixelOffset = targetPrevPixel - reprojectedMinPointInt;
        uint targetPrevPixelIndex = onlyVerticalMotion ? 2 * targetPrevPixelOffset.x + targetPrevPixelOffset.y : 2 * targetPrevPixelOffset.y + targetPrevPixelOffset.x;

        // RIS step 1: resample the previous 4 neighbor pixels
        RisState reprojectedRisState = RisState::createEmpty(float2(0.5f), float2(0), resampleResourceType == uint(ResamplingResourceType::DirectLighting) ? 2u : 1u);
        EvalContext selectedPrevNeighborContext = {};
        int selectedIdx = -1;
        uint2 selectedPrevPixel = reprojectedMinPointInt;
        int prevSampleSelectedShiftMapping = -1;
        float3 selectedPrevNormal = float3(0);
        PackedHitInfo selectedPrevPackedHit = {};

        for (uint i = 0; i < prevNeighborCount; i++)
        {
            // Get previous neighbor pixel top-left position
            uint2 prevNeighborPixel = reprojectedMinPointInt + prevNeighborOffsets[i];
            if (!isValidPixel(prevNeighborPixel)) continue;
            uint prevNeighborPixelIndex = getPixelIndex(prevNeighborPixel);

            // Load reserovir data (base path)
            Reservoir prevNeighborReservoir = prevReservoirs[prevNeighborPixelIndex];
            const LoadedLightSample prevNeighborLightSample = lights.loadLightSample(prevNeighborReservoir.lightSample, true);
            EvalContext prevNeighborContext = prevResEvalContext[prevNeighborPixelIndex];
            float3 prevRcNormalW = prevNeighborContext.N;
            prevNeighborReservoir.M = min(prevNeighborReservoir.M, currentReservoir.M * kMaxHistoryLength);
            JacobianEvalData prevNeighborJacoEvalData = computeJacobianEvalDataAtBase(frameDim, prevNeighborContext, prevNeighborReservoir, previousCameraData, prevNeighborPixel);

            // If use MIS shift, scale the confidence weight based on CoC
            float confidenceWeightScaler = 1.0f;
            if (shiftMappingModeRIS1 == uint(ShiftMappingModeInReusing::MIS))
            {
                confidenceWeightScaler = computeConfidenceWeightForTwoShifts(frameDim, prevPixelCenterEvalContext[prevNeighborPixelIndex], previousCameraData);
            }

            // Only DoF is enabled
            if (onlyReuseDoF)
            {
                if (temporalReuseMode == uint(TemporalReuseMode::FractionalMotion_Shifting2RIS))
                {
                    // Compute shifted lens and light sample at target pixel
                    float pHatSource = prevNeighborReservoir.targetPdf;
                    Reservoir shiftedNeighborReservoir = prevNeighborReservoir;
                    EvalContext shiftedNeighborContext = retracePrimaryRay( targetPrevPixel, frameDim, prevNeighborReservoir.pixelSampleUV, prevNeighborReservoir.lensSampleUV, true, previousCameraData );
                    float pHatTarget = shiftedNeighborContext.evalTargetFunction( prevNeighborLightSample, lights, true, kUnbiased, sg, prevNeighborReservoir.pathSample );
                    shiftedNeighborReservoir.targetPdf = pHatTarget;

                    // Compute MIS weight
                    float misWeight = 0;
                    for (uint j = 0; j < prevNeighborCount; j++)
                    {
                        // Load offset reservoir data
                        uint2 offsetPixel = reprojectedMinPointInt + prevNeighborOffsets[j];
                        Reservoir offsetReservoir = prevReservoirs[getPixelIndex(offsetPixel)];
                        offsetReservoir.M = min(offsetReservoir.M, currentReservoir.M * kMaxHistoryLength);

                        if (all(offsetPixel == prevNeighborPixel))
                        {
                            misWeight += prevNeighborReservoir.M * pHatSource;
                        }
                        else if (all(offsetPixel == targetPrevPixel))
                        {
                            misWeight += offsetReservoir.M * pHatTarget;
                        }
                        else
                        {
                            EvalContext offsetEvalContext = retracePrimaryRay( offsetPixel, frameDim, shiftedNeighborReservoir.pixelSampleUV, shiftedNeighborReservoir.lensSampleUV, true, previousCameraData );
                            float offsetTargetPdf = offsetEvalContext.evalTargetFunction( prevNeighborLightSample, lights, true, kUnbiased, sg, shiftedNeighborReservoir.pathSample);
                            misWeight += offsetReservoir.M * offsetTargetPdf;
                        }
                    }
                    misWeight = misWeight > 0.0f ? prevNeighborReservoir.M * pHatSource / misWeight : 0.0f;

                    print("misWeight = ", misWeight);
                    print("pHatTarget = ", pHatTarget);

                    // Start resampling
                    if (streamingResampleStepMis(reprojectedRisState, shiftedNeighborReservoir, sg, bilinearWeights[i], misWeight))
                    {
                        selectedIdx = targetPrevPixelIndex;
                        selectedPrevNeighborContext = shiftedNeighborContext;
                        selectedPrevPixel = targetPrevPixel;
                    }
                }
            }
            // Only anti-aliasing or both enabled
            else
            {
                // Check if the source domain sample in range
                bool isSourceSampleInRange = sampleInTargetRange( prevNeighborOffsets[i], prevNeighborReservoir.pixelSampleUV, reprojectedMinPointFrac, filterAlpha > 0.f ? filterRadius : 0.5); //areaScaler/ 2.f

                // TODO: if source sample in the range, for MIS shift, do we still need to do two passes? -> currently NO!
                // numPassesForFirstRIS = isSourceSampleInRange ? 1u : numPassesForFirstRIS;

                // Currently, we have 3 different ways to do the resampling, pick one of them
                if (temporalReuseMode == uint(TemporalReuseMode::FractionalMotion_Shifting2RIS))
                {
                    print("confidenceWeightScaler = ", confidenceWeightScaler);
                    float dummyPHat;
                    float dummyJacobian;
                    float2 dummyLensUV;
                    EvalContext dummyEvalContext = {};
                    JacobianEvalData dummyData = {};

                    if (optimizeShift2RIS)
                    {
                        // uint pass = 1;
                        // confidenceWeightScaler = 0.0f;

                        for (uint pass = 0; pass < numPassesForFirstRIS; pass++)
                        {
                            confidenceWeightScaler = pass == 0 ? confidenceWeightScaler : 1.0 - confidenceWeightScaler;
                            if (confidenceWeightScaler == 0.0f) continue;

                            uint selectedShiftMapping = findShiftMapping( shiftMappingModeRIS1, frameDim, prevNeighborContext.pos, previousCameraData, pass);

                            // Each center pixel has primary ray info of 8 surrounding pixel's sample
                            uint pHatMemoryOffset = shiftsPerPixel * getPixelIndex(prevNeighborPixel);
                            uint jacobianMemoryOffset = 8u * getPixelIndex(prevNeighborPixel);
                            float pHatSource = prevNeighborReservoir.targetPdf;
                            float pHatTarget = pHatSource;
                            float jacobianSourceToTarget = 1.0f;
                            EvalContext targetEvalContext = prevNeighborContext;
                            Reservoir targetReservoir = prevNeighborReservoir;
                            JacobianEvalData jEvalDataTarget = prevNeighborJacoEvalData;
                            float3 targetNormal = targetEvalContext.N;
                            PackedHitInfo targetPackedHit = targetEvalContext.hit.getData();

                            // uv is in ([0,2),[0,2)) range. Shift outside sample to target pixel
                            int2 targetPixel = prevNeighborOffsets[i]; // in [0, 2] range
                            float2 uv = targetPixel + prevNeighborReservoir.pixelSampleUV;
                            if (uv.y < reprojectedMinPointFrac.y) targetPixel.y += 1;
                            if (uv.y > 1 + reprojectedMinPointFrac.y) targetPixel.y -= 1;
                            if (uv.x < reprojectedMinPointFrac.x) targetPixel.x += 1;
                            if (uv.x > 1 + reprojectedMinPointFrac.x) targetPixel.x -= 1;
                            uint2 targetPixelAbsoluteInt = reprojectedMinPointInt + targetPixel;
                            // print("targetPixelAbsoluteInt = ", targetPixelAbsoluteInt);

                            // 0,0  1,0  2,0
                            // 1,0       2,1
                            // 2,0  2,1  2,2

                            // fetch shifted pHat at target pixel
                            int2 temp = targetPixel - int2(prevNeighborOffsets[i]) + int2(1, 1); // 8  different possibilites (or 16 for MIS)
                            int targetMemoryOffset = 3 * temp.y + temp.x;                        // target pixel relative to base pixel
                            if (targetMemoryOffset != 4)                                         // not "center" -> has shift
                            {
                                targetMemoryOffset = targetMemoryOffset < 4 ? targetMemoryOffset : targetMemoryOffset - 1;

                                // for MIS shift mode, pHat has 16 possibilites. If choose reconnection shift (pass = 1), offset into second part
                                int pHatTargetMemoryOffsetInPixel = targetMemoryOffset + 8 * pass;

                                // fetch pHat from memory
                                pHatTarget = temporalMISPDFs[pHatMemoryOffset + pHatTargetMemoryOffsetInPixel];
                                targetReservoir.targetPdf = pHatTarget;

                                // fetch primary hit
                                targetPackedHit = temporalMISPrimaryHits[jacobianMemoryOffset + targetMemoryOffset];
                                targetNormal = temporalMISPrimHitNormals[jacobianMemoryOffset + targetMemoryOffset];

                                // if (selectedShiftMapping == uint(ShiftMapping::RandomReplay))
                                //     targetEvalContext = temporalMISEvalContexts[pHatMemoryOffset + pHatTargetMemoryOffsetInPixel];

                                print("pHatTarget = ", pHatTarget);

                                if (pHatTarget > 0.0f && selectedShiftMapping == uint(ShiftMapping::PrimaryHitReconnection))
                                {
                                    jEvalDataTarget = temporalMISJacobianData[jacobianMemoryOffset + targetMemoryOffset];
                                    targetEvalContext.V = -jEvalDataTarget.rayDir;
                                    targetReservoir.lensSampleUV = jEvalDataTarget.lensUV;
                                    jacobianSourceToTarget = evalJacobianForReconnection(prevNeighborJacoEvalData, prevCameraNormalW, prevRcNormalW, jEvalDataTarget, prevCameraNormalW, prevRcNormalW);
                                }

                            }

                            // sum up MIS weights
                            float misWeight = 0.0f;
                            if (pHatTarget > 0.0f)
                            {
                                float pHatSum = bilinearWeights[i] * prevNeighborReservoir.M * pHatSource * (1.0f / jacobianSourceToTarget);
                                for (uint j = 0; j < prevNeighborCount; j++)
                                {
                                    uint2 offsetPixel = reprojectedMinPointInt + prevNeighborOffsets[j];
                                    Reservoir offsetReservoir = prevReservoirs[getPixelIndex(offsetPixel)];

                                    int2 diff = int2(prevNeighborOffsets[j]) - int2(prevNeighborOffsets[i]); // diffence to center
                                    if (diff.x == 0 && diff.y == 0) continue; // at center (source)
                                    int2 temp = diff + int2(1, 1);
                                    int otherMemoryOffset = 3 * temp.y + temp.x;
                                    otherMemoryOffset = otherMemoryOffset < 4 ? otherMemoryOffset : otherMemoryOffset - 1;

                                    int pHatOtherMemoryOffsetInPixel = otherMemoryOffset + 8 * pass;
                                    float pHatOther = temporalMISPDFs[pHatMemoryOffset + pHatOtherMemoryOffsetInPixel];

                                    // Compute jacobian from target to other (two) pixels (we need primary hit)
                                    float jacobianTargetToOther = 1.0f;

                                    if (selectedShiftMapping == uint(ShiftMapping::PrimaryHitReconnection))
                                    {
                                        if (any(offsetPixel != targetPixelAbsoluteInt) && pHatOther > 0.0f)
                                        {
                                            JacobianEvalData jEvalDataOther = temporalMISJacobianData[jacobianMemoryOffset + otherMemoryOffset];
                                            jacobianTargetToOther = evalJacobianForReconnection(jEvalDataTarget, prevCameraNormalW, prevRcNormalW, jEvalDataOther, prevCameraNormalW, prevRcNormalW);
                                        }
                                    }

                                    // fetch pHat from memory
                                    pHatSum += bilinearWeights[j] * offsetReservoir.M * pHatOther * jacobianTargetToOther;
                                }

                                // fetch MIS weight
                                float misWeightNum = bilinearWeights[i] * prevNeighborReservoir.M * pHatSource * (1.0f / jacobianSourceToTarget) * confidenceWeightScaler;
                                misWeight = pHatSum > 0.0f ? misWeightNum / pHatSum : 0.0f;
                            }

                            // Start resampling
                            if (streamingResampleStepMis( reprojectedRisState, targetReservoir, sg, bilinearWeights[i] * confidenceWeightScaler, misWeight, jacobianSourceToTarget ))
                            {
                                selectedIdx = 2 * targetPixel.y + targetPixel.x;
                                selectedPrevPixel = targetPixelAbsoluteInt;
                                selectedPrevNeighborContext = targetEvalContext;
                                selectedPrevPackedHit = targetPackedHit;
                                selectedPrevNormal = targetNormal;
                                prevSampleSelectedShiftMapping = int(selectedShiftMapping);
                            }
                        }
                    }
                    // Unoptimized part
                    else
                    {
                        for (uint pass = 0; pass < numPassesForFirstRIS; pass++)
                        {
                            confidenceWeightScaler = pass == 0 ? confidenceWeightScaler : 1.0 - confidenceWeightScaler;
                            if (confidenceWeightScaler == 0.0f) continue;

                            // Find the selected shift mapping first
                            uint selectedShiftMapping = findShiftMapping(shiftMappingModeRIS1, frameDim, prevNeighborContext.pos, previousCameraData, pass);

                            // Shift sample to other pixels if not in the reprojected square
                            float pHatSource = prevNeighborReservoir.targetPdf;
                            uint2 shiftedNeighborPixel = prevNeighborPixel;
                            uint shiftedNeighborPixelIndex = i;

                            float pHatTarget = 0;
                            float jacobianSourceToTarget = 1.0f;
                            float2 shiftedLensSampleUV = float2(0);
                            EvalContext shiftedNeighborContext = prevNeighborContext;
                            Reservoir shiftedNeighborReservoir = prevNeighborReservoir;

                            // Get the shifted sample in target domain
                            if (!isSourceSampleInRange)
                            {
                                for (uint j = 0; j < prevNeighborCount; j++)
                                {
                                    // Only one pixel will pass the check
                                    if (sampleInTargetRange(prevNeighborOffsets[j], prevNeighborReservoir.pixelSampleUV, reprojectedMinPointFrac))
                                    {
                                        // Store the shifted target pixel
                                        shiftedNeighborPixelIndex = j;
                                        shiftedNeighborPixel = reprojectedMinPointInt + prevNeighborOffsets[j];

                                        // Evaluate new pHat, jacobian, and samples at shifted path (note, invalid shift pHat = 0)
                                        evalPhatAndJacobian( selectedShiftMapping, frameDim, prevNeighborContext, prevNeighborReservoir, lights, kUnbiased, prevNeighborPixel, previousCameraData,
                                            shiftedNeighborPixel, previousCameraData, true, sg, pHatTarget, jacobianSourceToTarget, shiftedLensSampleUV, shiftedNeighborContext );

                                        shiftedNeighborReservoir.lensSampleUV = shiftedLensSampleUV;
                                        shiftedNeighborReservoir.targetPdf = pHatTarget;

                                        break;
                                    }
                                }
                            }
                            else
                            {
                                pHatTarget = pHatSource;
                            }

                            // Now, the base path should be shifted pixel.
                            selectedShiftMapping = findShiftMapping(shiftMappingModeRIS1, frameDim, shiftedNeighborContext.pos, previousCameraData, pass);

                            // Compute balanced heurstic MIS weight (target domain sample shift to all possible source domains)
                            //  -> if pHatTarget = 0, no need to compute this anymore
                            float misWeight = 0;
                            if (pHatTarget > 0.0f)
                            {
                                for (uint j = 0; j < prevNeighborCount; j++)
                                {
                                    // Load offset reservoir data
                                    uint2 offsetPixel = reprojectedMinPointInt + prevNeighborOffsets[j];
                                    Reservoir offsetReservoir = prevReservoirs[getPixelIndex(offsetPixel)];
                                    offsetReservoir.M = min(offsetReservoir.M, currentReservoir.M * kMaxHistoryLength);

                                    // Current iteration pixel is the target domain -> pHat already computed before
                                    if (all(offsetPixel == shiftedNeighborPixel))
                                    {
                                        misWeight += bilinearWeights[shiftedNeighborPixelIndex] * offsetReservoir.M * pHatTarget;
                                    }
                                    // Current iteration pixel is the source domain -> pHat already know
                                    else if (all(offsetPixel == prevNeighborPixel))
                                    {
                                        misWeight += bilinearWeights[i] * prevNeighborReservoir.M * pHatSource * (1.0f / jacobianSourceToTarget);
                                    }
                                    // The rest two pixels
                                    else
                                    {
                                        float pHatOtherSource = 0.0f;
                                        float jacobianTargetToSource = 1.0f;
                                        float2 dummyLensSampleUV = float2(0);
                                        EvalContext dummyEvalContext = {};

                                        // Evaluate pHat, jacobian, and samples at source domain
                                        uint inverseShiftMapping = selectedShiftMapping;
                                        evalPhatAndJacobian( inverseShiftMapping, frameDim, shiftedNeighborContext, shiftedNeighborReservoir, lights, kUnbiased, shiftedNeighborPixel,
                                            previousCameraData, offsetPixel, previousCameraData, true, sg, pHatOtherSource, jacobianTargetToSource, dummyLensSampleUV, dummyEvalContext );

                                        misWeight += bilinearWeights[j] * offsetReservoir.M * pHatOtherSource * jacobianTargetToSource;
                                    }
                                }

                                // Finalize MIS weight
                                float misWeightNum = bilinearWeights[i] * prevNeighborReservoir.M * pHatSource * (1.0f / jacobianSourceToTarget) * confidenceWeightScaler;
                                misWeight = misWeight > 0.0f ? misWeightNum / misWeight : 0.0f;
                            }

                            float bilinearWeight = bilinearWeights[i] * confidenceWeightScaler;

                            // Start resampling
                            if (streamingResampleStepMis(reprojectedRisState, shiftedNeighborReservoir, sg, bilinearWeight, misWeight, jacobianSourceToTarget))
                            {
                                selectedIdx = shiftedNeighborPixelIndex;
                                selectedPrevNeighborContext = shiftedNeighborContext;
                                selectedPrevPixel = shiftedNeighborPixel;
                            }
                        }
                    }
                }
                else if (temporalReuseMode == uint(TemporalReuseMode::FractionalMotion_Rejection2RIS))
                {
                    // Set outside sample pHat to 0
                    prevNeighborReservoir.targetPdf = isSourceSampleInRange ? prevNeighborReservoir.targetPdf : 0.0f;

                    float misWeight = 1.f;

                    if (filterAlpha > 0.f)
                    {
                        float2 pixelSamplePos = prevNeighborOffsets[i] - float2(1) + prevNeighborReservoir.pixelSampleUV;

                        float oldH = 0.f;
                        float sumH = 0.f;
                        // the four coords are
                        for (uint j = 0; j < prevNeighborCount; j++)
                        {
                            if (!isValidPixel(reprojectedMinPointInt + prevNeighborOffsets[j])) continue;
                            float2 squareCenterCoord = float2(prevNeighborOffsets[j]) - 0.5f;
                            // compute the local coords in all four neighbors (relative to pixel center)
                            float2 p = computeSquareLocalCoord(pixelSamplePos, squareCenterCoord, filterRadius);
                            if (p.x != 1e10f)
                            {
                                float h = gScene.camera.getFilterWeight(p + 0.5, filterRadius, filterAlpha, filterNorm);
                                sumH += h * bilinearWeights[j];
                                if (i == j) oldH = h;
                            }
                        }

                        // new coords in the fractional pixel (relative to pixel center)

                        float2 p = computeSquareLocalCoord(pixelSamplePos, reprojectedMinPointFrac - float2(0.5), filterRadius);
                        float newH = gScene.camera.getFilterWeight(p + 0.5, filterRadius, filterAlpha, filterNorm);

                        // old coords

                        // compute MIS weights (contains newH in pHat which we don't want to include in pHat after RIS (for automatic cancellation later))
                        misWeight = (sumH < 1e-6f || newH < 1e-6f) ? 0.f : bilinearWeights[i] * newH / sumH; // oldH / sumH * newH / oldH;
                    }

                    // if (streamingResampleStepMis( reprojectedRisState, prevNeighborReservoir, sg, bilinearWeights[i], 1.0f))
                    if (streamingResampleStepMis(reprojectedRisState, prevNeighborReservoir, sg, bilinearWeights[i], misWeight))
                    {
                        selectedIdx = i;
                        selectedPrevNeighborContext = prevNeighborContext;
                        selectedPrevPixel = prevNeighborPixel;
                    }
                }
            }
        }


        // Cap the confidence weight of final reservoir
        reprojectedRisState.M = min(reprojectedRisState.M, currentReservoir.M * kMaxHistoryLength);

        // Since subpixel uv sample is relative to the pixel min point, we also need to shift this to the reprojected pixel's domain
        if (selectedIdx >= 0 && !onlyReuseDoF)
            reprojectedRisState.pixelSampleUV = (selectedPrevPixel - reprojectedMinPointInt) + reprojectedRisState.pixelSampleUV - reprojectedMinPointFrac;

        // Ad-hoc fix for 1-3 pixels will go out of range. TODO: use better way
        // reprojectedRisState.pixelSampleUV = clamp(reprojectedRisState.pixelSampleUV, 0.0, 0.999999);
        reprojectedRisState.pixelSampleUV = clamp(reprojectedRisState.pixelSampleUV, -filterRadius + 0.5, filterRadius + 0.499999);

        // Finalize unbiased contribution weight
        reprojectedRisState.weight = reprojectedRisState.targetPdf > 0.f ? reprojectedRisState.weightSum / reprojectedRisState.targetPdf : 0.f;

        assert( all(reprojectedRisState.pixelSampleUV >= 0.0f) && all(reprojectedRisState.pixelSampleUV < 1.0f), "reprojectedRisState.pixelSampleUV not in valid range" );

        // RIS step 2: resample the result previous sample with current
        EvalContext selectedSampleAtCurrContext = currentContext;
        RisState risState = RisState::createEmpty(currentReservoir);
        {
            Reservoir resampledPrevReservoir = reprojectedRisState.toReservoir();
            float confidenceWeightSum = resampledPrevReservoir.M + currentReservoir.M;
            uint prevPixelIdxFixed = getPixelIndex(reprojectedMinPointInt + prevNeighborOffsets[selectedIdx]);

            resampledPrevReservoir.printReservoir();

            // If temporal reuse mode is bilinear, we cannot use reprojected range, instead we need to use the selected pixel as previous domain
            reprojectedMinPointInt = onlyReuseDoF ? selectedPrevPixel : reprojectedMinPointInt;
            reprojectedMinPointFrac = onlyReuseDoF ? float2(0) : reprojectedMinPointFrac;

            // In optimization mode, if the first RIS step selects the sample produced by random replay, we need to use stored surface hit to recreate context.
            // Otherwise, retracing the primary hit will result bias that blows up the render.
            bool ris2HasReconnectionShift = shiftMappingModeRIS2 == uint(ShiftMappingModeInReusing::MIS) || shiftMappingModeRIS2 == uint(ShiftMappingModeInReusing::OnlyReconnection);
            if (optimizeShift2RIS && ris2HasReconnectionShift && prevSampleSelectedShiftMapping == uint(ShiftMapping::RandomReplay))
            {
                Ray ray = computePrevRayThinlensSubPixelRandom( reprojectedMinPointInt, frameDim, resampledPrevReservoir.pixelSampleUV + reprojectedMinPointFrac,
                    resampledPrevReservoir.lensSampleUV, previousCameraData );
                selectedPrevNeighborContext = EvalContext::createFromHit(HitInfo(selectedPrevPackedHit), ray.origin, ray.dir, true);

                // Even we use the same hit info to load shading data, the normal may also be different (why?), so we have to also store the normal
                selectedPrevNeighborContext.N = selectedPrevNormal;
                selectedPrevNeighborContext.NdotV = dot(selectedPrevNeighborContext.V, selectedPrevNeighborContext.N);
            }

            // Pairwise resampling previous (candidate) sample.
            // do we need to discard special case when selectedIdx = -1? -> NO!
            // resampleAreaReservoirPairwiseMIS( risState, currentReservoir, currentContext, resampledPrevReservoir, selectedPrevNeighborContext, prevPixelCenterEvalContext[prevPixelIdxFixed],
            //     frameDim, pixel, reprojectedMinPointInt + reprojectedMinPointFrac, lights, useMFactor, confidenceWeightSum, 1.0f, kUsePrevFrameSceneData, true, shiftMappingModeRIS2, sg,
            //     selectedSampleAtCurrContext, kUnbiased, previousCameraData );

            resampleAreaReservoirPairwiseMIS( risState, currentReservoir, currentContext, resampledPrevReservoir, selectedPrevNeighborContext, pixelCenterEvalContext[pixelIndex],
                frameDim, pixel, reprojectedMinPointInt + reprojectedMinPointFrac, lights, useMFactor, confidenceWeightSum, 1.0f, kUsePrevFrameSceneData, true, shiftMappingModeRIS2, sg,
                selectedSampleAtCurrContext, kUnbiased, previousCameraData );

            // Start current (canonical) sample resampling
            if (streamingResampleFinalizeMis(risState, currentReservoir, currentReservoir.targetPdf, sg))
                selectedSampleAtCurrContext = currentContext;
        }

        // Finalize unbiased contribution weight
        risState.weight = risState.targetPdf > 0.f ? risState.weightSum / risState.targetPdf : 0.f;
        risState.M = min(risState.M, currentReservoir.M * kMaxHistoryLength);

        print("risState.weight = ", risState.weight);

        // Save reservoir.
        reservoirs[pixelIndex] = risState.toReservoir();

        // Store primary hit surface data
        resEvalContext[pixelIndex] = selectedSampleAtCurrContext;
    }
};

cbuffer CB
{
    uint resampleResourceType;
    TemporalResampling gTemporalResampling;
}

[numthreads(16, 4, 1)]
void main(uint3 dispatchThreadId : SV_DispatchThreadID)
{
    gTemporalResampling.process_2RIS(dispatchThreadId.xy);
}
