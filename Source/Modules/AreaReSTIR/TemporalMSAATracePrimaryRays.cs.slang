/***************************************************************************
 # Copyright (c) 2015-22, NVIDIA CORPORATION. All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/
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

struct Pass
{
    // Static configuration.
    static const bool kUnbiased = UNBIASED;
    static const int2 kPrevNeighborOffsets[8] = {
        int2(-1, -1), int2(0, -1), int2(1, -1), int2(-1, 0), int2(1, 0),   int2(-1, 1), int2(0, 1),  int2(1, 1)
    };

    StructuredBuffer<Reservoir> prevReservoirs;
    StructuredBuffer<EvalContext> prevResEvalContext;
    StructuredBuffer<CameraData> prevCameraData;
    RWStructuredBuffer<float> temporalMISPDFs;
    RWStructuredBuffer<EvalContext> temporalMISEvalContexts;
    RWStructuredBuffer<JacobianEvalData> temporalMISJacobianData;
    RWStructuredBuffer<float3> temporalMISViewDirs;
    RWStructuredBuffer<float3> temporalMISPrimHitNormals;
    RWStructuredBuffer<PackedHitInfo> temporalMISPrimaryHits;
    Lights lights;

    uint2 frameDim;
    uint frameIndex;
    uint shiftMappingMode;
    uint shiftsPerPixel;
    uint restirPassIdx;

    // -----------------------------------------
    // Helper funtions
    // -----------------------------------------
    uint getPixelIndex(uint2 pixel) { return pixel.y * frameDim.x + pixel.x; }

    // -----------------------------------------
    // Temporal Resampling functions
    // -----------------------------------------

    // For random replay shift, now we trace 1 primary ray + 1 secondary shadow ray
    // For reconnection shift, now we trace 1 primary shadow ray + 1 secondary shadow ray
    void doWork(uint2 pixel, uint shiftID)
    {
        if (any(pixel >= frameDim)) return;

        // printSetPixel(pixel);
        print("temporal MSAA Trace Primary Rays ---------------------", 1);

        const uint pixelIndex = getPixelIndex(pixel);

        // Compute pixel index in reservoir buffer
        uint pixelIdxInReservoirBuffer = getPixelIndex(frameDim, pixel, restirPassIdx);

        uint sampleNum = frameIndex + 8 * restirPassIdx;
        TinyUniformSampleGenerator sg = TinyUniformSampleGenerator(pixel, sampleNum);

        // map thread to primary rays

        // fetching lens and subpixel sample
        Reservoir prevReservoir = prevReservoirs[pixelIdxInReservoirBuffer];
        EvalContext prevContext = prevResEvalContext[pixelIdxInReservoirBuffer];

        int2 offset2D = kPrevNeighborOffsets[shiftID];
        CameraData previousCameraData = prevCameraData[0];

        // Values to evaluate
        float pHatOffset = 0.0f;
        float offsetJacobian = 1.0f;
        float2 offsetLensSampleUV = float2(0);
        EvalContext offsetEvalContext = {};
        JacobianEvalData jacobianEvalData = {};


        if (shiftMappingMode == uint(ShiftMappingModeInReusing::OnlyRandomReplay))
        {
            // Shift center pixel's sample to neighbor pixel
            evalPhatAndJacobianForRandomReplay( frameDim, prevReservoir, prevContext, lights, kUnbiased, int2(pixel) + offset2D, previousCameraData, true, sg, pHatOffset, offsetJacobian,
                offsetLensSampleUV, offsetEvalContext );
            temporalMISPDFs[shiftsPerPixel * pixelIdxInReservoirBuffer + shiftID] = pHatOffset;
            temporalMISPrimaryHits[shiftsPerPixel * pixelIdxInReservoirBuffer + shiftID] = offsetEvalContext.hit.getData();
            temporalMISViewDirs[shiftsPerPixel * pixelIdxInReservoirBuffer + shiftID] = offsetEvalContext.V;
            temporalMISPrimHitNormals[shiftsPerPixel * pixelIdxInReservoirBuffer + shiftID] = offsetEvalContext.N;
        }
        else if (shiftMappingMode == uint(ShiftMappingModeInReusing::OnlyReconnection))
        {
            // Note, reconnection shift's jacobian is not equal to 1, and its evaluation is postpone to later temporal reusing pass
            evalPhatAndJacobianForReconnection( frameDim, prevContext, prevReservoir, lights, kUnbiased, pixel, previousCameraData, int2(pixel) + offset2D, previousCameraData, true,
                sg, pHatOffset, offsetJacobian, offsetLensSampleUV, offsetEvalContext, jacobianEvalData );
            temporalMISPDFs[shiftsPerPixel * pixelIdxInReservoirBuffer + shiftID] = pHatOffset;
            temporalMISJacobianData[shiftsPerPixel * pixelIdxInReservoirBuffer + shiftID] = jacobianEvalData;
        }
        else
        {
            // 0 -> random replay, 1 -> reconnection
            // Memory: 0000 0000 1111 1111
            uint shiftsPerPixelHalf = shiftsPerPixel / 2u;

            // pHat for random replay shift
            evalPhatAndJacobianForRandomReplay( frameDim, prevReservoir, prevContext, lights, kUnbiased, int2(pixel) + offset2D, previousCameraData, true, sg, pHatOffset, offsetJacobian,
                offsetLensSampleUV, offsetEvalContext );
            temporalMISPDFs[shiftsPerPixel * pixelIdxInReservoirBuffer + shiftID] = pHatOffset;
            temporalMISPrimaryHits[shiftsPerPixelHalf * pixelIdxInReservoirBuffer + shiftID] = offsetEvalContext.hit.getData();
            temporalMISPrimHitNormals[shiftsPerPixelHalf * pixelIdxInReservoirBuffer + shiftID] = offsetEvalContext.N;

            // pHat for reconnection shift
            pHatOffset = 0;
            evalPhatAndJacobianForReconnection( frameDim, prevContext, prevReservoir, lights, kUnbiased, pixel, previousCameraData, int2(pixel) + offset2D, previousCameraData, true,
                sg, pHatOffset, offsetJacobian, offsetLensSampleUV, offsetEvalContext, jacobianEvalData );
            temporalMISPDFs[shiftsPerPixel * pixelIdxInReservoirBuffer + shiftID + shiftsPerPixelHalf] = pHatOffset;
            temporalMISJacobianData[shiftsPerPixelHalf * pixelIdxInReservoirBuffer + shiftID] = jacobianEvalData;
        }
    }
};

cbuffer CB
{
    uint resampleResourceType;
    Pass gPass;
}

// [numthreads(8, 8, SHIFTS_PER_PIXEL)]
[numthreads(8, 8, 8)]
void main(uint3 dispatchThreadId : SV_DispatchThreadID)
{
    gPass.doWork(dispatchThreadId.xy, dispatchThreadId.z);
}
