/***************************************************************************
 # Copyright (c) 2015-22, NVIDIA CORPORATION. All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/

/** Dummy compute program for reflection of the structured buffer types.

    Falcor requires reflection data to be able to create a structured buffer,
    but the programs are not yet ready to be used when we create the buffers.
    Therefore we create this dummy program to make it easier.
 */

import SurfaceData;
import LocalEmissiveTriangle;
import FinalSample;
import Lights;
import Reservoir;

import EvalContext;
import PixelAreaSampleData;

#define USE_PACKED_DATA 0

StructuredBuffer<PackedLocalEmissiveTriangle> emissiveTriangles;
StructuredBuffer<PackedHitInfo> finalPrimaryHits;
#if USE_PACKED_DATA
StructuredBuffer<PackedSurfaceData> surfaceData;
StructuredBuffer<PackedFinalSample> finalSamples;
StructuredBuffer<PackedLoadedLightSample> lightTileData;
StructuredBuffer<PackedReservoir> reservoirs;
#else
StructuredBuffer<SurfaceData> surfaceData;
StructuredBuffer<FinalSample> finalSamples;
StructuredBuffer<LoadedLightSample> lightTileData;
StructuredBuffer<Reservoir> reservoirs;
#endif

StructuredBuffer<float> temporalMISPDFs;
StructuredBuffer<float3> temporalMISViewDirs;
// StructuredBuffer<float2> temporalMISLensUV;
StructuredBuffer<JacobianEvalData> temporalMISJacobianData;
StructuredBuffer<float3> temporalMISPrimHitNormals;
StructuredBuffer<PackedHitInfo> temporalMISPrimaryHits;

StructuredBuffer<EvalContext> resEvalContext;
StructuredBuffer<CameraData> prevCameraData;

void main() {}
