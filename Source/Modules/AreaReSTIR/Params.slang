/***************************************************************************
 # Copyright (c) 2015-22, NVIDIA CORPORATION. All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/
#pragma once
#include "Utils/HostDeviceShared.slangh"

BEGIN_NAMESPACE_FALCOR

/** Enumeration of available debug outputs.
    Note: Keep in sync with definition in Params.slang
*/
enum class DebugOutput : uint32_t
{
    Disabled = 0,
    Position = 1,
    Depth = 2,
    Normal = 3,
    FaceNormal = 4,
    DiffuseWeight = 5,
    SpecularWeight = 6,
    SpecularRoughness = 7,
    PackedNormal = 8,
    PackedDepth = 9,
    InitialWeight = 10,
    TemporalReuse = 11,
    SpatialReuse = 12,
    FinalSampleDir = 13,
    FinalSampleDistance = 14,
    FinalSampleLi = 15,
};
FALCOR_ENUM_INFO(
    DebugOutput,
    {
        {DebugOutput::Disabled, "Disabled"},
        {DebugOutput::Position, "Position"},
        {DebugOutput::Depth, "Depth"},
        {DebugOutput::Normal, "Normal"},
        {DebugOutput::FaceNormal, "FaceNormal"},
        {DebugOutput::DiffuseWeight, "DiffuseWeight"},
        {DebugOutput::SpecularWeight, "SpecularWeight"},
        {DebugOutput::PackedNormal, "PackedNormal"},
        {DebugOutput::PackedDepth, "PackedDepth"},
        {DebugOutput::InitialWeight, "InitialWeight"},
        {DebugOutput::TemporalReuse, "TemporalReuse"},
        {DebugOutput::SpatialReuse, "SpatialReuse"},
        {DebugOutput::FinalSampleDir, "FinalSampleDir"},
        {DebugOutput::FinalSampleDistance, "FinalSampleDistance"},
        {DebugOutput::FinalSampleLi, "FinalSampleLi"},
    }
);
FALCOR_ENUM_REGISTER(DebugOutput);

// Currently unused
enum class ReSTIRMode : uint32_t
{
    InputOnly = 0,
    TemporalOnly = 1,
    TemporalAndBiasedSpatial = 2,
    TemporalAndUnbiasedSpatial = 3
};
FALCOR_ENUM_INFO(
    ReSTIRMode,
    {
        {ReSTIRMode::InputOnly, "InputOnly"},
        {ReSTIRMode::TemporalOnly, "TemporalOnly"},
        {ReSTIRMode::TemporalAndBiasedSpatial, "TemporalAndBiasedSpatial"},
        {ReSTIRMode::TemporalAndUnbiasedSpatial, "TemporalAndUnbiasedSpatial"},
    }
);
FALCOR_ENUM_REGISTER(ReSTIRMode);

// Currently unused
enum class TargetPDF : uint32_t
{
    IncomingRadiance = 0,
    OutgoingRadiance = 1
};
FALCOR_ENUM_INFO(
    TargetPDF,
    {
        {TargetPDF::IncomingRadiance, "IncomingRadiance"},
        {TargetPDF::OutgoingRadiance, "OutgoingRadiance"},
    }
);
FALCOR_ENUM_REGISTER(TargetPDF);

enum class TemporalReuseMode : uint32_t
{
    IntegerMotion = 0,
    FractionalMotion_Rejection2RIS = 1,
    FractionalMotion_Shifting2RIS = 2,
};
FALCOR_ENUM_INFO(
    TemporalReuseMode,
    {
        { TemporalReuseMode::IntegerMotion, "IntegerMotion (Origin)" },
        { TemporalReuseMode::FractionalMotion_Rejection2RIS, "FractionalMotion_Rejection2RIS" },
        { TemporalReuseMode::FractionalMotion_Shifting2RIS, "FractionalMotion_Shifting2RIS"},
    }
);
FALCOR_ENUM_REGISTER(TemporalReuseMode);

enum class ResamplingResourceType : uint32_t
{
   DirectLighting = 0,
   Emission = 1,
};
FALCOR_ENUM_INFO(
   ResamplingResourceType,
   {
       {ResamplingResourceType::DirectLighting, "DirectLighting"},
       {ResamplingResourceType::Emission, "Emission"},
   }
)
FALCOR_ENUM_REGISTER(ResamplingResourceType);

enum class ResampleEmissionMode : uint32_t
{
    None = 0,
    OneMergedReservoir = 1,
};
FALCOR_ENUM_INFO(
    ResampleEmissionMode,
    {
        {ResampleEmissionMode::None, "None"},
        {ResampleEmissionMode::OneMergedReservoir, "OneMergedReservoir"},
    }
);
FALCOR_ENUM_REGISTER(ResampleEmissionMode);

enum class ShiftMapping : uint32_t
{
    RandomReplay = 0, // -> this is lens vertex shift in the paper
    PrimaryHitReconnection = 1,
};
FALCOR_ENUM_INFO(
    ShiftMapping,
    {
        { ShiftMapping::RandomReplay, "RandomReplay" },
        { ShiftMapping::PrimaryHitReconnection, "PrimaryHitReconnection" },
    }
);
FALCOR_ENUM_REGISTER(ShiftMapping);

enum class ShiftMappingModeInReusing
{
    OnlyRandomReplay = 0,
    OnlyReconnection = 1,
    MIS = 2,
};
FALCOR_ENUM_INFO(
    ShiftMappingModeInReusing,
    {
        { ShiftMappingModeInReusing::OnlyRandomReplay, "OnlyRandomReplay" },
        { ShiftMappingModeInReusing::OnlyReconnection, "OnlyReconnection" },
        { ShiftMappingModeInReusing::MIS, "MIS" },
    }
);
FALCOR_ENUM_REGISTER(ShiftMappingModeInReusing);

enum class SpatialSampleSelectionInMisShift
{
    DuplicateSample = 0,
    SelectUniqueSample_FixedWeight = 1,
    SelectUniqueSample_CenterPixelCoC = 2,
    SelectUniqueSample_NeighborPixelCoC = 3,
};
FALCOR_ENUM_INFO(
    SpatialSampleSelectionInMisShift,
    {
        { SpatialSampleSelectionInMisShift::DuplicateSample, "DuplicateSample" },
        { SpatialSampleSelectionInMisShift::SelectUniqueSample_FixedWeight, "SelectUniqueSample_FixedWeight" },
        { SpatialSampleSelectionInMisShift::SelectUniqueSample_CenterPixelCoC, "SelectUniqueSample_CenterPixelCoC" },
        { SpatialSampleSelectionInMisShift::SelectUniqueSample_NeighborPixelCoC, "SelectUniqueSample_NeighborPixelCoC" },
    }
);
FALCOR_ENUM_REGISTER(SpatialSampleSelectionInMisShift);

// Import static specialization constants.
#ifndef HOST_CODE
__exported import StaticParams;
#endif

END_NAMESPACE_FALCOR
