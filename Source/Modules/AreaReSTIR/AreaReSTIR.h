/***************************************************************************
 # Copyright (c) 2015-23, NVIDIA CORPORATION. All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/
#pragma once
#include "Utils/Sampling/AliasTable.h"
#include "Utils/Debug/PixelDebug.h"
#include "Utils/Scripting/ScriptBindings.h"
#include "Utils/StringUtils.h"
#include "Scene/Scene.h"
#include "Scene/Lights/LightCollection.h"
#include "Scene/Lights/Light.h"
#include "../RenderPasses/GBuffer/VBuffer/VBufferRT.h"

#include "Params.slang"

#include <cmath>
#include <memory>
#include <random>
#include <tuple>
#include <vector>

namespace Falcor
{
    /** Implementation of Area ReSTIR for direct illumination.

        The direct illumination part (ReSTIR DI) is based on
        "Spatiotemporal reservoir resampling for real-time ray tracing with dynamic direct lighting"
        by Benedikt Bitterli et al. from 2020.

        Integrating this module into a renderer requires a few steps:

        - Host:   Call AreaReSTIR::beginFrame() on to begin a new frame.
        - Device: Populate surface data (GBuffer) using AreaReSTIR::setSurfaceData()/setInvalidSurfaceData().

        For ReSTIR DI:

        - Host:   Call AreaReSTIR::updateReSTIRDI() on to run the ReSTIR DI algorithm.
        - Device: Get final light samples using AreaReSTIR::getFinalSample() and perform shading.

        Finally at the end of frame:
        - Host:   Call AreaReSTIR::endFrame() to end the frame.
    */
    class AreaReSTIR
    {
    public:
        /** Configuration options.
        */
        struct Options
        {
            // Common Options for ReSTIR DI and GI.
            float normalThreshold = 0.5f;               ///< Normal cosine threshold for reusing temporal samples or spatial neighbor samples.
            float depthThreshold = 0.1f;                ///< Relative depth threshold for reusing temporal samples or spatial neighbor samples.

            // Options for ReSTIR DI only.
            bool useMFactor = false; // enable this will cause large variance for pairwise MIS (this is disabled in Area ReSTIR)

            // Light sampling options.
            float envLightWeight = 1.f;                 ///< Relative weight for selecting the env map when sampling a light.
            float emissiveLightWeight = 1.f;            ///< Relative weight for selecting an emissive light when sampling a light.
            float analyticLightWeight = 1.f;            ///< Relative weight for selecting an analytical light when sampling a light.

            bool useEmissiveTextureForSampling = true; ///< Use emissive texture for light sample evaluation.
            bool useEmissiveTextureForShading = true;  ///< Use emissive texture for shading.
            bool useLocalEmissiveTriangles = false;      ///< Use local emissive triangle data structure (for more efficient sampling/evaluation).

            // Light tile options.
            uint32_t lightTileCount = 128;              ///< Number of light tiles to compute.
            uint32_t lightTileSize = 1024;              ///< Number of lights per light tile.

            // Visibility options.
            bool useAlphaTest = true;                   ///< Use alpha testing on non-opaque triangles.
            bool useInitialVisibility = true;           ///< Check visibility on inital sample.
            bool useFinalVisibility = true;             ///< Check visibility on final sample.
            bool reuseFinalVisibility = false;          ///< Reuse final visibility temporally.

            // Initial resampling options.
            uint32_t screenTileSize = 8;                ///< Size of screen tile that samples from the same light tile.
            uint32_t initialLightSampleCount = 32;      ///< Number of initial light samples to resample per pixel.
            uint32_t initialBRDFSampleCount = 1;        ///< Number of initial BRDF samples to resample per pixel.
            uint32_t initialPathSampleCount = 1;        ///< Number of initial path samples to resample per pixel.
            float brdfCutoff = 0.f;                     ///< Value in range [0,1] to determine how much to shorten BRDF rays.

            // Temporal resampling options.
            bool useTemporalResampling = true;          ///< Enable temporal resampling.
            bool optimizeShift2RIS = true;
            uint32_t maxHistoryLength = 20;             ///< Maximum temporal history length.
            ShiftMappingModeInReusing temporalShiftMappingModeRIS1 = ShiftMappingModeInReusing::OnlyRandomReplay;
            ShiftMappingModeInReusing temporalShiftMappingModeRIS2 = ShiftMappingModeInReusing::OnlyRandomReplay;

            // Spatial resampling options.
            bool useSpatialResampling = true;           ///< Enable spatial resampling.
            bool rejectNeighborPixelForNormalDepth = false;
            bool rejectNeighborPixelForHitType = false;
            uint32_t spatialIterations = 1;             ///< Number of spatial resampling iterations.
            uint32_t spatialNeighborCount = 4;          ///< Number of neighbor samples to resample per pixel and iteration.
            uint32_t spatialGatherRadius = 30;          ///< Radius to gather samples from.
            ShiftMappingModeInReusing spatialShiftMappingMode = ShiftMappingModeInReusing::OnlyRandomReplay;
            float randomReplaySampleWeight = 0.5f;
            SpatialSampleSelectionInMisShift spatialMisSampleSelection = SpatialSampleSelectionInMisShift::SelectUniqueSample_FixedWeight;

            // Area ReSTIR general options
            bool reuseLensSample = false;
            bool reuseSubpixelSample = false;
            bool usePrevFrameSceneData = true;
            bool useRandomHybridThreshold = false;
            bool scaleTwoShiftsWeightForMIS = true;
            bool betterScaleFuntionForMIS = true;
            uint scalingFunctionIndex = 1;

            GBufferBase::SubPixelRandom subPixelRandom = GBufferBase::SubPixelRandom::None;
            TemporalReuseMode temporalMode = TemporalReuseMode::FractionalMotion_Shifting2RIS;
            ResampleEmissionMode resampleEmissionMode = ResampleEmissionMode::None;

            // Other general options
            bool unbiased = true;                      ///< Use unbiased version of ReSTIR by querying extra visibility rays.
            uint numReSTIRPasses = 1u;
            DebugOutput debugOutput = DebugOutput::Disabled;

            // Note: Empty constructor needed for clang due to the use of the nested struct constructor in the parent constructor.
            Options() {}

            template<typename Archive>
            void serialize(Archive& ar)
            {
                ar("normalThreshold", normalThreshold);
                ar("depthThreshold", depthThreshold);

                ar("envLightWeight", envLightWeight);
                ar("emissiveLightWeight", emissiveLightWeight);
                ar("analyticLightWeight", analyticLightWeight);

                ar("useEmissiveTextureForSampling", useEmissiveTextureForSampling);
                ar("useEmissiveTextureForShading", useEmissiveTextureForShading);
                ar("useLocalEmissiveTriangles", useLocalEmissiveTriangles);

                ar("lightTileCount", lightTileCount);
                ar("lightTileSize", lightTileSize);

                ar("useAlphaTest", useAlphaTest);
                ar("useInitialVisibility", useInitialVisibility);
                ar("useFinalVisibility", useFinalVisibility);
                ar("reuseFinalVisibility", reuseFinalVisibility);

                ar("screenTileSize", screenTileSize);
                ar("initialLightSampleCount", initialLightSampleCount);
                ar("initialBRDFSampleCount", initialBRDFSampleCount);
                ar("initialPathSampleCount", initialPathSampleCount);
                ar("brdfCutoff", brdfCutoff);

                ar("useTemporalResampling", useTemporalResampling);
                ar("maxHistoryLength", maxHistoryLength);

                ar("useSpatialResampling", useSpatialResampling);
                ar("spatialIterations", spatialIterations);
                ar("spatialNeighborCount", spatialNeighborCount);
                ar("spatialGatherRadius", spatialGatherRadius);

                ar("unbiased", unbiased);
                ar("reuseLensSample", reuseLensSample);
                ar("reuseSubpixelSample", reuseSubpixelSample);
                ar("resampleEmissionMode", resampleEmissionMode);

                ar("temporalMode", temporalMode);
                ar("temporalShiftMappingModeRIS1", temporalShiftMappingModeRIS1);
                ar("temporalShiftMappingModeRIS2", temporalShiftMappingModeRIS2);
                ar("spatialShiftMappingMode", spatialShiftMappingMode);
                ar("scaleTwoShiftsWeightForMIS", scaleTwoShiftsWeightForMIS);
                ar("optimizeShift2RIS", optimizeShift2RIS);
                ar("betterScaleFuntionForMIS", betterScaleFuntionForMIS);
                ar("numReSTIRPasses", numReSTIRPasses);
            }
        };

        static_assert(std::is_trivially_copyable<Options>() , "Options needs to be trivially copyable");

        /** Create a new instance of the ReSTIR sampler.
            \param[in] pScene Scene.
            \param[in] options Configuration options.
        */
        AreaReSTIR(const ref<Scene>& pScene, const Options& options = Options(), const DefineList& ownerDefines = DefineList());

        /** Get a list of shader defines for using the ReSTIR sampler.
            \return Returns a list of defines.
        */
        DefineList getDefines() const;


        /** Bind the ReSTIR sampler to a given shader var.
            \param[in] var The shader variable to set the data into.
        */
        void setShaderData(const ShaderVar& var) const;

        /** Render the GUI.
            \return True if options were changed, false otherwise.
        */
        bool renderUI(Gui::Widgets& widget);

        bool onKeyEvents(const KeyboardEvent& keyEvent);

        /** Returns the current configuration.
        */
        Options getOptions() const { return mOptions; }

        /** Set the configuration.
        */
        void setOptions(const Options& options);

        /** Begin a frame.
            Must be called once at the beginning of each frame.
            \param[in] pRenderContext Render context.
            \param[in] frameDim Current frame dimension.
        */
        void beginFrame(RenderContext* pRenderContext, const uint2& frameDim, const uint frameCount, float filterRadius, float filterAlpha,
            float filterNorm, bool dofEnabled, bool aaEnabled);

        /** End a frame.
            Must be called one at the end of each frame.
            \param[in] pRenderContext Render context.
        */
        void endFrame(RenderContext* pRenderContext);

        /** Update the ReSTIR sampler.
            This runs the ReSTIR DI algorithm and prepares a set of final samples to be queried afterwards.
            Must be called once between beginFrame() and endFrame().
            \param[in] pRenderContext Render context.
            \param[in] pMotionVectors Motion vectors for temporal reprojection.
        */
        void updateReSTIRDI(RenderContext* pRenderContext, const ref<Texture>& pMotionVectors, const ref<Texture>& pSubPixelUV = nullptr,
            const ref<Texture>& pLensUV = nullptr, const ref<Texture>& pViewDir = nullptr);

        /** Get the debug output texture.
            \return Returns the debug output texture.
        */
        const ref<Texture>& getDebugOutputTexture() const { return mpDebugOutputTexture; }

        /** Get the pixel debug component.
            \return Returns the pixel debug component.
        */
        const std::unique_ptr<PixelDebug>& getPixelDebug() const { return mpPixelDebug; }

        //static void scriptBindings(pybind11::module& m);

        void setOwnerDefines(DefineList defines);
        void setSubPixelRandomOption(GBufferBase::SubPixelRandom subpixelRandom) { mOptions.subPixelRandom = subpixelRandom; }
        void updatePrograms();
        void setRecompile(bool recompile) { mRecompile = recompile; }


    private:
        ref<Scene> mpScene;                           ///< Scene.
        ref<Device> mpDevice;                   ///< GPU device.
        Options mOptions;                                   ///< Configuration options.
        DefineList mOwnerDefines; ///< Share defines with inline path tracer

        std::mt19937 mRng;                                  ///< Random generator.

        std::unique_ptr<PixelDebug> mpPixelDebug;                 ///< Pixel debug component.

        uint2 mFrameDim = uint2(0);                         ///< Current frame dimensions.
        uint32_t mFrameIndex = 0;                           ///< Current frame index.

        ref<ComputePass> mpReflectTypes;              ///< Pass for reflecting types.

        // ReSTIR DI passes.
        ref<ComputePass> mpUpdateEmissiveTriangles;   ///< Pass for updating the local emissive triangle data.
        ref<ComputePass> mpGenerateLightTiles;        ///< Pass for generating the light tiles.
        ref<ComputePass> mpInitialResampling;         ///< Pass for initial resampling.
        ref<ComputePass> mpTemporalResampling;        ///< Pass for temporal resampling.
        ref<ComputePass> mpTemporalResamplingTracePrimaryRays;
        ref<ComputePass> mpTemporalResamplingFloatMotion; // Pairwise is enabled as default
        ref<ComputePass> mpSpatialResampling;         ///< Pass for spatial resampling.
        ref<ComputePass> mpEvaluateFinalSamples;      ///< Pass for evaluating the final samples.

        // ReSTIR DI resources.
        ref<Buffer> mpEnvLightLuminance;              ///< Buffer with luminance values of the env map.
        float mEnvLightLuminanceFactor;                     ///< Scalar luminance factor based on env map intensity and tint.
        ref<Buffer> mpEmissiveTriangles;              ///< Buffer with emissive triangle data.

        std::unique_ptr<AliasTable> mpEnvLightAliasTable;         ///< Alias table for sampling the env map.
        std::unique_ptr<AliasTable> mpEmissiveLightAliasTable;    ///< Alias table for sampling emissive lights.
        std::unique_ptr<AliasTable> mpAnalyticLightAliasTable;    ///< Alias table for sampling analytic lights.

        ref<Buffer> mpSurfaceData;                    ///< Buffer with the current frame surface data (GBuffer).
        ref<Buffer> mpPrevSurfaceData;                ///< Buffer with the previous frame surface data (GBuffer).
        ref<Buffer> mpPrevCameraData;
        ref<Buffer> mpLightTileData;                  ///< Buffer with the light tiles (light samples).
        ref<Buffer> mpCenterSurfaceData;
        ref<Buffer> mpCenterPrevSurfaceData;

        ref<Texture> mpNormalDepthTexture;            ///< Compact normal/depth texture used for fast neighbor pixel validation.
        ref<Texture> mpPrevNormalDepthTexture;        ///< Compact normal/depth texture used for fast neighbor pixel validation.
        ref<Texture> mpDebugOutputTexture;            ///< Debug output texture.
        ref<Texture> mpNeighborOffsets;               ///< 1D texture containing neighbor offsets within a unit circle.
        ref<Texture> mpSubPixelUvTexture;
        ref<Texture> mpPrevSubPixelUvTexture;
        ref<Texture> mpLensUvTexture;
        ref<Texture> mpPrevLensUvTexture;

        bool mRecompile = true;                             ///< Recompile programs on next frame if set to true.
        bool mResetTemporalReservoirs = true;               ///< Reset temporal reservoir buffer on next frame if set to true.

        // Guassian filter parameters
        float mFilterRadius = 0.5f;
        float mFilterAlpha = 0.f;
        float mFilterNorm = 1.f;

        struct
        {
            float envLight = 0.f;
            float emissiveLights = 0.f;
            float analyticLights = 0.f;

            /** Compute a discrete set of sample counts given the current selection probabilities.
            */
            std::tuple<uint32_t, uint32_t, uint32_t> getSampleCount(uint32_t totalCount)
            {
                uint32_t envCount = (uint32_t)std::floor(envLight * totalCount);
                uint32_t emissiveCount = (uint32_t)std::floor(emissiveLights * totalCount);
                uint32_t analyticCount = (uint32_t)std::floor(analyticLights * totalCount);
                if (envCount > 0) envCount = totalCount - emissiveCount - analyticCount;
                else if (emissiveCount > 0) emissiveCount = totalCount - envCount - analyticCount;
                else if (analyticCount > 0) analyticCount = totalCount - envCount - emissiveCount;
                return { envCount, emissiveCount, analyticCount };
            }
        }
        mLightSelectionProbabilities;

        // Shared ReSTIR resampling resource. Those resources will be changed during resampling process
        struct ResamplingResources
        {
            ResamplingResourceType type;
            uint perPixelMsaaShiftsCount = 8u;

            ref<Buffer> pReservoirs;
            ref<Buffer> pPrevReservoirs;                 ///< Buffer containing the previous reservoirs.
            ref<Buffer> pFinalSamples;                   ///< Buffer with the final samples.
            ref<Buffer> pFinalPrimaryHits;
            ref<Buffer> pResEvalContext;
            ref<Buffer> pPrevResEvalContext;
            ref<Buffer> pPixelCenterEvalContext;
            ref<Buffer> pPrevPixelCenterEvalContext;
            ref<Buffer> pTemporalMISPDFs;

            // only reconnection shift needs following buffer
            ref<Buffer> pTemporalMISEvalContexts;
            ref<Buffer> pTemporalMISViewDirs;
            //ref<Buffer> pTemporalMISLensUV;
            ref<Buffer> pTemporalMISJacobianData;
            ref<Buffer> pTemporalMISPrimHitNormals;
            ref<Buffer> pTemporalMISPrimaryHits;
        } mDirectLightingResources, mEmissiveResources;


        // Functions
        void prepareResources(RenderContext* pRenderContext);
        void prepareLighting(RenderContext* pRenderContext);
        void updateEmissiveTriangles(RenderContext* pRenderContext);
        void generateLightTiles(RenderContext* pRenderContext);

        // ReSTIR passes
        void initialResampling(RenderContext* pRenderContext, const ref<Texture>& pViewDir, ResamplingResources& resources);
        void temporalMSAATracePrimaryRays(RenderContext* pRenderContext, ResamplingResources& resources);
        void temporalResampling(RenderContext* pRenderContext, const ref<Texture>& pMotionVectors, const ref<ComputePass>& pTemporalPass,
            ResamplingResources& resources);
        void spatialResampling(RenderContext* pRenderContext, ResamplingResources& resources);
        void evaluateFinalSamples(RenderContext* pRenderContext, ResamplingResources& resources);

        // Defines and shader data utilities
        DefineList getLightsDefines() const;
        void setLightsShaderData(const ShaderVar& var) const;
        void setAreaSampleShaderData(const ShaderVar& var) const;
        void setResamplingShaderData(const ShaderVar& var) const;

        std::vector<float> computeEnvLightLuminance(RenderContext* pRenderContext, const ref<Texture>& texture, std::vector<float3>& radiances);
        std::unique_ptr<AliasTable> buildEnvLightAliasTable(uint32_t width, uint32_t height, const std::vector<float>& luminances, std::mt19937& rng);
        std::unique_ptr<AliasTable> buildEmissiveLightAliasTable(RenderContext* pRenderContext, const ref<LightCollection>& lightCollection, std::mt19937& rng);
        std::unique_ptr<AliasTable> buildAnalyticLightAliasTable(RenderContext* pRenderContext, const std::vector<ref<Light>>& lights, std::mt19937& rng);

        /** Create a 1D texture with random offsets within a unit circle around (0,0).
            The texture is RG8Snorm for compactness and has no mip maps.
            \param[in] sampleCount Number of samples in the offset texture.
        */
        ref<Texture> createNeighborOffsetTexture(uint32_t sampleCount);
    };
}
