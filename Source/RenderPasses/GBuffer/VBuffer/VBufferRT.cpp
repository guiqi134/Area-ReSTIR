/***************************************************************************
 # Copyright (c) 2015-23, NVIDIA CORPORATION. All rights reserved.
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
#include "VBufferRT.h"
#include "Scene/HitInfo.h"
#include "RenderGraph/RenderPassStandardFlags.h"
#include "RenderGraph/RenderPassHelpers.h"

namespace
{
    const std::string kProgramRaytraceFile = "RenderPasses/GBuffer/VBuffer/VBufferRT.rt.slang";
    const std::string kProgramComputeFile = "RenderPasses/GBuffer/VBuffer/VBufferRT.cs.slang";
    const std::string kComputeDerivativesFile = "RenderPasses/GBuffer/VBuffer/ComputeDerivatives.cs.slang";

    // Scripting options.
    const char kUseTraceRayInline[] = "useTraceRayInline";
    const char kUseDOF[] = "useDOF";
    const char kComputeDerivativeMaually[] = "computeDerivativeMaually";

    // Ray tracing settings that affect the traversal stack size. Set as small as possible.
    const uint32_t kMaxPayloadSizeBytes = 4; // TODO: The shader doesn't need a payload, set this to zero if it's possible to pass a null payload to TraceRay()
    const uint32_t kMaxRecursionDepth = 1;

    const std::string kVBufferName = "vbuffer";
    const std::string kVBufferCenterName = "vbufferCenter";
    const std::string kVBufferDesc = "V-buffer in packed format (indices + barycentrics)";

    // Additional output channels.
    const ChannelList kVBufferExtraChannels =
    {
        { "depth",          "gDepth",           "Depth buffer (NDC)",               true /* optional */, ResourceFormat::R32Float    },
        { "linearDepth",    "gLinearDepth",     "Depth buffer (View)",              true /* optional */, ResourceFormat::RGBA32Float    },
        { "mvec",           "gMotionVector",    "Motion vector",                    true /* optional */, ResourceFormat::RG32Float   },
        { "viewW",          "gViewW",           "View direction in world space",    true /* optional */, ResourceFormat::RGBA32Float },
        { "time",           "gTime",            "Per-pixel execution time",         true /* optional */, ResourceFormat::R32Uint     },
        { "mask",           "gMask",            "Mask",                             true /* optional */, ResourceFormat::R32Float    },
        { "subPixelUV", "gSubPixelUV", "Sub-pixel UV position", true, ResourceFormat::RG32Float },
        { "lensUV", "gLensUV", "Pixel lens UV position", true, ResourceFormat::RG32Float },
        { "debugMotion", "gDebugMotion", "Debug motion vector", true, ResourceFormat::RG32Float},
    };

    // Additional output channels for SVGF denoising
    const ChannelList kVBufferExtraChannelsForSVGF =
    {
        { "posW", "gPosW", "Position in world space", true /* optional */, ResourceFormat::RGBA32Float },
        { "normW", "gNormW", "Shading normal in world space", true /* optional */, ResourceFormat::RGBA32Float },
        { "albedo", "gAlbedo", "Albedo", true /* optional */, ResourceFormat::RGBA32Float },
        { "linearZ", "gLinearZ", "Linear Z and slope", true /* optional */, ResourceFormat::RG32Float },
        { "emissive", "gEmissive", "Emissive color", true /* optional */, ResourceFormat::RGBA32Float },
        { "pnFwidth",  "gPosNormalFwidth", "Position and guide normal filter width", true /* optional */, ResourceFormat::RG32Float },
    };
};

VBufferRT::VBufferRT(ref<Device> pDevice, const Properties& props)
    : GBufferBase(pDevice)
{
    if (!mpDevice->isFeatureSupported(Device::SupportedFeatures::RaytracingTier1_1))
    {
        throw RuntimeError("VBufferRT: Raytracing Tier 1.1 is not supported by the current device");
    }

    parseProperties(props);

    // Create sample generator
    mpSampleGenerator = SampleGenerator::create(mpDevice, SAMPLE_GENERATOR_DEFAULT);

    mpPixelDebug = std::make_unique<PixelDebug>(pDevice);
}

RenderPassReflection VBufferRT::reflect(const CompileData& compileData)
{
    RenderPassReflection reflector;
    const uint2 sz = RenderPassHelpers::calculateIOSize(mOutputSizeSelection, mFixedOutputSize, compileData.defaultTexDims);

    // Add the required output. This always exists.
    reflector.addOutput(kVBufferName, kVBufferDesc).bindFlags(ResourceBindFlags::UnorderedAccess).format(mVBufferFormat).texture2D(sz.x, sz.y);

    reflector.addOutput(kVBufferCenterName, kVBufferDesc).bindFlags(ResourceBindFlags::UnorderedAccess).format(
        mVBufferFormat).texture2D(sz.x, sz.y).flags(RenderPassReflection::Field::Flags::Optional);

    // Add all the other outputs.
    addRenderPassOutputs(reflector, kVBufferExtraChannels, ResourceBindFlags::UnorderedAccess, sz);
    addRenderPassOutputs(reflector, kVBufferExtraChannelsForSVGF, ResourceBindFlags::UnorderedAccess, sz);

    return reflector;
}

void VBufferRT::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    GBufferBase::execute(pRenderContext, renderData);

    //logInfo("VBufferRT::execute() called");

    // Update frame dimension based on render pass output.
    auto pOutput = renderData.getTexture(kVBufferName);
    FALCOR_ASSERT(pOutput);
    updateFrameDim(uint2(pOutput->getWidth(), pOutput->getHeight()));

    // If there is no scene, clear the output and return.
    if (mpScene == nullptr)
    {
        pRenderContext->clearUAV(pOutput->getUAV().get(), uint4(0));
        clearRenderPassChannels(pRenderContext, kVBufferExtraChannels, renderData);
        clearRenderPassChannels(pRenderContext, kVBufferExtraChannelsForSVGF, renderData);
        return;
    }

    mpPixelDebug->beginFrame(pRenderContext, uint2(pOutput->getWidth(), pOutput->getHeight()));

    // Check for scene changes.
    if (is_set(mpScene->getUpdates(), Scene::UpdateFlags::GeometryChanged) ||
        //is_set(mpScene->getUpdates(), Scene::UpdateFlags::GeometryMoved) ||
        //is_set(mpScene->getUpdates(), Scene::UpdateFlags::LightCollectionChanged) ||
        is_set(mpScene->getUpdates(), Scene::UpdateFlags::SDFGridConfigChanged))
    {
        recreatePrograms();
    }

    // Configure depth-of-field.
    // When DOF is enabled, two PRNG dimensions are used. Pass this info to subsequent passes via the dictionary.
    mComputeDOF = mUseDOF && mpScene->getCamera()->getApertureRadius() > 0.f;
    if (mUseDOF)
    {
        renderData.getDictionary()[Falcor::kRenderPassPRNGDimension] = mComputeDOF ? 2u : 0u;
    }

    renderData.getDictionary()["filterRadius"] = mUseGaussianFilter ? 0.5f * mAreaScaler : 0.5f;
    renderData.getDictionary()["filterAlpha"] = mUseGaussianFilter ? mFilterAlpha : 0.f;
    renderData.getDictionary()["dofEnabled"] = mComputeDOF;
    renderData.getDictionary()["aaEnabled"] = mSubPixelRandom != SubPixelRandom::None;

    mUseTraceRayInline ? executeCompute(pRenderContext, renderData) : executeRaytrace(pRenderContext, renderData);

    mpPixelDebug->endFrame(pRenderContext);

    mFrameCount++;
}

void VBufferRT::renderUI(Gui::Widgets& widget)
{
    GBufferBase::renderUI(widget);

    if (widget.checkbox("Use TraceRayInline", mUseTraceRayInline))
    {
        mOptionsChanged = true;
    }

    if (widget.checkbox("Use depth-of-field", mUseDOF))
    {
        mOptionsChanged = true;
    }
    widget.tooltip(
        "This option enables stochastic depth-of-field when the camera's aperture radius is nonzero. Disable it to force the use of a "
        "pinhole camera.",
        true
    );

    if (auto group = widget.group("Debugging"))
    {
        mOptionsChanged |= group.checkbox("Use fixed seed", mUseFixedSeed);
        group.tooltip("Forces a fixed random seed for each frame.\n\n"
            "This should produce exactly the same image each frame, which can be useful for debugging.");

        if (mUseFixedSeed)
        {
            mOptionsChanged |= group.var("Seed", mFixedSeed);
        }

        group.text("Frame Count (random seed) = " + std::to_string(mFrameCount));

        mpPixelDebug->renderUI(group);
    }
}

Properties VBufferRT::getProperties() const
{
    Properties props = GBufferBase::getProperties();
    props[kUseTraceRayInline] = mUseTraceRayInline;
    props[kUseDOF] = mUseDOF;

    return props;
}

void VBufferRT::setScene(RenderContext* pRenderContext, const ref<Scene>& pScene)
{
    GBufferBase::setScene(pRenderContext, pScene);

    recreatePrograms();
}

void VBufferRT::parseProperties(const Properties& props)
{
    GBufferBase::parseProperties(props);

    for (const auto& [key, value] : props)
    {
        if (key == kUseTraceRayInline) mUseTraceRayInline = value;
        else if (key == kUseDOF) mUseDOF = value;
        else if (key == kUseDOF) mComputeDerivativeMaually = value;
        // TODO: Check for unparsed fields, including those parsed in base classes.
    }
}

void VBufferRT::recreatePrograms()
{
    logInfo("VBuffer recreatePrograms() called");
    mRaytrace.pProgram = nullptr;
    mRaytrace.pVars = nullptr;
    mpComputePass = nullptr;
    mpComputeDerivativesPass = nullptr;
}

void VBufferRT::executeRaytrace(RenderContext* pRenderContext, const RenderData& renderData)
{
    if (!mRaytrace.pProgram || !mRaytrace.pVars)
    {
        DefineList defines;
        defines.add(mpScene->getSceneDefines());
        defines.add(mpSampleGenerator->getDefines());
        defines.add(getShaderDefines(renderData));

        // Create ray tracing program.
        ProgramDesc desc;
        desc.addShaderModules(mpScene->getShaderModules());
        desc.addShaderLibrary(kProgramRaytraceFile);
        desc.addTypeConformances(mpScene->getTypeConformances());
        desc.setMaxPayloadSize(kMaxPayloadSizeBytes);
        desc.setMaxAttributeSize(mpScene->getRaytracingMaxAttributeSize());
        desc.setMaxTraceRecursionDepth(kMaxRecursionDepth);

        ref<RtBindingTable> sbt = RtBindingTable::create(1, 1, mpScene->getGeometryCount());
        sbt->setRayGen(desc.addRayGen("rayGen"));
        sbt->setMiss(0, desc.addMiss("miss"));
        sbt->setHitGroup(0, mpScene->getGeometryIDs(Scene::GeometryType::TriangleMesh), desc.addHitGroup("closestHit", "anyHit"));

        // Add hit group with intersection shader for triangle meshes with displacement maps.
        if (mpScene->hasGeometryType(Scene::GeometryType::DisplacedTriangleMesh))
        {
            sbt->setHitGroup(0, mpScene->getGeometryIDs(Scene::GeometryType::DisplacedTriangleMesh), desc.addHitGroup("displacedTriangleMeshClosestHit", "", "displacedTriangleMeshIntersection"));
        }

        // Add hit group with intersection shader for curves (represented as linear swept spheres).
        if (mpScene->hasGeometryType(Scene::GeometryType::Curve))
        {
            sbt->setHitGroup(0, mpScene->getGeometryIDs(Scene::GeometryType::Curve), desc.addHitGroup("curveClosestHit", "", "curveIntersection"));
        }

        // Add hit group with intersection shader for SDF grids.
        if (mpScene->hasGeometryType(Scene::GeometryType::SDFGrid))
        {
            sbt->setHitGroup(0, mpScene->getGeometryIDs(Scene::GeometryType::SDFGrid), desc.addHitGroup("sdfGridClosestHit", "", "sdfGridIntersection"));
        }

        mRaytrace.pProgram = Program::create(mpDevice, desc, defines);
        mRaytrace.pVars = RtProgramVars::create(mpDevice, mRaytrace.pProgram, sbt);

        // Bind static resources.
        ShaderVar var = mRaytrace.pVars->getRootVar();
        mpSampleGenerator->bindShaderData(var);
    }

    mRaytrace.pProgram->addDefines(getShaderDefines(renderData));

    ShaderVar var = mRaytrace.pVars->getRootVar();
    bindShaderData(var, renderData);

    // Dispatch the rays.
    mpScene->raytrace(pRenderContext, mRaytrace.pProgram.get(), mRaytrace.pVars, uint3(mFrameDim, 1));
}

void VBufferRT::executeCompute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // Create compute pass.
    if (!mpComputePass)
    {
    	ProgramDesc desc;
        desc.addShaderModules(mpScene->getShaderModules());
    	desc.addShaderLibrary(kProgramComputeFile).csEntry("main").setShaderModel(ShaderModel::SM6_6);
        desc.addTypeConformances(mpScene->getTypeConformances());

        DefineList defines;
        defines.add(mpScene->getSceneDefines());
        defines.add(mpSampleGenerator->getDefines());
        defines.add(getShaderDefines(renderData));

    	mpComputePass = ComputePass::create(mpDevice, desc, defines, true);

        // Bind static resources
        ShaderVar var = mpComputePass->getRootVar();
        mpScene->setRaytracingShaderData(pRenderContext, var);
        mpSampleGenerator->bindShaderData(var);
    }

    //logInfo("Vbuffer executeCompute() called");

    mpComputePass->getProgram()->addDefines(getShaderDefines(renderData));

    ShaderVar var = mpComputePass->getRootVar();
    bindShaderData(var, renderData);
    mpScene->setRaytracingShaderData(pRenderContext, var); // always update TLAS

    mpPixelDebug->prepareProgram(mpComputePass->getProgram(), var);
    mpComputePass->execute(pRenderContext, uint3(mFrameDim, 1));

    // Compute derivatives for SVGF if enabled -> not needed, we can use ddx, ddy, fwidth in SM 6.6
    //if (mpDevice->isShaderModelSupported(ShaderModel::SM6_6) && renderData["pnFwidth"] != nullptr)
    if (mComputeDerivativeMaually && renderData["pnFwidth"] != nullptr)
    {
        //logInfo("SM 6.6 is not supported, use extra compute shader to compute derivatives");
        if (!mpComputeDerivativesPass)
        {
            ProgramDesc desc;
            desc.addShaderModules(mpScene->getShaderModules());
            desc.addShaderLibrary(kComputeDerivativesFile).csEntry("main");
            desc.addTypeConformances(mpScene->getTypeConformances());

            DefineList defines;
            defines.add(mpScene->getSceneDefines());
            mpComputeDerivativesPass = ComputePass::create(mpDevice, desc, defines, true);
        }

        // No ray tracing, no need to call setRaytracingShaderData() 

        auto rootVar = mpComputeDerivativesPass->getRootVar();
        rootVar["gPosW"] = renderData.getTexture("posW");
        rootVar["gNormW"] = renderData.getTexture("normW");
        rootVar["gPosNormalFwidth"] = renderData.getTexture("pnFwidth");
        rootVar["gLinearZ"] = renderData.getTexture("linearZ");
        rootVar["CB"]["gFrameDim"] = mFrameDim;
        mpPixelDebug->prepareProgram(mpComputeDerivativesPass->getProgram(), var);
        mpComputeDerivativesPass->execute(pRenderContext, uint3(mFrameDim, 1));
    }
}

DefineList VBufferRT::getShaderDefines(const RenderData& renderData) const
{
    DefineList defines;
    defines.add("COMPUTE_DEPTH_OF_FIELD", mComputeDOF ? "1" : "0");
    defines.add("USE_ALPHA_TEST", mUseAlphaTest ? "1" : "0");
    defines.add("COMPUTE_DERIVATIVE_MAUALLY", mComputeDerivativeMaually ? "1" : "0");

    // Setup ray flags.
    RayFlags rayFlags = RayFlags::None;
    if (mForceCullMode && mCullMode == RasterizerState::CullMode::Front) rayFlags = RayFlags::CullFrontFacingTriangles;
    else if (mForceCullMode && mCullMode == RasterizerState::CullMode::Back) rayFlags = RayFlags::CullBackFacingTriangles;
    defines.add("RAY_FLAGS", std::to_string((uint32_t)rayFlags));

    // For optional I/O resources, set 'is_valid_<name>' defines to inform the program of which ones it can access.
    // TODO: This should be moved to a more general mechanism using Slang.
    defines.add(getValidResourceDefines(kVBufferExtraChannels, renderData));
    defines.add(getValidResourceDefines(kVBufferExtraChannelsForSVGF, renderData));
    return defines;
}

void VBufferRT::bindShaderData(const ShaderVar& var, const RenderData& renderData)
{
    var["gVBufferRT"]["frameDim"] = mFrameDim;
    var["gVBufferRT"]["invFrameDim"] = mInvFrameDim;
    var["gVBufferRT"]["frameCount"] = mFrameCount;
    var["gVBufferRT"]["subPixelRandom"] = (uint)mSubPixelRandom;
    var["gVBufferRT"]["areaScaler"] = mUseGaussianFilter ? mAreaScaler : 1.f;
    var["gVBufferRT"]["isSceneAnimated"] = mpScene->isAnimated();
    var["gVBufferRT"]["clampMotionVec"] = mClampMotionVector;
    var["gVBufferRT"]["mvecClampThreshold"] = mMotionVecThreshold;
    var["gVBufferRT"]["sameSubpixelRandomForAllPixels"] = mSameSubpixelRandomForAllPixels;
    var["gVBufferRT"]["useSVGF"] = renderData["pnFwidth"] != nullptr;

    // Bind resources.
    var["gVBuffer"] = getOutput(renderData, kVBufferName);
    var["gVBufferCenter"] = getOutput(renderData, kVBufferCenterName);

    // Bind output channels as UAV buffers.
    auto bind = [&](const ChannelDesc& channel)
    {
        ref<Texture> pTex = getOutput(renderData, channel.name);
        var[channel.texname] = pTex;
    };
    for (const auto& channel : kVBufferExtraChannels) bind(channel);
    for (const auto& channel : kVBufferExtraChannelsForSVGF) bind(channel);
}
