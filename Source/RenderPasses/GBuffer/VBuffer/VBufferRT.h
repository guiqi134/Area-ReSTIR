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
#pragma once
#include "../GBufferBase.h"
#include "Utils/Sampling/SampleGenerator.h"
#include "Utils/Debug/PixelDebug.h"
#include "RenderGraph/RenderPassStandardFlags.h"

using namespace Falcor;

/** Ray traced V-buffer pass.

    This pass renders a visibility buffer using ray tracing.
    The visibility buffer encodes the mesh instance ID and primitive index,
    as well as the barycentrics at the hit point.
*/
class VBufferRT : public GBufferBase
{
public:
    FALCOR_PLUGIN_CLASS(VBufferRT, "VBufferRT", "Ray traced V-buffer generation pass.");

    static ref<VBufferRT> create(ref<Device> pDevice, const Properties& props) { return make_ref<VBufferRT>(pDevice, props); }

    VBufferRT(ref<Device> pDevice, const Properties& props);

    RenderPassReflection reflect(const CompileData& compileData) override;
    void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    void renderUI(Gui::Widgets& widget) override;
    virtual bool onMouseEvent(const MouseEvent& mouseEvent) { return mpPixelDebug->onMouseEvent(mouseEvent); }
    Properties getProperties() const override;
    void setScene(RenderContext* pRenderContext, const ref<Scene>& pScene) override;

private:
    void parseProperties(const Properties& props) override;

    void executeRaytrace(RenderContext* pRenderContext, const RenderData& renderData);
    void executeCompute(RenderContext* pRenderContext, const RenderData& renderData);

    DefineList getShaderDefines(const RenderData& renderData) const;
    void bindShaderData(const ShaderVar& var, const RenderData& renderData);
    void recreatePrograms();

    // Internal state
    bool mComputeDOF = false;           ///< Flag indicating if depth-of-field is computed for the current frame.
    ref<SampleGenerator> mpSampleGenerator;
    std::unique_ptr<PixelDebug> mpPixelDebug;

    // UI variables
    bool mUseTraceRayInline = true;
    bool mUseDOF = false;                ///< Option for enabling depth-of-field when camera's aperture radius is nonzero.

    struct
    {
        ref<Program> pProgram;
        ref<RtProgramVars> pVars;
    } mRaytrace;

    ref<ComputePass> mpComputePass;
};
