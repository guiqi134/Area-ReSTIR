add_plugin(PathTracer)

target_sources(PathTracer PRIVATE
    ColorType.slang
    GeneratePaths.cs.slang
    GuideData.slang
    LoadShadingData.slang
    NRDHelpers.slang
    Params.slang
    PathState.slang
    PathTracer.slang
    PathTracer.cpp
    PathTracer.h
    PathTracerNRD.slang
    ReflectTypes.cs.slang
    ResolvePass.cs.slang
    StaticParams.slang
    TracePass.rt.slang
)

target_link_libraries(PathTracer PUBLIC AreaReSTIR)
# add_dependencies(PathTracer AreaReSTIR)

target_source_group(PathTracer "RenderPasses")

# AreaReSTIR source files
# Note: These added after calling target_source_group macro as they are outside the current folder.
target_sources(PathTracer PRIVATE
    ../../Modules/AreaReSTIR/EvalContext.slang
    ../../Modules/AreaReSTIR/EvaluateFinalSamples.cs.slang
    ../../Modules/AreaReSTIR/FinalSample.slang
    ../../Modules/AreaReSTIR/GenerateLightTiles.cs.slang
    ../../Modules/AreaReSTIR/InitialResampling.cs.slang
    ../../Modules/AreaReSTIR/Lights.slang
    ../../Modules/AreaReSTIR/LightSample.slang
    ../../Modules/AreaReSTIR/LocalEmissiveTriangle.slang
    ../../Modules/AreaReSTIR/Params.slang
    ../../Modules/AreaReSTIR/ReflectTypes.cs.slang
    ../../Modules/AreaReSTIR/Resampling.slang
    ../../Modules/AreaReSTIR/Reservoir.slang
    ../../Modules/AreaReSTIR/AreaReSTIR.slang
    ../../Modules/AreaReSTIR/SpatialResampling.cs.slang
    ../../Modules/AreaReSTIR/SurfaceData.slang
    ../../Modules/AreaReSTIR/TemporalResampling.cs.slang
    ../../Modules/AreaReSTIR/UpdateEmissiveTriangles.cs.slang
    ../../Modules/AreaReSTIR/LoadShadingData.slang
    ../../Modules/AreaReSTIR/PixelAreaSampleData.slang
    ../../Modules/AreaReSTIR/TemporalResampling_FloatMotion.cs.slang
    ../../Modules/AreaReSTIR/SimpleCameraData.slang
    ../../Modules/AreaReSTIR/StaticParams.slang
    ../../Modules/AreaReSTIR/TemporalMSAATracePrimaryRays.cs.slang
)

target_copy_shaders(PathTracer RenderPasses/PathTracer)
