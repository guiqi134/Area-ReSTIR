from falcor import *

def render_graph_PathTracer():
    g = RenderGraph("PathTracer")

    gVBufferParams = {
        'samplePattern': "Center",
        'sampleCount': 1,
        'useAlphaTest': True,
        'subPixelRandom' : "UnitQuad",
        'useDOF' : False,
        'computeDerivativeMaually' : True,
    }
    gPathTracerParams = {
        'samplesPerPixel': 1,
        'useReSTIR': True,
        'emissiveSampler': "Power",
        'maxSurfaceBounces' : 0,
        'maxDiffuseBounces' : 0,
        'maxSpecularBounces' : 0,
        'maxTransmissionBounces' : 0,
        'disableCaustics' : True,
        'areaReSTIROptions' : {
            "reuseLensSample" : False, "reuseSubpixelSample" : True,
            "resampleEmissionMode" : "OneMergedReservoir", "useTemporalResampling" : True, "useSpatialResampling" : True,
            "temporalMode" : "FractionalMotion_Shifting2RIS",
            "temporalShiftMappingModeRIS1" : "OnlyRandomReplay", "temporalShiftMappingModeRIS2" : "MIS", "spatialShiftMappingMode" : "MIS",
        }
    }
    gSVGFParams = {
       'Enabled': True,
       'Iterations': 4,
       'FeedbackTap': 1,
       'VarianceEpsilon': 9.999999747378752e-05,
       'PhiColor': 10.0,
       'PhiNormal': 128.0,
       'Alpha': 0.05000000074505806,
       'MomentsAlpha': 0.20000000298023224
    }

    VBufferRT = createPass("VBufferRT", gVBufferParams)
    g.addPass(VBufferRT, "VBufferRT")
    PathTracer = createPass("PathTracer", gPathTracerParams)
    g.addPass(PathTracer, "PathTracer")
    SVGFPass = createPass("SVGFPass", gSVGFParams)
    g.addPass(SVGFPass, "SVGFPass")
    AccumulatePass = createPass("AccumulatePass", {'enabled': False, 'precisionMode': 'Single'})
    g.addPass(AccumulatePass, "AccumulatePass")
    ToneMapper = createPass("ToneMapper", {'autoExposure': False, 'exposureCompensation': 0.0})
    g.addPass(ToneMapper, "ToneMapper")

    g.addEdge("VBufferRT.vbuffer", "PathTracer.vbuffer")
    g.addEdge("VBufferRT.vbufferCenter", "PathTracer.vbufferCenter")
    g.addEdge("VBufferRT.viewW", "PathTracer.viewW")
    g.addEdge("VBufferRT.depth", "PathTracer.depth")
    g.addEdge("VBufferRT.mvec", "PathTracer.mvec")
    g.addEdge("VBufferRT.subPixelUV", "PathTracer.subPixelUV")
    g.addEdge("VBufferRT.lensUV", "PathTracer.lensUV")

    g.addEdge("PathTracer.color", "SVGFPass.Color")
    g.addEdge("VBufferRT.mvec", "SVGFPass.MotionVec")
    g.addEdge("VBufferRT.posW", "SVGFPass.WorldPosition")
    g.addEdge("VBufferRT.normW", "SVGFPass.WorldNormal")
    g.addEdge("VBufferRT.albedo", "SVGFPass.Albedo")
    g.addEdge("VBufferRT.linearZ", "SVGFPass.LinearZ")
    g.addEdge("VBufferRT.emissive", "SVGFPass.Emission")
    g.addEdge("VBufferRT.pnFwidth", "SVGFPass.PositionNormalFwidth")

    g.addEdge("SVGFPass.Filtered image", "AccumulatePass.input")
    g.addEdge("AccumulatePass.output", "ToneMapper.src")

    g.markOutput("ToneMapper.dst")
    return g

PathTracer = render_graph_PathTracer()
try: m.addGraph(PathTracer)
except NameError: None

pauseClock = True
if pauseClock:
    m.clock.stop()

scene = "../data/Bistro/BistroExteriorOrigin.pyscene"
m.loadScene(scene, buildFlags=(SceneBuilderFlags.DontMergeMaterials | SceneBuilderFlags.DontOptimizeMaterials))
m.scene.camera.animated = False
m.scene.animated = True

# Update toneMapping pass
PathTracer.updatePass("ToneMapper", {'autoExposure': False, 'exposureCompensation': 0.0})
