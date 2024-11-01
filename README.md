# Area ReSTIR (DI)
![](teaser.png)

## Introduction
- This repo includes the source code (only the direct lighting part) for the following SIGGRAPH 2024 paper

> **Area ReSTIR: Resampling for Real-Time Defocus and Antialiasing**<br>
> Song Zhang* (University of Utah), Daqi Lin* (NVIDIA), Markus Kettunen (NVIDIA), Cem Yuksel (University of Utah), Chris Wyman (NVIDIA)<br>
> (*Joint first authors) <br>
> [Utah Graphics Research Page](https://graphics.cs.utah.edu/research/projects/area-restir/) <br>
> [NVIDIA Real-Time Graphics Research Page](https://research.nvidia.com/labs/rtr/publication/zhang2024area/)

Area ReSTIR extends ReSTIR reservoirs to also integrate each pixel's 4D ray space, including 2D areas on the film and lens. To maximize resampling quality on subpixel and lens region, it also includes novel subpixel-tracking temporal reuse and reconnection shift mapping.
Compare to [[Bitterli et al. 2020]](https://cs.dartmouth.edu/~wjarosz/publications/bitterli20spatiotemporal.html) (ReSTIR DI) and [[Lin et al. 2022]](https://research.nvidia.com/publication/2022-07_generalized-resampled-importance-sampling-foundations-restir) (ReSTIR PT),
our Area ReSTIR has significant improvements on bokeh, foliage, hair, and detailed normal maps.

- The method is implemented as a rendering module called "AreaReSTIR" (`Source\Modules\AreaReSTIR`) in Falcor 7.0.
See README_Falcor.md for the original README file provided by Falcor.
- A script `RunAreaReSTIR.bat` is provided to show how the method works in an animated version of the Bistro scene (from [NVIDIA Ocra's Amazon Lumberyard Bistro](https://developer.nvidia.com/orca/amazon-lumberyard-bistro)).
- Before running the scripts, you need to compile the program and download the scene files following the instruction below.
- Note: although Area ReSTIR is implemented as a module for `PathTracer` render pass in Falcor, we also change some Falcor's source code to support storing previous frame's scene data, so just copying `Source\Modules\AreaReSTIR` to another Falcor project (or your project) will lead to bias/errors.

## Prerequisites
- Windows 10 version 20H2 or newer
- Visual Studio 2022
- [Windows 10 SDK version 10.0.19041.1 Or Newer](https://developer.microsoft.com/en-us/windows/downloads/sdk-archive)
- NVIDIA driver 530.xx and above
- A GPU supports DirectX Raytracing
    - To successfully run the program under default settings, the GPU memory should be **at least 12 GB**. Otherwise, see "Run the demo" section for how to scale down.

## How to compile
**IMPORTANT:** We use git submodules to download dependencies! Downloading the git repository as a .zip (rather than using git clone) will ensure you lack required dependencies, and the build scripts will fail.

After cloning the repository:
- Run `setup_vs2022.bat`
- Open `build/window-vs2022/Falcor.sln` and the `Build Solution` in the `Release` configuration

## Run the demo
- Before running the program, download the Bistro scene demo from [Google Drive](https://drive.google.com/file/d/1tRyFISyMozNMlHVF1q_iTNbpRvpEgyTp/view?usp=sharing).
- Unzip and put everything under `data\Bistro\`.
- Execute `RunAreaReSTIR_Bistro.bat`.
- The GUI contains self-explanatory settings to turn on/off different components of Area ReSTIR.
    - To quickly switch between baseline ReSTIR DI and Area ReSTIR, press key "1" to use ReSTIR DI and key "2" to use Area ReSTIR.
    - If you're experiencing out of memory issue, add `"optimizeShift2RIS" : False` at the end of `areaReSITROptions` inside `PathTracerAreaReSTIR_BistroDemo.py`. In this case, the memeory cost can be reduce, but performance will be worse.

## Test with more scenes
- You can test your custom scene by running Bin\x64\Release\Mogwai.exe first, then load `PathTracerAreaReSTIR.py`, and finally load a scene file.
- See `BistroExteriorOrigin.pyscene` for how to create a camera with custom apertures and animation path ([more details](docs/usage/scripting.md)).


## Denoising

If you really want to see the denoised results, we also include the SVGF denoised results in another script `PathTracerAreaReSTIR_SVGFDenoised_BistroDemo`, though it's **not recommended**. This is because modern denoiser is not designed to remove the noise from a noisy G-Buffer (i.e. different subpixel and lens location in our case).

So, to minimize the artifacts, the above script only enables AA and Area ReSTIR only reuses subpixel location.

If you encouter any shader errors when loading the scripts, try following solutions:
- Update your driver to the latest.
- Set `"computeDerivativeMaually : True"` in the script.
