# Monte Carlo Path Tracer with Microfacet BSDF

## Project Description
Physically based rendering with global illumination inspired by [Srinath Ravichandran's Dartmouth Spring 2016 Rendering Competition submission](https://www.cs.dartmouth.edu/~rendering-competition/sp2016/submissions/srinathravichandran/final.html)

## Team Members
- **Junhao Liu** (COMP8610 / u7778766)
- **Qingchuan Rui** (COMP8610 / u7776331)
- **Jiaxin Xie** (COMP4610 / u8153316)
- **Jinghang Li** (COMP4610 / u8162481)

## Key Features
- We built on top of the code framework provided by [UCSB GAMES101 Assignment 6](https://sites.cs.ucsb.edu/~lingqi/teaching/games101.html), and added modifications to implement our Monte Carlo path tracing with BVH acceleration.
- Microfacet BSDF for realistic material rendering, inspired by the paper: ["Microfacet Models for Refraction through Rough Surfaces"](https://www.graphics.cornell.edu/~bjw/microfacetbsdf.pdf)
- Depth of Field implementation inspired by [this blog](https://blog.demofox.org/2018/07/04/pathtraced-depth-of-field-bokeh/)
- [Gem-like refraction inspired by Josh Wiseman’s 2005 CS348B Final Project](https://graphics.stanford.edu/courses/cs348b-competition/cs348b-05/gems2/index.html)

## Final Rendering Result
Image Result Rendered in 2 hours on CPU, using 8-threaded OpenMP parallel Programming with specs: 

1920x1080, spp=2048, direct light sample = 32, Russian Roulette = 0.4, depth of field enabled
![final_render_result_with_dof](https://github.com/user-attachments/assets/3ed90044-718c-4778-9419-fbfd3bba1bd0)

Image Result Rendered in 2 hours and 10 minutes on CPU, using 8-threaded OpenMP parallel Programming with specs: 

1920x1080, spp=2048, direct light sample = 32, Russian Roulette = 0.4, depth of field disabled
![final_render_result_dark_without_dof](https://github.com/user-attachments/assets/60c3b617-04d4-4a52-ab85-825ec163bd50)

We have a DEMO mode for testing inside cornell box settings. You may replicate the results from the experiment section of our report using `make DEMO=1 && ./RayTracing`.  
<img width="373" alt="cornellbox_demo" src="https://github.com/user-attachments/assets/8429e95e-f5f7-4447-bb2e-d59852eb6374" />


## Installation / Build Instructions

```bash
# Clone the repository and cd into the repository root

# install openMP to your computer

# Build the project
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RELEASE

# Build options:
make DEMO=1    # Build with demo mode (material testing)
make            # Build for final scene (chessboard)

# Specify your scene configurations according to the usage section below.

# Render image:
./RayTracing
```
## Usage

We support various user controls via the `conf.json` file located in the root directory. Rendering result is named `output.png`, and can be found under build.

Running `cmake .. -DCMAKE_BUILD_TYPE=RELEASE` will create a copy of the `conf.json` file inside the `build/` directory. Users may freely modify the copied JSON file in the `build/` directory to experiment with different settings without affecting the default configuration in the root directory.

### Camera Settings (`camera` section in `conf.json`)
- `"width"` / `"height"`: screen resolution
- `"fov"`: field of view
- `"position"`: camera position in 3D space
- `"target"`: direction the camera is looking at
- `"useDOF"`: enable or disable depth of field
- `"focusDistance"`: distance at which objects appear in focus
- `"apertureRadius"`: size of aperture affecting DoF strength

### Render Quality
- `"spp"` (samples per pixel): controls image quality
  - Default: `2048` (used for final render)
  - For quicker preview renders, use lower values (e.g., `32`, `128`, `256`)

### Scene Settings (`scene` section in `conf.json`)
- Move the main objects’ positions
- Change objects’ surface materials or texture maps  
  - Supported materials:
    - `rough_red_conductor`
    - `rough_white_conductor`
    - `gold_conductor`
    - `silver_mirror`
    - `green_mirror`
    - `smooth_glass`
    - `clear_rough_plastic`
    - `rough_plastic`
- `"includeShadow"`: toggle shadow effect on/off
- `"lightPosition"` / `"lightBrightness"`: control the light source’s location and intensity
- `"envMap"`: change the environment map for image-based lighting

