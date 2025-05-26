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
TO BE ADDED ....

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

We support various user controls via the `conf.json` file located in the root directory.  
Running `cmake .. -DCMAKE_BUILD_TYPE=RELEASE` will create a copy of the `conf.json` file inside the `build/` directory.  
Users may freely modify the copied JSON file in the `build/` directory to experiment with different settings without affecting the default configuration in the root directory.

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

