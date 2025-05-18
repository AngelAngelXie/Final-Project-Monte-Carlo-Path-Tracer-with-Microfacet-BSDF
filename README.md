# Graphics_Final_Project
Junhao Liu(u7778766), Qingchuan Rui(u7776331), Jiaxin Xie(u8153316), Jinghang Li(u8162481)

[Project Detail Doc](https://docs.google.com/document/d/1A8Ojkn1N-bKzjAkaoPQIdWHmSiVXNFrjWJzCkogCvlA/edit?tab=t.0)

# Monte Carlo Path Tracer with Microfacet BSDF

Demo image using 8 pre-defined material types (left). Demo image with depth of field applied (right).
<img width="767" alt="demo" src="https://github.com/user-attachments/assets/639f76d2-f49d-4f89-b7f5-c21cf826b853" />


## Project Description

This is a final project for ANU COMP 4610/8610 Computer Graphics Course, implementing a Monte Carlo path tracer featuring:

- Microfacet BSDF for realistic material rendering inspired by the paper [Microfacet Models for Refraction through Rough Surfaces](https://www.graphics.cornell.edu/~bjw/microfacetbsdf.pdf)
- Support for smooth/rough dielectric/conductor materials
- Advanced lighting effects including [depth of field](https://blog.demofox.org/2018/07/04/pathtraced-depth-of-field-bokeh/) and soft shadows

Inspired by Srinath Ravichandran's Dartmouth Spring 2016 Rendering Competition submission:  
[Dartmouth Rendering Competition Entry](https://www.cs.dartmouth.edu/~rendering-competition/sp2016/submissions/srinathravichandran/final.html)

## Features

- Physically-based rendering with global illumination
- depth of field
- environment map
- Material system supporting:
  - Rough/smooth conductors (metals)
  - Dielectrics (glass, plastic)
  - Textured floor surface
- Configurable scene elements via JSON
- Demo mode for a close up look at materials in the cornel box

## Installation & Usage

### Build Instructions

```bash
# Clone the repository and cd into the repository root

# install openMP to your computer

# Build the project
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RELEASE

# Build options:
make DEMO=1    # Build with demo mode (material testing)
make            # Build for final scene (chessboard)
```
# Notes 
- when modified conf.json located under the root directory, please run cmake .. -DCMAKE_BUILD_TYPE=RELEASE to ensure that the build folder gets a updated copy of the json file. Alternatively, you may directly modify the conf.json file located in the build directory if you would like to play around with the scene without needing to run cmake and make repeatedly.
