# Graphics_Final_Project
Junhao Liu(u7778766), Qingchuan Rui(u7776331), Jiaxin Xie(u8153316), Jinghang Li(u8162481)

[Project Detail Doc](https://docs.google.com/document/d/1A8Ojkn1N-bKzjAkaoPQIdWHmSiVXNFrjWJzCkogCvlA/edit?tab=t.0)

# Monte Carlo Path Tracer with Microfacet BSDF

Demo image using 7 pre-defined material types (left). Demo image with depth of field applied (right).

<img width="467" alt="demo_original" src="https://github.com/user-attachments/assets/0c92e1f7-ad3f-47f5-9b21-7dc7b879176e" />
<img width="467" alt="demo_dof" src="https://github.com/user-attachments/assets/3a2e464f-f3ec-4a91-a88e-23baeb374579" />


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

# Build the project
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RELEASE

# Build options:
make DEMO=1    # Build with demo mode (material testing)
make            # Build for final scene (chessboard)
