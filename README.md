# MeshPolygonization
## Introduction

MeshPolygonization is the implementation of the MVS (Multi-view Stereo) 
building mesh simplification method described in the following 
[paper](https://www.sciencedirect.com/science/article/pii/S0924271620301969):
```
Vasileios Bouzas, Hugo Ledoux, and  Liangliang Nan.
Structure-aware Building Mesh Polygonization. 
ISPRS Journal of Photogrammetry and Remote Sensing. 167(2020), 432-442, 2020.
```
Please consider citing the above paper if you use the code/program (or part of it). 

---

The main characteristic of this method is structure awareness — namely, the recovery 
and preservation, for the input mesh, of both its primitives and the interrelationships 
between them (their configuration in 3D space). This awareness asserts that the 
resulting mesh closely follows the original and at the same time, dictates the 
geometric operations needed for its construction in the first place — thus providing 
accuracy, along with computational efficiency.

The proposed methodology consists of three main stages: (a) extracting planar primitives 
via mesh segmentation, (b) encoding primitive interrelationships in a structure graph, and
(c) mesh polygonization. In particular, polygonization is accomplished here by 
approximating the primitive borders with a building scaffold, out of which a set of 
candidate faces is defined. The selection of faces from the candidate set to form 
the simplified mesh is achieved through a linear binary programming formulation, in which
certain hard constraints are enforced to ensure that the result is manifold and watertight.

![](images/overview.png)



## Structure
This repository is structured as follows:
  - The **data** directory stores some mesh models of urban buildings. The results will also 
    be written into this directory by default.
  - The **src** directory contains the source code.
  
## Build

*MeshPolygonization* depends on CGAL. Please make sure CGAL exists on your machine before you build 
the program. During the development of *MeshPolygonization*, [CGAL v4.13](https://github.com/CGAL/cgal/releases/tag/releases/CGAL-4.13) was
 used, and later [CGAL v5.1](https://github.com/CGAL/cgal/releases/tag/v5.1) has also been tested. 
 Newer versions should also work.

To build *MeshPolygonization*, you need [CMake](https://cmake.org/download/) (`>= 3.1`) and of course a compiler 
that supports `C++11 (or higher)`.

*MeshPolygonization* has been tested on macOS (Xcode >= 8), Windows (MSVC >=2015), and 
Linux (GCC >= 4.8, Clang >= 3.3). Machines nowadays typically provide higher 
[supports](https://en.cppreference.com/w/cpp/compiler_support), so you should be 
able to build *MeshPolygonization* on almost all platforms.

There are many options to build *MeshPolygonization*. Choose one of the following (or 
whatever you are familiar with):

- Option 1: Use any IDE (e.g., [CLion](https://www.jetbrains.com/clion/) or 
[QtCreator](https://www.qt.io/product)) that can directly handle CMakeLists files to open 
the `CMakeLists.txt` file in the root directory. Then you should have obtained a usable project 
and just build it.
- Option 2: Use CMake to generate project files for your IDE. Then load the project to your IDE and 
build it.
- Option 3: Use CMake to generate Makefiles and then `make` (on Linux/macOS) or `nmake`(on Windows with Microsoft 
Visual Studio).

Don't have any experience with C/C++ programming? Have a look at [How to build *MeshPolygonization* 
step by step](./How_to_build.md).


## About the parameters

The distance & importance threshold are related to our segmentation technique which dissolves the input mesh into planar segments.

- **distance threshold**: It defines the maximum allowed distance of a mesh vertex to the fitted plane during the planar decomposition step. If this threshold is satisfied, then the vertex is considered as part of the planar region. In general, you will need to choose this parameter for the plane extraction step such that necessary planes capturing the desired geometric features can be obtained, which is, however, usually not obvious due to different levels of complexity, noise, outliers, and missing data. In practice, I would suggest starting from a coarse model and then tuning the parameters to add more (but limited) details.

- **importance threshold**: It defines which planar regions should be considered for further processing after the segmentation. In other segmentation techniques, it is a function of the number of faces out of which a planar region consists. Here, we prefer to define it as the proportion of the area for a given planar region to the entire mesh area.

Apart from the distance threshold (the program already provides a suggested value), there are not really any recommendations on the importance value. Of course, it should be in the range (0, 100) - in other words, from 0% of the total mesh area up to 100%. To get an idea on how you can play around with these parameters and how the algorithm actually works, I urge you to first run some of the examples in the [data](https://github.com/VasileiosBouzas/MeshPolygonization/tree/master/data) directory where each one is provided with some tested parameters.

For more theoretical details, please refer to the original paper.

