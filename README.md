# Scan3D

Scan3D is a desktop application for processing 3D scan data. It loads point clouds and meshes, cleans and registers them, reconstructs surfaces, textures them, and renders the result with OpenGL.

It is written in C++11 with Qt5 and OpenGL and builds with CMake on Windows, macOS and Linux. The current version is 2.2.0.

## Features

| Area | What you can do |
| --- | --- |
| **File I/O** | Read and write PLY, OBJ, OFF, STL, POLY, 3MF meshes, and PLY, OBJ, BIN, PTX, LAS/LAZ, XYZ/ASC/BXYZ, VG/BVG point clouds |
| **Point cloud cleaning** | Down-sampling, statistical outlier removal, radius filter, bilateral filter |
| **Normals and features** | Estimate, re-orient, invert and normalize normals; estimate curvature; compute FPFH features |
| **Registration** | Align two point clouds (PPF matching + ICP) |
| **Reconstruction** | Poisson surface reconstruction (with density-based trimming), Delaunay surface reconstruction |
| **Mesh processing** | Isotropic remeshing, mesh fixing (MeshFix), invert orientation, detect/remesh self-intersections (requires CGAL) |
| **Texturing** | Texture a mesh from calibrated photos using OpenMVS (seam leveling, atlas packing) |
| **Rendering** | Points, lines and surfaces with SSAO, shadows, eye-dome lighting, transparency, clipping planes, snapshots |
| **Interaction** | Rectangle/lasso/click selection on point clouds and meshes, model list, per-model rendering settings |

## Repository layout

```
Scan3D/
├── Scan3D_ui/        Qt5 application: main window, OpenGL canvas, dialogs, widgets, selection tools
├── module/           Static libraries (namespace scan3d)
│   ├── util/         logging (glog), file system, progress, timers
│   ├── core/         Model, PointCloud, SurfaceMesh (halfedge), Graph, property arrays, math types
│   ├── fileio/       readers/writers, chosen by file extension
│   ├── kdtree/       k-d tree search (ANN, ETH, FLANN, nanoflann back-ends)
│   ├── algo/         filters, normals, registration, reconstruction, remeshing, texturing
│   ├── algo_ext/     CGAL-based self-intersection tools (built only when CGAL is found)
│   ├── renderer/     OpenGL drawables, camera, shaders, effects
│   ├── gui/          picking of models, points and faces
│   └── viewer/       standalone GLFW + ImGui viewer (not used by the app)
├── resources/        GLSL shaders, colormaps, fonts (copied next to the executable at build time)
├── 3rd_party/        18 vendored libraries (OpenMVS, lib3mf, Eigen, glog, LAStools, Poisson Recon, MeshFix, ...)
└── cmake/            Find helpers for Qt5, OpenMP and CGAL
```

### Architecture

The app is a Qt5 shell on top of nine static libraries. Each layer links only to the layers below it:

```
Scan3D_ui (MainWindow, PaintCanvas, dialogs, widgets, tools)
    │
    ├── renderer ── gui            presentation
    ├── algo ── algo_ext ── fileio ── kdtree    processing and I/O
    ├── core ── util               foundation
    └── 3rd_party + system libraries
```

One exception: `renderer` also links `algo` and `fileio`. Removing that link is on the cleanup list.

Every processing step runs the same way:

1. A dialog reads the parameters.
2. The matching class in `module/algo` runs.
3. The algorithm either changes the current model in place (for example, it adds a `v:normal` property) or creates a new model (for example, Poisson outputs a new mesh).
4. `Renderer::update()` rebuilds the GPU buffers and `PaintCanvas` repaints.

Steps run synchronously on the UI thread. Some algorithms use OpenMP internally. The window does not respond until a step finishes.

## Installation

### 1. Prerequisites

| Dependency | Required | Notes |
| --- | --- | --- |
| C++11 compiler | Yes | MSVC 2017+, Clang or GCC |
| CMake ≥ 3.1 | Yes | A recent CMake (3.16+) is recommended |
| Qt 5 (Core, Gui, Widgets, OpenGL) | Yes, for the app | Without Qt, only the libraries are built |
| Boost (iostreams, system, serialization, program_options) | Yes | Used by the vendored OpenMVS |
| Eigen3 | Yes | Used by OpenMVS (a copy is also vendored for the rest of the code) |
| OpenCV | Yes | Used by OpenMVS and by mesh texturing |
| libpng, libjpeg, libtiff | Recommended | Image formats for texturing |
| OpenMP | Optional | Multithreads normals, filters, Poisson and picking |
| CGAL (+ GMP, MPFR) | Optional | Enables self-intersection detection and remeshing |
| GPU with OpenGL 4.3 core profile | Yes | The app requests a 4.3 core context at startup |

All other libraries (glew, glfw, imgui, glog, LAStools, rply, lib3mf, triangle, libtess, Poisson Recon, MeshFix, ...) are in `3rd_party/` and built from source.

### 2. Install the dependencies

The top-level `CMakeLists.txt` expects a toolchain file in `CMAKE_TOOLCHAIN_FILE`. The easiest setup is [vcpkg](https://github.com/microsoft/vcpkg) on every platform:

```bash
git clone https://github.com/microsoft/vcpkg.git
./vcpkg/bootstrap-vcpkg.sh          # Windows: .\vcpkg\bootstrap-vcpkg.bat

./vcpkg/vcpkg install qt5-base boost-iostreams boost-system boost-serialization \
    boost-program-options eigen3 opencv libpng libjpeg-turbo tiff
# optional
./vcpkg/vcpkg install cgal
```

On Windows, add `--triplet x64-windows` to the install command.

You can also use system packages instead:

- **macOS (Homebrew):** `brew install qt@5 boost eigen opencv libomp cgal`
- **Ubuntu/Debian:** `sudo apt install qtbase5-dev libboost-all-dev libeigen3-dev libopencv-dev libpng-dev libjpeg-dev libtiff-dev libcgal-dev`

If you use system packages instead of vcpkg, set `CMAKE_TOOLCHAIN_FILE` to an empty value (see below), and point CMake to Qt with `-DQt5_DIR=...` or `-DQT5_ROOT_PATH=...` if it is not found automatically.

### 3. Configure and build

Build in a separate folder. CMake refuses to build inside the source folder.

```bash
git clone https://github.com/WinterCodeForEverything/Scan3D.git
cd Scan3D
mkdir build && cd build

# with vcpkg
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_TOOLCHAIN_FILE=/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake

# or with system packages
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE= \
         -DQt5_DIR=$(brew --prefix qt@5)/lib/cmake/Qt5     # macOS example

cmake --build . --config Release -j 8
```

Useful CMake options:

| Option | Default | Effect |
| --- | --- | --- |
| `3DMODEL_ENABLE_QT` | `ON` | Build the Qt application |
| `QT5_ROOT_PATH` | empty | Qt5 root folder (the one that contains `bin/`) |
| `CGAL_DIR` | empty | Folder with `CGALConfig.cmake`, if CGAL is not found automatically |

On Windows you can also open the generated `Scan3D.sln` in Visual Studio and build the `Scan3D_ui` target.

### 4. Output

- The executable is in `build/bin/` (on macOS: `build/bin/Scan3D_ui.app`).
- Static libraries are in `build/lib/`.
- `resources/` (shaders, colormaps, fonts) is copied into `build/` when you run CMake. The app looks for a `resources` folder next to the executable and up to five parent folders. Keep that folder with the executable when you move or package the app. If you edit shaders, re-run CMake to copy them again.

## Usage

### Start the app

```bash
./build/bin/Scan3D_ui                       # Linux
open build/bin/Scan3D_ui.app                # macOS
build\bin\Release\Scan3D_ui.exe             # Windows (Visual Studio)
```

Log messages (glog) are written to the console.

### Typical workflow

1. **Open data:** use **File → Open** (`Ctrl+O`) or drag and drop files onto the window. You can open several files at once. Each file appears in the model list, and the selected model is the *current model* that menu actions apply to.
2. **Clean the point cloud:** **Point Cloud → Down sampling**, **Statistic outlier removal**, **Radius filter** or **Bilateral filter**.
3. **Compute normals:** **Point Cloud → Estimate normals**, then **Re-orient normals** so they point consistently outwards. Use **Invert normals orientation** if they point inwards. Poisson reconstruction needs good normals.
4. **Align scans (optional):** **Point Cloud → Compute feature FPFH** and **Point cloud registration** align two point clouds.
5. **Reconstruct a surface:** **Point Cloud → Poisson surface reconstruction** (the default octree depth is 8; higher means more detail and more time), then **Trim** to remove low-density surface. You can also use **Delaunay surface reconstruction**. The result is added as a new mesh.
6. **Repair the mesh:** **Surface Mesh → Remeshing**, **Remesh fixing**, **Detect / Remesh self-intersections** (the self-intersection tools need CGAL).
7. **Texture (optional):** **Surface Mesh → Texturing**. Pick a working folder that contains an `images/` folder with the photos and a `camera.xml` (OpenCV FileStorage format with `camIntrinsics`, `imageSize` and `camExternals`), plus a relative-pose `.txt` file. The mesh must have been loaded from a file. The textured mesh is saved as `<name>_texture.obj`.
8. **Save:** use **File → Save** (`Ctrl+S`). The writer is chosen by file extension, and PLY is the default.

Each step can take seconds to minutes on large data. The window does not respond while a step runs.

### Menus

| Menu | Actions |
| --- | --- |
| File | Open, Save, Recent files, Exit |
| View | Snapshot, Copy/Paste camera, Save/Restore camera state, Background color, Settings |
| Point Cloud | Down sampling · Estimate / Re-orient / Invert / Normalize normals · Poisson / Delaunay surface reconstruction · Statistic outlier removal, Radius filter, Bilateral filter · Estimate curvature · Compute feature FPFH, Point cloud registration |
| Surface Mesh | Invert surface orientation, Remeshing · Remesh fixing, Detect / Remesh self-intersections · Texturing |
| Help | Manual (`F1`), About |

The menus also show *RANSAC primitive extraction* and *Delaunay triangulation 2D/3D*, but these actions are not connected yet.

### Mouse and keyboard

Press `F1` in the app to print the full list.

| Input | Action |
| --- | --- |
| Left drag | Rotate the camera |
| Right drag | Pan |
| Wheel / middle drag, `Ctrl` + `+`/`-` | Zoom |
| Arrow keys (± `Ctrl`) | Turn / move the camera |
| `f` / `c` | Fit all models / the current model on screen |
| `z` + left click | Zoom to the clicked point |
| `Shift` + left / right click | Set / clear the rotation target point |
| `Ctrl` + left drag | Zoom to a region |
| `p` | Toggle perspective / orthographic projection |
| `<` / `>` | Switch the current model |
| `Fn` + `Delete` | Delete the current model |
| `-` / `=` | Smaller / larger points |
| `{` / `}` | Thinner / thicker lines |
| `a`, `b`, `e`, `v` | Toggle axes, borders, edges, vertices |
| `m` | Toggle smooth shading (meshes) |
| `d` | Print model info (drawables, properties) |
| `s` | Snapshot |

Use the model list on the side panel to show, hide and select models. Use the rendering panels to change colors, point size, the scalar field used for coloring, and effects such as SSAO, shadows and eye-dome lighting.

## Known limitations

- There are no automated tests or CI yet.
- The app requests OpenGL 4.3. macOS supports at most OpenGL 4.1, so some rendering features may not work there.
- Some source files are not UTF-8 (their comments are GBK-encoded Chinese) and use CRLF line endings.
- The `viewer` module and `module/algo/ICP_Princeton/ICP_main.cpp` are unused.

See the architecture analysis report for the full list of risks and planned improvements.

## Acknowledgements

Scan3D builds on [Easy3D](https://github.com/LiangliangNan/Easy3D) by Liangliang Nan. Its data structures (`PointCloud`, halfedge `SurfaceMesh`, property arrays), rendering engine (drawables, camera and manipulated frames, shaders, effects), file I/O, k-d tree wrappers and much of the Qt viewer follow Easy3D's design and code. We thank the Easy3D authors for making their work available.

Scan3D also uses the third-party libraries in `3rd_party/`, including OpenMVS, lib3mf, Eigen, glog, LAStools, Poisson Surface Reconstruction, MeshFix, Triangle, GLFW, Dear ImGui and GLEW. Each library keeps its own license in its folder.
