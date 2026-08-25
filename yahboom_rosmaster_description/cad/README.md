# ROSMASTER X3 CAD visual source

`rosmaster_unified.glb` is the canonical source for the simulator's generated
visual meshes. It is an assembled, material-bearing derivative of Yahboom's
official `520 Motor chassis Mecanum wheel-X3.STEP` model with the LiDAR and
RGB-D camera used by the challenge website.

Source record:

- GLB SHA-256: `5db7240c8e8ea715ac7a30f89547ab6e338cacadd650b0c93eff36f601a9446c`
- Website repository revision: `AIRclub-UdeSA/jar_site@0becc46e35a900604efe2efa9379f9e6ca405cca`
- Yahboom archive SHA-256: `616c5950610bfe2aefd42435b00739bc47be75077ade0df665ac082968b578cf`
- Selected archive member: `Pendulum suspension-Medium/520 Motor chassis Mecanum wheel-X3.STEP`
- Selected STEP SHA-256: `c61cdfd6cc5ec0f8190e6651dcfe9548c8c52fd889fbeb497ba144db17d8dbb1`

The original STEP file and complete archive are intentionally not committed.
The GLB is developer input and is not installed with the ROS package. Runtime
uses the generated OBJ/MTL files under `meshes/rosmaster_x3/cad_visual`.

The converter first merges the source into nine link/material sets. It writes
those sets as 27 deterministic OBJ groups capped at 20,000 triangles each,
because the Assimp import path used by Gazebo and RViz expands OBJ corners and
requires each imported submesh to stay below the 16-bit vertex-index limit.
This compatibility split does not decimate or otherwise remove CAD geometry.

To reproduce the runtime assets from the package directory:

```bash
python3 -m venv .cad-venv
.cad-venv/bin/pip install -r tools/requirements-cad.txt
.cad-venv/bin/python tools/convert_rosmaster_cad.py
```

Use `--check` to regenerate into a temporary directory and byte-compare the
result with the committed assets.
