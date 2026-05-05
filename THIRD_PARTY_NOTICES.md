# Third-Party Notices

This repository is GPLv3-licensed at the top level. That license applies to
project-authored files unless a subdirectory, bundled dependency, generated
asset, or other file-specific notice states otherwise.

Included third-party components keep their own licenses:

- `libs/libera-core` is a git submodule with its own `LICENSE` file.
- `libs/libera-core/libs/helios_dac` contains mixed-license material. Its host
  SDK is MIT-licensed, but its hardware and firmware subdirectories include
  additional non-commercial restrictions. See
  `libs/libera-core/libs/helios_dac/LICENSE.md` for the authoritative terms.
- Build-time dependencies fetched by CMake, such as GLFW and Dear ImGui, remain
  under their respective upstream licenses.
- Bundled/generated font files in `fonts/` and ILDA test patterns in
  `patterns/` retain any applicable upstream or asset-specific terms.

Before redistributing binaries or vendoring submodule contents, review the
licenses shipped with those dependencies.
