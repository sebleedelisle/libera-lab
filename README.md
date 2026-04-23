# Libera Lab

Libera Lab is a cross-platform desktop app for discovering, testing, and
previewing Libera-compatible laser controllers. It can stream generated test
patterns or ILDA files, manage controller plugins, and inspect output with live
preview and scope tools.

## OMNIA LIBERA INTER SE
### Any software. Any Laser.

Libera Lab is part of the Libera series of apps and open source code, with the
mission to improve interoperability between lasers and software.

## Build

This project uses CMake. Typical flow:

```sh
cmake --preset debug
cmake --build --preset debug
```

The build pulls GLFW and Dear ImGui at configure time and uses
`libs/libera-core` for controller discovery and output.
