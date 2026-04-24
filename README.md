# Libera Lab

Libera Lab is a cross-platform desktop app for discovering, testing, and
previewing Libera-compatible laser controllers. It can stream generated test
patterns or ILDA files, manage controller plugins, and inspect output with live
preview and scope tools.

## OMNIA LIBERA INTER SE
### Any software. Any laser.

Libera Lab is part of the growing Libera ecosystem - interoperable tools
designed to connect lasers and software without restriction.

The laser industry has been held back by closed systems and vendor lock-in.
Libera is built to break that cycle.

## Build

This project uses CMake. Typical flow:

```sh
cmake --preset debug
cmake --build --preset debug
```

The build pulls GLFW and Dear ImGui at configure time and uses
`libs/libera-core` for controller discovery and output.
