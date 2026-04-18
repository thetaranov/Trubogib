#pragma once
#include "bending_program.h"
#include <vector>
#include <array>

// ─── Mesh data for OpenGL rendering ─────────────────────────────────
struct Vertex {
    float x, y, z;       // position
    float nx, ny, nz;    // normal
};

struct TubeSegmentMesh {
    std::vector<Vertex>   vertices;
    std::vector<uint32_t> indices;
    int   stepId = 0;
    enum Type { Blue, Red, Green, Gray } type = Red;
};

// ─── Build tube geometry ────────────────────────────────────────────
/// Build renderable tube mesh from a BendingProgram.
/// Uses the same kinematic algorithm as v1.1 (reverse order processing).
/// If maxSteps >= 0, only process steps [0..maxSteps-1] (for animation).
/// If partialFrac in (0,1), the last included step is partially built.
std::vector<TubeSegmentMesh> buildTubeGeometry(
    const BendingProgram& prog,
    double fullLTotal = -1,
    int maxSteps = -1,
    double partialFrac = 1.0);
