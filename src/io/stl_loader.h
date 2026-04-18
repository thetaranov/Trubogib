#pragma once
// ─────────────────────────────────────────────────────────────────────
// stl_loader.h — Binary/ASCII STL file loader
// ─────────────────────────────────────────────────────────────────────
#include <string>
#include <vector>
#include <filesystem>

struct StlMesh {
    // Interleaved: px, py, pz, nx, ny, nz (6 floats per vertex)
    std::vector<float> vertices;
    int triangleCount = 0;

    // Bounding box (computed after load)
    float minX = 0, minY = 0, minZ = 0;
    float maxX = 0, maxY = 0, maxZ = 0;
};

// Load an STL file (auto-detects binary vs ASCII)
// Uses std::filesystem::path for proper Cyrillic/Unicode path support on Windows
bool loadStl(const std::filesystem::path& path, StlMesh& out);
