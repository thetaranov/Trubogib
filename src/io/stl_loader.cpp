// ─────────────────────────────────────────────────────────────────────
// stl_loader.cpp — Binary/ASCII STL file loader
// ─────────────────────────────────────────────────────────────────────
#include "stl_loader.h"
#include <fstream>
#include <cstring>
#include <cstdint>
#include <sstream>
#include <algorithm>
#include <iostream>
#include <cfloat>

namespace fs = std::filesystem;

static bool isBinaryStl(const fs::path& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    // Read first 80 bytes (header) + 4 bytes (triangle count)
    char header[84];
    f.read(header, 84);
    if (!f) return false;
    uint32_t numTri;
    memcpy(&numTri, header + 80, 4);
    // Check file size matches binary format
    f.seekg(0, std::ios::end);
    auto size = f.tellg();
    auto expected = 84 + numTri * 50; // 50 bytes per tri
    return (size == expected) || (size == expected + 1); // some files have trailing byte
}

static bool loadBinaryStl(const fs::path& path, StlMesh& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;

    char header[80];
    f.read(header, 80);

    uint32_t numTri;
    f.read(reinterpret_cast<char*>(&numTri), 4);

    out.triangleCount = (int)numTri;
    out.vertices.reserve(numTri * 18); // 3 verts * 6 floats

    for (uint32_t i = 0; i < numTri; i++) {
        float normal[3], v1[3], v2[3], v3[3];
        f.read(reinterpret_cast<char*>(normal), 12);
        f.read(reinterpret_cast<char*>(v1), 12);
        f.read(reinterpret_cast<char*>(v2), 12);
        f.read(reinterpret_cast<char*>(v3), 12);
        uint16_t attr;
        f.read(reinterpret_cast<char*>(&attr), 2);

        // Vertex 1
        out.vertices.push_back(v1[0]); out.vertices.push_back(v1[1]); out.vertices.push_back(v1[2]);
        out.vertices.push_back(normal[0]); out.vertices.push_back(normal[1]); out.vertices.push_back(normal[2]);
        // Vertex 2
        out.vertices.push_back(v2[0]); out.vertices.push_back(v2[1]); out.vertices.push_back(v2[2]);
        out.vertices.push_back(normal[0]); out.vertices.push_back(normal[1]); out.vertices.push_back(normal[2]);
        // Vertex 3
        out.vertices.push_back(v3[0]); out.vertices.push_back(v3[1]); out.vertices.push_back(v3[2]);
        out.vertices.push_back(normal[0]); out.vertices.push_back(normal[1]); out.vertices.push_back(normal[2]);
    }
    return true;
}

static bool loadAsciiStl(const fs::path& path, StlMesh& out) {
    std::ifstream f(path);
    if (!f) return false;

    std::string line;
    float nx = 0, ny = 0, nz = 0;
    int vertInFacet = 0;

    while (std::getline(f, line)) {
        // Trim
        size_t start = line.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) continue;
        line = line.substr(start);

        if (line.rfind("facet normal", 0) == 0) {
            std::istringstream ss(line.substr(12));
            ss >> nx >> ny >> nz;
            vertInFacet = 0;
        } else if (line.rfind("vertex", 0) == 0) {
            float x, y, z;
            std::istringstream ss(line.substr(6));
            ss >> x >> y >> z;
            out.vertices.push_back(x); out.vertices.push_back(y); out.vertices.push_back(z);
            out.vertices.push_back(nx); out.vertices.push_back(ny); out.vertices.push_back(nz);
            vertInFacet++;
        } else if (line.rfind("endfacet", 0) == 0) {
            if (vertInFacet == 3) out.triangleCount++;
        }
    }
    return out.triangleCount > 0;
}

static void computeBounds(StlMesh& mesh) {
    mesh.minX = mesh.minY = mesh.minZ = FLT_MAX;
    mesh.maxX = mesh.maxY = mesh.maxZ = -FLT_MAX;
    for (size_t i = 0; i + 5 < mesh.vertices.size(); i += 6) {
        float x = mesh.vertices[i], y = mesh.vertices[i+1], z = mesh.vertices[i+2];
        if (x < mesh.minX) mesh.minX = x;
        if (y < mesh.minY) mesh.minY = y;
        if (z < mesh.minZ) mesh.minZ = z;
        if (x > mesh.maxX) mesh.maxX = x;
        if (y > mesh.maxY) mesh.maxY = y;
        if (z > mesh.maxZ) mesh.maxZ = z;
    }
}

bool loadStl(const fs::path& path, StlMesh& out) {
    out.vertices.clear();
    out.triangleCount = 0;
    bool ok = false;
    if (isBinaryStl(path)) {
        ok = loadBinaryStl(path, out);
    } else {
        ok = loadAsciiStl(path, out);
    }
    if (ok && !out.vertices.empty()) {
        computeBounds(out);
    }
    return ok;
}
