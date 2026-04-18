#pragma once
// ─────────────────────────────────────────────────────────────────────
// step_tessellator.h — Load STEP model and tessellate to StlMesh
// Uses OCCT BRepMesh to produce solid-looking triangulated mesh
// without visible polygon edges (like Fusion 360 renders STEP).
// ─────────────────────────────────────────────────────────────────────
#include "stl_loader.h"   // StlMesh struct (pos+normal interleaved)
#include <filesystem>

/// Load a STEP file and tessellate all shapes into a single StlMesh.
/// @param path      Path to .stp/.step file (supports Unicode/Cyrillic)
/// @param out       Output mesh (same format as STL loader: 6 floats per vertex)
/// @param linDefl   Linear deflection in mm (smaller = finer mesh, default 0.5)
/// @param angDefl   Angular deflection in radians (default 0.5 ≈ 28°)
/// @return true if at least one face was tessellated
bool tessellateStep(const std::filesystem::path& path, StlMesh& out,
                    double linDefl = 0.5, double angDefl = 0.5);

/// Tessellate with binary cache — on first call tessellates and saves a .mesh_cache
/// file; on subsequent calls loads from cache if the STP file hasn't changed.
bool tessellateStepCached(const std::filesystem::path& path, StlMesh& out,
                          double linDefl = 0.5, double angDefl = 0.5);

/// Save/load mesh binary cache (internal helpers, also usable directly)
bool saveMeshCache(const std::filesystem::path& cachePath, const StlMesh& mesh,
                   uint64_t srcTimestamp);
bool loadMeshCache(const std::filesystem::path& cachePath, StlMesh& mesh,
                   uint64_t srcTimestamp);
