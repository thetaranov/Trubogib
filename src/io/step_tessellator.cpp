// ─────────────────────────────────────────────────────────────────────
// step_tessellator.cpp — Load STEP model and tessellate to StlMesh
// ─────────────────────────────────────────────────────────────────────
#include "step_tessellator.h"
#include <iostream>
#include <cmath>
#include <cfloat>

#ifdef _WIN32
#include <windows.h>
#endif

// OCCT includes for STEP reading
#include <STEPControl_Reader.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopExp_Explorer.hxx>

// OCCT includes for tessellation
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

namespace fs = std::filesystem;

// ─── Helper: convert wide path to short 8.3 ASCII for OCCT reader ──
static std::string toOcctPath(const fs::path& path) {
    std::string result = path.string();
#ifdef _WIN32
    std::wstring wpath = path.wstring();
    wchar_t shortPath[MAX_PATH] = {};
    DWORD shortLen = GetShortPathNameW(wpath.c_str(), shortPath, MAX_PATH);
    if (shortLen > 0 && shortLen < MAX_PATH) {
        int nLen = WideCharToMultiByte(CP_ACP, 0, shortPath, -1, nullptr, 0, nullptr, nullptr);
        if (nLen > 0) {
            std::string narrow(nLen - 1, '\0');
            WideCharToMultiByte(CP_ACP, 0, shortPath, -1, narrow.data(), nLen, nullptr, nullptr);
            result = narrow;
        }
    }
#endif
    return result;
}

// ─── Compute smooth vertex normal by averaging face normals ─────────
// For a solid-looking mesh we compute per-vertex normals from triangulation
// rather than flat face normals.  This gives Phong-like smooth shading
// without any visible polygon edges.
static void computeSmoothedNormals(
    const Handle(Poly_Triangulation)& tri,
    const TopLoc_Location& loc,
    bool reversed,
    std::vector<gp_Vec>& normals)
{
    int nbNodes = tri->NbNodes();
    int nbTris  = tri->NbTriangles();
    normals.assign(nbNodes, gp_Vec(0, 0, 0));

    const gp_Trsf& trsf = loc.Transformation();
    bool hasTrsf = !loc.IsIdentity();

    for (int i = 1; i <= nbTris; i++) {
        int n1, n2, n3;
        tri->Triangle(i).Get(n1, n2, n3);
        if (reversed) std::swap(n2, n3);

        gp_Pnt p1 = tri->Node(n1);
        gp_Pnt p2 = tri->Node(n2);
        gp_Pnt p3 = tri->Node(n3);
        if (hasTrsf) { p1.Transform(trsf); p2.Transform(trsf); p3.Transform(trsf); }

        gp_Vec v1(p1, p2), v2(p1, p3);
        gp_Vec faceN = v1.Crossed(v2);
        double mag = faceN.Magnitude();
        if (mag > 1e-12) faceN.Divide(mag);

        // Weight by triangle area (magnitude of cross product / 2)
        normals[n1 - 1] += faceN;
        normals[n2 - 1] += faceN;
        normals[n3 - 1] += faceN;
    }

    // Normalize all accumulated normals
    for (auto& n : normals) {
        double mag = n.Magnitude();
        if (mag > 1e-12) n.Divide(mag);
        else             n = gp_Vec(0, 0, 1);
    }
}

// ─── Main tessellation function ─────────────────────────────────────
bool tessellateStep(const fs::path& path, StlMesh& out,
                    double linDefl, double angDefl)
{
    out.vertices.clear();
    out.triangleCount = 0;

    // 1. Read STEP file
    STEPControl_Reader reader;
    std::string readPath = toOcctPath(path);

    auto status = reader.ReadFile(readPath.c_str());
    if (status != IFSelect_RetDone) {
        std::cerr << "[StepTess] Failed to read: " << path.u8string() << "\n";
        return false;
    }
    reader.TransferRoots();
    TopoDS_Shape shape = reader.OneShape();
    if (shape.IsNull()) {
        std::cerr << "[StepTess] Empty shape in: " << path.u8string() << "\n";
        return false;
    }

    // 2. Tessellate the shape
    BRepMesh_IncrementalMesh mesher(shape, linDefl, Standard_False, angDefl, Standard_True);
    mesher.Perform();
    if (!mesher.IsDone()) {
        std::cerr << "[StepTess] Tessellation failed: " << path.u8string() << "\n";
        return false;
    }

    // 3. Extract triangles from each face
    int totalTris = 0;
    for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face& face = TopoDS::Face(ex.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull()) continue;

        bool reversed = (face.Orientation() == TopAbs_REVERSED);
        const gp_Trsf& trsf = loc.Transformation();
        bool hasTrsf = !loc.IsIdentity();

        int nbTris  = tri->NbTriangles();

        // Compute smooth per-vertex normals (Phong-like, no polygon edges visible)
        std::vector<gp_Vec> smoothN;
        computeSmoothedNormals(tri, loc, reversed, smoothN);

        // Reserve space
        out.vertices.reserve(out.vertices.size() + nbTris * 18);

        for (int i = 1; i <= nbTris; i++) {
            int n1, n2, n3;
            tri->Triangle(i).Get(n1, n2, n3);
            if (reversed) std::swap(n2, n3);

            gp_Pnt p1 = tri->Node(n1);
            gp_Pnt p2 = tri->Node(n2);
            gp_Pnt p3 = tri->Node(n3);
            if (hasTrsf) {
                p1.Transform(trsf);
                p2.Transform(trsf);
                p3.Transform(trsf);
            }

            const gp_Vec& sn1 = smoothN[n1 - 1];
            const gp_Vec& sn2 = smoothN[n2 - 1];
            const gp_Vec& sn3 = smoothN[n3 - 1];

            // Vertex 1
            out.vertices.push_back((float)p1.X());
            out.vertices.push_back((float)p1.Y());
            out.vertices.push_back((float)p1.Z());
            out.vertices.push_back((float)sn1.X());
            out.vertices.push_back((float)sn1.Y());
            out.vertices.push_back((float)sn1.Z());
            // Vertex 2
            out.vertices.push_back((float)p2.X());
            out.vertices.push_back((float)p2.Y());
            out.vertices.push_back((float)p2.Z());
            out.vertices.push_back((float)sn2.X());
            out.vertices.push_back((float)sn2.Y());
            out.vertices.push_back((float)sn2.Z());
            // Vertex 3
            out.vertices.push_back((float)p3.X());
            out.vertices.push_back((float)p3.Y());
            out.vertices.push_back((float)p3.Z());
            out.vertices.push_back((float)sn3.X());
            out.vertices.push_back((float)sn3.Y());
            out.vertices.push_back((float)sn3.Z());

            totalTris++;
        }
    }

    out.triangleCount = totalTris;

    // 4. Compute bounding box
    out.minX = out.minY = out.minZ =  FLT_MAX;
    out.maxX = out.maxY = out.maxZ = -FLT_MAX;
    for (size_t i = 0; i + 5 < out.vertices.size(); i += 6) {
        float x = out.vertices[i], y = out.vertices[i+1], z = out.vertices[i+2];
        if (x < out.minX) out.minX = x;
        if (y < out.minY) out.minY = y;
        if (z < out.minZ) out.minZ = z;
        if (x > out.maxX) out.maxX = x;
        if (y > out.maxY) out.maxY = y;
        if (z > out.maxZ) out.maxZ = z;
    }

    std::cout << "[StepTess] " << path.filename().u8string()
              << ": " << totalTris << " triangles, bounds ["
              << out.minX << ".." << out.maxX << "] x ["
              << out.minY << ".." << out.maxY << "] x ["
              << out.minZ << ".." << out.maxZ << "]\n";

    return totalTris > 0;
}

// ─── Binary mesh cache ──────────────────────────────────────────────
// Format: "MCSH" (4), version u32, srcTimestamp u64, triCount u32,
//         vertSize u32, bbox 6*float, vertex data

static const uint32_t CACHE_MAGIC   = 0x4843534D; // "MSCH"
static const uint32_t CACHE_VERSION = 1;

bool saveMeshCache(const fs::path& cachePath, const StlMesh& mesh, uint64_t srcTs) {
#ifdef _WIN32
    FILE* f = _wfopen(cachePath.wstring().c_str(), L"wb");
#else
    FILE* f = fopen(cachePath.string().c_str(), "wb");
#endif
    if (!f) return false;
    fwrite(&CACHE_MAGIC, 4, 1, f);
    fwrite(&CACHE_VERSION, 4, 1, f);
    fwrite(&srcTs, 8, 1, f);
    uint32_t tc = (uint32_t)mesh.triangleCount;
    uint32_t vs = (uint32_t)(mesh.vertices.size() * sizeof(float));
    fwrite(&tc, 4, 1, f);
    fwrite(&vs, 4, 1, f);
    float bbox[6] = {mesh.minX, mesh.minY, mesh.minZ, mesh.maxX, mesh.maxY, mesh.maxZ};
    fwrite(bbox, sizeof(bbox), 1, f);
    if (!mesh.vertices.empty())
        fwrite(mesh.vertices.data(), 1, vs, f);
    fclose(f);
    std::cout << "[Cache] Saved " << cachePath.filename().u8string() << " (" << tc << " tris)\n";
    return true;
}

bool loadMeshCache(const fs::path& cachePath, StlMesh& mesh, uint64_t srcTs) {
#ifdef _WIN32
    FILE* f = _wfopen(cachePath.wstring().c_str(), L"rb");
#else
    FILE* f = fopen(cachePath.string().c_str(), "rb");
#endif
    if (!f) return false;
    uint32_t magic = 0, ver = 0;
    uint64_t ts = 0;
    fread(&magic, 4, 1, f);
    fread(&ver, 4, 1, f);
    fread(&ts, 8, 1, f);
    if (magic != CACHE_MAGIC || ver != CACHE_VERSION || ts != srcTs) {
        fclose(f);
        return false; // stale or incompatible cache
    }
    uint32_t tc = 0, vs = 0;
    fread(&tc, 4, 1, f);
    fread(&vs, 4, 1, f);
    float bbox[6];
    fread(bbox, sizeof(bbox), 1, f);
    mesh.minX = bbox[0]; mesh.minY = bbox[1]; mesh.minZ = bbox[2];
    mesh.maxX = bbox[3]; mesh.maxY = bbox[4]; mesh.maxZ = bbox[5];
    mesh.triangleCount = (int)tc;
    size_t numFloats = vs / sizeof(float);
    mesh.vertices.resize(numFloats);
    if (numFloats > 0)
        fread(mesh.vertices.data(), 1, vs, f);
    fclose(f);
    std::cout << "[Cache] Loaded " << cachePath.filename().u8string() << " (" << tc << " tris)\n";
    return true;
}

bool tessellateStepCached(const fs::path& path, StlMesh& out,
                          double linDefl, double angDefl) {
    // Compute cache path: same directory, .mesh_cache extension
    fs::path cacheDir = path.parent_path() / L"cache";
    try { fs::create_directories(cacheDir); } catch (...) {}
    fs::path cachePath = cacheDir / (path.stem().wstring() + L".mesh_cache");

    // Get source file timestamp
    uint64_t srcTs = 0;
    try {
        auto ftime = fs::last_write_time(path);
        srcTs = (uint64_t)ftime.time_since_epoch().count();
    } catch (...) {}

    // Try loading from cache
    if (loadMeshCache(cachePath, out, srcTs))
        return out.triangleCount > 0;

    // Cache miss: tessellate and save
    if (!tessellateStep(path, out, linDefl, angDefl))
        return false;

    saveMeshCache(cachePath, out, srcTs);
    return true;
}