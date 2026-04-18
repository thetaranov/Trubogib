// ─────────────────────────────────────────────────────────────────────
// step_import.cpp — STEP import using OpenCASCADE
// ─────────────────────────────────────────────────────────────────────
#include "step_import.h"

#include <iostream>
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

#ifdef _WIN32
#include <windows.h>
#endif

// OCCT includes
#include <STEPControl_Reader.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopExp_Explorer.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Geom_Plane.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Torus.hxx>
#include <gp_Pln.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopExp.hxx>

// ─── Helpers ────────────────────────────────────────────────────────
namespace {

struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    Vec3(const gp_Pnt& p) : x(p.X()), y(p.Y()), z(p.Z()) {}
    Vec3(const gp_Dir& d) : x(d.X()), y(d.Y()), z(d.Z()) {}
    Vec3(const gp_Vec& v) : x(v.X()), y(v.Y()), z(v.Z()) {}
    double len() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const { double l = len(); return l > 1e-12 ? Vec3(x/l, y/l, z/l) : Vec3(); }
    double dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const { return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x}; }
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double s) const { return {x*s, y*s, z*s}; }
    double dist(const Vec3& o) const { return (*this - o).len(); }
};

struct CylInfo {
    Vec3 origin, direction;
    double radius = 0;
    int faceIdx = -1;
};

struct TorusInfo {
    Vec3 center, axis;
    double majorR = 0, minorR = 0;
    int faceIdx = -1;
};

struct PlaneInfo {
    Vec3 origin, normal;
    int faceIdx = -1;
};

struct Segment {
    Vec3 start, end, direction;
    double length = 0;
    bool isBend = false;
    double bendAngle = 0;    // degrees
    double bendRadius = 0;   // R (CLR)
    int cylinderFace = -1;
    std::vector<int> adjacentPlaneFaces; // plane faces adjacent to this cylinder
};

} // anon namespace

// ─── Import ─────────────────────────────────────────────────────────
BendingProgram importStep(const std::string& filepath)
{
    BendingProgram prog;

    // 1. Read STEP file
    STEPControl_Reader reader;

    // Convert UTF-8 path to short 8.3 path (guaranteed ASCII) for OCCT
    std::string readPath = filepath;
#ifdef _WIN32
    {
        int wlen = MultiByteToWideChar(CP_UTF8, 0, filepath.c_str(), -1, nullptr, 0);
        if (wlen > 0) {
            std::wstring wpath(wlen, L'\0');
            MultiByteToWideChar(CP_UTF8, 0, filepath.c_str(), -1, wpath.data(), wlen);
            wchar_t shortPath[MAX_PATH] = {};
            DWORD shortLen = GetShortPathNameW(wpath.c_str(), shortPath, MAX_PATH);
            if (shortLen > 0 && shortLen < MAX_PATH) {
                int nLen = WideCharToMultiByte(CP_ACP, 0, shortPath, -1, nullptr, 0, nullptr, nullptr);
                std::string narrow(nLen - 1, '\0');
                WideCharToMultiByte(CP_ACP, 0, shortPath, -1, narrow.data(), nLen, nullptr, nullptr);
                readPath = narrow;
            }
        }
    }
#endif
    auto status = reader.ReadFile(readPath.c_str());
    if (status != IFSelect_RetDone) {
        std::cerr << "[STEP Import] Failed to read file: " << filepath << "\n";
        return prog;
    }
    reader.TransferRoots();
    TopoDS_Shape shape = reader.OneShape();
    std::cout << "[STEP Import] File loaded: " << filepath << "\n";

    // 2. Classify all faces: cylinders, tori, planes
    std::vector<CylInfo>   cylinders;
    std::vector<TorusInfo> tori;
    std::vector<PlaneInfo> planes;

    TopTools_IndexedMapOfShape faceMap;
    TopExp::MapShapes(shape, TopAbs_FACE, faceMap);

    for (int i = 1; i <= faceMap.Extent(); i++) {
        const TopoDS_Face& face = TopoDS::Face(faceMap(i));
        BRepAdaptor_Surface surf(face);

        switch (surf.GetType()) {
        case GeomAbs_Cylinder: {
            gp_Cylinder cyl = surf.Cylinder();
            auto ax = cyl.Position();
            CylInfo ci;
            ci.origin    = Vec3(ax.Location());
            ci.direction = Vec3(ax.Direction());
            ci.radius    = cyl.Radius();
            ci.faceIdx   = i;
            cylinders.push_back(ci);
            std::cout << "  Cylinder #" << i << " R=" << ci.radius
                      << " at (" << ci.origin.x << "," << ci.origin.y << "," << ci.origin.z << ")\n";
            break;
        }
        case GeomAbs_Torus: {
            gp_Torus tor = surf.Torus();
            auto ax = tor.Position();
            TorusInfo ti;
            ti.center = Vec3(ax.Location());
            ti.axis   = Vec3(ax.Direction());
            ti.majorR = tor.MajorRadius();
            ti.minorR = tor.MinorRadius();
            ti.faceIdx = i;
            tori.push_back(ti);
            std::cout << "  Torus #" << i << " R_maj=" << ti.majorR << " R_min=" << ti.minorR << "\n";
            break;
        }
        case GeomAbs_Plane: {
            gp_Pln pln = surf.Plane();
            PlaneInfo pi;
            pi.origin  = Vec3(pln.Location());
            pi.normal  = Vec3(pln.Axis().Direction());
            pi.faceIdx = i;
            planes.push_back(pi);
            std::cout << "  Plane #" << i << " N=(" << pi.normal.x << "," << pi.normal.y << "," << pi.normal.z << ")\n";
            break;
        }
        default: break;
        }
    }

    // 3. Pair cylinders (outer + inner by co-axial check)
    struct CylPair { CylInfo outer, inner; };
    std::vector<CylPair> cylPairs;
    std::set<int> usedCyl;

    for (size_t i = 0; i < cylinders.size(); i++) {
        if (usedCyl.count(i)) continue;
        for (size_t j = i + 1; j < cylinders.size(); j++) {
            if (usedCyl.count(j)) continue;
            auto& a = cylinders[i];
            auto& b = cylinders[j];
            auto da = a.direction.normalized();
            auto db = b.direction.normalized();
            if (std::abs(std::abs(da.dot(db)) - 1.0) < 0.01) {
                auto diff = b.origin - a.origin;
                auto along = da * diff.dot(da);
                auto perp = diff - along;
                if (perp.len() < 1.0) {
                    CylPair cp;
                    if (a.radius >= b.radius) { cp.outer = a; cp.inner = b; }
                    else                      { cp.outer = b; cp.inner = a; }
                    cylPairs.push_back(cp);
                    usedCyl.insert(i);
                    usedCyl.insert(j);
                    break;
                }
            }
        }
    }

    // 4. Pair tori
    struct TorusPair { TorusInfo outer, inner; };
    std::vector<TorusPair> torusPairs;
    std::set<int> usedTor;
    for (size_t i = 0; i < tori.size(); i++) {
        if (usedTor.count(i)) continue;
        for (size_t j = i + 1; j < tori.size(); j++) {
            if (usedTor.count(j)) continue;
            auto& a = tori[i]; auto& b = tori[j];
            if (a.center.dist(b.center) < 1.0 && std::abs(a.majorR - b.majorR) < 1.0) {
                TorusPair tp;
                if (a.minorR >= b.minorR) { tp.outer = a; tp.inner = b; }
                else                      { tp.outer = b; tp.inner = a; }
                torusPairs.push_back(tp);
                usedTor.insert(i); usedTor.insert(j);
                break;
            }
        }
    }

    // 5. Determine D and R
    if (!cylPairs.empty()) {
        prog.D = cylPairs[0].outer.radius * 2;
    }
    if (!torusPairs.empty()) {
        prog.R = torusPairs[0].outer.majorR;
    }
    double outerR = prog.D / 2.0;

    std::cout << "[STEP Import] D=" << prog.D << " R=" << prog.R
              << " Cylinders=" << cylPairs.size()
              << " Tori=" << torusPairs.size()
              << " Planes=" << planes.size() << "\n";

    // 6. Build straight segments from cylinder pairs
    //    Determine actual tube segment endpoints by intersecting the cylinder axis
    //    with neighboring torus center spheres.
    std::vector<Segment> segments;
    for (auto& cp : cylPairs) {
        Segment seg;
        auto dir = cp.outer.direction.normalized();
        auto origin = cp.outer.origin;

        // Find extent along axis by checking adjacency with torus centers
        double tMin = -1e9, tMax = 1e9;
        bool hasMin = false, hasMax = false;

        for (auto& tp : torusPairs) {
            auto toCenter = tp.outer.center - origin;
            double t = toCenter.dot(dir);
            auto perp = toCenter - dir * t;
            if (perp.len() < tp.outer.majorR + outerR * 2) {
                if (!hasMin || t > tMin) { tMin = t; hasMin = true; }
                if (!hasMax || t < tMax) { tMax = t; hasMax = true; }
                // Actually we want the one closest to each extreme
            }
        }

        // Better approach: project torus tangent points onto cylinder axis
        // For each torus, the tangent point on the cylinder is at distance R from torus center
        std::vector<double> touchPoints;
        for (auto& tp : torusPairs) {
            auto toCenter = tp.outer.center - origin;
            double along = toCenter.dot(dir);
            auto perp = toCenter - dir * along;
            if (perp.len() < tp.outer.majorR * 2) {
                touchPoints.push_back(along);
            }
        }

        if (touchPoints.empty()) {
            // Free-floating cylinder — use some default length along axis
            seg.start = origin;
            seg.end = origin + dir * 100; // placeholder
            seg.length = 100;
        } else if (touchPoints.size() == 1) {
            // One end touches torus, other end is free
            double t = touchPoints[0];
            // Determine which side is "free" based on plane locations
            seg.start = origin + dir * t;
            seg.end = origin; // will adjust
            // Try to find extent from plane distances
            double freeT = t;
            for (auto& pl : planes) {
                auto toPlane = pl.origin - origin;
                double tPlane = toPlane.dot(dir);
                // Check if plane normal is roughly perpendicular to axis
                double nDotD = std::abs(pl.normal.dot(dir));
                if (nDotD > 0.1) { // not perpendicular — oblique or straight cut
                    if ((tPlane - t) * (tPlane - freeT) > 0 || !hasMin) {
                        freeT = tPlane;
                    }
                }
            }
            if (freeT > t) { seg.start = origin + dir * t; seg.end = origin + dir * freeT; }
            else           { seg.start = origin + dir * freeT; seg.end = origin + dir * t; }
        } else {
            std::sort(touchPoints.begin(), touchPoints.end());
            seg.start = origin + dir * touchPoints.front();
            seg.end   = origin + dir * touchPoints.back();
        }

        seg.direction = (seg.end - seg.start).normalized();
        seg.length = seg.start.dist(seg.end);
        seg.cylinderFace = cp.outer.faceIdx;
        if (seg.length > 0.1)
            segments.push_back(seg);
    }

    // 7. Sort segments along the tube chain
    // Start from the segment whose one end is NOT near any torus center ("free end")
    auto isNearTorus = [&](const Vec3& pt) -> bool {
        for (auto& tp : torusPairs) {
            if (pt.dist(tp.outer.center) < tp.outer.majorR + outerR * 2) return true;
        }
        return false;
    };

    // Simple chain sort: for 2-segment tube (1 bend), just sort by which touches torus
    if (segments.size() >= 2) {
        // Chain segments start-to-end linked through tori
        std::vector<Segment> sorted;
        std::vector<bool> used(segments.size(), false);

        // Find the segment with START not near torus as the first
        int first = -1;
        for (size_t i = 0; i < segments.size(); i++) {
            if (!isNearTorus(segments[i].start)) { first = (int)i; break; }
            if (!isNearTorus(segments[i].end))   {
                // Flip so start is the free end
                std::swap(segments[i].start, segments[i].end);
                segments[i].direction = (segments[i].end - segments[i].start).normalized();
                first = (int)i;
                break;
            }
        }
        if (first < 0) first = 0;

        sorted.push_back(segments[first]);
        used[first] = true;

        while (sorted.size() < segments.size()) {
            auto& last = sorted.back();
            int best = -1;
            double bestDist = 1e9;
            bool flip = false;
            for (size_t i = 0; i < segments.size(); i++) {
                if (used[i]) continue;
                double d1 = last.end.dist(segments[i].start);
                double d2 = last.end.dist(segments[i].end);
                // Through torus — endpoints may not match exactly
                double d = std::min(d1, d2);
                // Allow large distance because segments connect through torus arcs
                if (d < bestDist) {
                    bestDist = d;
                    best = (int)i;
                    flip = (d2 < d1);
                }
            }
            if (best < 0) break;
            if (flip) {
                std::swap(segments[best].start, segments[best].end);
                segments[best].direction = (segments[best].end - segments[best].start).normalized();
            }
            sorted.push_back(segments[best]);
            used[best] = true;
        }
        segments = sorted;
    }

    // 8. Build BendingProgram steps
    for (size_t i = 0; i < segments.size(); i++) {
        BendStep step;
        step.id = (int)i + 1;
        step.feedLength = segments[i].length;
        step.rotation = 0;
        step.bendAngle = 0;

        // Compute bend angle between consecutive segments
        if (i + 1 < segments.size()) {
            auto d1 = segments[i].direction.normalized();
            auto d2 = segments[i+1].direction.normalized();
            double cosA = d1.dot(d2);
            cosA = std::clamp(cosA, -1.0, 1.0);
            step.bendAngle = std::acos(cosA) * 180.0 / M_PI;

            // Compute rotation (dihedral angle between consecutive bend planes)
            if (i > 0) {
                auto d0 = segments[i-1].direction.normalized();
                auto n_prev = d0.cross(d1).normalized();
                auto n_curr = d1.cross(d2).normalized();
                if (n_prev.len() > 0.01 && n_curr.len() > 0.01) {
                    double cosR = n_prev.dot(n_curr);
                    cosR = std::clamp(cosR, -1.0, 1.0);
                    double sinR = n_prev.cross(n_curr).dot(d1);
                    step.rotation = std::atan2(sinR, cosR) * 180.0 / M_PI;
                }
            }
        }

        prog.steps.push_back(step);
    }

    // 9. Detect cut planes at tube ends
    // Use OCCT topology: find plane faces that share edges with the first/last cylinder faces
    auto findAdjacentPlanes = [&](int cylFaceIdx) -> std::vector<PlaneInfo> {
        std::vector<PlaneInfo> result;
        if (cylFaceIdx < 1 || cylFaceIdx > faceMap.Extent()) return result;

        const TopoDS_Face& cylFace = TopoDS::Face(faceMap(cylFaceIdx));

        // Get all edges of the cylinder face
        TopTools_IndexedMapOfShape cylEdges;
        TopExp::MapShapes(cylFace, TopAbs_EDGE, cylEdges);

        // For each plane, check if it shares an edge with this cylinder
        for (auto& pl : planes) {
            if (pl.faceIdx < 1 || pl.faceIdx > faceMap.Extent()) continue;
            const TopoDS_Face& planeFace = TopoDS::Face(faceMap(pl.faceIdx));

            TopTools_IndexedMapOfShape planeEdges;
            TopExp::MapShapes(planeFace, TopAbs_EDGE, planeEdges);

            // Check for shared edges
            bool shared = false;
            for (int ce = 1; ce <= cylEdges.Extent() && !shared; ce++) {
                for (int pe = 1; pe <= planeEdges.Extent() && !shared; pe++) {
                    if (cylEdges(ce).IsSame(planeEdges(pe))) {
                        shared = true;
                    }
                }
            }
            if (shared) {
                result.push_back(pl);
            }
        }
        return result;
    };

    if (!segments.empty()) {
        // First segment: planes at the free end (start)
        auto planesFirst = findAdjacentPlanes(segments.front().cylinderFace);
        // Last segment: planes at the free end (end)
        auto planesLast  = findAdjacentPlanes(segments.back().cylinderFace);

        std::cout << "[STEP Import] Planes at first segment: " << planesFirst.size()
                  << ", at last segment: " << planesLast.size() << "\n";

        // Convert plane normals to tube-local coordinates
        auto worldToLocal = [](const Vec3& worldNormal, const Vec3& tubeDir) -> Vec3 {
            Vec3 tubeY = tubeDir.normalized();
            Vec3 refUp = (std::abs(tubeY.z) < 0.9) ? Vec3(0,0,1) : Vec3(1,0,0);
            Vec3 tubeX = refUp.cross(tubeY).normalized();
            Vec3 tubeZ = tubeY.cross(tubeX).normalized();
            return { worldNormal.dot(tubeX), worldNormal.dot(tubeY), worldNormal.dot(tubeZ) };
        };

        // CutBefore = first segment's free-end planes (Y=0 in renderer)
        if (!planesFirst.empty()) {
            auto segDir = segments.front().direction.normalized();
            for (auto& pl : planesFirst) {
                auto localN = worldToLocal(pl.normal.normalized(), segDir);
                CutPlane cp;
                cp.nx = localN.x; cp.ny = localN.y; cp.nz = localN.z;
                cp.offsetAlongAxis = 0;
                prog.cutBefore.planes.push_back(cp);
                std::cout << "  CutBefore plane: NX=" << cp.nx << " NY=" << cp.ny << " NZ=" << cp.nz << "\n";
            }
        }

        // CutAfter = last segment's free-end planes (Y=LTotal in renderer)
        if (!planesLast.empty()) {
            double totalLen = prog.lTotal();
            auto segDir = segments.back().direction.normalized();
            for (auto& pl : planesLast) {
                auto localN = worldToLocal(pl.normal.normalized(), segDir);
                CutPlane cp;
                cp.nx = localN.x; cp.ny = localN.y; cp.nz = localN.z;
                cp.offsetAlongAxis = totalLen;
                prog.cutAfter.planes.push_back(cp);
                std::cout << "  CutAfter plane: NX=" << cp.nx << " NY=" << cp.ny << " NZ=" << cp.nz << "\n";
            }
        }
    }

    std::cout << "[STEP Import] Program: " << prog.steps.size() << " steps, D=" << prog.D
              << " R=" << prog.R << " LTotal=" << prog.lTotal() << "\n";
    return prog;
}
