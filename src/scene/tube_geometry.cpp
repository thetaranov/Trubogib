// ─────────────────────────────────────────────────────────────────────
// tube_geometry.cpp — Build renderable tube mesh
// ─────────────────────────────────────────────────────────────────────
#include "tube_geometry.h"
#include <cmath>
#include <algorithm>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

struct V3 {
    double x = 0, y = 0, z = 0;
    V3() = default;
    V3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    double len() const { return std::sqrt(x*x + y*y + z*z); }
    V3 norm() const { double l = len(); return l > 1e-12 ? V3(x/l, y/l, z/l) : V3(); }
    V3 cross(const V3& o) const { return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x}; }
    double dot(const V3& o) const { return x*o.x + y*o.y + z*o.z; }
    V3 operator+(const V3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    V3 operator-(const V3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    V3 operator*(double s) const { return {x*s, y*s, z*s}; }
};

V3 rotateAround(const V3& v, const V3& axis, double angle) {
    V3 a = axis.norm();
    double c = std::cos(angle), s = std::sin(angle);
    double d = a.dot(v);
    V3 cr = a.cross(v);
    return a * d * (1 - c) + v * c + cr * s;
}

constexpr int CIRC_SEGMENTS = 32;
constexpr int ARC_STEPS_PER_5DEG = 1; // 1 ring per 5 degrees

// Create a cylinder mesh from start to end with given radius and color type
TubeSegmentMesh createCylinder(
    const V3& start, const V3& end, double radius,
    int stepId, TubeSegmentMesh::Type type,
    const CutDefinition* cutAtStart = nullptr,
    const CutDefinition* cutAtEnd = nullptr)
{
    TubeSegmentMesh mesh;
    mesh.stepId = stepId;
    mesh.type = type;

    auto axis = end - start;
    double len = axis.len();
    if (len < 0.001) return mesh;
    auto dir = axis.norm();

    // Frame: tubeX = cross(refUp, dir), tubeZ = cross(dir, tubeX)
    V3 refUp = (std::abs(dir.z) < 0.9) ? V3(0,0,1) : V3(1,0,0);
    V3 tubeX = refUp.cross(dir).norm();
    V3 tubeZ = dir.cross(tubeX).norm();

    // Precompute cut anchoring
    double minDeltaStart = 0, maxDeltaEnd = 0;
    if (cutAtStart && !cutAtStart->empty()) {
        double off0 = cutAtStart->planes[0].offsetAlongAxis;
        double mn = 1e9;
        for (int k = 0; k < 360; k++) {
            double t = 2 * M_PI * k / 360.0;
            mn = std::min(mn, cutAtStart->contourY(t, radius) - off0);
        }
        minDeltaStart = mn;
    }
    if (cutAtEnd && !cutAtEnd->empty()) {
        double off0 = cutAtEnd->planes[0].offsetAlongAxis;
        double mx = -1e9;
        for (int k = 0; k < 360; k++) {
            double t = 2 * M_PI * k / 360.0;
            mx = std::max(mx, cutAtEnd->contourY(t, radius) - off0);
        }
        maxDeltaEnd = mx;
    }

    // Ring 0 (start)
    for (int j = 0; j <= CIRC_SEGMENTS; j++) {
        double theta = 2.0 * M_PI * j / CIRC_SEGMENTS;
        V3 n = tubeZ * std::cos(theta) + tubeX * std::sin(theta);
        V3 p = start + n * radius;
        if (cutAtStart && !cutAtStart->empty()) {
            double fY = cutAtStart->contourY(theta, radius) - cutAtStart->planes[0].offsetAlongAxis;
            p = p + dir * (fY - minDeltaStart);
        }
        mesh.vertices.push_back({(float)p.x, (float)p.y, (float)p.z,
                                 (float)n.x, (float)n.y, (float)n.z});
    }

    // Ring 1 (end)
    for (int j = 0; j <= CIRC_SEGMENTS; j++) {
        double theta = 2.0 * M_PI * j / CIRC_SEGMENTS;
        V3 n = tubeZ * std::cos(theta) + tubeX * std::sin(theta);
        V3 p = end + n * radius;
        if (cutAtEnd && !cutAtEnd->empty()) {
            double fY = cutAtEnd->contourY(theta, radius) - cutAtEnd->planes[0].offsetAlongAxis;
            p = p + dir * (fY - maxDeltaEnd);
        }
        mesh.vertices.push_back({(float)p.x, (float)p.y, (float)p.z,
                                 (float)n.x, (float)n.y, (float)n.z});
    }

    // Triangles
    int n = CIRC_SEGMENTS + 1;
    for (int j = 0; j < CIRC_SEGMENTS; j++) {
        mesh.indices.push_back(j);     mesh.indices.push_back(n + j);     mesh.indices.push_back(j + 1);
        mesh.indices.push_back(j + 1); mesh.indices.push_back(n + j);     mesh.indices.push_back(n + j + 1);
    }
    return mesh;
}

// Create a torus arc mesh
TubeSegmentMesh createArc(
    const V3& pos, const V3& fwd, const V3& up, const V3& right,
    double R, double radius, double angleDeg, int stepId,
    V3& outPos, V3& outFwd, V3& outUp)
{
    TubeSegmentMesh mesh;
    mesh.stepId = stepId;
    mesh.type = TubeSegmentMesh::Green;

    double angleRad = angleDeg * M_PI / 180.0;
    V3 center = pos + right * R;
    int arcSteps = std::max(4, (int)(angleDeg / 5.0));

    int ringSize = CIRC_SEGMENTS + 1;
    for (int i = 0; i <= arcSteps; i++) {
        double t = angleRad * i / arcSteps;
        // Position on arc center line
        V3 spoke = (pos - center);  // from center to pos
        V3 arcPos = center + rotateAround(spoke, up, t);
        V3 arcFwd = rotateAround(fwd, up, t).norm();
        V3 arcRight = rotateAround(right, up, t).norm();

        // Local frame at arc point
        V3 tubeX = arcRight;
        V3 tubeZ = up;

        for (int j = 0; j <= CIRC_SEGMENTS; j++) {
            double theta = 2.0 * M_PI * j / CIRC_SEGMENTS;
            V3 n = tubeZ * std::cos(theta) + tubeX * std::sin(theta);
            V3 p = arcPos + n * radius;
            mesh.vertices.push_back({(float)p.x, (float)p.y, (float)p.z,
                                     (float)n.x, (float)n.y, (float)n.z});
        }
    }

    // Triangles
    for (int i = 0; i < arcSteps; i++) {
        for (int j = 0; j < CIRC_SEGMENTS; j++) {
            int a = i * ringSize + j;
            int b = (i + 1) * ringSize + j;
            mesh.indices.push_back(a);     mesh.indices.push_back(b);     mesh.indices.push_back(a + 1);
            mesh.indices.push_back(a + 1); mesh.indices.push_back(b);     mesh.indices.push_back(b + 1);
        }
    }

    // Output updated frame
    outPos = center + rotateAround(pos - center, up, angleRad);
    outFwd = rotateAround(fwd, up, angleRad).norm();
    outUp  = up;

    return mesh;
}

} // anon

std::vector<TubeSegmentMesh> buildTubeGeometry(const BendingProgram& prog, double fullLTotal,
                                                int maxSteps, double partialFrac)
{
    std::vector<TubeSegmentMesh> result;
    double radius = prog.D / 2.0;
    double R = prog.R;

    // Collect only bend (non-cut) steps for geometry
    std::vector<const BendStep*> bSteps;
    for (auto& s : prog.steps) {
        if (!s.isCut) bSteps.push_back(&s);
    }
    int numSteps = (int)bSteps.size();
    if (maxSteps >= 0 && maxSteps < numSteps) numSteps = maxSteps;

    double consumed = 0;
    for (int i = 0; i < numSteps; i++) {
        auto& s = *bSteps[i];
        double feedLen = s.feedLength;
        double bendAngle = s.bendAngle;
        // For the last shown step, apply partialFrac
        if (i == numSteps - 1 && partialFrac < 1.0) {
            feedLen *= partialFrac;
            bendAngle *= partialFrac;
        }
        consumed += feedLen + (M_PI * R * std::abs(bendAngle) / 180.0);
    }
    double totalLen = (fullLTotal > 0) ? fullLTotal : consumed;
    double blueLen = std::max(0.0, totalLen - consumed);

    // Blue tail — Point0 from STP: bend point at X=-R, Z=397 (from Точка гибки R76 Ф38.stp)
    V3 Point0(-(double)prog.R, 0, 397); // dynamic: follows bend radius

    // Gray clamp zone: from Point0 (Y=0) to Point0 + clampLength in +Y direction
    // This is the part of tube held by the clamp, between the bender and the bar stock
    double clampLen = prog.clampLength;
    if (clampLen > 0.5 && blueLen > 0.5) {
        V3 clampEnd = Point0 + V3(0, 1, 0) * std::min(clampLen, blueLen);
        auto grayMesh = createCylinder(Point0, clampEnd, radius, 0, TubeSegmentMesh::Gray);
        result.push_back(grayMesh);
    }

    // Blue bar stock: only from clampLength onwards (Y > clampLength)
    double blueStart = clampLen;
    double pureBluLen = blueLen - blueStart;
    if (pureBluLen > 0.5) {
        V3 bStart = Point0 + V3(0, 1, 0) * blueStart;
        V3 bEnd   = Point0 + V3(0, 1, 0) * blueLen;
        auto mesh = createCylinder(bStart, bEnd, radius, 0, TubeSegmentMesh::Blue,
                                   nullptr, prog.cutAfter.empty() ? nullptr : &prog.cutAfter);
        result.push_back(mesh);
    }

    if (numSteps <= 0) return result;

    // Process steps in reverse (same as v1.1)
    V3 pos = Point0;
    V3 fwd(0, -1, 0);
    V3 up(0, 0, 1);
    V3 right = up.cross(fwd).norm();

    for (int i = numSteps - 1; i >= 0; i--) {
        auto& step = *bSteps[i];

        // For animation: the last shown step may have partial feed/angle
        double stepFeed = step.feedLength;
        double stepBendAngle = step.bendAngle;
        double stepRotation = step.rotation;
        if (i == numSteps - 1 && partialFrac < 1.0) {
            stepFeed *= partialFrac;
            stepBendAngle *= partialFrac;
            // rotation applied only when partialFrac >= 1
            if (partialFrac < 0.99) stepRotation = 0;
        }

        // 1. Bend (green arc)
        if (stepBendAngle > 0.001) {
            V3 newPos, newFwd, newUp;
            auto arcMesh = createArc(pos, fwd, up, right, R, radius, stepBendAngle, step.id,
                                     newPos, newFwd, newUp);
            result.push_back(arcMesh);
            pos = newPos; fwd = newFwd; up = newUp;
            right = up.cross(fwd).norm();
        }

        // 2. Rotation
        if (std::abs(stepRotation) > 0.001) {
            double rotRad = stepRotation * M_PI / 180.0;
            up = rotateAround(up, fwd, rotRad);
            right = up.cross(fwd).norm();
        }

        // 3. Feed (red segment)
        if (stepFeed > 0.001) {
            V3 feedEnd = pos + fwd * stepFeed;

            const CutDefinition* cutStart = nullptr;
            const CutDefinition* cutEnd = nullptr;

            if (i == 0 && !prog.cutBefore.empty())
                cutEnd = &prog.cutBefore;
            if (i == numSteps - 1 && blueLen <= 0.5 && !prog.cutAfter.empty())
                cutStart = &prog.cutAfter;

            auto feedMesh = createCylinder(pos, feedEnd, radius, step.id, TubeSegmentMesh::Red,
                                           cutStart, cutEnd);
            result.push_back(feedMesh);
            pos = feedEnd;
        }
    }

    return result;
}
