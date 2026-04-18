// ─────────────────────────────────────────────────────────────────────
// scene_animator.cpp — Unified machine-state → 3D scene projection.
//
// INVARIANT: the current step's bend is ALWAYS anchored at Point0 (roller).
// Cumulative clearance pushes from prior bent steps are represented by
// explicit 0-angle "spacer" BendSteps injected between completed steps in
// tempProg. buildTubeGeometry processes them naturally: each spacer pushes
// the preceding (older) step's draw position further back from Point0,
// which is EXACTLY the physical effect of a clearance push.
//
// No tubeTransform is used — the tube mesh is always uploaded in world
// coordinates anchored at Point0. The carriage moves independently based
// on zAbs via setCarriageOffset, and because the tube's blue tail grows
// to fill (whipLength - consumed_with_spacers), the visual carriage-tail
// relationship stays consistent.
//
// The logic operates purely on MachineState (populated from sim OR hw in
// Machine::getSnapshot), so the 3D scene is identical in both modes.
// ─────────────────────────────────────────────────────────────────────
#include "scene_animator.h"
#include "renderer.h"
#include "tube_geometry.h"
#include "logger.h"

#include <algorithm>
#include <cmath>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

constexpr float  PT0_Y = 0.0f;
constexpr float  PT0_Z = 397.0f;
constexpr double CAD_CARRIAGE_Y_BASE = 6000.0;
constexpr double RENDERER_CARRIAGE_SIGN = -1.0;

inline double carriageRenderFeed(double zAbs, double whipLength) {
    const double yTarget = whipLength - std::max(0.0, zAbs);
    const double offsetModel = yTarget - CAD_CARRIAGE_Y_BASE;
    return RENDERER_CARRIAGE_SIGN * offsetModel;
}

const char* phaseText(int p) {
    switch ((MachPhase)p) {
    case MachPhase::Idle:              return "Idle";
    case MachPhase::ServoOn:           return "ServoOn";
    case MachPhase::HomingZ:           return "HomingZ";
    case MachPhase::HomingC:           return "HomingC";
    case MachPhase::UnclampWait1:      return "UnclampWait1";
    case MachPhase::Feed:              return "Feed";
    case MachPhase::Rotation:          return "Rotation";
    case MachPhase::LaserCutBefore:    return "LaserCutBefore";
    case MachPhase::LaserCutBeforeWait:return "LaserCutBeforeWait";
    case MachPhase::ClampWait:         return "ClampWait";
    case MachPhase::SimulatedBendWait: return "SimulatedBendWait";
    case MachPhase::BendStart:         return "BendStart";
    case MachPhase::BendWait:          return "BendWait";
    case MachPhase::UnclampWait2:      return "UnclampWait2";
    case MachPhase::LaserCutAfter:     return "LaserCutAfter";
    case MachPhase::LaserCutAfterWait: return "LaserCutAfterWait";
    case MachPhase::Clearance:         return "Clearance";
    case MachPhase::ReturnBend:        return "ReturnBend";
    case MachPhase::ReturnBendWait:    return "ReturnBendWait";
    case MachPhase::StepDone:          return "StepDone";
    case MachPhase::ProgramDone:       return "ProgramDone";
    case MachPhase::Error:             return "Error";
    }
    return "?";
}

double computeMaterialConsumed(const BendingProgram& tempProg) {
    double m = 0.0;
    for (const auto& s : tempProg.steps) {
        if (s.isCut) continue;
        m += s.feedLength;
        m += M_PI * tempProg.R * std::abs(s.bendAngle) / 180.0;
    }
    return m;
}

BendStep makeSpacer(int id, double feedLength) {
    BendStep sp;
    sp.id = id;
    sp.isCut = false;
    sp.feedLength = feedLength;
    sp.rotation = 0.0;
    sp.bendAngle = 0.0;
    return sp;
}

} // namespace

// ─────────────────────────────────────────────────────────────────────
void SceneAnimator::apply(const MachineState& s,
                          const BendingProgram& prog,
                          double whipLength) {
    if (!renderer_) return;

    // DEBUG: Log apply entry
    static int applyLogCount = 0;
    if (++applyLogCount % 120 == 0) {  // ~2Hz at 60FPS
        Logger::log(std::string("[SCENE_APPLY] phase=") + std::to_string((int)s.phaseId) +
                   " step=" + std::to_string(s.stepIdx) + "/" + std::to_string((int)prog.steps.size()) +
                   " zAbs=" + std::to_string(s.zAbs) +
                   " cAbs=" + std::to_string(s.cAbs) +
                   " bAbs=" + std::to_string(s.bAbs) +
                   " isSimulation=" + std::to_string((int)s.isSimulation));
    }

    const int stepIdx    = s.stepIdx;
    const int totalSteps = (int)prog.steps.size();

    // ── 1. Carriage + bender roller from the same snapshot ─────────
    renderer_->setCarriageOffset((float)carriageRenderFeed(s.zAbs, whipLength));
    renderer_->setBenderRotation((float)std::max(0.0, s.bAbs));

    // ── 2. Empty program → clear visuals ───────────────────────────
    if (totalSteps == 0) {
        renderer_->clearTubeTransform();
        clampTarget_ = ClampClosed;
        return;
    }

    const int stepIdxClamped = std::clamp(stepIdx, 0, totalSteps - 1);
    const auto phase = (MachPhase)s.phaseId;
    const BendStep& step = prog.steps[stepIdxClamped];
    const double clampLen = prog.clampLength;

    // ── 3. Build partial CURRENT step from phase + live snapshot ───
    const double localZ = std::max(0.0, s.zAbs - s.baseFeedAcc);
    const double localC = s.cAbs - s.baseRotAcc;

    BendStep partial = step;
    partial.feedLength = 0.0;
    partial.rotation   = 0.0;
    partial.bendAngle  = 0.0;

    switch (phase) {
    case MachPhase::Feed:
        partial.feedLength = std::clamp(localZ, 0.0, step.feedLength);
        break;

    case MachPhase::Rotation:
        partial.feedLength = step.feedLength;
        partial.rotation   = (step.rotation >= 0.0)
            ? std::clamp(localC, 0.0, step.rotation)
            : std::clamp(localC, step.rotation, 0.0);
        break;

    case MachPhase::ClampWait:
        partial.feedLength = step.feedLength;
        partial.rotation   = step.rotation;
        break;

    case MachPhase::BendStart:
    case MachPhase::BendWait:
    case MachPhase::SimulatedBendWait:
        partial.feedLength = step.feedLength;
        partial.rotation   = step.rotation;
        partial.bendAngle  = (step.bendAngle >= 0.0)
            ? std::clamp(s.bAbs, 0.0, step.bendAngle)
            : std::clamp(s.bAbs, step.bendAngle, 0.0);
        break;

    case MachPhase::UnclampWait2:
    case MachPhase::Clearance:
    case MachPhase::ReturnBend:
    case MachPhase::ReturnBendWait:
    case MachPhase::StepDone:
    case MachPhase::ProgramDone:
        partial.feedLength = step.feedLength;
        partial.rotation   = step.rotation;
        partial.bendAngle  = step.bendAngle;
        break;

    default:
        // Idle / ServoOn / HomingZ / HomingC / UnclampWait1 / LaserCut*
        // → partial stays (0,0,0): current step contributes nothing yet.
        break;
    }

    // ── 4. Build tempProg: prior steps + spacers + partial ─────────
    BendingProgram tempProg = prog;
    tempProg.steps.clear();

    for (int k = 0; k < stepIdxClamped; ++k) {
        const BendStep& prior = prog.steps[k];
        tempProg.steps.push_back(prior);
        // A spacer is permanently present after every completed BENT step,
        // because Clearance pushes the bar +clampLength AFTER each bend.
        if (!prior.isCut && std::abs(prior.bendAngle) > 0.001 && clampLen > 0.001) {
            tempProg.steps.push_back(makeSpacer(9000 + k, clampLen));
        }
    }
    tempProg.steps.push_back(partial);

    // ── 5. Compute current-step growing clearance from zAbs invariant ──
    // During Clearance phase of the CURRENT step, zAbs grows by up to
    // clampLen while material stays fixed — the delta is the growing
    // spacer. In all other phases it's zero by construction.
    const double materialSoFar = computeMaterialConsumed(tempProg);
    double currentClearance = s.zAbs - materialSoFar;
    if (currentClearance < 0.0) currentClearance = 0.0;
    if (currentClearance > clampLen) currentClearance = clampLen;

    if (currentClearance > 0.01) {
        tempProg.steps.push_back(makeSpacer(9999, currentClearance));
    }

    // ── 6. Upload geometry — NO tubeTransform, ever ────────────────
    auto meshes = buildTubeGeometry(tempProg, whipLength);
    renderer_->uploadTubeSegments(meshes);
    renderer_->clearTubeTransform();

    // ── 7. Phase/step transition log + world-coord diagnostics ─────
    if (s.phaseId != lastLoggedPhaseId_ || stepIdx != lastLoggedStepIdx_) {
        // Diagnostic world coordinates for the current mesh build:
        //   • curArcWorld — first vertex of the current step's mesh
        //                   (should be at Point0 ≈ (-R, 0, 397) during Bend)
        //   • tailWorld   — last vertex of the blue/gray tail (stepId==0)
        //                   (where the bar stock ends; tells us how far the
        //                    tube extends toward the carriage)
        const int curStepId = step.id;
        const Vertex* curV = nullptr;
        const Vertex* tailV = nullptr;
        for (const auto& m : meshes) {
            if (m.vertices.empty()) continue;
            if (m.stepId == curStepId && !curV) {
                curV = &m.vertices.front();
            }
            if (m.stepId == 0 && m.type == TubeSegmentMesh::Blue) {
                tailV = &m.vertices.back();
            }
        }
        // Fallback: any stepId==0 mesh if no Blue one exists
        if (!tailV) {
            for (const auto& m : meshes) {
                if (m.stepId == 0 && !m.vertices.empty()) {
                    tailV = &m.vertices.back();
                    break;
                }
            }
        }

        std::string msg = std::string("[SCENE] phase=") + phaseText(s.phaseId)
                  + " step=" + std::to_string(stepIdx + 1) + "/" + std::to_string(totalSteps)
                  + " zAbs=" + std::to_string(s.zAbs)
                  + " cAbs=" + std::to_string(s.cAbs)
                  + " bAbs=" + std::to_string(s.bAbs)
                  + " baseFeedAcc=" + std::to_string(s.baseFeedAcc)
                  + " baseRotAcc=" + std::to_string(s.baseRotAcc)
                  + " material=" + std::to_string(materialSoFar)
                  + " curClr=" + std::to_string(currentClearance)
                  + " tempProgSteps=" + std::to_string((int)tempProg.steps.size());
        if (curV) {
            msg += " curArcWorld=(" + std::to_string(curV->x) + ","
                                    + std::to_string(curV->y) + ","
                                    + std::to_string(curV->z) + ")";
        }
        if (tailV) {
            msg += " tailWorld=(" + std::to_string(tailV->x) + ","
                                  + std::to_string(tailV->y) + ","
                                  + std::to_string(tailV->z) + ")";
        }
        Logger::log(msg);
        lastLoggedPhaseId_ = s.phaseId;
        lastLoggedStepIdx_ = stepIdx;
    }

    // ── 8. Clamp target for App-level interpolation ────────────────
    // During a bend cycle the phase dictates state. Outside a running program
    // (Idle / StepDone / ProgramDone / Error) the manual clamp/unclamp buttons
    // take over — here we must mirror the physical relay state from
    // s.clampClosed so that pressing ПРИЖИМ/РАЗЖИМ animates the clamp.
    if (phase == MachPhase::ClampWait
        || phase == MachPhase::BendStart
        || phase == MachPhase::BendWait
        || phase == MachPhase::SimulatedBendWait) {
        clampTarget_ = ClampOpenForBend;
    } else if (phase == MachPhase::Idle
            || phase == MachPhase::ProgramDone
            || phase == MachPhase::StepDone
            || phase == MachPhase::Error) {
        clampTarget_ = s.clampClosed ? ClampClosed : ClampOpenForBend;
    } else {
        clampTarget_ = ClampClosed;
    }
}
