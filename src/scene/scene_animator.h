#pragma once
// ─────────────────────────────────────────────────────────────────────
// scene_animator.h — Отображение состояния станка в 3D сцене.
//
// ЕДИНАЯ ЛОГИКА для симуляции и реального железа. Потребляет MachineState
// (единый источник истины) и один раз за кадр применяет его к Renderer:
//   • каретка (setCarriageOffset)     — от MachineState.zAbs
//   • гибочный ролик (setBenderRotation) — от MachineState.bAbs
//   • геометрия трубы (uploadTubeSegments) — частичный BendStep из той же zAbs
//   • tubeTransform                   — той же zAbs для Clearance/Return
//
// ⚠️ КРИТИЧНО: ВСЕ визуалы выводятся из ОДНОГО снимка MachineState.
// Это исключает дёрганье каретки vs. длины трубы (разные источники).
// ─────────────────────────────────────────────────────────────────────
#include "bending_program.h"
#include "machine.h"  // MachineState, MachPhase

class Renderer;

class SceneAnimator {
public:
    SceneAnimator() = default;

    void setRenderer(Renderer* r) { renderer_ = r; }

    // Apply one snapshot to the scene. Must be called every frame when a
    // program is running (or whenever machine state changes).
    //
    // whipLength — current physical whip length (scene-dependent).
    // The function deterministically derives all 3D visuals from `s` only —
    // no cross-frame state, no hidden filtering. Smoothing (if needed) must
    // be applied upstream in Machine::getSnapshot().
    void apply(const MachineState& s,
               const BendingProgram& prog,
               double whipLength);

    // After apply(), these tell App what the clamp target should be.
    // App still owns clamp interpolation for Logic2 UI modes.
    enum ClampVisualState { ClampClosed = 0, ClampOpenForBend = 1 };
    ClampVisualState clampTarget() const { return clampTarget_; }

private:
    Renderer* renderer_ = nullptr;
    ClampVisualState clampTarget_ = ClampClosed;

    // Logging dedup
    int lastLoggedPhaseId_ = -1;
    int lastLoggedStepIdx_ = -1;
};
