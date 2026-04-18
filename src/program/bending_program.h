#pragma once
#include <vector>
#include <string>
#include <cmath>
#include <array>
#include <nlohmann/json.hpp>

// ─── Cut plane ──────────────────────────────────────────────────────
struct CutPlane {
    double nx = 0, ny = -1, nz = 0;   // normal in tube-local frame
    double offsetAlongAxis = 0;         // mm along unrolled tube axis

    /// ΔY(θ) = r * (nz·cos(θ) + nx·sin(θ)) / ny
    double deltaY(double theta, double radius) const {
        if (std::abs(ny) < 1e-9) return 0;
        return radius * (nz * std::cos(theta) + nx * std::sin(theta)) / ny;
    }

    bool isStraight() const {
        return std::abs(std::abs(ny) - 1.0) < 0.05;
    }
};

// ─── Cut definition (1 or 2 planes) ────────────────────────────────
enum class CutType { None, Straight, Oblique, Saddle };

struct CutDefinition {
    std::vector<CutPlane> planes;

    std::string typeName() const {
        switch (type()) {
            case CutType::None:     return u8"Нет";
            case CutType::Straight: return u8"Прямой";
            case CutType::Oblique:  return u8"Косой";
            case CutType::Saddle:   return u8"Седловой";
        }
        return "?";
    }

    CutType type() const {
        if (planes.empty()) return CutType::None;
        if (planes.size() == 1) return planes[0].isStraight() ? CutType::Straight : CutType::Oblique;
        return CutType::Saddle;
    }

    /// Contour Y at angle θ (picks min of 2 planes for saddle)
    double contourY(double theta, double radius, bool useMin = true) const {
        if (planes.empty()) return 0;
        double result = planes[0].offsetAlongAxis + planes[0].deltaY(theta, radius);
        for (size_t i = 1; i < planes.size(); i++) {
            double yi = planes[i].offsetAlongAxis + planes[i].deltaY(theta, radius);
            result = useMin ? std::min(result, yi) : std::max(result, yi);
        }
        return result;
    }

    bool empty() const { return planes.empty(); }
};

// ─── Bend step ──────────────────────────────────────────────────────
struct BendStep {
    int    id         = 0;
    double feedLength = 0;    // mm
    double rotation   = 0;    // degrees
    double bendAngle  = 0;    // degrees
    bool   isCut      = false; // true = this step is a cut, not a bend
    CutDefinition cutDef;      // only used when isCut==true

    double arcLength(double R) const {
        return isCut ? 0.0 : M_PI * R * std::abs(bendAngle) / 180.0;
    }
};

// ─── Bending program ────────────────────────────────────────────────
struct BendingProgram {
    double D = 38;            // outer diameter, mm
    double R = 76;            // bend radius (CLR), mm
    double clampLength = 60;  // straight clamp section, mm
    std::vector<BendStep> steps;

    CutDefinition cutBefore;   // far end of tube (Y=0)
    CutDefinition cutAfter;    // near end / machine side (Y=LTotal)

    double lTotal() const {
        double sum = clampLength;
        for (auto& s : steps) {
            sum += s.feedLength + s.arcLength(R); // ключаем подачу реза, чтобы анимация кнута (каретки) совпадала с физикой
        }
        return sum;
    }

    void addStep(double feed = 50, double rot = 0, double ang = 0) {
        int id = steps.empty() ? 1 : steps.back().id + 1;
        BendStep s;
        s.id = id; s.feedLength = feed; s.rotation = rot; s.bendAngle = ang;
        steps.push_back(s);
    }

    /// Derive cutBefore/cutAfter from unified steps list.
    /// First isCut step before first bend-step → cutBefore.
    /// Last  isCut step after  last  bend-step → cutAfter.
    /// Call this before animation/geometry rebuild.
    void deriveCuts() {
        cutBefore = {};
        cutAfter  = {};
        // Find first and last bend (non-cut) step indices
        int firstBend = -1, lastBend = -1;
        for (int i = 0; i < (int)steps.size(); i++) {
            if (!steps[i].isCut) {
                if (firstBend < 0) firstBend = i;
                lastBend = i;
            }
        }
        // Cuts before firstBend → cutBefore, cuts after lastBend → cutAfter
        for (int i = 0; i < (int)steps.size(); i++) {
            if (!steps[i].isCut) continue;
            if (firstBend < 0) {
                // No bend steps at all — treat first cut as before, rest as after
                if (cutBefore.empty())
                    cutBefore = steps[i].cutDef;
                else
                    cutAfter = steps[i].cutDef;
            } else if (i < firstBend) {
                cutBefore = steps[i].cutDef;
            } else if (i > lastBend) {
                cutAfter = steps[i].cutDef;
            }
        }
    }

    /// Get only bend (non-cut) steps for geometry/animation
    std::vector<BendStep> bendSteps() const {
        std::vector<BendStep> result;
        for (auto& s : steps)
            if (!s.isCut) result.push_back(s);
        return result;
    }

    void clear() {
        steps.clear();
        cutBefore = {};
        cutAfter  = {};
    }

    // Convenience JSON methods
    nlohmann::json toJson() const;
    static BendingProgram fromJson(const nlohmann::json& j);
};

// JSON serialization
void to_json(nlohmann::json& j, const CutPlane& p);
void from_json(const nlohmann::json& j, CutPlane& p);
void to_json(nlohmann::json& j, const CutDefinition& c);
void from_json(const nlohmann::json& j, CutDefinition& c);
void to_json(nlohmann::json& j, const BendStep& s);
void from_json(const nlohmann::json& j, BendStep& s);
void to_json(nlohmann::json& j, const BendingProgram& p);
void from_json(const nlohmann::json& j, BendingProgram& p);
