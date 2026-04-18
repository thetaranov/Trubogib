#include "bending_program.h"
#include <iostream>

void to_json(nlohmann::json& j, const CutPlane& p) {
    j = {{"nx", p.nx}, {"ny", p.ny}, {"nz", p.nz}, {"offset", p.offsetAlongAxis}};
}
void from_json(const nlohmann::json& j, CutPlane& p) {
    j.at("nx").get_to(p.nx); j.at("ny").get_to(p.ny); j.at("nz").get_to(p.nz);
    j.at("offset").get_to(p.offsetAlongAxis);
}
void to_json(nlohmann::json& j, const CutDefinition& c) { j = {{"planes", c.planes}}; }
void from_json(const nlohmann::json& j, CutDefinition& c) { j.at("planes").get_to(c.planes); }
void to_json(nlohmann::json& j, const BendStep& s) {
    j = {{"id", s.id}, {"feedLength", s.feedLength}, {"rotation", s.rotation},
         {"bendAngle", s.bendAngle}, {"isCut", s.isCut}};
    if (s.isCut) j["cutDef"] = s.cutDef;
}
void from_json(const nlohmann::json& j, BendStep& s) {
    // Support both v2.0 format {"id","feedLength","rotation","bendAngle"}
    // and v1.1 format {"LFeed","Rotation","Angle"}
    if (j.contains("feedLength")) {
        j.at("id").get_to(s.id);
        j.at("feedLength").get_to(s.feedLength);
        j.at("rotation").get_to(s.rotation);
        j.at("bendAngle").get_to(s.bendAngle);
        if (j.contains("isCut")) j.at("isCut").get_to(s.isCut);
        if (j.contains("cutDef")) j.at("cutDef").get_to(s.cutDef);
    } else if (j.contains("LFeed")) {
        // v1.1 format
        s.id = 0;
        j.at("LFeed").get_to(s.feedLength);
        j.at("Rotation").get_to(s.rotation);
        j.at("Angle").get_to(s.bendAngle);
    }
}
void to_json(nlohmann::json& j, const BendingProgram& p) {
    j = {{"D", p.D}, {"R", p.R}, {"clampLength", p.clampLength},
         {"steps", p.steps}, {"cutBefore", p.cutBefore}, {"cutAfter", p.cutAfter}};
}
void from_json(const nlohmann::json& j, BendingProgram& p) {
    j.at("D").get_to(p.D); j.at("R").get_to(p.R);

    // Support both v2.0 ("steps") and v1.1 ("Steps") format
    if (j.contains("steps")) {
        j.at("steps").get_to(p.steps);
    } else if (j.contains("Steps")) {
        // v1.1 format: "Steps" array with {LFeed, Rotation, Angle}
        p.steps.clear();
        const auto& arr = j.at("Steps");
        for (int i = 0; i < (int)arr.size(); i++) {
            BendStep s;
            from_json(arr[i], s);
            s.id = i + 1;
            p.steps.push_back(s);
        }
        std::cout << "[JSON] Imported v1.1 format: " << p.steps.size() << " steps\n";
    }

    if (j.contains("clampLength")) j.at("clampLength").get_to(p.clampLength);

    // Legacy: read cutBefore/cutAfter and inject as cut steps
    if (j.contains("cutBefore")) j.at("cutBefore").get_to(p.cutBefore);
    if (j.contains("cutAfter"))  j.at("cutAfter").get_to(p.cutAfter);

    // If old-format file has cutBefore/cutAfter but no isCut steps, inject them
    bool hasAnyCutStep = false;
    for (auto& s : p.steps) if (s.isCut) { hasAnyCutStep = true; break; }
    if (!hasAnyCutStep) {
        if (!p.cutBefore.empty()) {
            BendStep cs;
            cs.id = 0; cs.isCut = true; cs.cutDef = p.cutBefore;
            p.steps.insert(p.steps.begin(), cs);
        }
        if (!p.cutAfter.empty()) {
            BendStep cs;
            cs.id = 0; cs.isCut = true; cs.cutDef = p.cutAfter;
            p.steps.push_back(cs);
        }
        // Renumber
        for (int i = 0; i < (int)p.steps.size(); i++)
            p.steps[i].id = i + 1;
    }
    // Always derive cutBefore/cutAfter from steps
    p.deriveCuts();
}

nlohmann::json BendingProgram::toJson() const {
    nlohmann::json j;
    to_json(j, *this);
    return j;
}
BendingProgram BendingProgram::fromJson(const nlohmann::json& j) {
    BendingProgram p;
    from_json(j, p);
    return p;
}
