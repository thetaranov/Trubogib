#pragma once
#include "bending_program.h"
#include <string>

/// Import a STEP file and extract a BendingProgram (tube geometry + cut planes).
/// Uses OpenCASCADE for proper STEP parsing — no manual text parsing needed.
BendingProgram importStep(const std::string& filepath);
