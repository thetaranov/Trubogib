#pragma once
#include "bending_program.h"
#include <string>

/// Export a BendingProgram to STEP AP242 format using OpenCASCADE.
bool exportStep(const BendingProgram& prog, const std::string& filepath);
