// ─────────────────────────────────────────────────────────────────────
// step_export.cpp — STEP export using OpenCASCADE BRep builder
// ─────────────────────────────────────────────────────────────────────
#include "step_export.h"

#include <iostream>
#include <cmath>
#include <string>

#ifdef _WIN32
#include <windows.h>
#endif

// OCCT
#include <STEPControl_Writer.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <gp_Ax2.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <Interface_Static.hxx>

bool exportStep(const BendingProgram& prog, const std::string& filepath)
{
    double radius = prog.D / 2.0;
    double R = prog.R;
    double wall = (prog.D > 10) ? 2.0 : prog.D * 0.1; // default wall thickness
    double innerR = radius - wall;
    if (innerR < 0.1) innerR = radius * 0.8;

    // Build tube as a chain of cylinders and torus arcs
    gp_Pnt pos(0, 0, 0);
    gp_Dir fwd(0, -1, 0);   // initial direction: -Y (tube exits machine)
    gp_Dir up(0, 0, 1);     // Z up

    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);

    for (size_t i = 0; i < prog.steps.size(); i++) {
        auto& step = prog.steps[i];

        // --- Feed (straight cylinder) ---
        if (step.feedLength > 0.1) {
            gp_Ax2 ax(pos, gp_Dir(fwd));

            // Outer cylinder
            BRepPrimAPI_MakeCylinder outerCyl(ax, radius, step.feedLength);
            builder.Add(compound, outerCyl.Shape());

            // Inner cylinder (hollow tube)
            if (innerR > 0.1) {
                BRepPrimAPI_MakeCylinder innerCyl(ax, innerR, step.feedLength);
                builder.Add(compound, innerCyl.Shape());
            }

            // Move position
            pos.Translate(gp_Vec(fwd) * step.feedLength);
        }

        // --- Rotation ---
        if (std::abs(step.rotation) > 0.001) {
            double rotRad = step.rotation * M_PI / 180.0;
            gp_Trsf rot;
            rot.SetRotation(gp_Ax1(pos, fwd), rotRad);
            up.Transform(rot);
        }

        // --- Bend (torus arc) ---
        if (step.bendAngle > 0.001) {
            double bendRad = step.bendAngle * M_PI / 180.0;

            // Bend center is at pos + right * R where right = fwd x up
            gp_Dir right = fwd.Crossed(up);
            gp_Pnt center = pos.Translated(gp_Vec(right) * R);

            // Torus axis is along 'up'
            gp_Ax2 torAx(center, gp_Dir(up.IsOpposite(gp_Dir(0,0,1), 0.01) ? gp_Dir(0,0,-1) : up));

            // Outer torus
            try {
                BRepPrimAPI_MakeTorus outerTor(torAx, R, radius, -M_PI/2, -M_PI/2 + bendRad);
                builder.Add(compound, outerTor.Shape());
            } catch (...) {
                std::cerr << "[STEP Export] Failed to create outer torus for step " << i << "\n";
            }
            if (innerR > 0.1) {
                try {
                    BRepPrimAPI_MakeTorus innerTor(torAx, R, innerR, -M_PI/2, -M_PI/2 + bendRad);
                    builder.Add(compound, innerTor.Shape());
                } catch (...) {}
            }

            // Update position and direction after bend
            gp_Trsf bendTrsf;
            bendTrsf.SetRotation(gp_Ax1(center, gp_Dir(up)), bendRad);
            pos.Transform(bendTrsf);
            fwd.Transform(bendTrsf);
        }
    }

    // Write to STEP
    STEPControl_Writer writer;
    Interface_Static::SetCVal("write.step.schema", "AP242");

    auto stat = writer.Transfer(compound, STEPControl_AsIs);
    if (stat != IFSelect_RetDone) {
        std::cerr << "[STEP Export] Transfer failed\n";
        return false;
    }

    // Convert UTF-8 path to short 8.3 path for OCCT on Windows
    std::string writePath = filepath;
#ifdef _WIN32
    {
        int wlen = MultiByteToWideChar(CP_UTF8, 0, filepath.c_str(), -1, nullptr, 0);
        if (wlen > 0) {
            std::wstring wpath(wlen, L'\0');
            MultiByteToWideChar(CP_UTF8, 0, filepath.c_str(), -1, wpath.data(), wlen);
            // Create file first so short path can be obtained
            HANDLE hf = CreateFileW(wpath.c_str(), GENERIC_WRITE, 0, nullptr,
                                    CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
            if (hf != INVALID_HANDLE_VALUE) CloseHandle(hf);
            wchar_t shortPath[MAX_PATH] = {};
            DWORD shortLen = GetShortPathNameW(wpath.c_str(), shortPath, MAX_PATH);
            if (shortLen > 0 && shortLen < MAX_PATH) {
                int nLen = WideCharToMultiByte(CP_ACP, 0, shortPath, -1, nullptr, 0, nullptr, nullptr);
                std::string narrow(nLen - 1, '\0');
                WideCharToMultiByte(CP_ACP, 0, shortPath, -1, narrow.data(), nLen, nullptr, nullptr);
                writePath = narrow;
            }
        }
    }
#endif
    auto writeStat = writer.Write(writePath.c_str());
    if (writeStat != IFSelect_RetDone) {
        std::cerr << "[STEP Export] Write failed: " << filepath << "\n";
        return false;
    }

    std::cout << "[STEP Export] Written: " << filepath << "\n";
    return true;
}
