#pragma once
#include "tube_geometry.h"
#include "stl_loader.h"
#include <vector>

// ─── OpenGL Renderer ────────────────────────────────────────────────
class Renderer {
public:
    bool init(int width, int height);
    void shutdown();
    void resize(int width, int height);

    // Camera
    void setCamera(float yaw, float pitch, float distance, float cx, float cy, float cz);
    void orbit(float dYaw, float dPitch);
    void zoom(float delta);
    void pan(float dx, float dy);

    // Scene
    void clearScene();
    void uploadTubeSegments(const std::vector<TubeSegmentMesh>& segments);
    void uploadBenderSTL(const StlMesh& stlMesh);

    // FBO (off-screen rendering for ImGui Image)
    void ensureFBO(int w, int h);
    void renderToFBO();
    unsigned int fboTextureId() const { return m_fboTexture; }
    int fboWidth() const { return m_fboW; }
    int fboHeight() const { return m_fboH; }

    // Render
    void beginFrame();
    void renderScene(int vpX, int vpY, int vpW, int vpH);
    void endFrame();

    // Picking
    int pickSegment(int mouseX, int mouseY); // returns stepId or -1

    void setShowBender(bool v) { m_showBender = v; }
    void setBenderRotation(float angleDeg) { m_benderAngleDeg = angleDeg; }
    void setBenderOpaque(bool v) { m_benderOpaque = v; }

    // Logic 2 models (roller, clamp, static frame, carriage)
    void uploadRollerSTL(const StlMesh& mesh);
    void uploadClampSTL(const StlMesh& mesh);
    void uploadStaticSTL(const StlMesh& mesh);
    void uploadCarriageSTL(const StlMesh& mesh);
    void setShowLogic2(bool v) { m_showLogic2 = v; }
    void setClampOffset(float xOffset) { m_clampXOffset = xOffset; }
    void setCarriageOffset(float yOffset) { m_carriageYOffset = yOffset; }
    // Roller rotation uses same m_benderAngleDeg as bender

    // Laser head / beam visualization
    void setLaserHeadPos(float x, float y, float headZ, float tubeTopZ);
    void setShowLaserHead(bool v) { m_showLaserHead = v; }
    void setShowLaserBeam(bool v) { m_showLaserBeam = v; if (!v) clearCutContour(); }

    // Camera getters for UI gizmo
    float yaw() const { return m_yaw; }
    float pitch() const { return m_pitch; }

    // Cut contour on tube surface (called each frame during cut phases)
    // contourYOffsets: 360 values of Y-offset from laser Y for each degree
    void setCutContour(const std::vector<float>& contourYOffsets,
                       float tubeCX, float tubeCZ, float tubeR, float laserY);
    void clearCutContour();

    // Tube transform for laser cut animation (rotation around Y + feed in -Y)
    void setTubeTransform(float rotYDeg, float feedY, float px, float py, float pz);
    void clearTubeTransform();

    // Exclusion zone cylinder visualization (straight + bent arc)
    void setShowExclZone(bool v) { m_showExclZone = v; }
    void updateExclZone(float cx, float cz, float radius,
                        float yMin, float bendR, float bendAngleDeg);
    void setCollisionMarker(float x, float y, float z, bool isFloor, float floorZ);
    void clearCollisionMarker();

    int width() const { return m_width; }
    int height() const { return m_height; }

    // Checkered floor
    void setFloorLevel(float z);
    void setShowFloor(bool v) { m_showFloor = v; }
    void setBackfaceCulling(bool v) { m_backfaceCulling = v; }

private:
    int m_width = 1280, m_height = 720;

    // FBO for off-screen 3D rendering
    unsigned int m_fbo = 0;
    unsigned int m_fboTexture = 0;
    unsigned int m_fboDepth = 0;
    int m_fboW = 0, m_fboH = 0;

    // Camera state
    float m_yaw = -90, m_pitch = 25, m_dist = 800;
    float m_cx = -76, m_cy = -100, m_cz = 347;

    // Shader programs
    unsigned int m_mainShader = 0;
    unsigned int m_pickShader = 0;

    // GPU objects for tube segments
    struct GPUSegment {
        unsigned int vao = 0, vbo = 0, ebo = 0;
        int indexCount = 0;
        int stepId = 0;
        TubeSegmentMesh::Type type;
    };
    std::vector<GPUSegment> m_segments;

    // GPU objects for bender STL
    unsigned int m_benderVAO = 0, m_benderVBO = 0;
    int m_benderVertCount = 0;
    bool m_showBender = true;
    bool m_benderOpaque = true;
    float m_benderAngleDeg = 0.0f; // rotation around Z axis (0,0,1)

    // Logic 2 models
    unsigned int m_rollerVAO = 0, m_rollerVBO = 0;
    int m_rollerVertCount = 0;
    unsigned int m_clampVAO = 0, m_clampVBO = 0;
    int m_clampVertCount = 0;
    unsigned int m_staticVAO = 0, m_staticVBO = 0;
    int m_staticVertCount = 0;
    unsigned int m_carriageVAO = 0, m_carriageVBO = 0;
    int m_carriageVertCount = 0;
    bool m_showLogic2 = false;
    float m_clampXOffset = 0.0f;    // 0=clamped, negative=released (local X, rotates with roller)
    float m_carriageYOffset = 0.0f; // carriage translation along Y (feed)

    // Laser head/beam
    bool m_showLaserHead = false;
    bool m_showLaserBeam = false;
    unsigned int m_laserVAO = 0, m_laserVBO = 0;
    int m_laserHeadVertCount = 0;
    int m_laserBeamVertCount = 0;
    float m_laserX = -76.f, m_laserY = -60.f, m_laserZ = 371.f, m_laserTubeTopZ = 366.f;

    // Cut contour on tube surface
    unsigned int m_cutContourVAO = 0, m_cutContourVBO = 0;
    int m_cutContourVertCount = 0;
    bool m_cutContourDirty = false;

    // Tube transform (laser cut animation)
    bool m_hasTubeTransform = false;
    float m_tubeModel[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    float m_tubeNormMat[9] = {1,0,0, 0,1,0, 0,0,1};

    // Exclusion zone cylinder + sphere visualization
    bool m_showExclZone = false;
    unsigned int m_exclVAO = 0, m_exclVBO = 0;
    int m_exclVertCount = 0;

    // Collision preview marker
    unsigned int m_collisionVAO = 0, m_collisionVBO = 0;
    int m_collisionVertCount = 0;
    bool m_showCollisionMarker = false;
    bool m_collisionMarkerIsFloor = false;

    // Checkered floor
    bool m_showFloor = true;
    bool m_backfaceCulling = false;
    bool m_floorGenerated = false;
    float m_floorZ = 0.0f;
    unsigned int m_floorVAO[2] = {};
    unsigned int m_floorVBO[2] = {};
    int m_floorVertCount[2] = {};
    void generateFloor(float z);

    void compileShaders();
    void uploadMesh(GPUSegment& gpu, const TubeSegmentMesh& mesh);
    void cleanupSegments();

    // Matrix helpers
    void getViewMatrix(float* mat) const;
    void getProjMatrix(float* mat) const;
};
