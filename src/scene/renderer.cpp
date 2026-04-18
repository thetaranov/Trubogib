// ─────────────────────────────────────────────────────────────────────
// renderer.cpp — OpenGL renderer with lit Phong shading
// ─────────────────────────────────────────────────────────────────────
#include "renderer.h"
#include <glad/glad.h>
#include <cmath>
#include <cstring>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ─── Shader sources ─────────────────────────────────────────────────
static const char* vertSrc = R"(
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
uniform mat4 uMVP;
uniform mat4 uModel;
uniform mat3 uNormalMat;
out vec3 vWorldPos;
out vec3 vNormal;
void main() {
    vWorldPos = vec3(uModel * vec4(aPos, 1.0));
    vNormal = normalize(uNormalMat * aNormal);
    gl_Position = uMVP * vec4(aPos, 1.0);
}
)";

static const char* fragSrc = R"(
#version 330 core
in vec3 vWorldPos;
in vec3 vNormal;
uniform vec3 uColor;
uniform vec3 uLightDir;
uniform vec3 uCamPos;
uniform float uAlpha;
uniform float uSpecular;
out vec4 FragColor;
void main() {
    vec3 N = normalize(vNormal);
    vec3 L = normalize(uLightDir);
    float diff = max(dot(N, L), 0.0) * 0.7 + 0.3; // ambient + diffuse
    vec3 V = normalize(uCamPos - vWorldPos);
    vec3 H = normalize(L + V);
    float spec = pow(max(dot(N, H), 0.0), 32.0) * uSpecular;
    vec3 col = uColor * diff + vec3(1.0) * spec;
    FragColor = vec4(col, uAlpha);
}
)";

// ─── Matrix math (simple) ───────────────────────────────────────────
namespace {

void identity(float* m) { memset(m, 0, 16*sizeof(float)); m[0]=m[5]=m[10]=m[15]=1; }

void perspective(float* m, float fovY, float aspect, float near, float far) {
    memset(m, 0, 16*sizeof(float));
    float f = 1.0f / tanf(fovY * 0.5f);
    m[0] = f / aspect;
    m[5] = f;
    m[10] = (far + near) / (near - far);
    m[11] = -1.0f;
    m[14] = 2.0f * far * near / (near - far);
}

void lookAt(float* m, float ex, float ey, float ez, float cx, float cy, float cz, float ux, float uy, float uz) {
    float fx = cx-ex, fy = cy-ey, fz = cz-ez;
    float fl = sqrtf(fx*fx+fy*fy+fz*fz);
    fx/=fl; fy/=fl; fz/=fl;
    float sx = fy*uz-fz*uy, sy = fz*ux-fx*uz, sz = fx*uy-fy*ux;
    float sl = sqrtf(sx*sx+sy*sy+sz*sz);
    sx/=sl; sy/=sl; sz/=sl;
    float upx = sy*fz-sz*fy, upy = sz*fx-sx*fz, upz = sx*fy-sy*fx;
    memset(m, 0, 16*sizeof(float));
    m[0]=sx; m[4]=sy; m[8]=sz;
    m[1]=upx; m[5]=upy; m[9]=upz;
    m[2]=-fx; m[6]=-fy; m[10]=-fz;
    m[12]=-(sx*ex+sy*ey+sz*ez);
    m[13]=-(upx*ex+upy*ey+upz*ez);
    m[14]=(fx*ex+fy*ey+fz*ez);
    m[15]=1;
}

void mulMat(float* out, const float* a, const float* b) {
    float tmp[16];
    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++) {
            tmp[j*4+i]=0;
            for (int k=0; k<4; k++) tmp[j*4+i] += a[k*4+i]*b[j*4+k];
        }
    memcpy(out, tmp, 16*sizeof(float));
}

unsigned int compileShader(const char* src, GLenum type) {
    unsigned int s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    int ok; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512]; glGetShaderInfoLog(s, 512, nullptr, log);
        std::cerr << "Shader error: " << log << "\n";
    }
    return s;
}

unsigned int linkProgram(unsigned int vs, unsigned int fs) {
    unsigned int p = glCreateProgram();
    glAttachShader(p, vs); glAttachShader(p, fs);
    glLinkProgram(p);
    int ok; glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512]; glGetProgramInfoLog(p, 512, nullptr, log);
        std::cerr << "Link error: " << log << "\n";
    }
    glDeleteShader(vs); glDeleteShader(fs);
    return p;
}

} // anon

// ─── Laser geometry helpers ─────────────────────────────────────────
static void pushV(std::vector<float>& v, float x, float y, float z,
                  float nx, float ny, float nz) {
    v.push_back(x); v.push_back(y); v.push_back(z);
    v.push_back(nx); v.push_back(ny); v.push_back(nz);
}

static void generateCylinder(std::vector<float>& out,
    float cx, float cy, float z0, float z1, float r, int nseg)
{
    for (int i = 0; i < nseg; i++) {
        float a0 = 2.0f * (float)M_PI * i / nseg;
        float a1 = 2.0f * (float)M_PI * (i + 1) / nseg;
        float c0 = cosf(a0), s0 = sinf(a0);
        float c1 = cosf(a1), s1 = sinf(a1);
        float x0 = cx + r*c0, y0 = cy + r*s0;
        float x1 = cx + r*c1, y1 = cy + r*s1;
        // Side quad (2 triangles)
        pushV(out, x0,y0,z0, c0,s0,0);
        pushV(out, x1,y1,z0, c1,s1,0);
        pushV(out, x0,y0,z1, c0,s0,0);
        pushV(out, x1,y1,z0, c1,s1,0);
        pushV(out, x1,y1,z1, c1,s1,0);
        pushV(out, x0,y0,z1, c0,s0,0);
    }
    // Top cap
    for (int i = 0; i < nseg; i++) {
        float a0 = 2.0f * (float)M_PI * i / nseg;
        float a1 = 2.0f * (float)M_PI * (i + 1) / nseg;
        pushV(out, cx,cy,z1, 0,0,1);
        pushV(out, cx+r*cosf(a0),cy+r*sinf(a0),z1, 0,0,1);
        pushV(out, cx+r*cosf(a1),cy+r*sinf(a1),z1, 0,0,1);
    }
    // Bottom cap
    for (int i = 0; i < nseg; i++) {
        float a0 = 2.0f * (float)M_PI * i / nseg;
        float a1 = 2.0f * (float)M_PI * (i + 1) / nseg;
        pushV(out, cx,cy,z0, 0,0,-1);
        pushV(out, cx+r*cosf(a1),cy+r*sinf(a1),z0, 0,0,-1);
        pushV(out, cx+r*cosf(a0),cy+r*sinf(a0),z0, 0,0,-1);
    }
}

// ─── Renderer implementation ────────────────────────────────────────
bool Renderer::init(int width, int height) {
    m_width = width; m_height = height;
    compileShaders();
    glEnable(GL_DEPTH_TEST);
    if (m_backfaceCulling) glEnable(GL_CULL_FACE);
    else glDisable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    return true;
}

void Renderer::compileShaders() {
    auto vs = compileShader(vertSrc, GL_VERTEX_SHADER);
    auto fs = compileShader(fragSrc, GL_FRAGMENT_SHADER);
    m_mainShader = linkProgram(vs, fs);
}

void Renderer::shutdown() {
    cleanupSegments();
    if (m_benderVAO) { glDeleteVertexArrays(1, &m_benderVAO); m_benderVAO = 0; }
    if (m_benderVBO) { glDeleteBuffers(1, &m_benderVBO); m_benderVBO = 0; }
    if (m_rollerVAO) { glDeleteVertexArrays(1, &m_rollerVAO); m_rollerVAO = 0; }
    if (m_rollerVBO) { glDeleteBuffers(1, &m_rollerVBO); m_rollerVBO = 0; }
    if (m_clampVAO) { glDeleteVertexArrays(1, &m_clampVAO); m_clampVAO = 0; }
    if (m_clampVBO) { glDeleteBuffers(1, &m_clampVBO); m_clampVBO = 0; }
    if (m_staticVAO) { glDeleteVertexArrays(1, &m_staticVAO); m_staticVAO = 0; }
    if (m_staticVBO) { glDeleteBuffers(1, &m_staticVBO); m_staticVBO = 0; }
    if (m_carriageVAO) { glDeleteVertexArrays(1, &m_carriageVAO); m_carriageVAO = 0; }
    if (m_carriageVBO) { glDeleteBuffers(1, &m_carriageVBO); m_carriageVBO = 0; }
    if (m_laserVAO) { glDeleteVertexArrays(1, &m_laserVAO); m_laserVAO = 0; }
    if (m_laserVBO) { glDeleteBuffers(1, &m_laserVBO); m_laserVBO = 0; }
    if (m_collisionVAO) { glDeleteVertexArrays(1, &m_collisionVAO); m_collisionVAO = 0; }
    if (m_collisionVBO) { glDeleteBuffers(1, &m_collisionVBO); m_collisionVBO = 0; }
    if (m_mainShader) { glDeleteProgram(m_mainShader); m_mainShader = 0; }
    // FBO cleanup
    if (m_fbo) { glDeleteFramebuffers(1, &m_fbo); m_fbo = 0; }
    if (m_fboTexture) { glDeleteTextures(1, &m_fboTexture); m_fboTexture = 0; }
    if (m_fboDepth) { glDeleteRenderbuffers(1, &m_fboDepth); m_fboDepth = 0; }
    m_fboW = m_fboH = 0;
}

void Renderer::resize(int w, int h) { m_width = w; m_height = h; glViewport(0, 0, w, h); }

void Renderer::setCamera(float yaw, float pitch, float dist, float cx, float cy, float cz) {
    m_yaw = yaw; m_pitch = pitch; m_dist = dist;
    m_cx = cx; m_cy = cy; m_cz = cz;
}

void Renderer::orbit(float dYaw, float dPitch) {
    m_yaw += dYaw; m_pitch += dPitch;
    m_pitch = std::clamp(m_pitch, -89.0f, 89.0f);
}

void Renderer::zoom(float delta) {
    m_dist *= (1.0f - delta * 0.1f);
    m_dist = std::clamp(m_dist, 10.0f, 5000.0f);
}

void Renderer::pan(float dx, float dy) {
    float yawRad = m_yaw * (float)M_PI / 180.f;
    float pitchRad = m_pitch * (float)M_PI / 180.f;
    float speed = m_dist * 0.001f;
    // Screen-right vector (world space)
    float rx = -sinf(yawRad);
    float ry =  cosf(yawRad);
    // Screen-up vector (world space, accounts for pitch)
    float ux = -cosf(yawRad) * sinf(pitchRad);
    float uy = -sinf(yawRad) * sinf(pitchRad);
    float uz =  cosf(pitchRad);
    m_cx += (rx * dx - ux * dy) * speed;
    m_cy += (ry * dx - uy * dy) * speed;
    m_cz -= uz * dy * speed;
}

void Renderer::getViewMatrix(float* m) const {
    float yawRad   = m_yaw * (float)M_PI / 180.f;
    float pitchRad = m_pitch * (float)M_PI / 180.f;
    float ex = m_cx + m_dist * cosf(pitchRad) * cosf(yawRad);
    float ey = m_cy + m_dist * cosf(pitchRad) * sinf(yawRad);
    float ez = m_cz + m_dist * sinf(pitchRad);
    lookAt(m, ex, ey, ez, m_cx, m_cy, m_cz, 0, 0, 1);
}

void Renderer::getProjMatrix(float* m) const {
    float aspect = (m_height > 0) ? (float)m_width / m_height : 1.0f;
    float farPlane = std::max(100000.0f, m_dist + 120000.0f);
    perspective(m, 45.0f * (float)M_PI / 180.f, aspect, 1.0f, farPlane);
}

void Renderer::cleanupSegments() {
    for (auto& s : m_segments) {
        if (s.vao) glDeleteVertexArrays(1, &s.vao);
        if (s.vbo) glDeleteBuffers(1, &s.vbo);
        if (s.ebo) glDeleteBuffers(1, &s.ebo);
    }
    m_segments.clear();
}

void Renderer::clearScene() { cleanupSegments(); }

void Renderer::uploadMesh(GPUSegment& gpu, const TubeSegmentMesh& mesh) {
    glGenVertexArrays(1, &gpu.vao);
    glGenBuffers(1, &gpu.vbo);
    glGenBuffers(1, &gpu.ebo);

    glBindVertexArray(gpu.vao);
    glBindBuffer(GL_ARRAY_BUFFER, gpu.vbo);
    glBufferData(GL_ARRAY_BUFFER, mesh.vertices.size() * sizeof(Vertex), mesh.vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gpu.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.indices.size() * sizeof(uint32_t), mesh.indices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
    gpu.indexCount = (int)mesh.indices.size();
    gpu.stepId = mesh.stepId;
    gpu.type = mesh.type;
}

void Renderer::uploadTubeSegments(const std::vector<TubeSegmentMesh>& segs) {
    cleanupSegments();
    for (auto& s : segs) {
        GPUSegment gpu;
        uploadMesh(gpu, s);
        m_segments.push_back(gpu);
    }
}

void Renderer::uploadBenderSTL(const StlMesh& stlMesh) {
    if (m_benderVAO) { glDeleteVertexArrays(1, &m_benderVAO); m_benderVAO = 0; }
    if (m_benderVBO) { glDeleteBuffers(1, &m_benderVBO); m_benderVBO = 0; }
    if (stlMesh.vertices.empty()) return;

    glGenVertexArrays(1, &m_benderVAO);
    glGenBuffers(1, &m_benderVBO);
    glBindVertexArray(m_benderVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_benderVBO);
    glBufferData(GL_ARRAY_BUFFER, stlMesh.vertices.size() * sizeof(float), stlMesh.vertices.data(), GL_STATIC_DRAW);
    // Stride: 6 floats (pos + normal)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
    m_benderVertCount = (int)(stlMesh.vertices.size() / 6);
}

// ─── Logic 2 model uploads ──────────────────────────────────────────
static void uploadGenericSTL(unsigned int& vao, unsigned int& vbo, int& vertCount, const StlMesh& mesh) {
    if (vao) { glDeleteVertexArrays(1, &vao); vao = 0; }
    if (vbo) { glDeleteBuffers(1, &vbo); vbo = 0; }
    if (mesh.vertices.empty()) return;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, mesh.vertices.size() * sizeof(float), mesh.vertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
    vertCount = (int)(mesh.vertices.size() / 6);
}

void Renderer::uploadRollerSTL(const StlMesh& mesh) {
    uploadGenericSTL(m_rollerVAO, m_rollerVBO, m_rollerVertCount, mesh);
}
void Renderer::uploadClampSTL(const StlMesh& mesh) {
    uploadGenericSTL(m_clampVAO, m_clampVBO, m_clampVertCount, mesh);
}
void Renderer::uploadStaticSTL(const StlMesh& mesh) {
    uploadGenericSTL(m_staticVAO, m_staticVBO, m_staticVertCount, mesh);
}
void Renderer::uploadCarriageSTL(const StlMesh& mesh) {
    uploadGenericSTL(m_carriageVAO, m_carriageVBO, m_carriageVertCount, mesh);
}

void Renderer::beginFrame() {
    glViewport(0, 0, m_width, m_height);
    glClearColor(0.28f, 0.28f, 0.31f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

// ─── FBO management ─────────────────────────────────────────────────
void Renderer::ensureFBO(int w, int h) {
    if (w <= 0) w = 1;
    if (h <= 0) h = 1;
    if (m_fbo && m_fboW == w && m_fboH == h) return; // already correct size

    // Delete old
    if (m_fbo) { glDeleteFramebuffers(1, &m_fbo); m_fbo = 0; }
    if (m_fboTexture) { glDeleteTextures(1, &m_fboTexture); m_fboTexture = 0; }
    if (m_fboDepth) { glDeleteRenderbuffers(1, &m_fboDepth); m_fboDepth = 0; }

    m_fboW = w;
    m_fboH = h;

    // Create color texture
    glGenTextures(1, &m_fboTexture);
    glBindTexture(GL_TEXTURE_2D, m_fboTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    // Create depth renderbuffer
    glGenRenderbuffers(1, &m_fboDepth);
    glBindRenderbuffer(GL_RENDERBUFFER, m_fboDepth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, w, h);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    // Create FBO
    glGenFramebuffers(1, &m_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_fboTexture, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, m_fboDepth);

    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "FBO incomplete: 0x" << std::hex << status << std::dec << "\n";
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Renderer::renderToFBO() {
    if (!m_fbo || m_fboW <= 0 || m_fboH <= 0) return;

    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glViewport(0, 0, m_fboW, m_fboH);
    glClearColor(0.28f, 0.28f, 0.31f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    renderScene(0, 0, m_fboW, m_fboH);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Renderer::renderScene(int vpX, int vpY, int vpW, int vpH) {
    if (vpW <= 0 || vpH <= 0) return;
    glViewport(vpX, vpY, vpW, vpH);
    glEnable(GL_SCISSOR_TEST);
    glScissor(vpX, vpY, vpW, vpH);
    if (m_backfaceCulling) glEnable(GL_CULL_FACE);
    else glDisable(GL_CULL_FACE);

    float view[16], proj[16], mvp[16], model[16];
    identity(model);
    getViewMatrix(view);

    // Use viewport aspect ratio for projection
    float aspect = (float)vpW / (float)vpH;
    float p[16];
    memset(p, 0, 16*sizeof(float));
    float f = 1.0f / tanf(45.0f * (float)M_PI / 180.0f * 0.5f);
    float farPlane = std::max(100000.0f, m_dist + 120000.0f);
    p[0] = f / aspect;
    p[5] = f;
    p[10] = (farPlane + 1.0f) / (1.0f - farPlane);
    p[11] = -1.0f;
    p[14] = 2.0f * farPlane * 1.0f / (1.0f - farPlane);
    memcpy(proj, p, sizeof(proj));

    mulMat(mvp, proj, view);

    // Camera position for specular
    float yawRad = m_yaw * (float)M_PI / 180.f;
    float pitchRad = m_pitch * (float)M_PI / 180.f;
    float camX = m_cx + m_dist * cosf(pitchRad) * cosf(yawRad);
    float camY = m_cy + m_dist * cosf(pitchRad) * sinf(yawRad);
    float camZ = m_cz + m_dist * sinf(pitchRad);

    glUseProgram(m_mainShader);

    // Identity normal matrix (for bender / laser head later)
    float normMat[9] = {1,0,0, 0,1,0, 0,0,1};

    glUniform3f(glGetUniformLocation(m_mainShader, "uLightDir"), 0.3f, -0.5f, 0.8f);
    glUniform3f(glGetUniformLocation(m_mainShader, "uCamPos"), camX, camY, camZ);
    glUniform1f(glGetUniformLocation(m_mainShader, "uSpecular"), 0.3f); // default specular for all objects

    // ── Checkered floor ─────────────────────────────────────────────
    if (m_showFloor && m_floorGenerated) {
        glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uMVP"), 1, GL_FALSE, mvp);
        glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uModel"), 1, GL_FALSE, model);
        glUniformMatrix3fv(glGetUniformLocation(m_mainShader, "uNormalMat"), 1, GL_FALSE, normMat);
        glUniform1f(glGetUniformLocation(m_mainShader, "uAlpha"), 1.0f);
        glUniform1f(glGetUniformLocation(m_mainShader, "uSpecular"), 0.0f); // no specular on floor
        glUniform3f(glGetUniformLocation(m_mainShader, "uColor"), 0.22f, 0.22f, 0.22f);
        glBindVertexArray(m_floorVAO[0]);
        glDrawArrays(GL_TRIANGLES, 0, m_floorVertCount[0]);
        glUniform3f(glGetUniformLocation(m_mainShader, "uColor"), 0.16f, 0.16f, 0.16f);
        glBindVertexArray(m_floorVAO[1]);
        glDrawArrays(GL_TRIANGLES, 0, m_floorVertCount[1]);
        glUniform1f(glGetUniformLocation(m_mainShader, "uSpecular"), 0.3f); // restore specular
    }

    // Tube MVP/Model/Normal — may include laser cut animation transform
    if (m_hasTubeTransform) {
        float tubeMVP[16], temp[16];
        mulMat(temp, view, m_tubeModel);
        mulMat(tubeMVP, proj, temp);
        glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uMVP"), 1, GL_FALSE, tubeMVP);
        glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uModel"), 1, GL_FALSE, m_tubeModel);
        glUniformMatrix3fv(glGetUniformLocation(m_mainShader, "uNormalMat"), 1, GL_FALSE, m_tubeNormMat);
    } else {
        glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uMVP"), 1, GL_FALSE, mvp);
        glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uModel"), 1, GL_FALSE, model);
        glUniformMatrix3fv(glGetUniformLocation(m_mainShader, "uNormalMat"), 1, GL_FALSE, normMat);
    }

    // Render tube segments
    for (auto& seg : m_segments) {
        float r, g, b;
        switch (seg.type) {
            case TubeSegmentMesh::Blue:  r=0.12f; g=0.56f; b=1.0f;  break;
            case TubeSegmentMesh::Red:   r=1.0f;  g=0.27f; b=0.0f;  break;
            case TubeSegmentMesh::Green: r=0.0f;  g=0.8f;  b=0.0f;  break;
            case TubeSegmentMesh::Gray:  r=0.45f; g=0.45f; b=0.45f; break;
        }
        glUniform3f(glGetUniformLocation(m_mainShader, "uColor"), r, g, b);
        glUniform1f(glGetUniformLocation(m_mainShader, "uAlpha"), 1.0f);

        glBindVertexArray(seg.vao);
        glDrawElements(GL_TRIANGLES, seg.indexCount, GL_UNSIGNED_INT, 0);
    }

    // Restore identity transforms after tube rendering
    if (m_hasTubeTransform) {
        glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uMVP"), 1, GL_FALSE, mvp);
        glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uModel"), 1, GL_FALSE, model);
        glUniformMatrix3fv(glGetUniformLocation(m_mainShader, "uNormalMat"), 1, GL_FALSE, normMat);
    }

    // Render bender STL (with rotation around Z)
    if (m_showBender && m_benderVAO && m_benderVertCount > 0) {
        float benderAlpha = m_benderOpaque ? 1.0f : 0.35f;
        if (!m_benderOpaque) glDepthMask(GL_FALSE);

        // Build bender model matrix: RotateZ(benderAngle) around origin
        float benderModel[16];
        identity(benderModel);
        if (std::abs(m_benderAngleDeg) > 0.01f) {
            float a = m_benderAngleDeg * (float)M_PI / 180.0f;
            float ca = cosf(a), sa = sinf(a);
            benderModel[0] = ca;  benderModel[4] = -sa;
            benderModel[1] = sa;  benderModel[5] = ca;
            // benderModel[10] = 1, benderModel[15] = 1 already set by identity
        }
        float benderMVP[16];
        float temp[16];
        mulMat(temp, view, benderModel);  // temp = view * benderModel
        mulMat(benderMVP, proj, temp);    // benderMVP = proj * view * benderModel
        glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uMVP"), 1, GL_FALSE, benderMVP);
        glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uModel"), 1, GL_FALSE, benderModel);

        // Normal matrix for bender (2D rotation around Z)
        float benderNorm[9] = {1,0,0, 0,1,0, 0,0,1};
        if (std::abs(m_benderAngleDeg) > 0.01f) {
            float a = m_benderAngleDeg * (float)M_PI / 180.0f;
            float ca = cosf(a), sa = sinf(a);
            benderNorm[0] = ca;  benderNorm[3] = -sa;
            benderNorm[1] = sa;  benderNorm[4] = ca;
        }
        glUniformMatrix3fv(glGetUniformLocation(m_mainShader, "uNormalMat"), 1, GL_FALSE, benderNorm);

        glUniform3f(glGetUniformLocation(m_mainShader, "uColor"), 0.6f, 0.6f, 0.6f);
        glUniform1f(glGetUniformLocation(m_mainShader, "uAlpha"), benderAlpha);
        glBindVertexArray(m_benderVAO);
        glDrawArrays(GL_TRIANGLES, 0, m_benderVertCount);
        if (!m_benderOpaque) glDepthMask(GL_TRUE);

        // Restore identity transforms for subsequent renders
        glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uMVP"), 1, GL_FALSE, mvp);
        glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uModel"), 1, GL_FALSE, model);
        glUniformMatrix3fv(glGetUniformLocation(m_mainShader, "uNormalMat"), 1, GL_FALSE, normMat);
    }

    // ── Logic 2: render roller + clamp + static frame ──────────────
    if (m_showLogic2) {
        float l2Alpha = m_benderOpaque ? 1.0f : 0.55f;
        float l2StaticAlpha = m_benderOpaque ? 1.0f : 0.35f;
        if (!m_benderOpaque) glDepthMask(GL_FALSE);

        // Build rotation matrix around Z (same as bender)
        float rotZ[16];
        identity(rotZ);
        float rotNorm[9] = {1,0,0, 0,1,0, 0,0,1};
        if (std::abs(m_benderAngleDeg) > 0.01f) {
            float a = m_benderAngleDeg * (float)M_PI / 180.0f;
            float ca = cosf(a), sa = sinf(a);
            rotZ[0] = ca;  rotZ[4] = -sa;
            rotZ[1] = sa;  rotZ[5] = ca;
            rotNorm[0] = ca; rotNorm[3] = -sa;
            rotNorm[1] = sa; rotNorm[4] = ca;
        }

        // 1) Roller — rotates around Z (same as bender axis)
        if (m_rollerVAO && m_rollerVertCount > 0) {
            float rMVP[16], temp2[16];
            mulMat(temp2, view, rotZ);
            mulMat(rMVP, proj, temp2);
            glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uMVP"), 1, GL_FALSE, rMVP);
            glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uModel"), 1, GL_FALSE, rotZ);
            glUniformMatrix3fv(glGetUniformLocation(m_mainShader, "uNormalMat"), 1, GL_FALSE, rotNorm);
            glUniform3f(glGetUniformLocation(m_mainShader, "uColor"), 0.5f, 0.55f, 0.6f);
            glUniform1f(glGetUniformLocation(m_mainShader, "uAlpha"), l2Alpha);
            glBindVertexArray(m_rollerVAO);
            glDrawArrays(GL_TRIANGLES, 0, m_rollerVertCount);
        }

        // 2) Clamp — fixed to the roller, opens/closes only along its local X axis
        if (m_clampVAO && m_clampVertCount > 0) {
            // Keep the v2.1 behavior: local X translation first, then roller rotation.
            // This keeps the clamp rigidly attached to the die and prevents teleporting.
            float trans[16];
            identity(trans);
            trans[12] = m_clampXOffset;
            float clampModel[16];
            mulMat(clampModel, rotZ, trans);
            float clampNorm[9];
            memcpy(clampNorm, rotNorm, 9 * sizeof(float));
            float cMVP[16], temp2[16];
            mulMat(temp2, view, clampModel);
            mulMat(cMVP, proj, temp2);
            glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uMVP"), 1, GL_FALSE, cMVP);
            glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uModel"), 1, GL_FALSE, clampModel);
            glUniformMatrix3fv(glGetUniformLocation(m_mainShader, "uNormalMat"), 1, GL_FALSE, clampNorm);
            glUniform3f(glGetUniformLocation(m_mainShader, "uColor"), 0.6f, 0.5f, 0.4f);
            glUniform1f(glGetUniformLocation(m_mainShader, "uAlpha"), l2Alpha);
            glBindVertexArray(m_clampVAO);
            glDrawArrays(GL_TRIANGLES, 0, m_clampVertCount);
        }

        // 3) Carriage — translates along Y by m_carriageYOffset
        if (m_carriageVAO && m_carriageVertCount > 0) {
            float carrModel[16];
            identity(carrModel);
            carrModel[13] = -m_carriageYOffset;  // translate in Y (feed)
            float crMVP[16], temp2[16];
            mulMat(temp2, view, carrModel);
            mulMat(crMVP, proj, temp2);
            glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uMVP"), 1, GL_FALSE, crMVP);
            glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uModel"), 1, GL_FALSE, carrModel);
            glUniformMatrix3fv(glGetUniformLocation(m_mainShader, "uNormalMat"), 1, GL_FALSE, normMat);
            glUniform3f(glGetUniformLocation(m_mainShader, "uColor"), 0.4f, 0.5f, 0.55f);
            glUniform1f(glGetUniformLocation(m_mainShader, "uAlpha"), l2Alpha);
            glBindVertexArray(m_carriageVAO);
            glDrawArrays(GL_TRIANGLES, 0, m_carriageVertCount);
        }

        // 4) Static frame — no movement
        if (m_staticVAO && m_staticVertCount > 0) {
            glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uMVP"), 1, GL_FALSE, mvp);
            glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uModel"), 1, GL_FALSE, model);
            glUniformMatrix3fv(glGetUniformLocation(m_mainShader, "uNormalMat"), 1, GL_FALSE, normMat);
            glUniform3f(glGetUniformLocation(m_mainShader, "uColor"), 0.45f, 0.45f, 0.45f);
            glUniform1f(glGetUniformLocation(m_mainShader, "uAlpha"), l2StaticAlpha);
            glBindVertexArray(m_staticVAO);
            glDrawArrays(GL_TRIANGLES, 0, m_staticVertCount);
        }

        if (!m_benderOpaque) glDepthMask(GL_TRUE);
        // Restore identity transforms
        glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uMVP"), 1, GL_FALSE, mvp);
        glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uModel"), 1, GL_FALSE, model);
        glUniformMatrix3fv(glGetUniformLocation(m_mainShader, "uNormalMat"), 1, GL_FALSE, normMat);
    }

    // ── Exclusion zone cylinder + sphere (always opaque, unaffected by benderOpaque) ──
    if (m_showExclZone && m_exclVAO && m_exclVertCount > 0) {
        glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uMVP"), 1, GL_FALSE, mvp);
        glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uModel"), 1, GL_FALSE, model);
        glUniformMatrix3fv(glGetUniformLocation(m_mainShader, "uNormalMat"), 1, GL_FALSE, normMat);
        glUniform3f(glGetUniformLocation(m_mainShader, "uColor"), 0.0f, 0.8f, 0.8f); // cyan
        glUniform1f(glGetUniformLocation(m_mainShader, "uAlpha"), 1.0f);
        glBindVertexArray(m_exclVAO);
        glDrawArrays(GL_TRIANGLES, 0, m_exclVertCount);
    }

    // Collision preview marker
    if (m_showCollisionMarker && m_collisionVAO && m_collisionVertCount > 0) {
        glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uMVP"), 1, GL_FALSE, mvp);
        glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uModel"), 1, GL_FALSE, model);
        glUniformMatrix3fv(glGetUniformLocation(m_mainShader, "uNormalMat"), 1, GL_FALSE, normMat);
        glUniform1f(glGetUniformLocation(m_mainShader, "uAlpha"), 1.0f);
        glUniform1f(glGetUniformLocation(m_mainShader, "uSpecular"), 0.0f);
        if (m_collisionMarkerIsFloor)
            glUniform3f(glGetUniformLocation(m_mainShader, "uColor"), 1.0f, 0.55f, 0.10f);
        else
            glUniform3f(glGetUniformLocation(m_mainShader, "uColor"), 1.0f, 0.15f, 0.15f);
        glBindVertexArray(m_collisionVAO);
        glDrawArrays(GL_TRIANGLES, 0, m_collisionVertCount);
        glUniform1f(glGetUniformLocation(m_mainShader, "uSpecular"), 0.3f);
    }

    // Render laser head / beam (after bender, same identity transforms)
    if (m_showLaserHead && m_laserVAO && m_laserHeadVertCount > 0) {
        glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uMVP"), 1, GL_FALSE, mvp);
        glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uModel"), 1, GL_FALSE, model);
        glUniformMatrix3fv(glGetUniformLocation(m_mainShader, "uNormalMat"), 1, GL_FALSE, normMat);

        // Laser head body (dark red, semi-transparent)
        glDepthMask(GL_FALSE);
        glUniform3f(glGetUniformLocation(m_mainShader, "uColor"), 0.7f, 0.2f, 0.2f);
        glUniform1f(glGetUniformLocation(m_mainShader, "uAlpha"), 0.75f);
        glBindVertexArray(m_laserVAO);
        glDrawArrays(GL_TRIANGLES, 0, m_laserHeadVertCount);
        glDepthMask(GL_TRUE);

        // Laser beam (bright red, opaque, only during cut phases)
        if (m_showLaserBeam && m_laserBeamVertCount > 0) {
            glUniform3f(glGetUniformLocation(m_mainShader, "uColor"), 1.0f, 0.0f, 0.0f);
            glUniform1f(glGetUniformLocation(m_mainShader, "uAlpha"), 1.0f);
            glDrawArrays(GL_TRIANGLES, m_laserHeadVertCount, m_laserBeamVertCount);
        }
    }

    // Render cut contour on tube surface (bright yellow, during cut phases)
    if (m_showLaserBeam && m_cutContourVAO && m_cutContourVertCount > 0) {
        // Apply tube transform if active
        if (m_hasTubeTransform) {
            float tubeMVP[16], temp[16];
            mulMat(temp, view, m_tubeModel);
            mulMat(tubeMVP, proj, temp);
            glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uMVP"), 1, GL_FALSE, tubeMVP);
            glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uModel"), 1, GL_FALSE, m_tubeModel);
            glUniformMatrix3fv(glGetUniformLocation(m_mainShader, "uNormalMat"), 1, GL_FALSE, m_tubeNormMat);
        }
        glUniform3f(glGetUniformLocation(m_mainShader, "uColor"), 1.0f, 1.0f, 0.0f);
        glUniform1f(glGetUniformLocation(m_mainShader, "uAlpha"), 1.0f);
        glBindVertexArray(m_cutContourVAO);
        glDrawArrays(GL_TRIANGLES, 0, m_cutContourVertCount);
        // Restore identity transforms
        if (m_hasTubeTransform) {
            glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uMVP"), 1, GL_FALSE, mvp);
            glUniformMatrix4fv(glGetUniformLocation(m_mainShader, "uModel"), 1, GL_FALSE, model);
            glUniformMatrix3fv(glGetUniformLocation(m_mainShader, "uNormalMat"), 1, GL_FALSE, normMat);
        }
    }

    glBindVertexArray(0);
    glDisable(GL_SCISSOR_TEST);
}

void Renderer::endFrame() {
    // Nothing — GLFW swap is handled in main loop
}

int Renderer::pickSegment(int mouseX, int mouseY) {
    // Simple color-based picking could be added later
    return -1;
}

// ─── Laser head / beam geometry ─────────────────────────────────────
void Renderer::setLaserHeadPos(float x, float y, float z, float tubeTopZ) {
    if (m_laserVAO != 0 &&
        m_laserX == x && m_laserY == y && m_laserZ == z && m_laserTubeTopZ == tubeTopZ)
        return; // unchanged

    m_laserX = x; m_laserY = y; m_laserZ = z; m_laserTubeTopZ = tubeTopZ;

    // Cleanup old
    if (m_laserVAO) { glDeleteVertexArrays(1, &m_laserVAO); m_laserVAO = 0; }
    if (m_laserVBO) { glDeleteBuffers(1, &m_laserVBO); m_laserVBO = 0; }

    std::vector<float> verts;

    // Laser head body: cylinder, 10mm diameter, 40mm tall, centered at (x,y), from z to z+40
    generateCylinder(verts, x, y, z, z + 40.0f, 5.0f, 8);
    m_laserHeadVertCount = (int)(verts.size() / 6);

    // Laser beam: thin cylinder, 0.6mm diameter, from tubeTopZ up to z
    generateCylinder(verts, x, y, tubeTopZ, z, 0.3f, 4);
    m_laserBeamVertCount = (int)(verts.size() / 6) - m_laserHeadVertCount;

    // Upload
    glGenVertexArrays(1, &m_laserVAO);
    glGenBuffers(1, &m_laserVBO);
    glBindVertexArray(m_laserVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_laserVBO);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
}

// ─── Cut contour on tube surface ────────────────────────────────────
void Renderer::setCutContour(const std::vector<float>& contourYOffsets,
                             float tubeCX, float tubeCZ, float tubeR, float laserY)
{
    // Cleanup old
    if (m_cutContourVAO) { glDeleteVertexArrays(1, &m_cutContourVAO); m_cutContourVAO = 0; }
    if (m_cutContourVBO) { glDeleteBuffers(1, &m_cutContourVBO); m_cutContourVBO = 0; }
    m_cutContourVertCount = 0;

    if (contourYOffsets.empty()) return;

    int nseg = (int)contourYOffsets.size();
    std::vector<float> verts;
    float stripR = tubeR + 0.15f;   // slightly outside tube surface
    float stripW = 1.2f;            // 1.2mm wide ribbon on tube surface

    for (int i = 0; i < nseg; i++) {
        int i1 = (i + 1) % nseg;
        float a0 = 2.0f * (float)M_PI * i / nseg;
        float a1 = 2.0f * (float)M_PI * i1 / nseg;
        float c0 = cosf(a0), s0 = sinf(a0);
        float c1 = cosf(a1), s1 = sinf(a1);

        // Y positions from contourYOffsets (absolute mesh Y)
        float y0 = laserY + contourYOffsets[i];
        float y1 = laserY + contourYOffsets[i1];

        // Center of strip on tube surface — match tube_geometry frame:
        // For dir=(0,-1,0): tubeX=(1,0,0), tubeZ=(0,0,1)
        //   n(θ) = tubeZ·cos(θ) + tubeX·sin(θ) = (sin(θ), 0, cos(θ))
        float x0 = tubeCX + stripR * s0, z0 = tubeCZ + stripR * c0;
        float x1 = tubeCX + stripR * s1, z1 = tubeCZ + stripR * c1;

        // Ribbon: inner and outer edges (offset in Y direction)
        float halfW = stripW * 0.5f;
        float x0i = x0, z0i = z0, y0i = y0 - halfW;
        float x0o = x0, z0o = z0, y0o = y0 + halfW;
        float x1i = x1, z1i = z1, y1i = y1 - halfW;
        float x1o = x1, z1o = z1, y1o = y1 + halfW;

        // Normal pointing radially outward (matching tube frame)
        float nx0 = s0, nz0 = c0;
        float nx1 = s1, nz1 = c1;

        // Two triangles for this segment
        pushV(verts, x0i, y0i, z0i, nx0, 0, nz0);
        pushV(verts, x0o, y0o, z0o, nx0, 0, nz0);
        pushV(verts, x1i, y1i, z1i, nx1, 0, nz1);
        pushV(verts, x0o, y0o, z0o, nx0, 0, nz0);
        pushV(verts, x1o, y1o, z1o, nx1, 0, nz1);
        pushV(verts, x1i, y1i, z1i, nx1, 0, nz1);
    }

    m_cutContourVertCount = (int)(verts.size() / 6);
    if (m_cutContourVertCount == 0) return;

    glGenVertexArrays(1, &m_cutContourVAO);
    glGenBuffers(1, &m_cutContourVBO);
    glBindVertexArray(m_cutContourVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_cutContourVBO);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
}

void Renderer::clearCutContour() {
    if (m_cutContourVAO) { glDeleteVertexArrays(1, &m_cutContourVAO); m_cutContourVAO = 0; }
    if (m_cutContourVBO) { glDeleteBuffers(1, &m_cutContourVBO); m_cutContourVBO = 0; }
    m_cutContourVertCount = 0;
}

void Renderer::setCollisionMarker(float x, float y, float z, bool isFloor, float floorZ) {
    m_showCollisionMarker = true;
    m_collisionMarkerIsFloor = isFloor;

    if (m_collisionVAO) { glDeleteVertexArrays(1, &m_collisionVAO); m_collisionVAO = 0; }
    if (m_collisionVBO) { glDeleteBuffers(1, &m_collisionVBO); m_collisionVBO = 0; }

    std::vector<float> verts;
    const float markerHalf = 14.0f;
    generateCylinder(verts, x, y, z - markerHalf, z + markerHalf, 4.5f, 12);

    float stemTop = z - markerHalf - 2.0f;
    float stemBottom = std::min(stemTop, floorZ);
    if (stemTop > stemBottom + 0.5f)
        generateCylinder(verts, x, y, stemBottom, stemTop, 1.5f, 8);

    m_collisionVertCount = (int)(verts.size() / 6);
    if (m_collisionVertCount <= 0) {
        m_showCollisionMarker = false;
        return;
    }

    glGenVertexArrays(1, &m_collisionVAO);
    glGenBuffers(1, &m_collisionVBO);
    glBindVertexArray(m_collisionVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_collisionVBO);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
}

void Renderer::clearCollisionMarker() {
    m_showCollisionMarker = false;
    if (m_collisionVAO) { glDeleteVertexArrays(1, &m_collisionVAO); m_collisionVAO = 0; }
    if (m_collisionVBO) { glDeleteBuffers(1, &m_collisionVBO); m_collisionVBO = 0; }
    m_collisionVertCount = 0;
}

// ─── Tube transform for laser cut animation ─────────────────────────
void Renderer::setTubeTransform(float rotYDeg, float feedY,
                                float px, float py, float pz)
{
    m_hasTubeTransform = true;

    float a = rotYDeg * (float)M_PI / 180.0f;
    float ca = cosf(a), sa = sinf(a);

    // Combined: M = T(0,-feedY,0) * T(pivot) * Ry(angle) * T(-pivot)
    // M*v = Ry*(v - pivot) + pivot + (0, -feedY, 0)
    //     = Ry*v + (-Ry*pivot + pivot + (0,-feedY,0))
    float rpx = ca * px + sa * pz;   // Ry * pivot
    float rpy = py;
    float rpz = -sa * px + ca * pz;

    float tx = -rpx + px;
    float ty = -rpy + py - feedY;
    float tz = -rpz + pz;

    // Column-major 4x4
    identity(m_tubeModel);
    m_tubeModel[0]  = ca;   m_tubeModel[4]  = 0;  m_tubeModel[8]  = sa;   m_tubeModel[12] = tx;
    m_tubeModel[1]  = 0;    m_tubeModel[5]  = 1;  m_tubeModel[9]  = 0;    m_tubeModel[13] = ty;
    m_tubeModel[2]  = -sa;  m_tubeModel[6]  = 0;  m_tubeModel[10] = ca;   m_tubeModel[14] = tz;
    // [3],[7],[11]=0, [15]=1 — already set by identity

    // Normal matrix (3x3 = rotation part only)
    m_tubeNormMat[0] = ca;   m_tubeNormMat[3] = 0;  m_tubeNormMat[6] = sa;
    m_tubeNormMat[1] = 0;    m_tubeNormMat[4] = 1;  m_tubeNormMat[7] = 0;
    m_tubeNormMat[2] = -sa;  m_tubeNormMat[5] = 0;  m_tubeNormMat[8] = ca;
}

void Renderer::clearTubeTransform() {
    m_hasTubeTransform = false;
    identity(m_tubeModel);
    m_tubeNormMat[0]=1; m_tubeNormMat[1]=0; m_tubeNormMat[2]=0;
    m_tubeNormMat[3]=0; m_tubeNormMat[4]=1; m_tubeNormMat[5]=0;
    m_tubeNormMat[6]=0; m_tubeNormMat[7]=0; m_tubeNormMat[8]=1;
}

// ─── Exclusion zone: 3 sections ─────────────────────────────────────
// 1. Static cylinder Y=0..+R (feed channel above Point0)
// 2. Torus arc 0°..bendAngle
// 3. Rotating cylinder at end of torus (clamp, moves with roller)
void Renderer::updateExclZone(float cx, float cz, float radius,
                               float yMin, float bendR, float bendAngleDeg) {
    const int segs = 32;
    const float rW = 0.5f;
    float clampLen = -yMin; // positive length of clamp section
    float bcx = cx + bendR;  // bend center X
    float bRad = bendAngleDeg * (float)M_PI / 180.0f;
    std::vector<float> verts;
    auto pv = [&](float x, float y, float z, float nx, float ny, float nz) {
        verts.push_back(x); verts.push_back(y); verts.push_back(z);
        verts.push_back(nx); verts.push_back(ny); verts.push_back(nz);
    };

    // ── Section 1: Static cylinder Y from 0 to +bendR at (cx, *, cz) ──
    for (int i = 0; i < segs; i++) {
        float a = 2.0f * (float)M_PI * i / segs;
        float c = cosf(a), s = sinf(a);
        float x = cx + radius * c, z = cz + radius * s;
        float ox = -s * rW, oz = c * rW;
        pv(x-ox, 0,       z-oz, c,0,s);
        pv(x+ox, 0,       z+oz, c,0,s);
        pv(x+ox, bendR,   z+oz, c,0,s);
        pv(x-ox, 0,       z-oz, c,0,s);
        pv(x+ox, bendR,   z+oz, c,0,s);
        pv(x-ox, bendR,   z-oz, c,0,s);
    }
    // Cap ring at Y=+bendR
    for (int i = 0; i < segs; i++) {
        float a0 = 2.0f * (float)M_PI * i / segs;
        float a1 = 2.0f * (float)M_PI * (i+1) / segs;
        float x0 = cx + radius*cosf(a0), z0 = cz + radius*sinf(a0);
        float x1 = cx + radius*cosf(a1), z1 = cz + radius*sinf(a1);
        float ri = radius - rW*2;
        float x0i = cx + ri*cosf(a0), z0i = cz + ri*sinf(a0);
        float x1i = cx + ri*cosf(a1), z1i = cz + ri*sinf(a1);
        pv(x0i, bendR, z0i, 0,1,0);
        pv(x0,  bendR, z0,  0,1,0);
        pv(x1,  bendR, z1,  0,1,0);
        pv(x0i, bendR, z0i, 0,1,0);
        pv(x1,  bendR, z1,  0,1,0);
        pv(x1i, bendR, z1i, 0,1,0);
    }

    // ── Section 2: Torus arc 0° → bendAngleDeg ──
    if (bRad > 0.001f) {
        int aSegs = std::max(4, (int)(segs * bendAngleDeg / 360.0f));

        // Transverse rings along arc
        for (int a = 0; a <= aSegs; a++) {
            float t = bRad * a / aSegs;
            float ct = cosf(t), st = sinf(t);
            float oxx = st * rW, oyy = -ct * rW;
            for (int i = 0; i < segs; i++) {
                float p0 = 2.0f * (float)M_PI * i / segs;
                float p1 = 2.0f * (float)M_PI * (i+1) / segs;
                float rr0 = bendR + radius*cosf(p0);
                float rr1 = bendR + radius*cosf(p1);
                float x0 = bcx - rr0*ct, y0 = -rr0*st, z0 = cz + radius*sinf(p0);
                float x1 = bcx - rr1*ct, y1 = -rr1*st, z1 = cz + radius*sinf(p1);
                float nx0 = -cosf(p0)*ct, ny0 = -cosf(p0)*st, nz0 = sinf(p0);
                pv(x0-oxx, y0-oyy, z0, nx0,ny0,nz0);
                pv(x0+oxx, y0+oyy, z0, nx0,ny0,nz0);
                pv(x1+oxx, y1+oyy, z1, nx0,ny0,nz0);
                pv(x0-oxx, y0-oyy, z0, nx0,ny0,nz0);
                pv(x1+oxx, y1+oyy, z1, nx0,ny0,nz0);
                pv(x1-oxx, y1-oyy, z1, nx0,ny0,nz0);
            }
        }

        // Longitude lines along the arc
        int longN = std::max(8, segs / 2);
        for (int i = 0; i < longN; i++) {
            float phi = 2.0f * (float)M_PI * i / longN;
            float cp = cosf(phi), sp = sinf(phi);
            float rr = bendR + radius * cp;
            float pz = cz + radius * sp;
            for (int a = 0; a < aSegs; a++) {
                float t0 = bRad * a / aSegs;
                float t1 = bRad * (a+1) / aSegs;
                float x0 = bcx - rr*cosf(t0), y0 = -rr*sinf(t0);
                float x1 = bcx - rr*cosf(t1), y1 = -rr*sinf(t1);
                float tm = (t0+t1) * 0.5f;
                float nx = -cp*cosf(tm), ny = -cp*sinf(tm), nz = sp;
                pv(x0, y0, pz-rW, nx,ny,nz);
                pv(x0, y0, pz+rW, nx,ny,nz);
                pv(x1, y1, pz+rW, nx,ny,nz);
                pv(x0, y0, pz-rW, nx,ny,nz);
                pv(x1, y1, pz+rW, nx,ny,nz);
                pv(x1, y1, pz-rW, nx,ny,nz);
            }
        }
    }

    // ── Section 3: Rotating cylinder at end of torus (clamp) ──
    {
        float ct = cosf(bRad), st = sinf(bRad);
        // Arc endpoint at θ=bendAngle
        float ex = bcx - bendR * ct, ey = -bendR * st;
        // Tangent direction at θ: (sin(θ), -cos(θ), 0)
        float tx = st, ty = -ct;
        // Cross-section axes perpendicular to tangent:
        // u1 = (-cos(θ), -sin(θ), 0), u2 = (0, 0, 1)
        float u1x = -ct, u1y = -st;

        // Cylinder body — lines along tangent direction
        for (int i = 0; i < segs; i++) {
            float a = 2.0f * (float)M_PI * i / segs;
            float ca = cosf(a), sa = sinf(a);
            float offx = radius * ca * u1x, offy = radius * ca * u1y;
            float offz = radius * sa;
            float nx = ca * u1x, ny = ca * u1y, nz = sa;
            float sx = ex + offx, sy = ey + offy, sz = cz + offz;
            float fx = ex + clampLen*tx + offx, fy = ey + clampLen*ty + offy;
            float rwx = tx * rW, rwy = ty * rW;
            pv(sx-rwx, sy-rwy, sz,  nx,ny,nz);
            pv(sx+rwx, sy+rwy, sz,  nx,ny,nz);
            pv(fx+rwx, fy+rwy, sz,  nx,ny,nz);
            pv(sx-rwx, sy-rwy, sz,  nx,ny,nz);
            pv(fx+rwx, fy+rwy, sz,  nx,ny,nz);
            pv(fx-rwx, fy-rwy, sz,  nx,ny,nz);
        }
        // Transverse rings along clamp cylinder (start, middle, end)
        int ringCount = std::max(2, (int)(clampLen / 20.0f));
        for (int r = 0; r <= ringCount; r++) {
            float d = clampLen * r / ringCount;
            float rcx = ex + d * tx, rcy = ey + d * ty;
            for (int i = 0; i < segs; i++) {
                float a0 = 2.0f * (float)M_PI * i / segs;
                float a1 = 2.0f * (float)M_PI * (i+1) / segs;
                float x0 = rcx + radius*cosf(a0)*u1x, y0 = rcy + radius*cosf(a0)*u1y, z0 = cz + radius*sinf(a0);
                float x1 = rcx + radius*cosf(a1)*u1x, y1 = rcy + radius*cosf(a1)*u1y, z1 = cz + radius*sinf(a1);
                float ri = radius - rW*2;
                float x0i = rcx + ri*cosf(a0)*u1x, y0i = rcy + ri*cosf(a0)*u1y, z0i = cz + ri*sinf(a0);
                float x1i = rcx + ri*cosf(a1)*u1x, y1i = rcy + ri*cosf(a1)*u1y, z1i = cz + ri*sinf(a1);
                float rn = (r == ringCount) ? 1.0f : (r == 0 ? -1.0f : 0.0f);
                float rnx = tx * rn, rny = ty * rn;
                pv(x0i, y0i, z0i, rnx,rny,0);
                pv(x0,  y0,  z0,  rnx,rny,0);
                pv(x1,  y1,  z1,  rnx,rny,0);
                pv(x0i, y0i, z0i, rnx,rny,0);
                pv(x1,  y1,  z1,  rnx,rny,0);
                pv(x1i, y1i, z1i, rnx,rny,0);
            }
        }
    }

    m_exclVertCount = (int)(verts.size() / 6);
    if (m_exclVertCount == 0) return;

    if (!m_exclVAO) {
        glGenVertexArrays(1, &m_exclVAO);
        glGenBuffers(1, &m_exclVBO);
    }
    glBindVertexArray(m_exclVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_exclVBO);
    glBufferData(GL_ARRAY_BUFFER, verts.size()*sizeof(float), verts.data(), GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glBindVertexArray(0);
}

// ─── Checkered floor ────────────────────────────────────────────────
void Renderer::setFloorLevel(float z) {
    if (!m_floorGenerated || std::abs(z - m_floorZ) > 0.01f)
        generateFloor(z);
}

void Renderer::generateFloor(float z) {
    m_floorZ = z;
    std::vector<float> lightV, darkV;
    const int gridN = 120;
    const float cellSize = 1000.0f;
    const float half = gridN * cellSize * 0.5f;

    for (int i = 0; i < gridN; i++) {
        for (int j = 0; j < gridN; j++) {
            float x0 = -half + i * cellSize;
            float y0 = -half + j * cellSize;
            float x1 = x0 + cellSize;
            float y1 = y0 + cellSize;
            auto& v = ((i + j) % 2 == 0) ? lightV : darkV;
            pushV(v, x0, y0, z, 0, 0, 1);
            pushV(v, x1, y0, z, 0, 0, 1);
            pushV(v, x1, y1, z, 0, 0, 1);
            pushV(v, x0, y0, z, 0, 0, 1);
            pushV(v, x1, y1, z, 0, 0, 1);
            pushV(v, x0, y1, z, 0, 0, 1);
        }
    }

    for (int c = 0; c < 2; c++) {
        auto& vd = (c == 0) ? lightV : darkV;
        if (m_floorVAO[c]) glDeleteVertexArrays(1, &m_floorVAO[c]);
        if (m_floorVBO[c]) glDeleteBuffers(1, &m_floorVBO[c]);
        glGenVertexArrays(1, &m_floorVAO[c]);
        glGenBuffers(1, &m_floorVBO[c]);
        glBindVertexArray(m_floorVAO[c]);
        glBindBuffer(GL_ARRAY_BUFFER, m_floorVBO[c]);
        glBufferData(GL_ARRAY_BUFFER, vd.size() * sizeof(float), vd.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);
        m_floorVertCount[c] = (int)(vd.size() / 6);
    }
    m_floorGenerated = true;
}