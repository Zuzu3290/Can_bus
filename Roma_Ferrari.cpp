#define SDL_MAIN_HANDLED
#include <SDL2/SDL.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

struct Vec3 {
    float x, y, z;
};

struct Face {
    std::vector<int> indices;
};

struct Edge {
    int a, b;

    Edge(int v1, int v2) {
        if (v1 < v2) { a = v1; b = v2; }
        else         { a = v2; b = v1; }
    }

    bool operator<(const Edge& other) const {
        if (a != other.a) return a < other.a;
        return b < other.b;
    }
};

struct Mesh {
    std::vector<Vec3> vertices;
    std::vector<Face> faces;
    std::vector<Edge> edges;
};

static const int WIDTH  = 1280;
static const int HEIGHT = 720;

static float g_yaw = 25.0f;
static float g_pitch = 15.0f;
static float g_distance = 8.0f;
static bool  g_dragging = false;
static bool  g_autoRotate = false;
static int   g_lastMouseX = 0;
static int   g_lastMouseY = 0;

static Mesh g_mesh;

static Vec3 g_center = {0.0f, 0.0f, 0.0f};
static float g_scale = 1.0f;

static bool startsWith(const std::string& s, const std::string& prefix) {
    return s.rfind(prefix, 0) == 0;
}

static void computeBoundsAndNormalize(Mesh& mesh) {
    if (mesh.vertices.empty()) return;

    Vec3 minV = mesh.vertices[0];
    Vec3 maxV = mesh.vertices[0];

    for (const auto& v : mesh.vertices) {
        minV.x = std::min(minV.x, v.x);
        minV.y = std::min(minV.y, v.y);
        minV.z = std::min(minV.z, v.z);

        maxV.x = std::max(maxV.x, v.x);
        maxV.y = std::max(maxV.y, v.y);
        maxV.z = std::max(maxV.z, v.z);
    }

    g_center.x = (minV.x + maxV.x) * 0.5f;
    g_center.y = (minV.y + maxV.y) * 0.5f;
    g_center.z = (minV.z + maxV.z) * 0.5f;

    float sx = maxV.x - minV.x;
    float sy = maxV.y - minV.y;
    float sz = maxV.z - minV.z;
    float maxDim = std::max({sx, sy, sz});

    if (maxDim < 0.0001f) maxDim = 1.0f;
    g_scale = 4.0f / maxDim;
}

static int parseObjIndex(const std::string& token) {
    // Supports formats:
    // f v1 v2 v3
    // f v1/vt1 v2/vt2 v3/vt3
    // f v1/vt1/vn1 ...
    // f v1//vn1 ...
    size_t slash = token.find('/');
    std::string vStr = (slash == std::string::npos) ? token : token.substr(0, slash);
    int idx = std::stoi(vStr);
    return idx - 1; // OBJ is 1-based
}

static bool loadOBJ(const std::string& filename, Mesh& mesh) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open OBJ file: " << filename << "\n";
        return false;
    }

    mesh.vertices.clear();
    mesh.faces.clear();
    mesh.edges.clear();

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;

        if (startsWith(line, "v ")) {
            std::istringstream iss(line);
            char c;
            Vec3 v{};
            iss >> c >> v.x >> v.y >> v.z;
            mesh.vertices.push_back(v);
        }
        else if (startsWith(line, "f ")) {
            std::istringstream iss(line);
            char c;
            iss >> c;

            Face face;
            std::string token;
            while (iss >> token) {
                int idx = parseObjIndex(token);
                if (idx >= 0 && idx < (int)mesh.vertices.size()) {
                    face.indices.push_back(idx);
                }
            }

            if (face.indices.size() >= 2) {
                mesh.faces.push_back(face);
            }
        }
    }

    std::set<Edge> uniqueEdges;

    for (const auto& face : mesh.faces) {
        int n = (int)face.indices.size();
        for (int i = 0; i < n; ++i) {
            int a = face.indices[i];
            int b = face.indices[(i + 1) % n];
            if (a != b) {
                uniqueEdges.insert(Edge(a, b));
            }
        }
    }

    mesh.edges.assign(uniqueEdges.begin(), uniqueEdges.end());

    computeBoundsAndNormalize(mesh);

    std::cout << "Loaded OBJ: " << filename << "\n";
    std::cout << "Vertices: " << mesh.vertices.size() << "\n";
    std::cout << "Faces:    " << mesh.faces.size() << "\n";
    std::cout << "Edges:    " << mesh.edges.size() << "\n";

    return true;
}

static void setupProjection(int width, int height) {
    if (height <= 0) height = 1;

    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (double)width / (double)height, 0.1, 200.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

static void drawGround() {
    glBegin(GL_LINES);
    for (int i = -20; i <= 20; ++i) {
        glVertex3f((float)i, 0.0f, -20.0f);
        glVertex3f((float)i, 0.0f,  20.0f);

        glVertex3f(-20.0f, 0.0f, (float)i);
        glVertex3f( 20.0f, 0.0f, (float)i);
    }
    glEnd();
}

static void drawMeshWireframe(const Mesh& mesh) {
    glBegin(GL_LINES);
    for (const auto& e : mesh.edges) {
        const Vec3& a = mesh.vertices[e.a];
        const Vec3& b = mesh.vertices[e.b];
        glVertex3f(a.x, a.y, a.z);
        glVertex3f(b.x, b.y, b.z);
    }
    glEnd();
}

static void renderScene() {
    glClearColor(0.f, 0.f, 0.f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glTranslatef(0.0f, 0.0f, -g_distance);
    glRotatef(g_pitch, 1.0f, 0.0f, 0.0f);
    glRotatef(g_yaw,   0.0f, 1.0f, 0.0f);

    glColor3f(0.22f, 0.22f, 0.22f);
    glLineWidth(1.0f);
    drawGround();

    glPushMatrix();

    // center and scale mesh
    glScalef(g_scale, g_scale, g_scale);
    glTranslatef(-g_center.x, -g_center.y, -g_center.z);

    glColor3f(1.f, 1.f, 1.f);
    glLineWidth(1.0f);
    drawMeshWireframe(g_mesh);

    glPopMatrix();
}

static void resetView() {
    g_yaw = 25.0f;
    g_pitch = 15.0f;
    g_distance = 8.0f;
    g_autoRotate = false;
}

int main() {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << "\n";
        return 1;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    SDL_Window* window = SDL_CreateWindow(
        "Roma Spider OBJ Wireframe",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        WIDTH,
        HEIGHT,
        SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    );

    if (!window) {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << "\n";
        SDL_Quit();
        return 1;
    }

    SDL_GLContext glContext = SDL_GL_CreateContext(window);
    if (!glContext) {
        std::cerr << "SDL_GL_CreateContext failed: " << SDL_GetError() << "\n";
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    SDL_GL_SetSwapInterval(1);

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glDisable(GL_CULL_FACE);

    if (!loadOBJ("roma_spider.obj", g_mesh)) {
        SDL_GL_DeleteContext(glContext);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    setupProjection(w, h);

    bool running = true;

    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }

            if (event.type == SDL_WINDOWEVENT &&
                event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
                setupProjection(event.window.data1, event.window.data2);
            }

            if (event.type == SDL_KEYDOWN) {
                switch (event.key.keysym.sym) {
                    case SDLK_ESCAPE:
                        running = false;
                        break;
                    case SDLK_a:
                        g_autoRotate = !g_autoRotate;
                        break;
                    case SDLK_r:
                        resetView();
                        break;
                    default:
                        break;
                }
            }

            if (event.type == SDL_MOUSEBUTTONDOWN &&
                event.button.button == SDL_BUTTON_LEFT) {
                g_dragging = true;
                g_lastMouseX = event.button.x;
                g_lastMouseY = event.button.y;
            }

            if (event.type == SDL_MOUSEBUTTONUP &&
                event.button.button == SDL_BUTTON_LEFT) {
                g_dragging = false;
            }

            if (event.type == SDL_MOUSEMOTION && g_dragging) {
                int dx = event.motion.x - g_lastMouseX;
                int dy = event.motion.y - g_lastMouseY;

                g_lastMouseX = event.motion.x;
                g_lastMouseY = event.motion.y;

                g_yaw += dx * 0.35f;
                g_pitch += dy * 0.35f;

                if (g_pitch > 89.0f)  g_pitch = 89.0f;
                if (g_pitch < -89.0f) g_pitch = -89.0f;
            }

            if (event.type == SDL_MOUSEWHEEL) {
                g_distance -= event.wheel.y * 0.6f;
                if (g_distance < 2.0f)  g_distance = 2.0f;
                if (g_distance > 50.0f) g_distance = 50.0f;
            }
        }

        if (g_autoRotate) {
            g_yaw += 0.15f;
        }

        renderScene();
        SDL_GL_SwapWindow(window);
    }

    SDL_GL_DeleteContext(glContext);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}