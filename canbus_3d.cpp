/**
 * CAN Bus Vehicle Network Simulator — 3D Terminal (C++)
 * ═══════════════════════════════════════════════════════
 * ISO 11898 · Sedan Blueprint · ECU Nodes · Software 3D Renderer
 * Pure C++17 · ANSI Terminal · No external dependencies
 *
 * Controls:
 *   [1-6]  Fire ECU frames           [a] Startup scenario
 *   [q/e]  Rotate left/right         [b] Braking scenario
 *   [w/s]  Tilt up/down              [d] OBD diagnostic scan
 *   [+/-]  Zoom in/out               [r] Reset view
 *   [ESC]  Quit
 */

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <array>
#include <string>
#include <cmath>
#include <cstring>
#include <thread>
#include <mutex>
#include <queue>
#include <atomic>
#include <chrono>
#include <random>
#include <functional>
#include <algorithm>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>

// ════════════════════════════════════════════
//  ANSI COLOURS & TERMINAL
// ════════════════════════════════════════════
namespace ANS {
    const char* RST   = "\033[0m";
    const char* BOLD  = "\033[1m";
    const char* DIM   = "\033[2m";
    const char* BLUE  = "\033[38;2;0;180;255m";
    const char* CYAN  = "\033[38;2;0;255;231m";
    const char* WARN  = "\033[38;2;255;184;0m";
    const char* GREEN = "\033[38;2;0;255;136m";
    const char* RED   = "\033[38;2;255;59;92m";
    const char* DIM_B = "\033[38;2;10;58;92m";
    const char* TEXT  = "\033[38;2;200;232;255m";
    const char* BG    = "\033[48;2;2;12;24m";

    void clear()          { std::cout << "\033[2J\033[H"; }
    void moveto(int r,int c){ std::cout << "\033[" << r << ";" << c << "H"; }
    void hide_cursor()    { std::cout << "\033[?25l"; }
    void show_cursor()    { std::cout << "\033[?25h"; }
    void flush()          { std::cout.flush(); }

    void get_size(int &rows, int &cols) {
        struct winsize w;
        ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
        rows = w.ws_row ? w.ws_row : 40;
        cols = w.ws_col ? w.ws_col : 120;
    }
}

// ════════════════════════════════════════════
//  MATH  (Vec3, Mat4x4)
// ════════════════════════════════════════════
struct Vec3 {
    double x=0, y=0, z=0;
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double s)      const { return {x*s,   y*s,   z*s  }; }
    Vec3 lerp(const Vec3& o, double t)  const { return *this + (o - *this)*t; }
    double dot(const Vec3& o)     const { return x*o.x + y*o.y + z*o.z; }
    double len()                  const { return std::sqrt(x*x+y*y+z*z); }
    Vec3 norm()                   const { double l=len(); return l>0?*this*(1/l):*this; }
};

// Simple projection: rotate Y then X, then perspective divide
struct Camera {
    double azim = -55.0;   // degrees
    double elev =  22.0;
    double zoom =  18.0;   // scale for terminal coords

    // Project world → (col, row) on terminal
    // returns {col, row, depth}
    Vec3 project(Vec3 p, int W, int H) const {
        // Rotate around Y (azim)
        double ay = azim * M_PI / 180.0;
        double x1 =  p.x*cos(ay) + p.z*sin(ay);
        double z1 = -p.x*sin(ay) + p.z*cos(ay);
        // Rotate around X (elev)
        double ae = elev * M_PI / 180.0;
        double y2 =  p.y*cos(ae) - z1*sin(ae);
        double z2 =  p.y*sin(ae) + z1*cos(ae);

        // Perspective (simple)
        double fov = 3.5;
        double pz = z2 + fov;
        if (pz < 0.1) pz = 0.1;
        double sx = x1 / pz;
        double sy = y2 / pz;

        // Terminal aspect: chars are ~2x tall
        int col = (int)(W/2 + sx * zoom * 2.0);
        int row = (int)(H/2 - sy * zoom);
        return {(double)col, (double)row, z2};
    }
};

// ════════════════════════════════════════════
//  FRAMEBUFFER  (char + colour per cell)
// ════════════════════════════════════════════
struct Cell {
    char ch = ' ';
    const char* fg = ANS::TEXT;
    int depth_int = -9999;
};

struct Framebuffer {
    int W=0, H=0;
    std::vector<Cell> buf;

    void resize(int w, int h) {
        W=w; H=h;
        buf.assign(w*h, Cell{});
    }
    void clear() {
        for (auto& c : buf) { c.ch=' '; c.fg=ANS::TEXT; c.depth_int=-9999; }
    }
    Cell& at(int row, int col) {
        row = std::clamp(row,0,H-1);
        col = std::clamp(col,0,W-1);
        return buf[row*W+col];
    }
    void put(int row, int col, char ch, const char* fg, double depth) {
        if (row<0||row>=H||col<0||col>=W) return;
        int di = (int)(depth*100);
        Cell& c = buf[row*W+col];
        if (di > c.depth_int) { c.ch=ch; c.fg=fg; c.depth_int=di; }
    }
    void putstr(int row, int col, const std::string& s, const char* fg) {
        for (int i=0;i<(int)s.size();i++)
            put(row, col+i, s[i], fg, 9999);
    }

    // Draw a 3D line via Bresenham
    void line3d(Vec3 a, Vec3 b, const Camera& cam, char ch,
                const char* fg, int W2, int H2) {
        Vec3 pa = cam.project(a, W2, H2);
        Vec3 pb = cam.project(b, W2, H2);
        int x0=pa.x, y0=pa.y, x1=pb.x, y1=pb.y;
        int dx=std::abs(x1-x0), dy=-std::abs(y1-y0);
        int sx=(x0<x1)?1:-1, sy=(y0<y1)?1:-1;
        int err=dx+dy;
        double dep0=pa.z, dep1=pb.z;
        int steps=std::max(dx,-dy);
        for (int i=0; i<=steps; i++) {
            double t = steps>0?(double)i/steps:0;
            double dep = dep0*(1-t)+dep1*t;
            put(y0, x0, ch, fg, dep);
            if (x0==x1&&y0==y1) break;
            int e2=2*err;
            if (e2>=dy){err+=dy;x0+=sx;}
            if (e2<=dx){err+=dx;y0+=sy;}
        }
    }
};

// ════════════════════════════════════════════
//  CAN BUS DATA STRUCTURES
// ════════════════════════════════════════════
struct CANFrame {
    std::string src, dst, label, can_id, data;
    double timestamp;
    bool broadcast = false;
};

struct ECUNode {
    std::string name, id, full_name;
    Vec3 pos;
    const char* color;
    bool active = false;
    double active_timer = 0.0;
};

struct Packet {
    Vec3 pos;
    Vec3 src_pos, bus_src, bus_dst, dst_pos;
    double t = 0.0;
    bool alive = false;
    const char* color;
    std::string label;
};

// ════════════════════════════════════════════
//  ECU DEFINITIONS
// ════════════════════════════════════════════
std::vector<ECUNode> make_ecus() {
    return {
        {"ECU",  "0x28",  "Engine ECU",       {-1.2, 0.0,  0.55}, ANS::BLUE},
        {"TCM",  "0x2A",  "Transmission",     { 0.4, 0.0,  0.55}, ANS::BLUE},
        {"BCM",  "0x35",  "Body Control",     {-0.4, 0.0,  0.65}, ANS::CYAN},
        {"ABS",  "0x18",  "ABS Braking",      {-1.5,-0.5, -0.05}, ANS::BLUE},
        {"TPMS", "0x3F",  "Tyre Pressure",    { 1.5,-0.5, -0.05}, ANS::BLUE},
        {"OBD",  "0x7DF", "OBD-II Gateway",   { 0.0, 0.9,  0.15}, ANS::RED },
    };
}

// ════════════════════════════════════════════
//  SEDAN WIREFRAME  (3D line segments)
// ════════════════════════════════════════════
struct WireSeg {
    Vec3 a, b;
    const char* color;
    char ch;
    double alpha; // 0-1, used to decide if we draw (threshold)
};

std::vector<WireSeg> make_sedan() {
    std::vector<WireSeg> segs;
    const double W = 0.45;

    auto seg = [&](Vec3 a, Vec3 b, const char* c=nullptr, char ch='\xB7', double al=0.8){
        const char* col = (c == nullptr) ? ANS::BLUE : c;
        if(al > 0.4) segs.push_back({a,b,col,ch,al});
    };
    // lambda to add mirrored segments on both sides
    auto mirr = [&](std::vector<std::pair<Vec3,Vec3>> ps,
                    const char* col, char ch='-', double al=0.8) {
        for (auto [a,b] : ps) {
            segs.push_back({{a.x,-W,a.z},{b.x,-W,b.z},col,ch,al});
            segs.push_back({{a.x, W,a.z},{b.x, W,b.z},col,ch,al});
        }
    };

    const char* B = ANS::BLUE;
    const char* D = ANS::DIM_B;
    const char* W2 = ANS::WARN;
    const char* R = ANS::RED;

    // Body profile – side silhouette
    std::vector<std::pair<Vec3,Vec3>> body_pairs = {
        {{-2.0,0,-0.30},{-2.0,0, 0.00}},  // rear pillar base
        {{-2.0,0, 0.00},{-1.7,0, 0.45}},  // rear pillar
        {{-1.7,0, 0.45},{-0.3,0, 0.70}},  // roof rear
        {{-0.3,0, 0.70},{ 0.3,0, 0.70}},  // roof top
        {{ 0.3,0, 0.70},{ 1.5,0, 0.45}},  // windscreen
        {{ 1.5,0, 0.45},{ 2.0,0, 0.10}},  // hood slope
        {{ 2.0,0, 0.10},{ 2.0,0,-0.30}},  // front drop
        {{ 2.0,0,-0.30},{-2.0,0,-0.30}},  // underbody
    };
    mirr(body_pairs, B, '*', 0.9);

    // Cross ribs
    for (double z : {-0.30, 0.0, 0.45})
        segs.push_back({{-2.0,-W,z},{-2.0,W,z},D,'|',0.4});
    for (double x : {-0.3, 0.3})
        segs.push_back({{x,-W,0.70},{x,W,0.70},D,'|',0.4});
    segs.push_back({{2.0,-W, 0.10},{2.0,W, 0.10},D,'|',0.4});
    segs.push_back({{2.0,-W,-0.30},{2.0,W,-0.30},D,'|',0.4});

    // Door divider
    segs.push_back({{-0.3,-W,-0.30},{-0.3,-W,0.45},D,':',0.3});
    segs.push_back({{-0.3, W,-0.30},{-0.3, W,0.45},D,':',0.3});

    // Roof cross beam
    segs.push_back({{0.0,-W,0.70},{0.0,W,0.70},D,'-',0.3});

    // Wheels  (hexagonal approximation per side, both sides)
    for (double wx : {-1.4, 1.4}) {
        int N = 12;
        for (int i=0;i<N;i++) {
            double t0 = 2*M_PI*i/N, t1 = 2*M_PI*(i+1)/N;
            double r = 0.32;
            Vec3 p0{wx + r*cos(t0), 0, -0.30 + r*sin(t0)};
            Vec3 p1{wx + r*cos(t1), 0, -0.30 + r*sin(t1)};
            segs.push_back({{p0.x,-W-0.02,p0.z},{p1.x,-W-0.02,p1.z},B,'o',0.6});
            segs.push_back({{p0.x, W+0.02,p0.z},{p1.x, W+0.02,p1.z},B,'o',0.6});
        }
        // Spokes
        for (int i=0;i<6;i++) {
            double th = M_PI*i/3;
            for (double wy : {-W-0.02, W+0.02}) {
                segs.push_back({{wx,wy,-0.30},{wx+0.18*cos(th),wy,-0.30+0.18*sin(th)},D,'-',0.3});
            }
        }
    }

    // Headlights
    segs.push_back({{2.05,-W+0.05,-0.05},{2.05,-W+0.05,0.10},B,'|',0.9});
    segs.push_back({{2.05, W-0.05,-0.05},{2.05, W-0.05,0.10},B,'|',0.9});
    // Taillights
    segs.push_back({{-2.05,-W+0.05,-0.05},{-2.05,-W+0.05,0.10},R,'|',0.9});
    segs.push_back({{-2.05, W-0.05,-0.05},{-2.05, W-0.05,0.10},R,'|',0.9});

    // CAN Bus backbone — two rails
    segs.push_back({{-2.1, -0.85+0.03, 0.0},{2.1,-0.85+0.03,0.0},B,'-',0.95});
    segs.push_back({{-2.1, -0.85-0.03, 0.0},{2.1,-0.85-0.03,0.0},B,'-',0.95});
    // Termination blocks
    segs.push_back({{-2.1,-0.85+0.03,0.0},{-2.1,-0.85-0.03,0.0},W2,'#',0.9});
    segs.push_back({{ 2.1,-0.85+0.03,0.0},{ 2.1,-0.85-0.03,0.0},W2,'#',0.9});

    return segs;
}

// ════════════════════════════════════════════
//  KEYBOARD  (raw non-blocking)
// ════════════════════════════════════════════
struct RawTerm {
    termios orig;
    RawTerm() {
        tcgetattr(STDIN_FILENO, &orig);
        termios raw = orig;
        raw.c_lflag &= ~(ECHO|ICANON);
        raw.c_cc[VMIN]=0; raw.c_cc[VTIME]=0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }
    ~RawTerm() {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig);
        ANS::show_cursor();
    }
    int getch() {
        unsigned char c=0;
        return (read(STDIN_FILENO,&c,1)==1)?(int)c:-1;
    }
};

// ════════════════════════════════════════════
//  CAN BUS ENGINE
// ════════════════════════════════════════════
class CANBusEngine {
public:
    std::mutex mtx;
    std::queue<CANFrame> inbox;
    std::vector<std::string> log;
    double wall = 0.0;  // updated by renderer

    void send(const std::string& src, const std::string& dst,
              const std::string& label, const std::string& can_id,
              const std::string& data, bool bcast=false) {
        CANFrame f{src,dst,label,can_id,data,wall,bcast};
        std::lock_guard<std::mutex> lk(mtx);
        inbox.push(f);
        log.push_back("TX [" + can_id + "] " + label + "  " + src + "→" + dst);
        if(log.size()>30) log.erase(log.begin());
    }

    bool poll(CANFrame& out) {
        std::lock_guard<std::mutex> lk(mtx);
        if(inbox.empty()) return false;
        out = inbox.front(); inbox.pop(); return true;
    }

    void log_rx(const CANFrame& f) {
        std::lock_guard<std::mutex> lk(mtx);
        log.push_back("RX [" + f.can_id + "] " + f.label + "  →" + f.dst);
        if(log.size()>30) log.erase(log.begin());
    }

    // Scenarios — run in detached thread
    void scenario(std::vector<std::tuple<double,std::string,std::string,
                                         std::string,std::string,std::string>> steps) {
        std::thread([this, steps=std::move(steps)](){
            for (auto& [delay, s, d, l, id, dt] : steps) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds((int)(delay*1000)));
                send(s,d,l,id,dt);
            }
        }).detach();
    }

    void startup() {
        scenario({
            {0.0, "ECU","BCM","ECU_INIT",    "0x28","[0x01,0x00]"},
            {0.5, "TCM","BCM","TCM_READY",   "0x2A","[0x01]"},
            {0.9, "ABS","BCM","ABS_READY",   "0x18","[0x01,0x00]"},
            {1.3, "TPMS","BCM","TPMS_OK",    "0x3F","[0x1C,0x1C]"},
            {1.7, "BCM","ECU","START_ENGINE", "0x35","[0x05,0x01]"},
            {2.2, "ECU","BCM","RPM_IDLE",     "0x28","[0x03,0x20]"},
        });
    }
    void braking() {
        scenario({
            {0.0, "ABS","BCM","BRAKE_HARD",  "0x18","[0xFF,0xFF]"},
            {0.3, "ABS","BCM","ABS_ACTIVE",  "0x18","[0xFF,0x01]"},
            {0.6, "TCM","BCM","DOWNSHIFT",   "0x2A","[0x02,0x00]"},
            {0.9, "ECU","BCM","ENG_BRAKE",   "0x28","[0x00,0x50]"},
            {1.4, "ABS","BCM","ABS_RELEASE", "0x18","[0x00,0x00]"},
        });
    }
    void obd_scan() {
        scenario({
            {0.0, "OBD","ECU","OBD_QUERY_RPM",  "0x7DF","[0x02,0x01,0x0C]"},
            {0.4, "ECU","OBD","OBD_RESP_RPM",   "0x7E8","[0x04,0x41,0x1A]"},
            {0.8, "OBD","ECU","OBD_QUERY_SPD",  "0x7DF","[0x02,0x01,0x0D]"},
            {1.2, "ECU","OBD","OBD_RESP_SPD",   "0x7E8","[0x03,0x41,0x60]"},
            {1.6, "OBD","ECU","OBD_QUERY_TEMP", "0x7DF","[0x02,0x01,0x05]"},
            {2.0, "ECU","OBD","OBD_RESP_TEMP",  "0x7E8","[0x03,0x41,0x7B]"},
        });
    }
};

// ════════════════════════════════════════════
//  MAIN RENDERER
// ════════════════════════════════════════════
class Renderer {
    Framebuffer fb;
    Camera cam;
    CANBusEngine bus;
    std::vector<WireSeg> sedan;
    std::vector<ECUNode> ecus;
    Packet pkt;
    RawTerm raw;

    std::string last_frame_info;
    std::vector<std::string> frame_fields;
    int W=0, H=0;
    double t = 0.0;
    double hb_timer = 0.0;
    int hb_idx = 0;
    std::mt19937 rng{42};

    // frame info
    std::string fr_id, fr_label, fr_data, fr_dlc, fr_crc;

    using Clock = std::chrono::steady_clock;
    Clock::time_point t0 = Clock::now();

    double now_sec() {
        return std::chrono::duration<double>(Clock::now()-t0).count();
    }

    ECUNode* find_ecu(const std::string& name) {
        for (auto& e : ecus) if (e.name==name) return &e;
        return nullptr;
    }

    Vec3 bus_stub(const Vec3& pos) {
        return {pos.x, -0.85, 0.0};
    }

public:
    Renderer() {
        sedan = make_sedan();
        ecus  = make_ecus();
        ANS::hide_cursor();
        ANS::clear();
    }

    ~Renderer() { ANS::clear(); }

    void run() {
        double prev = now_sec();
        while (true) {
            double cur = now_sec();
            double dt  = cur - prev;
            prev = cur;
            t += dt;
            bus.wall = t;

            // Input
            int ch = raw.getch();
            if (ch == 27 || ch == 'x') break;
            handle_key(ch);

            // Bus poll
            CANFrame frame;
            if (bus.poll(frame)) {
                activate_packet(frame);
                set_frame_info(frame);
                auto* src = find_ecu(frame.src);
                auto* dst = find_ecu(frame.dst);
                if (src) { src->active=true; src->active_timer=0.5; }
                if (dst) { dst->active=true; dst->active_timer=0.5; }
            }

            // Animate packet
            if (pkt.alive) {
                pkt.t += dt * 2.2;  // speed
                Vec3 pos = packet_pos(pkt.t);
                pkt.pos = pos;
                if (pkt.t >= 1.0) {
                    pkt.alive = false;
                    bus.log_rx(CANFrame{});
                }
            }

            // ECU timers
            for (auto& e : ecus) {
                if (e.active) { e.active_timer -= dt; if(e.active_timer<=0) e.active=false; }
            }

            // Heartbeat
            hb_timer += dt;
            if (hb_timer > 3.5) {
                hb_timer = 0;
                std::string names[] = {"ECU","TCM","ABS","TPMS"};
                std::string n = names[hb_idx++ % 4];
                bus.send(n, "BCM", "HEARTBEAT", ecus[0].id, "[0x01]");
            }

            // Render
            ANS::get_size(H, W);
            // reserve right panel
            int VP_W = W - 36;
            int VP_H = H - 2;

            fb.resize(VP_W, VP_H);
            fb.clear();

            draw_sedan(VP_W, VP_H);
            draw_stubs(VP_W, VP_H);
            draw_nodes(VP_W, VP_H);
            if (pkt.alive) draw_packet(VP_W, VP_H);

            flush_fb(VP_W, VP_H);
            draw_right_panel(VP_W, VP_H);
            draw_header();
            draw_keybinds();
            ANS::flush();

            // ~30 fps
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
    }

private:
    void handle_key(int ch) {
        if (ch == 'q') cam.azim -= 5;
        if (ch == 'e') cam.azim += 5;
        if (ch == 'w') cam.elev += 3;
        if (ch == 's') cam.elev -= 3;
        if (ch == '+') cam.zoom += 1;
        if (ch == '-') cam.zoom -= 1;
        if (ch == 'r') { cam.azim=-55; cam.elev=22; cam.zoom=18; }
        // ECU triggers
        std::pair<std::string,std::string> triggers[] = {
            {"ECU","BCM"},{"TCM","BCM"},{"ABS","BCM"},
            {"TPMS","BCM"},{"OBD","ECU"},{"BCM","ECU"},
        };
        std::string labels[] = {"ENGINE_RPM","GEAR_SHIFT","ABS_ACTIVE","TYRE_PSI","OBD_QUERY","FUEL_REQ"};
        std::string ids[]    = {"0x28","0x2A","0x18","0x3F","0x7DF","0x35"};
        std::string data[]   = {"[0x0F,0xA0]","[0x03]","[0xFF,0x01]","[0x1C,0x1D]","[0x02,0x01]","[0x02,0x41]"};
        if (ch>='1'&&ch<='6') {
            int i = ch-'1';
            bus.send(triggers[i].first, triggers[i].second, labels[i], ids[i], data[i]);
        }
        if (ch=='a') bus.startup();
        if (ch=='b') bus.braking();
        if (ch=='d') bus.obd_scan();
    }

    void activate_packet(const CANFrame& f) {
        auto* src = find_ecu(f.src);
        auto* dst = find_ecu(f.dst);
        if (!src) return;
        Vec3 dp = dst ? dst->pos : Vec3{0,-0.85,0};
        pkt.src_pos = src->pos;
        pkt.bus_src = bus_stub(src->pos);
        pkt.bus_dst = bus_stub(dp);
        pkt.dst_pos = dp;
        pkt.t = 0.0;
        pkt.alive = true;
        pkt.color = src->color;
        pkt.label = f.label;
    }

    Vec3 packet_pos(double t) {
        // 4 waypoints: src → bus_src → bus_dst → dst
        std::vector<Vec3> wp = {pkt.src_pos, pkt.bus_src, pkt.bus_dst, pkt.dst_pos};
        std::vector<double> lens;
        double total = 0;
        for (int i=0;i<3;i++) {
            double l=(wp[i+1]-wp[i]).len();
            lens.push_back(l); total+=l;
        }
        double d = t * total;
        double acc = 0;
        for (int i=0;i<3;i++) {
            if (d <= acc+lens[i]) {
                double alpha = (d-acc)/lens[i];
                return wp[i].lerp(wp[i+1], alpha);
            }
            acc += lens[i];
        }
        return pkt.dst_pos;
    }

    void set_frame_info(const CANFrame& f) {
        fr_id    = f.can_id;
        fr_label = f.label;
        fr_data  = f.data;
        fr_dlc   = std::to_string(std::count(f.data.begin(),f.data.end(),',')+1);
        std::uniform_int_distribution<int> crc_dist(0,65535);
        std::ostringstream ss;
        ss << "0x" << std::hex << std::uppercase << crc_dist(rng);
        fr_crc = ss.str();
    }

    // ── draw helpers ──────────────────────────────

    void draw_sedan(int VW, int VH) {
        for (auto& s : sedan) {
            if (s.alpha < 0.35) continue;
            fb.line3d(s.a, s.b, cam, s.ch, s.color, VW, VH);
        }
    }

    void draw_stubs(int VW, int VH) {
        for (auto& e : ecus) {
            Vec3 bpt = bus_stub(e.pos);
            const char* col = e.active ? ANS::CYAN : ANS::DIM_B;
            fb.line3d(e.pos, bpt, cam, ':', col, VW, VH);
        }
    }

    void draw_nodes(int VW, int VH) {
        for (auto& e : ecus) {
            Vec3 p = cam.project(e.pos, VW, VH);
            int r=(int)p.y, c=(int)p.x;
            const char* col = e.active ? ANS::CYAN : e.color;
            // Draw node box
            fb.put(r-1, c-2, '[', col, p.z+1);
            fb.put(r-1, c+3, ']', col, p.z+1);
            for(int i=0;i<(int)e.name.size();i++)
                fb.put(r-1, c-1+i, e.name[i], col, p.z+1);
            // ID below
            for(int i=0;i<(int)e.id.size();i++)
                fb.put(r, c-2+i, e.id[i], ANS::DIM_B, p.z+0.5);
        }
    }

    void draw_packet(int VW, int VH) {
        Vec3 p = cam.project(pkt.pos, VW, VH);
        int r=(int)p.y, c=(int)p.x;
        // Draw glowing dot
        fb.put(r, c, '@', pkt.color, p.z+5);
        fb.put(r-1, c, '|', pkt.color, p.z+4);
        fb.put(r+1, c, '|', pkt.color, p.z+4);
        fb.put(r, c-1, '-', pkt.color, p.z+4);
        fb.put(r, c+1, '-', pkt.color, p.z+4);
        // Label
        for(int i=0;i<(int)pkt.label.size()&&i<12;i++)
            fb.put(r-2, c-2+i, pkt.label[i], ANS::TEXT, p.z+3);
    }

    void flush_fb(int VW, int VH) {
        const char* last_col = nullptr;
        for (int r=0; r<VH; r++) {
            ANS::moveto(r+2, 1);
            for (int c=0; c<VW; c++) {
                Cell& cell = fb.at(r,c);
                if (cell.fg != last_col) {
                    std::cout << cell.fg;
                    last_col = cell.fg;
                }
                std::cout << cell.ch;
            }
        }
        std::cout << ANS::RST;
    }

    void draw_header() {
        ANS::moveto(1, 1);
        std::cout << ANS::BG << ANS::BOLD << ANS::BLUE
                  << "  CAN BUS  ·  3D VEHICLE NETWORK SIMULATOR  ·  ISO 11898  ·  500 kbps  "
                  << ANS::RST;
    }

    void draw_right_panel(int VW, int VH) {
        int PC = VW + 1;
        auto row = [&](int r, int col, const std::string& s, const char* col2=ANS::TEXT){
            ANS::moveto(r, col);
            std::cout << col2 << s.substr(0, W-col) << ANS::RST;
        };

        // Panel border
        for (int r=2; r<=H; r++) {
            ANS::moveto(r, VW);
            std::cout << ANS::DIM_B << "│" << ANS::RST;
        }

        int r = 2;
        row(r++, PC, "┌─ ECU CONTROLS ─────────────┐", ANS::BLUE);
        std::pair<std::string,std::string> btn_info[] = {
            {"[1] ENGINE RPM ",  "ECU→BCM"},
            {"[2] GEAR SHIFT ",  "TCM→BCM"},
            {"[3] ABS TRIGGER",  "ABS→BCM"},
            {"[4] TYRE PSI   ",  "TPMS→BCM"},
            {"[5] OBD QUERY  ",  "OBD→ECU"},
            {"[6] FUEL REQ   ",  "BCM→ECU"},
        };
        for (auto& [label, dir] : btn_info) {
            ANS::moveto(r++, PC);
            std::cout << ANS::BLUE << "│ " << ANS::CYAN << label
                      << ANS::DIM_B << " " << dir << ANS::BLUE << " │" << ANS::RST;
        }
        row(r++, PC, "├─ SCENARIOS ───────────────┤", ANS::BLUE);
        row(r++, PC, "│ [a] Cold Startup Seq.     │", ANS::WARN);
        row(r++, PC, "│ [b] Emergency Braking     │", ANS::WARN);
        row(r++, PC, "│ [d] OBD Diagnostic Scan   │", ANS::WARN);
        row(r++, PC, "├─ CAN FRAME DECODER ───────┤", ANS::BLUE);

        auto field = [&](const std::string& name, const std::string& val, const char* vc) {
            ANS::moveto(r++, PC);
            std::string v = val.size()>18 ? val.substr(0,18) : val;
            std::cout << ANS::BLUE << "│ " << ANS::DIM_B << std::left << std::setw(5) << name
                      << ANS::RST << " " << vc << std::left << std::setw(20) << v
                      << ANS::BLUE << "│" << ANS::RST;
        };
        field("SOF:",  "0 (dominant)",         ANS::DIM_B);
        field("ID:",   fr_id.empty()?"—":fr_id,          ANS::GREEN);
        field("LABEL:",fr_label.empty()?"—":fr_label,    ANS::CYAN);
        field("DLC:",  fr_dlc.empty()?"—":fr_dlc+" bytes",ANS::WARN);
        field("DATA:", fr_data.empty()?"—":fr_data,      ANS::CYAN);
        field("CRC:",  fr_crc.empty()?"—":fr_crc,        ANS::DIM_B);
        field("ACK:",  fr_id.empty()?"—":"1 (ACK)",      ANS::GREEN);

        row(r++, PC, "├─ BUS LOG ─────────────────┤", ANS::BLUE);

        std::vector<std::string> log_snap;
        {
            std::lock_guard<std::mutex> lk(bus.mtx);
            log_snap = bus.log;
        }
        int log_rows = H - r - 1;
        int start = std::max(0,(int)log_snap.size()-log_rows);
        for (int i=start; i<(int)log_snap.size()&&r<H; i++) {
            ANS::moveto(r++, PC);
            std::string entry = log_snap[i];
            bool is_tx = entry.find("TX") != std::string::npos;
            bool is_rx = entry.find("RX") != std::string::npos;
            const char* lc = is_tx ? ANS::GREEN : (is_rx ? ANS::CYAN : ANS::TEXT);
            std::string trimmed = entry.size()>33 ? entry.substr(0,33) : entry;
            // pad to fill width
            while ((int)trimmed.size() < 34) trimmed += ' ';
            std::cout << ANS::BLUE << "│" << lc << trimmed << ANS::BLUE << "│" << ANS::RST;
        }
        // Fill remaining
        while (r < H) { row(r++, PC, "│                              │", ANS::BLUE); }
        row(r, PC, "└───────────────────────────┘", ANS::BLUE);
    }

    void draw_keybinds() {
        ANS::moveto(H, 1);
        std::cout << ANS::DIM_B
                  << " [q/e] Rotate  [w/s] Tilt  [+/-] Zoom  [r] Reset  "
                  << "[1-6] Fire ECU  [a/b/d] Scenarios  [ESC/x] Quit"
                  << ANS::RST;
    }
};

// ════════════════════════════════════════════
//  ENTRY POINT
// ════════════════════════════════════════════
int main() {
    std::cout << "\033[2J\033[H";
    std::cout << ANS::BLUE << ANS::BOLD << R"(
  ╔══════════════════════════════════════════════════════╗
  ║   CAN BUS 3D VEHICLE SIMULATOR  ─  C++ Edition      ║
  ║   ISO 11898 · 500 kbps · Software 3D Renderer       ║
  ╚══════════════════════════════════════════════════════╝
)" << ANS::RST;
    std::cout << ANS::TEXT
              << "  Starting 3D terminal renderer...\n\n"
              << "  NOTE: Best viewed in a terminal at least 120×40 chars.\n"
              << "  Set your terminal font to a monospace font for best results.\n\n"
              << ANS::WARN << "  Press any key to start..." << ANS::RST;
    std::cout.flush();
    // Wait for keypress
    {
        termios t; tcgetattr(0,&t);
        termios r=t; r.c_lflag&=~(ECHO|ICANON); r.c_cc[VMIN]=1;
        tcsetattr(0,TCSANOW,&r);
        getchar();
        tcsetattr(0,TCSANOW,&t);
    }

    Renderer renderer;
    renderer.run();

    std::cout << ANS::BLUE << "\n  CAN Bus Simulator terminated. Goodbye.\n" << ANS::RST;
    return 0;
}
