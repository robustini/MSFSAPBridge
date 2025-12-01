/*  
   MSFS 202x–ArduPilot Bridge
   A lightweight bridge connecting Microsoft Flight Simulator (MSFS)
   with ArduPilot-based tooling and SITL workflows.

   Contributions, testing and ideas are welcome!

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
   See the GNU General Public License for more details.
*/
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#define DIRECTINPUT_VERSION 0x0800
#define NOMINMAX
#include <cstdio>

#ifndef APP_TITLE_W
#define APP_TITLE_W L"MSFS 202x \u2194 ArduPilot SITL Bridge v1.0.0 by Marcopter"
#endif
#ifndef APP_TITLE_A
#define APP_TITLE_A  "MSFS 202x <-> ArduPilot SITL Bridge v1.0.0 by Marcopter"
#endif

#include <cstdint>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>
#include <thread>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <bitset>
#include <cstdlib>
#include <cstring>
#include <cwchar>
#include <csignal>
#include <exception>
#include <new>
#include <eh.h>
#include <clocale>

static inline uint64_t _now_ms() {
    using namespace std::chrono;
    return (uint64_t)duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <commctrl.h>
#include <shellapi.h>
#include <dinput.h>
#include <commdlg.h>
#include "resource.h"

#pragma comment(lib,"Ws2_32.lib")
#pragma comment(lib,"User32.lib")
#pragma comment(lib,"Gdi32.lib")
#pragma comment(lib,"Comctl32.lib")
#pragma comment(lib,"dinput8.lib")
#pragma comment(lib,"dxguid.lib")
#pragma comment(lib,"Comdlg32.lib")

#include "SimConnect.h"

#include <delayimp.h>
#include <cstring>

#pragma comment(lib, "delayimp")
#pragma comment(linker, "/DELAYLOAD:SimConnect.dll")

static void ShowSimConnectMissingDialog() {
    MessageBoxW(
    nullptr,
    L"SimConnect.dll was not found.\n\n"
    L"This application requires Microsoft Flight Simulator (2020/2024).\n\n"
    L"How to fix:\n"
    L"\u2022 Ensure MSFS is installed and up to date (SimConnect is included).\n"
    L"\u2022 Or install the SDK from MSFS: Dev Mode → Help → SDK Installer.\n"
    L"\u2022 Use the x64 build and ensure the Microsoft Visual C++ x64 runtime is installed.\n\n"
    L"Press OK to close.",
    L"Error: SimConnect missing",
    MB_OK | MB_ICONERROR | MB_SETFOREGROUND | MB_TOPMOST | MB_SYSTEMMODAL
    );
}

static FARPROC WINAPI DelayHook(unsigned dliNotify, PDelayLoadInfo pdli) {
    if (dliNotify == dliFailLoadLib && pdli && pdli->szDll &&

    _stricmp(pdli->szDll, "SimConnect.dll") == 0) {
        ShowSimConnectMissingDialog();
        ExitProcess(EXIT_FAILURE);
    }
    return nullptr;
}

extern "C" const PfnDliHook __pfnDliFailureHook2 = DelayHook;

static void load_ini();
static void save_ini();

#define IDC_SIMMAP_CB_BASE 5200

static HWND g_sitl_evt_cb[16] = {0};

struct SimEvtOpt { const wchar_t* label; const char* ev; };
static const SimEvtOpt kEvtOpts[] = {
    {L"(None)", ""},

    {L"Ailerons (AXIS_AILERONS_SET)",   "AXIS_AILERONS_SET"},
    {L"Elevator (AXIS_ELEVATOR_SET)",   "AXIS_ELEVATOR_SET"},
    {L"Rudder (AXIS_RUDDER_SET)",       "AXIS_RUDDER_SET"},

    {L"Throttle (all engines, THROTTLE_AXIS_SET_EX1)", "THROTTLE_AXIS_SET_EX1"},
    {L"Throttle1 (THROTTLE1_AXIS_SET_EX1)", "THROTTLE1_AXIS_SET_EX1"},
    {L"Throttle2 (THROTTLE2_AXIS_SET_EX1)", "THROTTLE2_AXIS_SET_EX1"},
    {L"Throttle3 (THROTTLE3_AXIS_SET_EX1)", "THROTTLE3_AXIS_SET_EX1"},
    {L"Throttle4 (THROTTLE4_AXIS_SET_EX1)", "THROTTLE4_AXIS_SET_EX1"},

    {L"Elev Trim (ELEVATOR_TRIM_SET)",  "ELEVATOR_TRIM_SET"},
    {L"Aileron Trim (AILERON_TRIM_SET)","AILERON_TRIM_SET"},
    {L"Rudder Trim (RUDDER_TRIM_SET)",  "RUDDER_TRIM_SET"},

    {L"Spoilers (SPOILERS_SET)",        "SPOILERS_SET"},
    {L"Spoilers Arm (SPOILERS_ARM_TOGGLE)", "SPOILERS_ARM_TOGGLE"},

    {L"Flaps set (FLAPS_SET)",          "FLAPS_SET"},
    {L"Flaps +1 (FLAPS_INCR)",          "FLAPS_INCR"},
    {L"Flaps -1 (FLAPS_DECR)",          "FLAPS_DECR"},

    {L"Slats set (LEADING_EDGE_FLAPS_SET)", "LEADING_EDGE_FLAPS_SET"},

    {L"Landing Gear toggle (GEAR_TOGGLE)",  "GEAR_TOGGLE"},
    {L"Landing Gear set 0/1 (GEAR_SET)",    "GEAR_SET"},
};

static int G_sim_evt_idx[16] = {
    1,2,3,4,
    0,0,0,0,
    0,0,0,0,
    0,0,0,0
};

static const char* get_sim_evt_by_idx(int idx){
    if (idx < 0) idx = 0;
    size_t n = sizeof(kEvtOpts)/sizeof(kEvtOpts[0]);
    if ((size_t)idx >= n) idx = 0;
    return kEvtOpts[idx].ev;
}

static int find_evt_idx_by_name(const wchar_t* w){
    if(!w) return 0;
    size_t n = sizeof(kEvtOpts)/sizeof(kEvtOpts[0]);
    for(size_t i=0;i<n;i++){
        wchar_t evw[128];
        MultiByteToWideChar(CP_UTF8, 0, kEvtOpts[i].ev, -1, evw, 128);
        if(_wcsicmp(w, evw)==0) return (int)i;
    }
    return 0;
}

static HWND g_lblSimDbg=0, g_lvSimDbg=0;
#pragma comment(lib,"SimConnect.lib")

#ifndef DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2
#define DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2 ((DPI_AWARENESS_CONTEXT)-4)
#endif

#define IDM_FILE_LOAD 1001
#define IDM_FILE_SAVE 1002
#define IDM_FILE_SAVEAS 1003
#define IDM_FILE_EXIT 1004
#define IDM_VIEW_SIMCONNECT 2001
#define IDM_HELP_ABOUT 3001
#define IDM_HELP_LOGGING 3002

static int   g_dpi = 96;
static HFONT g_uiFont = NULL;
static HFONT g_uiFontBold = NULL;
static HFONT g_hudFont = NULL;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline double deg2rad(double deg) { return deg * (M_PI / 180.0); }
static inline double ft2m(double ft) { return ft * 0.3048; }
static inline double kt2ms(double kt) { return kt * 0.514444; }

static int Dpi(HWND h){
    HMODULE hUser32 = LoadLibraryW(L"User32.dll");

    if (hUser32 && GetProcAddress(hUser32, "GetDpiForWindow")) {
        typedef UINT (WINAPI *PFN)(HWND);
        PFN p=(PFN)GetProcAddress(hUser32,"GetDpiForWindow");
        UINT d = p? p(h):96;
        FreeLibrary(hUser32);
        return (int)d;
    }
    if (hUser32) FreeLibrary(hUser32);
    return 96;
}

static int S(int px){ return MulDiv(px, g_dpi, 96); }

static HFONT MakeUIFont(HWND h, int pt, bool bold, const wchar_t* face = L"Segoe UI"){
    LOGFONTW lf{};
    lf.lfHeight = -MulDiv(pt, g_dpi, 72);
    wcscpy(lf.lfFaceName, face);
    lf.lfWeight = bold ? FW_SEMIBOLD : FW_NORMAL;
    lf.lfQuality = CLEARTYPE_QUALITY;
    return CreateFontIndirectW(&lf);
}

static void CreateFonts(HWND h){
    if (g_uiFont) DeleteObject(g_uiFont);
    if (g_uiFontBold) DeleteObject(g_uiFontBold);
    if (g_hudFont) DeleteObject(g_hudFont);
    g_uiFont = MakeUIFont(h, 9, false);
    g_uiFontBold = MakeUIFont(h, 9, true);
    g_hudFont = MakeUIFont(h, 8, true, L"Consolas");
}

static void ApplyUIFont(HWND parent){
    if(!g_uiFont) return;

    EnumChildWindows(parent, [](HWND w, LPARAM)->BOOL{
        SendMessageW(w, WM_SETFONT, (WPARAM)g_uiFont, TRUE);
        return TRUE;
    }, 0);
}

#ifndef SIO_UDP_CONNRESET
#define IOC_IN  0x80000000
#define IOC_VENDOR 0x18000000
#define _WSAIOW(x,y) (IOC_IN|(x)|(y))
#define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR,12)
#endif

// RAII helper to initialize and shut down WinSock2.
struct WsaInit {
    WsaInit(){ WSADATA w; WSAStartup(MAKEWORD(2,2), &w); }
    ~WsaInit(){ WSACleanup(); }
} static g_wsa;

static inline double clampd(double v,double lo,double hi){ return v<lo?lo:(v>hi?hi:v); }
template<typename T>
static inline T iclamp(T v, T lo, T hi){ return v<lo?lo:(v>hi?hi:v); }

#pragma pack(push,1)

// Compact UDP packet carrying 16 PWM servo channels.
struct servo_packet_16 {
    uint16_t magic = 18458;
    uint16_t frame_rate;
    uint32_t frame_count;
    uint16_t pwm[16];
};
static_assert(sizeof(servo_packet_16) == (4 + 4 + 16*2), "servo_packet_16 size mismatch");

// Compact UDP packet carrying 32 PWM servo channels.
struct servo_packet_32 {
    uint16_t magic = 29569;
    uint16_t frame_rate;
    uint32_t frame_count;
    uint16_t pwm[32];
};
static_assert(sizeof(servo_packet_32) == (4 + 4 + 32*2), "servo_packet_32 size mismatch");

#pragma pack(pop)

// Configuration of the remote SITL endpoint (IP and ports).
struct Dest {
    std::string ip="127.0.0.1";
    uint16_t port_tx=9003;
    uint16_t port_rx=9002;
};

static void disable_connreset(SOCKET s){
    DWORD bytes=0; BOOL b=FALSE;
    WSAIoctl(s, SIO_UDP_CONNRESET, &b, sizeof(b), NULL, 0, &bytes, NULL, NULL);
}

static std::mutex g_sitl_addr_mtx;
static struct sockaddr_in g_sitl_addr = {};
static bool g_sitl_addr_known = false;

// Thin wrapper around a UDP socket used for transmitting packets.
class UdpTx {
public:

    bool open(const std::string& ip, uint16_t port){
        close();
        sock_ = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
        if(sock_==INVALID_SOCKET) return false;
        disable_connreset(sock_);

        ip_ = "stateless"; port_ = 0;
        return true;
    }
    void close(){ if(sock_!=INVALID_SOCKET){ closesocket(sock_); sock_=INVALID_SOCKET; } }
    bool needs_reopen(const std::string& ip, uint16_t port) const {

        return ip!=ip_ || port!=port_;
    }

    bool send_buffer(const char* buf, int len, const struct sockaddr_in* dest){
        if(sock_==INVALID_SOCKET) return false;
        if (dest == nullptr || dest->sin_family != AF_INET || dest->sin_port == 0) return false;
        int sent = sendto(sock_, buf, len, 0, (const sockaddr*)dest, sizeof(struct sockaddr_in));
        return sent == len;
    }

    ~UdpTx(){ close(); }
private:
    SOCKET sock_=INVALID_SOCKET;
    sockaddr_in addr_{};
    std::string ip_="";
    uint16_t port_=0;
};

// Thin wrapper around a UDP socket used for receiving raw packets.
class UdpRxRaw {
public:

    bool open(uint16_t port){
        close();
        sock_ = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
        if(sock_==INVALID_SOCKET) return false;
        disable_connreset(sock_);
        sockaddr_in a{};
        a.sin_family=AF_INET;
        a.sin_port=htons(port);
        a.sin_addr.s_addr=INADDR_ANY;
        if(bind(sock_,(sockaddr*)&a,sizeof(a))==SOCKET_ERROR){ close(); return false; }
        DWORD to=10;
        setsockopt(sock_,SOL_SOCKET,SO_RCVTIMEO,(const char*)&to,sizeof(to));
        port_=port;
        return true;
    }
    void close(){ if(sock_!=INVALID_SOCKET){ closesocket(sock_); sock_=INVALID_SOCKET; } }
    bool needs_reopen(uint16_t port) const { return port!=port_; }

    int recv(uint8_t* out, int cap, struct sockaddr_in* from_addr){
        if(sock_==INVALID_SOCKET) return -1;

        int fromlen = sizeof(struct sockaddr_in);
        int len = recvfrom(sock_, (char*)out, cap, 0, (sockaddr*)from_addr, (socklen_t*)&fromlen);

        if(len==SOCKET_ERROR){
            int e=WSAGetLastError();
            if(e==WSAETIMEDOUT) return 0;
            return -1;
        }
        return len;
    }
    ~UdpRxRaw(){ close(); }
private:
    SOCKET sock_=INVALID_SOCKET;
    uint16_t port_=0;
};

enum DEF_ID { DEF_SENSORS=1 };
enum REQ_ID { REQ_SENSORS=1 };

enum EVT_ID {
    EVT_AIL=1, EVT_ELE, EVT_RUD, EVT_THR,
    EVT_AUX1, EVT_AUX2, EVT_AUX3, EVT_AUX4,
    EVT_AUX5, EVT_AUX6, EVT_AUX7, EVT_AUX8,
    EVT_AUX9, EVT_AUX10, EVT_AUX11, EVT_AUX12,
    EVT_THR_ALT,
    EVT_THR_PERCENT,
    EVT_RAW_AIL,
    EVT_RAW_ELE,
    EVT_RAW_RUD,
    EVT_RAW_THR
};

enum GRP_ID { GRP_INTERCEPT = 1 };

// Sensor snapshot populated from SimConnect for the current aircraft state.
struct RawSensors {
    double lat_deg=0, lon_deg=0;
    double alt_msl_ft=0, alt_agl_ft=0;
    double pitch_deg=0, bank_deg=0, hdg_true_deg=0;
    double ias_kt=0;
    double vel_e_fps=0, vel_n_fps=0, vel_u_fps=0;
    double p_rads=0, q_rads=0, r_rads=0;
    double accel_x_fps2=0, accel_y_fps2=0, accel_z_fps2=0;
    double engine_rpm=0, prop_rpm=0, prop_pitch_rad=0;
    double radio_height_ft=0, ground_alt_ft=0;

    double N_m=0, E_m=0, U_m=0;

    bool valid=false;
};

static HANDLE gSim=nullptr;

static const int g_sim_evt_map[16] = {
    EVT_AIL, EVT_ELE, EVT_THR, EVT_RUD,
    EVT_AUX1, EVT_AUX2, EVT_AUX3, EVT_AUX4,
    EVT_AUX5, EVT_AUX6, EVT_AUX7, EVT_AUX8,
    EVT_AUX9, EVT_AUX10, EVT_AUX11, EVT_AUX12
};

// Open a SimConnect session and subscribe to live aircraft sensor data.
static bool sim_open(){
    if(SimConnect_Open(&gSim, APP_TITLE_A, nullptr, 0, 0, 0)!=S_OK) return false;

    SimConnect_AddToDataDefinition(gSim, DEF_SENSORS, "PLANE LATITUDE","degrees",SIMCONNECT_DATATYPE_FLOAT64, 0.0f, 0);
    SimConnect_AddToDataDefinition(gSim, DEF_SENSORS, "PLANE LONGITUDE","degrees",SIMCONNECT_DATATYPE_FLOAT64, 0.0f, 1);
    SimConnect_AddToDataDefinition(gSim, DEF_SENSORS, "PLANE ALTITUDE","feet",SIMCONNECT_DATATYPE_FLOAT64, 0.0f, 2);
    SimConnect_AddToDataDefinition(gSim, DEF_SENSORS, "PLANE ALT ABOVE GROUND","feet",SIMCONNECT_DATATYPE_FLOAT64, 0.0f, 3);
    SimConnect_AddToDataDefinition(gSim, DEF_SENSORS, "PLANE PITCH DEGREES","degrees",SIMCONNECT_DATATYPE_FLOAT64, 0.0f, 4);
    SimConnect_AddToDataDefinition(gSim, DEF_SENSORS, "PLANE BANK DEGREES","degrees",SIMCONNECT_DATATYPE_FLOAT64, 0.0f, 5);
    SimConnect_AddToDataDefinition(gSim, DEF_SENSORS, "PLANE HEADING DEGREES TRUE","degrees",SIMCONNECT_DATATYPE_FLOAT64, 0.0f, 6);
    SimConnect_AddToDataDefinition(gSim, DEF_SENSORS, "AIRSPEED INDICATED","knots",SIMCONNECT_DATATYPE_FLOAT64, 0.0f, 7);
    SimConnect_AddToDataDefinition(gSim, DEF_SENSORS, "VELOCITY WORLD X","feet per second",SIMCONNECT_DATATYPE_FLOAT64, 0.0f, 8);
    SimConnect_AddToDataDefinition(gSim, DEF_SENSORS, "VELOCITY WORLD Z","feet per second",SIMCONNECT_DATATYPE_FLOAT64, 0.0f, 9);
    SimConnect_AddToDataDefinition(gSim, DEF_SENSORS, "VELOCITY WORLD Y","feet per second",SIMCONNECT_DATATYPE_FLOAT64, 0.0f, 10);
    SimConnect_AddToDataDefinition(gSim, DEF_SENSORS, "ROTATION VELOCITY BODY X","radians per second",SIMCONNECT_DATATYPE_FLOAT64, 0.0f, 11);
    SimConnect_AddToDataDefinition(gSim, DEF_SENSORS, "ROTATION VELOCITY BODY Y","radians per second",SIMCONNECT_DATATYPE_FLOAT64, 0.0f, 12);
    SimConnect_AddToDataDefinition(gSim, DEF_SENSORS, "ROTATION VELOCITY BODY Z","radians per second",SIMCONNECT_DATATYPE_FLOAT64, 0.0f, 13);
    SimConnect_AddToDataDefinition(gSim, DEF_SENSORS, "ACCELERATION BODY X","feet per second squared",SIMCONNECT_DATATYPE_FLOAT64, 0.0f, 14);
    SimConnect_AddToDataDefinition(gSim, DEF_SENSORS, "ACCELERATION BODY Y","feet per second squared",SIMCONNECT_DATATYPE_FLOAT64, 0.0f, 15);
    SimConnect_AddToDataDefinition(gSim, DEF_SENSORS, "ACCELERATION BODY Z","feet per second squared",SIMCONNECT_DATATYPE_FLOAT64, 0.0f, 16);
    SimConnect_AddToDataDefinition(gSim, DEF_SENSORS, "GENERAL ENG RPM:1","rpm",SIMCONNECT_DATATYPE_FLOAT64, 0.0f, 17);
    SimConnect_AddToDataDefinition(gSim, DEF_SENSORS, "PROP RPM:1","rpm",SIMCONNECT_DATATYPE_FLOAT64, 0.0f, 18);
    SimConnect_AddToDataDefinition(gSim, DEF_SENSORS, "PROP BETA:1","radians",SIMCONNECT_DATATYPE_FLOAT64, 0.0f, 19);
    SimConnect_AddToDataDefinition(gSim, DEF_SENSORS, "RADIO HEIGHT","feet",SIMCONNECT_DATATYPE_FLOAT64, 0.0f, 20);
    SimConnect_AddToDataDefinition(gSim, DEF_SENSORS, "GROUND ALTITUDE","feet",SIMCONNECT_DATATYPE_FLOAT64, 0.0f, 21);

    SimConnect_RequestDataOnSimObject(gSim, REQ_SENSORS, DEF_SENSORS, SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_SIM_FRAME, SIMCONNECT_DATA_REQUEST_FLAG_DEFAULT, 0,0,0);

    for (int i = 0; i < 16; i++) {
        SimConnect_MapClientEventToSimEvent(gSim, g_sim_evt_map[i], get_sim_evt_by_idx(G_sim_evt_idx[i]));
    }

    SimConnect_MapClientEventToSimEvent(gSim, EVT_THR_ALT, "AXIS_THROTTLE1_SET");
    SimConnect_MapClientEventToSimEvent(gSim, EVT_THR_PERCENT, "THROTTLE1_SET");

    SimConnect_MapClientEventToSimEvent(gSim, EVT_RAW_AIL, "AXIS_AILERONS_SET");
    SimConnect_MapClientEventToSimEvent(gSim, EVT_RAW_ELE, "AXIS_ELEVATOR_SET");
    SimConnect_MapClientEventToSimEvent(gSim, EVT_RAW_RUD, "AXIS_RUDDER_SET");
    SimConnect_MapClientEventToSimEvent(gSim, EVT_RAW_THR, "THROTTLE_AXIS_SET_EX1");

    SimConnect_MapInputEventToClientEvent(gSim, GRP_INTERCEPT, "AXIS_AILERONS_SET", EVT_RAW_AIL);
    SimConnect_MapInputEventToClientEvent(gSim, GRP_INTERCEPT, "AXIS_ELEVATOR_SET", EVT_RAW_ELE);
    SimConnect_MapInputEventToClientEvent(gSim, GRP_INTERCEPT, "AXIS_RUDDER_SET", EVT_RAW_RUD);
    SimConnect_MapInputEventToClientEvent(gSim, GRP_INTERCEPT, "THROTTLE_AXIS_SET_EX1", EVT_RAW_THR);

    SimConnect_SetInputGroupState(gSim, GRP_INTERCEPT, SIMCONNECT_STATE_ON);

    SimConnect_SetInputGroupPriority(gSim, GRP_INTERCEPT, SIMCONNECT_GROUP_PRIORITY_STANDARD);

    return true;
}

static void sim_close(){ if(gSim){ SimConnect_Close(gSim); gSim=nullptr; }}

#define IDC_GAIN_TRK   462
#define IDC_GAIN_VAL   463
#define IDC_IP      201
#define IDC_TX      202
#define IDC_RX      203
#define IDC_RATE    204
#define IDC_MATCH_SIM   210
#define IDC_RESAMPLE    211
#define ID_LBL_MATCH    212
#define ID_LBL_RESAMP   213
#define IDC_RESAMPLE_BTN 214
#define IDC_TIME_SYNC_CB 217
#define ID_LBL_TSYNC     218
#define IDC_NO_LOCKSTEP_CB 219
#define ID_LBL_LOCKSTEP    220
#define ID_LBL_POS_FMT     215
#define IDC_POS_FMT_CB     216

#define IDC_SAVE    205
#define IDC_STAT    206
#define IDC_JOYLBL  207
#define IDC_JOYCB   208

#define IDC_INVS_CH1    224
#define IDC_INVS_CH_BASE 224

#define IDC_JOYCAL_BTN  232
#define IDC_JOYBTN      233

#define WM_APP_STATUSTEXT (WM_APP+1)
#define WM_APP_SIM_STATUS (WM_APP+2)
#define WM_APP_TX_STATUS  (WM_APP+3)
#define WM_APP_RX_STATUS  (WM_APP+4)

#define IDC_AXLBL_BASE   300
#define IDC_AXPB_BASE    320
#define IDC_MAP_LBL_BASE 340
#define IDC_MAP_DST_CB   360
#define IDC_MAP_SRC_INV  380
#define IDC_MAP_OVR_CB   400
#define IDC_GAIN_LBL     460

#define IDC_MAP_OVR_LBL  470
#define ID_LBL_IP 1000
#define ID_LBL_TX 1001
#define ID_LBL_RX 1002
#define ID_LBL_HZ 1003
#define IDC_SIMDBG_LBL  7010
#define IDC_SIMDBG_LIST 7011

#define IDC_GRP_CONN    5001
#define IDC_GRP_JOY     5002
#define IDC_GRP_SITL_OUT 5003
#define IDC_GRP_DI_IN   5004
#define IDC_GRP_MAPPING 5005
#define IDC_GRP_GAIN    5006
#define IDC_GRP_STATUS  5007
#define IDC_HUD_DISPLAY 5008

#define IDC_SITL_OUT_LBL_BASE 6000
#define IDC_SITL_OUT_PB_BASE  6020
#define IDC_SITL_OUT_VAL_BASE 6040
#define IDC_AX_VAL_BASE       6060

#define IDC_STATUS_LED_SIM    6100
#define IDC_STATUS_LBL_SIM    6101
#define IDC_STATUS_LED_TX     6102
#define IDC_STATUS_LBL_TX     6103
#define IDC_STATUS_LED_RX     6104
#define IDC_STATUS_LBL_RX     6105
#define IDC_SITL_OUT_REV_LBL1 6106
#define IDC_SITL_OUT_REV_LBL2 6107

// Last PWM frame received from SITL plus basic timing metadata.
struct PWMLast{
    uint16_t rate_hz=0;
    uint32_t frame=0;
    std::vector<uint16_t> pwm;
    std::chrono::steady_clock::time_point tlast{};
};

static const wchar_t* AXIS_SRC_NAMES[] = {
    L"Axis 1 (X)", L"Axis 2 (Y)", L"Axis 3 (Z)",
    L"Axis 4 (Rx)", L"Axis 5 (Ry)", L"Axis 6 (Rz)",
    L"Axis 7 (Slider1)", L"Axis 8 (Slider2)",
    L"V-Axis 9 (POV1 Y)", L"V-Axis 10 (POV1 X)",
    L"V-Axis 11 (Button1)", L"V-Axis 12 (Button2)"
};
const int NUM_JOY_AXES = (sizeof(AXIS_SRC_NAMES) / sizeof(AXIS_SRC_NAMES[0]));

static const wchar_t* RC_DEST_NAMES[] = {
    L"(None)",
    L"RC1", L"RC2", L"RC3", L"RC4",
    L"RC5", L"RC6", L"RC7", L"RC8",
    L"RC9", L"RC10", L"RC11", L"RC12",
};
const int NUM_RC_DESTS = (sizeof(RC_DEST_NAMES) / sizeof(RC_DEST_NAMES[0]));

// Per-joystick-axis mapping to a RC channel and inversion/override flags.
struct JoyMapCfg {
    int rcDest = 0;
    int srcInv = +1;
    int overrideMode = 0;
};

// Shared state between GUI, SimConnect, joystick and networking threads.
struct Shared {
    bool invsim_ch[16]{};

    std::mutex m_tx;
    Dest dest{};
    int rate_hz=1000;
    bool match_sim_rate=false;
    int resample_mode=0;
    double sim_dt_ms=33.3;
    bool use_time_sync=true;
    bool no_lockstep=false;
    int json_pos_mode=0;

    bool sim_origin_set = true;

    double sim_origin_lat = -35.363261;
    double sim_origin_lon = 149.165230;
    double sim_origin_alt_m = 584.0;
    double sim_earth_radius = 6378137.0;

    int joy_index=0;
    double deadzone=0.02;

    JoyMapCfg joy_map[NUM_JOY_AXES];

    RawSensors R{};

    std::mutex m_rx;
    PWMLast pwm{};

    std::mutex m_gui;
    double raw_axes[NUM_JOY_AXES]{0};
    bool raw_buttons[8]{};
    double rc_out[12]{};

    double sitl_out_pwm[16]{};
    bool sitl_has_ch[16]{};
    int win_x = CW_USEDEFAULT, win_y = CW_USEDEFAULT;
    int win_w = 780, win_h = 740;
    std::atomic<bool> joy_ok{false};

    std::atomic<bool> status_sim_ok{false};
    std::atomic<bool> status_tx_ok{false};
    std::atomic<bool> status_rx_ok{false};
    std::atomic<double> status_sim_rate{0.0};
    std::atomic<double> status_tx_rate{0.0};
    std::atomic<double> status_rx_rate{0.0};

} static G;

static HWND g_simDbgPopup = NULL;
static const wchar_t* kSimDbgPopupClass = L"MSFS_AP_BRIDGE_SIMDBG_POPUP";

struct SimField { const wchar_t* name; const wchar_t* unit; };
static const SimField kSimFields[] = {
    { L"Latitude",           L"deg"     },
    { L"Longitude",          L"deg"     },
    { L"Alt MSL",            L"ft"      },
    { L"Alt AGL",            L"ft"      },
    { L"Pitch",              L"deg"     },
    { L"Bank",               L"deg"     },
    { L"Heading True",       L"deg"     },
    { L"Airspeed Indicated", L"kt"      },
    { L"Vel East",           L"ft/s"    },
    { L"Vel North",          L"ft/s"    },
    { L"Vel Up",             L"ft/s"    },
    { L"p (roll rate)",      L"rad/s"   },
    { L"q (pitch rate)",     L"rad/s"   },
    { L"r (yaw rate)",       L"rad/s"   },
    { L"Accel X",            L"ft/s^2"  },
    { L"Accel Y",            L"ft/s^2"  },
    { L"Accel Z",            L"ft/s^2"  },
    { L"Engine RPM",         L"rpm"     },
    { L"Prop RPM",           L"rpm"     },
    { L"Prop Beta",          L"rad"     },
    { L"Radio Height",       L"ft"      },
    { L"Ground Alt",         L"ft"      },
    { L"Valid",              L""        },
};
static const int kSimFieldCount = (int)(sizeof(kSimFields)/sizeof(kSimFields[0]));

static void InitSimDbgList(HWND lv){
    if (!lv) return;
    ListView_SetExtendedListViewStyle(lv, LVS_EX_FULLROWSELECT|LVS_EX_GRIDLINES|LVS_EX_DOUBLEBUFFER);
    LVCOLUMNW col{};
    col.mask = LVCF_TEXT|LVCF_WIDTH|LVCF_SUBITEM;
    col.pszText = const_cast<wchar_t*>(L"Key"); col.cx = S(160); col.iSubItem=0; ListView_InsertColumn(lv, 0, &col);
    col.pszText = const_cast<wchar_t*>(L"Value"); col.cx = S(140); col.iSubItem=1; ListView_InsertColumn(lv, 1, &col);
    col.pszText = const_cast<wchar_t*>(L"Unit"); col.cx = S(80); col.iSubItem=2; ListView_InsertColumn(lv, 2, &col);
    for(int i=0;i<kSimFieldCount;i++){
        LVITEMW it{};
        it.mask = LVIF_TEXT; it.iItem=i; it.iSubItem=0; it.pszText=const_cast<wchar_t*>(kSimFields[i].name);
        ListView_InsertItem(lv, &it);
        ListView_SetItemText(lv, i, 2, const_cast<wchar_t*>(kSimFields[i].unit));
    }
}

static void UpdateSimDbgValues(){
    if (!g_lvSimDbg) return;
    RawSensors R{};
    { std::lock_guard<std::mutex> lk(G.m_tx); R = G.R; }
    wchar_t b[128];
    swprintf(b,128,L"%.6f", R.lat_deg);           ListView_SetItemText(g_lvSimDbg,  0,1,b);
    swprintf(b,128,L"%.6f", R.lon_deg);           ListView_SetItemText(g_lvSimDbg,  1,1,b);
    swprintf(b,128,L"%.1f", R.alt_msl_ft);        ListView_SetItemText(g_lvSimDbg,  2,1,b);
    swprintf(b,128,L"%.1f", R.alt_agl_ft);        ListView_SetItemText(g_lvSimDbg,  3,1,b);
    swprintf(b,128,L"%.2f", R.pitch_deg);         ListView_SetItemText(g_lvSimDbg,  4,1,b);
    swprintf(b,128,L"%.2f", R.bank_deg);          ListView_SetItemText(g_lvSimDbg,  5,1,b);
    swprintf(b,128,L"%.2f", R.hdg_true_deg);      ListView_SetItemText(g_lvSimDbg,  6,1,b);
    swprintf(b,128,L"%.1f", R.ias_kt);            ListView_SetItemText(g_lvSimDbg,  7,1,b);
    swprintf(b,128,L"%.2f", R.vel_e_fps);         ListView_SetItemText(g_lvSimDbg,  8,1,b);
    swprintf(b,128,L"%.2f", R.vel_n_fps);         ListView_SetItemText(g_lvSimDbg,  9,1,b);
    swprintf(b,128,L"%.2f", R.vel_u_fps);         ListView_SetItemText(g_lvSimDbg, 10,1,b);
    swprintf(b,128,L"%.3f", R.p_rads);            ListView_SetItemText(g_lvSimDbg, 11,1,b);
    swprintf(b,128,L"%.3f", R.q_rads);            ListView_SetItemText(g_lvSimDbg, 12,1,b);
    swprintf(b,128,L"%.3f", R.r_rads);            ListView_SetItemText(g_lvSimDbg, 13,1,b);
    swprintf(b,128,L"%.2f", R.accel_x_fps2);      ListView_SetItemText(g_lvSimDbg, 14,1,b);
    swprintf(b,128,L"%.2f", R.accel_y_fps2);      ListView_SetItemText(g_lvSimDbg, 15,1,b);
    swprintf(b,128,L"%.2f", R.accel_z_fps2);      ListView_SetItemText(g_lvSimDbg, 16,1,b);
    swprintf(b,128,L"%.0f", R.engine_rpm);        ListView_SetItemText(g_lvSimDbg, 17,1,b);
    swprintf(b,128,L"%.0f", R.prop_rpm);          ListView_SetItemText(g_lvSimDbg, 18,1,b);
    swprintf(b,128,L"%.3f", R.prop_pitch_rad);    ListView_SetItemText(g_lvSimDbg, 19,1,b);
    swprintf(b,128,L"%.1f", R.radio_height_ft);   ListView_SetItemText(g_lvSimDbg, 20,1,b);
    swprintf(b,128,L"%.1f", R.ground_alt_ft);     ListView_SetItemText(g_lvSimDbg, 21,1,b);
    ListView_SetItemText(g_lvSimDbg, 22,1, const_cast<wchar_t*>(R.valid?L"YES":L"NO"));
}

// Window procedure for the popup that shows raw SimConnect sensor values.
static LRESULT CALLBACK SimDbgWndProc(HWND h, UINT m, WPARAM w, LPARAM l) {

    switch (m) {
        case WM_CREATE: {
            g_dpi = Dpi(h);

            g_lblSimDbg = CreateWindowExW(0,L"STATIC",L"SimConnect Live (10+ Hz):",WS_CHILD|WS_VISIBLE|SS_LEFT|SS_ENDELLIPSIS,
            S(10), S(10), S(260), S(24), h,(HMENU)(INT_PTR)IDC_SIMDBG_LBL, GetModuleHandle(NULL), NULL);
            g_lvSimDbg = CreateWindowExW(WS_EX_CLIENTEDGE, WC_LISTVIEW, L"", WS_CHILD|WS_VISIBLE|LVS_REPORT|LVS_SINGLESEL|LVS_NOSORTHEADER,
            S(10), S(40), S(400), S(500), h, (HMENU)(INT_PTR)IDC_SIMDBG_LIST, GetModuleHandle(NULL), NULL);
            InitSimDbgList(g_lvSimDbg);
            ApplyUIFont(h);

            SendMessageW(g_lblSimDbg, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SetTimer(h, 2, 100, NULL);
            return 0;
        }
        case WM_TIMER: {
            if (w == 2) UpdateSimDbgValues();
            return 0;
        }
        case WM_SIZE: {
            RECT rc; GetClientRect(h, &rc);
            int w_size = rc.right - rc.left;
            int h_win = rc.bottom - rc.top;
            if (g_lblSimDbg) MoveWindow(g_lblSimDbg, S(10), S(10), w_size - S(20), S(24), TRUE);
            if (g_lvSimDbg) MoveWindow(g_lvSimDbg, S(10), S(40), w_size - S(20), h_win - S(50), TRUE);
            return 0;
        }
        case WM_DPICHANGED: {
            g_dpi = HIWORD(w);
            ApplyUIFont(h);

            if(g_lblSimDbg) SendMessageW(g_lblSimDbg, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            RECT* prc = (RECT*)l;
            if (prc) SetWindowPos(h, NULL, prc->left, prc->top, prc->right-prc->left, prc->bottom-prc->top, SWP_NOZORDER|SWP_NOACTIVATE);
            return 0;
        }
        case WM_CLOSE:
        KillTimer(h, 2);
        g_lvSimDbg = NULL;
        g_lblSimDbg = NULL;
        DestroyWindow(h);
        g_simDbgPopup = NULL;
        return 0;
    }
    return DefWindowProcW(h, m, w, l);
}

// Create and show the SimConnect live sensor debug popup window.
static void ShowSimDbgPopup(HWND parent) {

    if (g_simDbgPopup) {
        SetForegroundWindow(g_simDbgPopup);
        return;
    }

    WNDCLASSEXW wc{sizeof(WNDCLASSEXW)};
    wc.lpfnWndProc=SimDbgWndProc;
    wc.hInstance=GetModuleHandle(NULL);
    wc.lpszClassName=kSimDbgPopupClass;
    wc.hCursor=LoadCursor(NULL,IDC_ARROW);
    wc.hbrBackground=(HBRUSH)(COLOR_WINDOW+1);
    RegisterClassExW(&wc);

    g_simDbgPopup = CreateWindowExW(WS_EX_TOOLWINDOW, kSimDbgPopupClass, L"SimConnect Live Sensor Data",
    WS_OVERLAPPEDWINDOW|WS_VISIBLE,
    CW_USEDEFAULT, CW_USEDEFAULT, S(440), S(600),
    parent, NULL, GetModuleHandle(NULL), NULL);
}

// Window class name used by the synthetic HUD control embedded in the main dialog.
static const wchar_t* kHudClass = L"MSFS_AP_BRIDGE_HUD";
// Window procedure that renders the synthetic glass-cockpit style HUD overlay.
static LRESULT CALLBACK HudWndProc(HWND h, UINT m, WPARAM w, LPARAM l) {
    switch (m) {
        case WM_ERASEBKGND:
            return 1;
        case WM_PAINT: {
            PAINTSTRUCT ps;
            HDC hdc = BeginPaint(h, &ps);
            RECT rc; GetClientRect(h, &rc);
            int w_px = rc.right - rc.left;
            int h_px = rc.bottom - rc.top;
            if (w_px <= 0 || h_px <= 0) {
                EndPaint(h, &ps);
                return 0;
            }
            HDC hdcMem = CreateCompatibleDC(hdc);
            HBITMAP hBmp = CreateCompatibleBitmap(hdc, w_px, h_px);
            HBITMAP hBmpOld = (HBITMAP)SelectObject(hdcMem, hBmp);
            HBRUSH hBrBlack = CreateSolidBrush(RGB(0, 0, 0));
            FillRect(hdcMem, &rc, hBrBlack);
            DeleteObject(hBrBlack);

            RawSensors R{};
            { std::lock_guard<std::mutex> lk(G.m_tx); R = G.R; }

            HBRUSH hBrSky1 = CreateSolidBrush(RGB(0, 76, 153));
            HBRUSH hBrSky2 = CreateSolidBrush(RGB(51, 127, 204));
            HBRUSH hBrGround1 = CreateSolidBrush(RGB(101, 67, 33));
            HBRUSH hBrGround2 = CreateSolidBrush(RGB(61, 43, 31));
            HPEN hPenHorizonSuper = CreatePen(PS_SOLID, 2, RGB(255, 255, 255));
            HPEN hPenHorizonGlow = CreatePen(PS_SOLID, 0, RGB(255, 255, 100));
            HPEN hPenWhiteThin = CreatePen(PS_SOLID, 1, RGB(255, 255, 255));
            HPEN hPenWhiteMed = CreatePen(PS_SOLID, 2, RGB(255, 255, 255));
            HPEN hPenYellow = CreatePen(PS_SOLID, 3, RGB(255, 255, 0));
            HPEN hPenGreen = CreatePen(PS_SOLID, 2, RGB(0, 255, 0));
            HPEN hPenDashed = CreatePen(PS_DOT, 1, RGB(255, 255, 100));
            HFONT hFontOld = (HFONT)SelectObject(hdcMem, g_hudFont);

            int cx = w_px / 2;
            int cy = h_px / 2;
            double roll_rad = deg2rad(R.bank_deg);
            double pitch_pixels_per_deg = S(5);
            double pitch_offset_px = R.pitch_deg * pitch_pixels_per_deg;

            SetGraphicsMode(hdcMem, GM_ADVANCED);
            XFORM xform;
            xform.eM11 = (FLOAT)cos(roll_rad);
            xform.eM12 = (FLOAT)sin(roll_rad);
            xform.eM21 = (FLOAT)-sin(roll_rad);
            xform.eM22 = (FLOAT)cos(roll_rad);
            xform.eDx = (FLOAT)cx;
            xform.eDy = (FLOAT)cy - (FLOAT)pitch_offset_px;
            SetWorldTransform(hdcMem, &xform);

            int R_size = (int)(sqrt((double)(w_px*w_px + h_px*h_px)) * 2);
            SelectObject(hdcMem, GetStockObject(NULL_PEN));
            SelectObject(hdcMem, hBrSky1);
            Rectangle(hdcMem, -R_size/2, -R_size*2, R_size/2, -R_size/2);
            SelectObject(hdcMem, hBrSky2);
            Rectangle(hdcMem, -R_size/2, -R_size/2, R_size/2, 0);
            SelectObject(hdcMem, hBrGround1);
            Rectangle(hdcMem, -R_size/2, 0, R_size/2, R_size/2);
            SelectObject(hdcMem, hBrGround2);
            Rectangle(hdcMem, -R_size/2, R_size/2, R_size/2, R_size*2);

            SelectObject(hdcMem, hPenHorizonGlow);
            MoveToEx(hdcMem, -R_size, 0, NULL);
            LineTo(hdcMem, R_size, 0);
            SelectObject(hdcMem, hPenHorizonSuper);
            MoveToEx(hdcMem, -R_size, 0, NULL);
            LineTo(hdcMem, R_size, 0);

            SetTextAlign(hdcMem, TA_CENTER | TA_BOTTOM);
            SetBkMode(hdcMem, TRANSPARENT);
            SetTextColor(hdcMem, RGB(255, 255, 255));

            for (int p = -90; p <= 90; p += 5) {
                if (p == 0) continue;
                int y_pos = (int)(-p * pitch_pixels_per_deg);
                if (p > 0) {
                    SelectObject(hdcMem, hPenWhiteMed);
                } else {
                    SelectObject(hdcMem, hPenDashed);
                }
                if (p % 10 == 0) {
                    int line_len = S(60);
                    MoveToEx(hdcMem, -line_len, y_pos, NULL);
                    LineTo(hdcMem, -line_len/3, y_pos);
                    MoveToEx(hdcMem, line_len/3, y_pos, NULL);
                    LineTo(hdcMem, line_len, y_pos);
                    if (p % 20 == 0) {
                        MoveToEx(hdcMem, -line_len/3, y_pos - S(5), NULL);
                        LineTo(hdcMem, -line_len/3, y_pos + S(5));
                        MoveToEx(hdcMem, line_len/3, y_pos - S(5), NULL);
                        LineTo(hdcMem, line_len/3, y_pos + S(5));
                    }
                    wchar_t buf[8];
                    swprintf(buf, 8, L"%d", abs(p));
                    TextOutW(hdcMem, -line_len - S(15), y_pos + S(5), buf, (int)wcslen(buf));
                    TextOutW(hdcMem, line_len + S(15), y_pos + S(5), buf, (int)wcslen(buf));
                } else {
                    int line_len = S(30);
                    MoveToEx(hdcMem, -line_len, y_pos, NULL);
                    LineTo(hdcMem, -line_len/2, y_pos);
                    MoveToEx(hdcMem, line_len/2, y_pos, NULL);
                    LineTo(hdcMem, line_len, y_pos);
                }
            }
            ModifyWorldTransform(hdcMem, NULL, MWT_IDENTITY);

            SelectObject(hdcMem, hPenYellow);
            SelectObject(hdcMem, GetStockObject(NULL_BRUSH));
            int wing_w = S(120);
            MoveToEx(hdcMem, cx - wing_w, cy, NULL);
            LineTo(hdcMem, cx - S(20), cy);
            MoveToEx(hdcMem, cx + S(20), cy, NULL);
            LineTo(hdcMem, cx + wing_w, cy);
            MoveToEx(hdcMem, cx - wing_w, cy, NULL);
            LineTo(hdcMem, cx - wing_w, cy + S(15));
            MoveToEx(hdcMem, cx + wing_w, cy, NULL);
            LineTo(hdcMem, cx + wing_w, cy + S(15));

            HBRUSH hBrYellow = CreateSolidBrush(RGB(255, 255, 0));
            SelectObject(hdcMem, hBrYellow);
            Ellipse(hdcMem, cx - S(6), cy - S(6), cx + S(6), cy + S(6));
            DeleteObject(hBrYellow);

            int roll_arc_y = S(80);
            SelectObject(hdcMem, hPenWhiteThin);
            SelectObject(hdcMem, GetStockObject(NULL_BRUSH));
            Arc(hdcMem, cx - S(70), roll_arc_y - S(70), cx + S(70), roll_arc_y + S(70),
                cx - S(60), roll_arc_y - S(35), cx + S(60), roll_arc_y - S(35));
            int roll_marks[] = {-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60};
            for (int i = 0; i < 11; i++) {
                double angle_rad = deg2rad(roll_marks[i] - 90);
                int x1 = cx + (int)(S(65) * cos(angle_rad));
                int y1 = roll_arc_y + (int)(S(65) * sin(angle_rad));
                int x2 = cx + (int)(S(roll_marks[i] % 30 == 0 ? 75 : 70) * cos(angle_rad));
                int y2 = roll_arc_y + (int)(S(roll_marks[i] % 30 == 0 ? 75 : 70) * sin(angle_rad));
                MoveToEx(hdcMem, x1, y1, NULL);
                LineTo(hdcMem, x2, y2);
            }

            SelectObject(hdcMem, hPenYellow);
            double roll_angle_rad = deg2rad(R.bank_deg - 90);
            int roll_x = cx + (int)(S(60) * cos(roll_angle_rad));
            int roll_y = roll_arc_y + (int)(S(60) * sin(roll_angle_rad));
            POINT triangle[3] = {
                {roll_x, roll_y},
                {roll_x - S(5), roll_y - S(10)},
                {roll_x + S(5), roll_y - S(10)}
            };
            if (abs(R.bank_deg) < 90) {
                triangle[1].x = cx + (int)(S(50) * cos(roll_angle_rad - 0.15));
                triangle[1].y = roll_arc_y + (int)(S(50) * sin(roll_angle_rad - 0.15));
                triangle[2].x = cx + (int)(S(50) * cos(roll_angle_rad + 0.15));
                triangle[2].y = roll_arc_y + (int)(S(50) * sin(roll_angle_rad + 0.15));
            }
            HBRUSH hBrTriangle = CreateSolidBrush(RGB(255, 255, 0));
            SelectObject(hdcMem, hBrTriangle);
            Polygon(hdcMem, triangle, 3);
            DeleteObject(hBrTriangle);

            HBRUSH hBrWhite = CreateSolidBrush(RGB(255, 255, 255));
            SelectObject(hdcMem, hBrWhite);
            POINT ref_triangle[3] = {
                {cx, roll_arc_y - S(70)},
                {cx - S(5), roll_arc_y - S(78)},
                {cx + S(5), roll_arc_y - S(78)}
            };
            Polygon(hdcMem, ref_triangle, 3);
            DeleteObject(hBrWhite);

            HBRUSH hBrPanel = CreateSolidBrush(RGB(20, 20, 20));
            SelectObject(hdcMem, hBrPanel);
            SelectObject(hdcMem, hPenGreen);


            int sensors_top = S(10);
            int sensors_bottom = S(80);

            int status_top = h_px - S(40);
            int status_bottom = h_px - S(10);


            Rectangle(hdcMem, cx - S(20), sensors_top, cx + S(20), sensors_top + S(35));


            int tape_margin = S(5);
            int tape_cy = cy;

            int tape_target_half = S(75);
            int avail_up = tape_cy - (sensors_bottom + tape_margin);
            int avail_down = (status_top - tape_margin) - tape_cy;
            int half_h = tape_target_half;
            if (half_h > avail_up) half_h = avail_up;
            if (half_h > avail_down) half_h = avail_down;
            if (half_h < S(40)) half_h = S(40);

            int tape_top = tape_cy - half_h;
            int tape_bottom = tape_cy + half_h;


            Rectangle(hdcMem, S(10), tape_top, S(70), tape_bottom);
            Rectangle(hdcMem, w_px - S(70), tape_top, w_px - S(10), tape_bottom);


            Rectangle(hdcMem, S(10), status_top, w_px - S(10), status_bottom);


            SetTextAlign(hdcMem, TA_LEFT | TA_TOP);
            SetTextColor(hdcMem, RGB(0, 255, 0));
            SetBkMode(hdcMem, OPAQUE);
            SetBkColor(hdcMem, RGB(0, 0, 0));
            wchar_t hud_buf[128];
            int x_text = S(15);
            int y_text = sensors_top + S(5);

            swprintf(hud_buf, 128, L"IAS: %3.0f KT", R.ias_kt);
            TextOutW(hdcMem, x_text, y_text, hud_buf, (int)wcslen(hud_buf));
            y_text += S(15);
            swprintf(hud_buf, 128, L"MSL: %5.0f FT", R.alt_msl_ft);
            TextOutW(hdcMem, x_text, y_text, hud_buf, (int)wcslen(hud_buf));
            y_text += S(15);
            swprintf(hud_buf, 128, L"AGL: %5.0f FT", R.alt_agl_ft);
            TextOutW(hdcMem, x_text, y_text, hud_buf, (int)wcslen(hud_buf));
            y_text += S(15);
            double vs_fpm = R.vel_u_fps * 60.0;
            swprintf(hud_buf, 128, L"V/S: %+4.0f FPM", vs_fpm);
            TextOutW(hdcMem, x_text, y_text, hud_buf, (int)wcslen(hud_buf));
            y_text += S(15);
            swprintf(hud_buf, 128, L"GS: %3.0f KT", sqrt(R.vel_e_fps*R.vel_e_fps + R.vel_n_fps*R.vel_n_fps) * 0.592484);
            TextOutW(hdcMem, x_text, y_text, hud_buf, (int)wcslen(hud_buf));


            y_text = sensors_top + S(5);
            SetTextAlign(hdcMem, TA_RIGHT | TA_TOP);
            int x_text_r = w_px - S(15);

            swprintf(hud_buf, 128, L"ROLL: %+5.1f B0", R.bank_deg);
            TextOutW(hdcMem, x_text_r, y_text, hud_buf, (int)wcslen(hud_buf));
            y_text += S(15);
            swprintf(hud_buf, 128, L"PITCH: %+5.1f B0", R.pitch_deg);
            TextOutW(hdcMem, x_text_r, y_text, hud_buf, (int)wcslen(hud_buf));
            y_text += S(15);


            swprintf(hud_buf, 128, L"RPM: %4.0f", R.engine_rpm);
            TextOutW(hdcMem, x_text_r, y_text, hud_buf, (int)wcslen(hud_buf));
            y_text += S(15);
            swprintf(hud_buf, 128, L"PROP: %4.0f", R.prop_rpm);
            TextOutW(hdcMem, x_text_r, y_text, hud_buf, (int)wcslen(hud_buf));


            SetBkMode(hdcMem, TRANSPARENT);


            SetTextAlign(hdcMem, TA_CENTER | TA_TOP);


            SetTextColor(hdcMem, RGB(255, 255, 0));
            swprintf(hud_buf, 128, L"HDG");
            TextOutW(hdcMem, cx, sensors_top + S(2), hud_buf, (int)wcslen(hud_buf));


            SetTextColor(hdcMem, RGB(0, 255, 0));
            swprintf(hud_buf, 128, L"%03d", (int)R.hdg_true_deg);
            TextOutW(hdcMem, cx, sensors_top + S(16), hud_buf, (int)wcslen(hud_buf));


            SetTextAlign(hdcMem, TA_CENTER | TA_TOP);


            for (int i = -5; i <= 5; i++) {
                int alt = (int)(R.alt_msl_ft / 100) * 100 + i * 100;
                int y_pos = tape_cy - i * S(20);
                if (y_pos > tape_top + S(5) && y_pos < tape_bottom - S(5)) {
                    swprintf(hud_buf, 128, L"%d", alt);
                    SetTextColor(hdcMem, RGB(200, 200, 200));
                    TextOutW(hdcMem, w_px - S(40), y_pos - S(5), hud_buf, (int)wcslen(hud_buf));
                }
            }

            COLORREF boxColor = RGB(255, 215, 0);
            HBRUSH hBrYellowBox = CreateSolidBrush(boxColor);
            SelectObject(hdcMem, hBrYellowBox);
            SelectObject(hdcMem, hPenYellow);
            Rectangle(hdcMem, w_px - S(65), tape_cy - S(12), w_px - S(15), tape_cy + S(12));
            SetTextColor(hdcMem, RGB(0, 0, 0));
            SetBkMode(hdcMem, OPAQUE);
            SetBkColor(hdcMem, boxColor);
            swprintf(hud_buf, 128, L"%d", (int)R.alt_msl_ft);
            TextOutW(hdcMem, w_px - S(40), tape_cy - S(8), hud_buf, (int)wcslen(hud_buf));
            SetBkMode(hdcMem, TRANSPARENT);


            for (int i = -5; i <= 5; i++) {
                int spd = (int)(R.ias_kt / 10) * 10 + i * 10;
                if (spd < 0) spd = 0;
                int y_pos = tape_cy - i * S(20);
                if (y_pos > tape_top + S(5) && y_pos < tape_bottom - S(5)) {
                    swprintf(hud_buf, 128, L"%d", spd);
                    SetTextColor(hdcMem, RGB(200, 200, 200));
                    TextOutW(hdcMem, S(40), y_pos - S(5), hud_buf, (int)wcslen(hud_buf));
                }
            }

            SelectObject(hdcMem, hBrYellowBox);
            SelectObject(hdcMem, hPenYellow);
            Rectangle(hdcMem, S(15), tape_cy - S(12), S(65), tape_cy + S(12));
            SetTextColor(hdcMem, RGB(0, 0, 0));
            SetBkMode(hdcMem, OPAQUE);
            SetBkColor(hdcMem, boxColor);
            swprintf(hud_buf, 128, L"%d", (int)R.ias_kt);
            TextOutW(hdcMem, S(40), tape_cy - S(8), hud_buf, (int)wcslen(hud_buf));
            SetBkMode(hdcMem, TRANSPARENT);
            DeleteObject(hBrYellowBox);

            SetTextAlign(hdcMem, TA_LEFT | TA_BOTTOM);
            SetTextColor(hdcMem, RGB(0, 255, 0));
            const char* status = R.valid ? "VALID" : "NO DATA";
            COLORREF status_color = R.valid ? RGB(0, 255, 0) : RGB(255, 0, 0);
            SetTextColor(hdcMem, status_color);
            swprintf(hud_buf, 128, L"STATUS: %S", status);
            TextOutW(hdcMem, S(15), h_px - S(15), hud_buf, (int)wcslen(hud_buf));
            SetTextAlign(hdcMem, TA_CENTER | TA_BOTTOM);
            SetTextColor(hdcMem, RGB(200, 200, 200));
            swprintf(hud_buf, 128, L"LAT: %.6f LON: %.6f", R.lat_deg, R.lon_deg);
            TextOutW(hdcMem, cx, h_px - S(15), hud_buf, (int)wcslen(hud_buf));
            SetTextAlign(hdcMem, TA_RIGHT | TA_BOTTOM);
            SYSTEMTIME st;
            GetLocalTime(&st);
            swprintf(hud_buf, 128, L"%02d:%02d:%02d", st.wHour, st.wMinute, st.wSecond);
            TextOutW(hdcMem, w_px - S(15), h_px - S(15), hud_buf, (int)wcslen(hud_buf));

            BitBlt(hdc, 0, 0, w_px, h_px, hdcMem, 0, 0, SRCCOPY);

            SelectObject(hdcMem, hFontOld);
            SelectObject(hdcMem, hBmpOld);
            DeleteObject(hBmp);
            DeleteDC(hdcMem);
            DeleteObject(hBrSky1);
            DeleteObject(hBrSky2);
            DeleteObject(hBrGround1);
            DeleteObject(hBrGround2);
            DeleteObject(hBrPanel);
            DeleteObject(hPenHorizonSuper);
            DeleteObject(hPenHorizonGlow);
            DeleteObject(hPenWhiteThin);
            DeleteObject(hPenWhiteMed);
            DeleteObject(hPenYellow);
            DeleteObject(hPenGreen);
            DeleteObject(hPenDashed);
            EndPaint(h, &ps);
            return 0;
        }
        case WM_TIMER: {
            if (w == 3) {
                InvalidateRect(h, NULL, FALSE);
            }
            return 0;
        }
        case WM_LBUTTONDOWN: {
            int x = (short)LOWORD(l);
            int y = (short)HIWORD(l);

            RECT rc;
            GetClientRect(h, &rc);
            int w_px = rc.right - rc.left;
            int h_px = rc.bottom - rc.top;
            int cx = w_px / 2;

            RECT rcLatLon;
            rcLatLon.left = cx - S(120);
            rcLatLon.right = cx + S(120);
            rcLatLon.top = h_px - S(30);
            rcLatLon.bottom = h_px - S(5);

            POINT pt;
            pt.x = x;
            pt.y = y;
            if (PtInRect(&rcLatLon, pt)) {
                RawSensors R;
                {
                    std::lock_guard<std::mutex> lk(G.m_tx);
                    R = G.R;
                }

                wchar_t buf[64];
                swprintf(buf, _countof(buf), L"%.6f, %.6f", R.lat_deg, R.lon_deg);

                if (OpenClipboard(h)) {
                    EmptyClipboard();
                    SIZE_T bytes = (wcslen(buf) + 1) * sizeof(wchar_t);
                    HGLOBAL hMem = GlobalAlloc(GMEM_MOVEABLE, bytes);
                    if (hMem) {
                        void* p = GlobalLock(hMem);
                        if (p) {
                            memcpy(p, buf, bytes);
                            GlobalUnlock(hMem);
                            SetClipboardData(CF_UNICODETEXT, hMem);
                        } else {
                            GlobalFree(hMem);
                        }
                    }
                    CloseClipboard();
                }
            }
            return 0;
        }
    }

    return DefWindowProcW(h, m, w, l);
}

// Paint a simple red/green status indicator LED inside a static control.
static void SetLedColor(HWND hLed, bool ok) {
    if (!hLed) return;
    HDC hdc = GetDC(hLed);
    if (!hdc) return;
    RECT rc; GetClientRect(hLed, &rc);
    HBRUSH hBrush = CreateSolidBrush(ok ? RGB(0, 200, 0) : RGB(200, 0, 0));
    FillRect(hdc, &rc, hBrush);
    DeleteObject(hBrush);
    ReleaseDC(hLed, hdc);
}


static HWND g_hwnd=0, g_stat=0, g_ip=0, g_tx=0, g_rx=0, g_rate=0, g_joycb=0;
static HWND g_match_sim=0, g_resample_cb=0;
static HWND g_time_sync_cb = 0;
static HWND g_no_lockstep_cb = 0;
static HWND g_pos_fmt_cb = 0;

static HWND g_lblSitlOut = 0;

static HWND g_sitl_out_lbl[16]{};
static HWND g_sitl_out_pb[16]{};
static HWND g_sitl_out_inv_chk[16]{};
static HWND g_sitl_out_val[16]{};

static HWND g_ax_pb[NUM_JOY_AXES]{};
static HWND g_ax_val[NUM_JOY_AXES]{};
static HWND g_lblJoy=0;

static HWND g_lblMapping=0, g_lblLivePreview=0, g_lblDest=0, g_lblReverseIn=0, g_lblOverride=0;

static HWND g_map_lbl[NUM_JOY_AXES]{};
static HWND g_map_dst_cb[NUM_JOY_AXES]{};
static HWND g_map_src_inv[NUM_JOY_AXES]{};
static HWND g_map_ovr_cb[NUM_JOY_AXES]{};

static HWND g_grp_conn=0, g_grp_joy=0, g_grp_sitl_out=0, g_grp_di_in=0, g_grp_status=0;
static HWND g_hud = 0;

static HWND g_led_sim = 0, g_lbl_sim_status = 0;
static HWND g_led_tx = 0, g_lbl_tx_status = 0;
static HWND g_led_rx = 0, g_lbl_rx_status = 0;

static HWND g_btnJoyCal = 0;

static LPDIRECTINPUT8       g_pDI = NULL;
static LPDIRECTINPUTDEVICE8 g_pJoy = NULL;
static std::vector<GUID>    g_joystickGUIDs;
static int                  g_selectedJoyIndex = -1;

static std::atomic<bool> RUN{true};
static std::atomic<bool> g_sim_ok{false};

static std::wstring g_ini_path;
static std::atomic<bool> g_logging_enabled{false};

static void LogSensorsToFile(const struct RawSensors& R);
static std::mutex g_log_mtx;
static FILE* g_log_file = nullptr;

static std::wstring get_log_path(){
    wchar_t mod[MAX_PATH];
    GetModuleFileNameW(NULL, mod, MAX_PATH);
    std::wstring p(mod);
    size_t dot = p.find_last_of(L'.');
    if (dot != std::wstring::npos) p = p.substr(0, dot);
    p += L".csv";
    return p;
}

static void OpenLogFile(void){
    std::lock_guard<std::mutex> lk(g_log_mtx);
    if (g_log_file){ fclose(g_log_file); g_log_file = nullptr; }
    std::wstring path = get_log_path();
    g_log_file = _wfopen(path.c_str(), L"w");

    if (g_log_file){
        fwprintf(g_log_file, L"utc_ms,utc_iso,local_iso,lat_deg,lon_deg,alt_msl_ft,alt_agl_ft,pitch_deg,bank_deg,hdg_true_deg,ias_kt,vel_e_fps,vel_n_fps,vel_u_fps,p_rads,q_rads,r_rads,accel_x_fps2,accel_y_fps2,accel_z_fps2,engine_rpm,prop_rpm,prop_pitch_rad,radio_height_ft,ground_alt_ft,valid,ch1_cmd,ch2_cmd,ch3_cmd,ch4_cmd\n");
        fflush(g_log_file);
    }
}

static void CloseLogFile(void){
    std::lock_guard<std::mutex> lk(g_log_mtx);
    if (g_log_file){ fclose(g_log_file); g_log_file = nullptr; }
}

static void LogSensorsToFile(const RawSensors& R){
    SYSTEMTIME st_utc; GetSystemTime(&st_utc);
    SYSTEMTIME st_loc; GetLocalTime(&st_loc);
    wchar_t utc_iso[40];
    wsprintfW(utc_iso, L"%04d-%02d-%02d %02d:%02d:%02d.%03d",
    st_utc.wYear, st_utc.wMonth, st_utc.wDay, st_utc.wHour, st_utc.wMinute, st_utc.wSecond, st_utc.wMilliseconds);
    wchar_t local_iso[40];
    wsprintfW(local_iso, L"%04d-%02d-%02d %02d:%02d:%02d.%03d",
    st_loc.wYear, st_loc.wMonth, st_loc.wDay, st_loc.wHour, st_loc.wMinute, st_loc.wSecond, st_loc.wMilliseconds);

    if (!g_logging_enabled.load()) return;
    std::lock_guard<std::mutex> lk(g_log_mtx);
    if (!g_log_file) return;

    unsigned long long now_ms = GetTickCount64();

    double ch_cmd[4] = { 0.5, 0.5, 0.0, 0.5 };
    {
        std::lock_guard<std::mutex> lk_gui(G.m_gui);
        if (G.sitl_has_ch[0]) ch_cmd[0] = G.sitl_out_pwm[0];
        if (G.sitl_has_ch[1]) ch_cmd[1] = G.sitl_out_pwm[1];
        if (G.sitl_has_ch[2]) ch_cmd[2] = G.sitl_out_pwm[2];
        if (G.sitl_has_ch[3]) ch_cmd[3] = G.sitl_out_pwm[3];
    }

    fwprintf(g_log_file, L"%llu,%ls,%ls,%.10f,%.10f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f,%.2f,%.2f,%.6f,%.4f,%.4f,%d,%.6f,%.6f,%.6f,%.6f\n", now_ms, utc_iso, local_iso,
    R.lat_deg, R.lon_deg,
    R.alt_msl_ft, R.alt_agl_ft,
    R.pitch_deg, R.bank_deg, R.hdg_true_deg, R.ias_kt,
    R.vel_e_fps, R.vel_n_fps, R.vel_u_fps,
    R.p_rads, R.q_rads, R.r_rads,
    R.accel_x_fps2, R.accel_y_fps2, R.accel_z_fps2,
    R.engine_rpm, R.prop_rpm, R.prop_pitch_rad,
    R.radio_height_ft, R.ground_alt_ft,
    (int)(R.valid ? 1 : 0),
    ch_cmd[0], ch_cmd[1], ch_cmd[2], ch_cmd[3]);
    fflush(g_log_file);
}
static std::atomic<uint64_t> g_next_log_ms{0};

static std::wstring get_ini_path(){
    wchar_t mod[MAX_PATH];
    GetModuleFileNameW(NULL,mod,MAX_PATH);
    std::wstring s(mod);
    size_t p=s.find_last_of(L'.');
    if(p!=std::wstring::npos) s=s.substr(0,p);
    s+=L".ini";
    return s;
}

static void enumerate_joysticks(HWND cb);
static void UpdateUIFromGlobals();

static void ApplyChanges(UINT code, UINT id, HWND hCtl);

static void load_settings_from_path(const std::wstring& path){

    for (int i = 0; i < 16; i++) {
        if (g_sitl_out_inv_chk[i]) {
            G.invsim_ch[i] = (SendMessageW(g_sitl_out_inv_chk[i], BM_GETCHECK, 0, 0) == BST_CHECKED);
        }
    }

    {
        int log_en = GetPrivateProfileIntW(L"bridge", L"logging_enabled", 0, path.c_str());
        g_logging_enabled.store(log_en != 0);
    }

    wchar_t wbuf[256];

    if(GetPrivateProfileStringW(L"bridge",L"ip",L"127.0.0.1",wbuf,256,path.c_str())>0){
        char t[256]; WideCharToMultiByte(CP_UTF8,0,wbuf,-1,t,256,NULL,NULL); G.dest.ip = t;
    }

    G.dest.port_tx = (uint16_t)GetPrivateProfileIntW(L"bridge",L"port_tx",9003,path.c_str());
    G.dest.port_rx = (uint16_t)GetPrivateProfileIntW(L"bridge",L"port_rx",9002,path.c_str());

    G.rate_hz      = GetPrivateProfileIntW(L"bridge",L"rate",G.rate_hz,path.c_str());
    G.match_sim_rate = GetPrivateProfileIntW(L"bridge",L"match_sim_rate", G.match_sim_rate?1:0, path.c_str())!=0;
    {
        wchar_t wres[64];

        if(GetPrivateProfileStringW(L"bridge",L"resample",L"off",wres,64,path.c_str())>0){
            if(!_wcsicmp(wres,L"off")) G.resample_mode=0; else if(!_wcsicmp(wres,L"zoh")) G.resample_mode=1; else if(!_wcsicmp(wres,L"linear")) G.resample_mode=2;
        }
    }

    G.use_time_sync = GetPrivateProfileIntW(L"bridge", L"use_time_sync", G.use_time_sync?1:0, path.c_str()) != 0;
    G.no_lockstep = GetPrivateProfileIntW(L"bridge", L"no_lockstep", G.no_lockstep?1:0, path.c_str()) != 0;
    G.json_pos_mode = GetPrivateProfileIntW(L"bridge", L"pos_mode", G.json_pos_mode, path.c_str());

    G.joy_index    = GetPrivateProfileIntW(L"bridge",L"joy_index",G.joy_index,path.c_str());

    {
        std::lock_guard<std::mutex> lk(G.m_tx);
        G.win_x   = GetPrivateProfileIntW(L"bridge",L"win_x",CW_USEDEFAULT,path.c_str());
        G.win_y   = GetPrivateProfileIntW(L"bridge",L"win_y",CW_USEDEFAULT,path.c_str());
        G.win_w   = GetPrivateProfileIntW(L"bridge", L"win_w", G.win_w, path.c_str());
        G.win_h   = GetPrivateProfileIntW(L"bridge", L"win_h", G.win_h, path.c_str());
    }

    for (int i = 0; i < 16; i++) {
        wchar_t key_inv[64];
        swprintf(key_inv, 64, L"invert_sim_ch%d", i + 1);
        G.invsim_ch[i] = GetPrivateProfileIntW(L"bridge", key_inv, G.invsim_ch[i] ? 1 : 0, path.c_str()) != 0;
    }

    {

        wchar_t sim_key[64];
        for(int i=0;i<16;i++){
            if (i == 0) wcscpy(sim_key, L"sim_roll_event");
            else if (i == 1) wcscpy(sim_key, L"sim_pitch_event");
            else if (i == 2) wcscpy(sim_key, L"sim_yaw_event");
            else if (i == 3) wcscpy(sim_key, L"sim_thr_event");
            else swprintf(sim_key, 64, L"sim_aux%d_event", i - 3);

            GetPrivateProfileStringW(L"bridge", sim_key, L"", wbuf, 256, path.c_str());
            G_sim_evt_idx[i] = find_evt_idx_by_name(wbuf);
        }
    }

    G.joy_map[0] = { 1, +1, 0 };
    G.joy_map[1] = { 2, +1, 0 };
    G.joy_map[2] = { 4, +1, 0 };
    G.joy_map[3] = { 0, +1, 0 };
    G.joy_map[4] = { 0, +1, 0 };
    G.joy_map[5] = { 0, +1, 0 };
    G.joy_map[6] = { 3, +1, 0 };
    G.joy_map[7] = { 0, +1, 0 };
    G.joy_map[8] = { 0, +1, 0 };
    G.joy_map[9] = { 0, +1, 0 };
    G.joy_map[10] = { 0, +1, 0 };
    G.joy_map[11] = { 0, +1, 0 };

    for(int i=0; i < NUM_JOY_AXES; i++) {
        wchar_t key_dst[64], key_inv[64], key_ovr[64];
        swprintf(key_dst, 64, L"joy_axis_%d_rc_dest", i + 1);
        swprintf(key_inv, 64, L"joy_axis_%d_reverse", i + 1);
        swprintf(key_ovr, 64, L"joy_axis_%d_override", i + 1);

        G.joy_map[i].rcDest = iclamp((int)GetPrivateProfileIntW(L"bridge", key_dst, G.joy_map[i].rcDest, path.c_str()), 0, NUM_RC_DESTS - 1);

        G.joy_map[i].srcInv = GetPrivateProfileIntW(L"bridge", key_inv, (G.joy_map[i].srcInv == -1 ? 1 : 0), path.c_str()) ? -1 : +1;
        G.joy_map[i].overrideMode = iclamp((int)GetPrivateProfileIntW(L"bridge", key_ovr, G.joy_map[i].overrideMode, path.c_str()), 0, 3);
    }

    {
        wchar_t wtmp[64]; char ctmp[64];
        if(GetPrivateProfileStringW(L"bridge",L"origin_lat",L"-35.363261",wtmp,64,path.c_str())>0) {
            WideCharToMultiByte(CP_UTF8,0,wtmp,-1,ctmp,64,NULL,NULL); G.sim_origin_lat = atof(ctmp);
        }
        if(GetPrivateProfileStringW(L"bridge",L"origin_lon",L"149.165230",wtmp,64,path.c_str())>0) {
            WideCharToMultiByte(CP_UTF8,0,wtmp,-1,ctmp,64,NULL,NULL); G.sim_origin_lon = atof(ctmp);
        }
        if(GetPrivateProfileStringW(L"bridge",L"origin_alt_m",L"584.0",wtmp,64,path.c_str())>0) {
            WideCharToMultiByte(CP_UTF8,0,wtmp,-1,ctmp,64,NULL,NULL); G.sim_origin_alt_m = atof(ctmp);
        }

        if(GetPrivateProfileStringW(L"bridge",L"earth_radius",L"6378137.0",wtmp,64,path.c_str())>0) {
            WideCharToMultiByte(CP_UTF8,0,wtmp,-1,ctmp,64,NULL,NULL); G.sim_earth_radius = atof(ctmp);
        }
    }

}

static void save_settings_to_path(const std::wstring& path){

    for (int i = 0; i < 16; i++) {
        wchar_t key_inv[64], b[8];
        swprintf(key_inv, 64, L"invert_sim_ch%d", i + 1);
        wsprintfW(b, L"%d", G.invsim_ch[i] ? 1 : 0);
        WritePrivateProfileStringW(L"bridge", key_inv, b, path.c_str());
    }

    {
        wchar_t b_log[8];
        wsprintfW(b_log, L"%d", g_logging_enabled.load() ? 1 : 0);
        WritePrivateProfileStringW(L"bridge", L"logging_enabled", b_log, path.c_str());
    }

    wchar_t b[64];

    wchar_t wip[256]; MultiByteToWideChar(CP_UTF8,0,G.dest.ip.c_str(),-1,wip,256);
    WritePrivateProfileStringW(L"bridge",L"ip",wip,path.c_str());
    wsprintfW(b,L"%u",G.dest.port_tx); WritePrivateProfileStringW(L"bridge",L"port_tx",b,path.c_str());
    wsprintfW(b,L"%u",G.dest.port_rx); WritePrivateProfileStringW(L"bridge",L"port_rx",b,path.c_str());
    wsprintfW(b,L"%d",G.rate_hz); WritePrivateProfileStringW(L"bridge",L"rate",b,path.c_str());
    WritePrivateProfileStringW(L"bridge", L"match_sim_rate", (G.match_sim_rate?L"1":L"0"), path.c_str());
    {
        const wchar_t* mode = L"Off"; if(G.resample_mode==1) mode=L"Zoh"; else if(G.resample_mode==2) mode=L"Linear";
        WritePrivateProfileStringW(L"bridge", L"resample", mode, path.c_str());
    }

    wsprintfW(b, L"%d", G.use_time_sync ? 1 : 0);
    WritePrivateProfileStringW(L"bridge", L"use_time_sync", b, path.c_str());
    wsprintfW(b, L"%d", G.no_lockstep ? 1 : 0);
    WritePrivateProfileStringW(L"bridge", L"no_lockstep", b, path.c_str());
    wsprintfW(b, L"%d", G.json_pos_mode);
    WritePrivateProfileStringW(L"bridge", L"pos_mode", b, path.c_str());

    wsprintfW(b,L"%d",G.joy_index); WritePrivateProfileStringW(L"bridge",L"joy_index",b,path.c_str());

    {
        std::lock_guard<std::mutex> lk(G.m_tx);
        if (G.win_x != CW_USEDEFAULT) {
            wsprintfW(b,L"%d",G.win_x); WritePrivateProfileStringW(L"bridge",L"win_x",b,path.c_str());
        }
        if (G.win_y != CW_USEDEFAULT) {
            wsprintfW(b,L"%d",G.win_y); WritePrivateProfileStringW(L"bridge",L"win_y",b,path.c_str());
        }
        wsprintfW(b,L"%d",G.win_w); WritePrivateProfileStringW(L"bridge",L"win_w",b,path.c_str());
        wsprintfW(b,L"%d",G.win_h); WritePrivateProfileStringW(L"bridge",L"win_h",b,path.c_str());
    }

    wchar_t sim_key[64];
    for (int i = 0; i < 16; i++) {
        if (i == 0) wcscpy(sim_key, L"sim_roll_event");
        else if (i == 1) wcscpy(sim_key, L"sim_pitch_event");
        else if (i == 2) wcscpy(sim_key, L"sim_yaw_event");
        else if (i == 3) wcscpy(sim_key, L"sim_thr_event");
        else swprintf(sim_key, 64, L"sim_aux%d_event", i - 3);

        const char* evt_name_utf8 = get_sim_evt_by_idx(G_sim_evt_idx[i]);
        wchar_t evt_name_w[128] = {0};

        if (evt_name_utf8 && evt_name_utf8[0] != '\0') {
            MultiByteToWideChar(CP_UTF8, 0, evt_name_utf8, -1, evt_name_w, 128);
        }
        WritePrivateProfileStringW(L"bridge", sim_key, evt_name_w, path.c_str());
    }

    for(int i=0; i < NUM_JOY_AXES; i++) {
        wchar_t key_dst[64], key_inv[64], key_ovr[64];
        swprintf(key_dst, 64, L"joy_axis_%d_rc_dest", i + 1);
        swprintf(key_inv, 64, L"joy_axis_%d_reverse", i + 1);
        swprintf(key_ovr, 64, L"joy_axis_%d_override", i + 1);

        wsprintfW(b, L"%d", G.joy_map[i].rcDest);
        WritePrivateProfileStringW(L"bridge", key_dst, b, path.c_str());

        wsprintfW(b, L"%d", G.joy_map[i].srcInv == -1 ? 1 : 0);
        WritePrivateProfileStringW(L"bridge", key_inv, b, path.c_str());

        wsprintfW(b, L"%d", G.joy_map[i].overrideMode);
        WritePrivateProfileStringW(L"bridge", key_ovr, b, path.c_str());
    }
}

static void DoFileLoad(HWND h) {

    wchar_t szFile[MAX_PATH];
    wcsncpy(szFile, g_ini_path.c_str(), MAX_PATH);

    OPENFILENAMEW ofn = {0};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = h;
    ofn.lpstrFile = szFile;
    ofn.nMaxFile = sizeof(szFile) / sizeof(wchar_t);
    ofn.lpstrFilter = L"INI Files (*.ini)\0*.ini\0All Files (*.*)\0*.*\0";
    ofn.nFilterIndex = 1;
    ofn.lpstrFileTitle = NULL;
    ofn.nMaxFileTitle = 0;
    ofn.lpstrInitialDir = NULL;
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

    if (GetOpenFileNameW(&ofn) == TRUE) {
        load_settings_from_path(ofn.lpstrFile);
        enumerate_joysticks(g_joycb);
        { int sel=(int)SendMessageW(g_joycb, CB_GETCURSEL, 0, 0); wchar_t name[256]=L""; if(sel>=0) SendMessageW(g_joycb, CB_GETLBTEXT, sel, (LPARAM)name); if(name[0]==0) lstrcpyW(name, L"Select joystick"); wchar_t cap[300]; lstrcpy(cap, name); }
        UpdateUIFromGlobals();

    }
}

static void DoFileSaveAs(HWND h) {

    wchar_t szFile[MAX_PATH];
    wcsncpy(szFile, g_ini_path.c_str(), MAX_PATH);

    OPENFILENAMEW ofn = {0};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = h;
    ofn.lpstrFile = szFile;
    ofn.nMaxFile = sizeof(szFile) / sizeof(wchar_t);
    ofn.lpstrFilter = L"INI Files (*.ini)\0*.ini\0All Files (*.*)\0*.*\0";
    ofn.nFilterIndex = 1;
    ofn.lpstrDefExt = L"ini";
    ofn.lpstrFileTitle = NULL;
    ofn.nMaxFileTitle = 0;
    ofn.lpstrInitialDir = NULL;
    ofn.Flags = OFN_OVERWRITEPROMPT;

    if (GetSaveFileNameW(&ofn) == TRUE) {
        save_settings_to_path(ofn.lpstrFile);
    }
}

struct EnumContext {
    HWND cb;
    int selected_index;
};

static BOOL CALLBACK DIEnumDevicesCallback(LPCDIDEVICEINSTANCE lpddi, LPVOID pvRef) {
    EnumContext* ctx = (EnumContext*)pvRef;

    int idx = (int)SendMessageW(ctx->cb, CB_ADDSTRING, 0, (LPARAM)lpddi->tszProductName);

    g_joystickGUIDs.push_back(lpddi->guidInstance);

    int vector_index = (int)g_joystickGUIDs.size() - 1;
    SendMessageW(ctx->cb, CB_SETITEMDATA, idx, (LPARAM)vector_index);

    if (vector_index == ctx->selected_index) {
        SendMessageW(ctx->cb, CB_SETCURSEL, idx, 0);
    }

    return DIENUM_CONTINUE;
}

static void enumerate_joysticks(HWND cb){
    SendMessageW(cb, CB_RESETCONTENT, 0, 0);
    g_joystickGUIDs.clear();

    if (!g_pDI) {

        if (FAILED(DirectInput8Create(GetModuleHandle(NULL), DIRECTINPUT_VERSION, IID_IDirectInput8, (VOID**)&g_pDI, NULL))) {
            return;
        }
    }

    EnumContext ctx;
    ctx.cb = cb;
    ctx.selected_index = G.joy_index;

    g_pDI->EnumDevices(DI8DEVCLASS_GAMECTRL, DIEnumDevicesCallback, &ctx, DIEDFL_ATTACHEDONLY);

    if (SendMessageW(cb, CB_GETCURSEL, 0, 0) == CB_ERR && g_joystickGUIDs.size() > 0) {
        SendMessageW(cb, CB_SETCURSEL, 0, 0);
        G.joy_index = 0;
    }
}

static BOOL CALLBACK DIEnumDeviceObjectsCallback(LPCDIDEVICEOBJECTINSTANCE lpddoi, LPVOID pvRef) {
    if (!g_pJoy) return DIENUM_STOP;

    if (lpddoi->dwType & DIDFT_AXIS) {
        DIPROPRANGE diprg;
        diprg.diph.dwSize = sizeof(DIPROPRANGE);
        diprg.diph.dwHeaderSize = sizeof(DIPROPHEADER);
        diprg.diph.dwHow = DIPH_BYID;
        diprg.diph.dwObj = lpddoi->dwType;
        diprg.lMin = -1000;
        diprg.lMax = +1000;

        if (FAILED(g_pJoy->SetProperty(DIPROP_RANGE, &diprg.diph))) {
        }

        DIPROPDWORD dipdw;
        dipdw.diph.dwSize = sizeof(DIPROPDWORD);
        dipdw.diph.dwHeaderSize = sizeof(DIPROPHEADER);
        dipdw.diph.dwHow = DIPH_BYID;
        dipdw.diph.dwObj = lpddoi->dwType;
        dipdw.dwData = (DWORD)(G.deadzone * 10000.0);

        g_pJoy->SetProperty(DIPROP_DEADZONE, &dipdw.diph);
    }

    return DIENUM_CONTINUE;
}

static bool select_joystick(int index) {

    if (index < 0 || index >= (int)g_joystickGUIDs.size()) {
        return false;
    }

    if (g_pJoy) {
        g_pJoy->Unacquire();
        g_pJoy->Release();
        g_pJoy = NULL;
    }

    if (!g_pDI) {
        return false;
    }

    GUID joy_guid = g_joystickGUIDs[index];

    if (FAILED(g_pDI->CreateDevice(joy_guid, &g_pJoy, NULL))) {
        return false;
    }

    if (FAILED(g_pJoy->SetDataFormat(&c_dfDIJoystick2))) {
        g_pJoy->Release(); g_pJoy = NULL;
        return false;
    }

    if (FAILED(g_pJoy->SetCooperativeLevel(g_hwnd, DISCL_BACKGROUND | DISCL_NONEXCLUSIVE))) {
        g_pJoy->Release(); g_pJoy = NULL;
        return false;
    }

    g_pJoy->EnumObjects(DIEnumDeviceObjectsCallback, (LPVOID)g_hwnd, DIDFT_AXIS);

    g_pJoy->Acquire();

    g_selectedJoyIndex = index;
    return true;
}

static void UpdateLayout(HWND h){
    RECT rc; GetClientRect(h,&rc);
    const int pad=S(10), rowh=S(24), gap=S(6);
    int y = pad;
    int x = pad;
    int w = rc.right - 2*pad;
    if (w < S(700)) w = S(700);

    if(g_stat) MoveWindow(g_stat, x, y, w, rowh, TRUE);
    y += rowh + gap;

    const int grp_conn_h = S(90);
    if (g_grp_conn) MoveWindow(g_grp_conn, x, y, w, grp_conn_h, TRUE);

    int y_grp = y + S(20);
    int x_stat = x + pad;
    const int led_w = S(16);
    const int led_h = S(16);
    int stat_w = (w - 2 * pad - 2 * gap) / 3;
    if (stat_w < S(150)) stat_w = S(150);
    int lbl_w = stat_w - led_w - gap;

    if (g_led_sim) MoveWindow(g_led_sim, x_stat, y_grp, led_w, led_h, TRUE);
    if (g_lbl_sim_status) MoveWindow(g_lbl_sim_status, x_stat + led_w + gap, y_grp, lbl_w, rowh, TRUE);
    x_stat += stat_w + gap;

    if (g_led_tx) MoveWindow(g_led_tx, x_stat, y_grp, led_w, led_h, TRUE);
    if (g_lbl_tx_status) MoveWindow(g_lbl_tx_status, x_stat + led_w + gap, y_grp, lbl_w, rowh, TRUE);
    x_stat += stat_w + gap;

    if (g_led_rx) MoveWindow(g_led_rx, x_stat, y_grp, led_w, led_h, TRUE);
    if (g_lbl_rx_status) MoveWindow(g_lbl_rx_status, x_stat + led_w + gap, y_grp, lbl_w, rowh, TRUE);

    y_grp += rowh + gap;
    int x_net = x + pad;

    const int lbl_w_ip = S(25);
    const int ip_w = S(100);
    const int port_w = S(45);
    const int rate_w = S(40);
    const int lbl_w_port_rx = S(120);
    const int lbl_w_port_tx = S(120);

    if(GetDlgItem(h,ID_LBL_IP)) MoveWindow(GetDlgItem(h,ID_LBL_IP), x_net, y_grp+S(4), lbl_w_ip, rowh, TRUE);
    x_net += lbl_w_ip + gap;
    if(g_ip) MoveWindow(g_ip, x_net, y_grp, ip_w, rowh, TRUE);
    x_net += ip_w + S(4);

    if(GetDlgItem(h,ID_LBL_RX)) MoveWindow(GetDlgItem(h,ID_LBL_RX), x_net, y_grp+S(4), lbl_w_port_rx, rowh, TRUE);
    x_net += lbl_w_port_rx + gap;
    if(g_rx) MoveWindow(g_rx, x_net, y_grp, port_w, rowh, TRUE);
    x_net += port_w + S(4);

    if(GetDlgItem(h,ID_LBL_TX)) MoveWindow(GetDlgItem(h,ID_LBL_TX), x_net, y_grp+S(4), lbl_w_port_tx, rowh, TRUE);
    x_net += lbl_w_port_tx + gap;
    if(g_tx) MoveWindow(g_tx, x_net, y_grp, port_w, rowh, TRUE);
    x_net += port_w + S(4);

    if(GetDlgItem(h,ID_LBL_HZ)) MoveWindow(GetDlgItem(h,ID_LBL_HZ), x_net, y_grp+S(4), S(25), rowh, TRUE);
    x_net += S(25) + gap;
    if(g_rate) MoveWindow(g_rate, x_net, y_grp, rate_w, rowh, TRUE);
    x_net += rate_w + gap;
    if(g_match_sim) MoveWindow(g_match_sim, x_net, y_grp, S(80), rowh, TRUE);
    x_net += S(80) + gap;

    if(GetDlgItem(h,ID_LBL_RESAMP)) MoveWindow(GetDlgItem(h,ID_LBL_RESAMP), x_net, y_grp+S(4), S(60), rowh, TRUE);
    x_net += S(60) + gap;
    if(g_resample_cb){ MoveWindow(g_resample_cb, x_net, y_grp, S(80), S(200), TRUE); SetWindowPos(g_resample_cb, HWND_TOP, 0,0,0,0, SWP_NOMOVE|SWP_NOSIZE); }
    x_net += S(80) + gap;

    const int pos_lbl_w = S(70);
    const int pos_cb_w = S(80);
    if(GetDlgItem(h,ID_LBL_POS_FMT)) MoveWindow(GetDlgItem(h,ID_LBL_POS_FMT), x_net, y_grp+S(4), pos_lbl_w, rowh, TRUE);
    x_net += pos_lbl_w + gap;
    if(g_pos_fmt_cb){ MoveWindow(g_pos_fmt_cb, x_net, y_grp, pos_cb_w, S(200), TRUE); SetWindowPos(g_pos_fmt_cb, HWND_TOP, 0,0,0,0, SWP_NOMOVE|SWP_NOSIZE); }
    x_net += pos_cb_w + gap;

    const int tsync_lbl_w = S(85);
    const int tsync_cb_w = S(25);
    if(GetDlgItem(h,ID_LBL_TSYNC)) MoveWindow(GetDlgItem(h,ID_LBL_TSYNC), x_net, y_grp+S(4), tsync_lbl_w, rowh, TRUE);
    x_net += tsync_lbl_w + gap;
    if(g_time_sync_cb) MoveWindow(g_time_sync_cb, x_net, y_grp, tsync_cb_w, rowh, TRUE);
    x_net += tsync_cb_w;

    const int lock_lbl_w = S(75);
    const int lock_cb_w = S(25);
    x_net += gap * 2;
    if(GetDlgItem(h,ID_LBL_LOCKSTEP)) MoveWindow(GetDlgItem(h,ID_LBL_LOCKSTEP), x_net, y_grp+S(4), lock_lbl_w, rowh, TRUE);
    x_net += lock_lbl_w + gap;
    if(g_no_lockstep_cb) MoveWindow(g_no_lockstep_cb, x_net, y_grp, lock_cb_w, rowh, TRUE);

    y += grp_conn_h + gap;

    const int grp_joy_h = S(55);
    if(g_grp_joy) MoveWindow(g_grp_joy, x, y, w, grp_joy_h, TRUE);
    y_grp = y + S(20);
    const int joy_lbl_w = S(70);
    if(g_lblJoy) MoveWindow(g_lblJoy, x + pad, y_grp+S(4), joy_lbl_w, rowh, TRUE);
    const int joy_btn_w = S(100);
    const int joy_btn_gap = S(10);
    int joy_cb_w = w - (joy_lbl_w + 2*pad + gap + joy_btn_w + joy_btn_gap);
    if (joy_cb_w > S(360)) joy_cb_w = S(360);
    if (joy_cb_w < S(160)) joy_cb_w = S(160);
    if(g_joycb){ MoveWindow(g_joycb, x + pad + joy_lbl_w + gap, y_grp, joy_cb_w, S(200), TRUE); SetWindowPos(g_joycb, HWND_TOP, 0,0,0,0, SWP_NOMOVE|SWP_NOSIZE); }
    if(g_btnJoyCal) MoveWindow(g_btnJoyCal, x + pad + joy_lbl_w + gap + joy_cb_w + joy_btn_gap, y_grp, joy_btn_w, rowh, TRUE);
    SetWindowPos(g_btnJoyCal, HWND_TOP, 0,0,0,0, SWP_NOMOVE|SWP_NOSIZE);
    y += grp_joy_h + gap;

    const int h_sim_evt_rows = S(24) * 4 + gap * 3;
    const int h_sitl_out_rows = (S(18) + S(4)) * 8;
    const int grp_sitl_out_h = S(20) + h_sim_evt_rows + S(10) + S(24) + S(4) + h_sitl_out_rows + S(10);

    if(g_grp_sitl_out) MoveWindow(g_grp_sitl_out, x, y, w, grp_sitl_out_h, TRUE);
    y_grp = y + S(20);

    {
        const int colw = (w - 2*pad) / 4;
        const int lbl_w_evt = S(90);
        const int cb_w_evt = colw - lbl_w_evt - gap;
        for(int i=0; i<16; i++){
            int col = i % 4;
            int row = i / 4;
            int x_evt = x + pad + col * colw;
            int y_evt = y_grp + row * (S(24) + gap);

            HWND lbl = GetDlgItem(h, 5100+i);
            if(lbl) MoveWindow(lbl, x_evt, y_evt+S(4), lbl_w_evt, rowh, TRUE);
            if(g_sitl_evt_cb[i]) MoveWindow(g_sitl_evt_cb[i], x_evt + lbl_w_evt + gap, y_evt, cb_w_evt, S(200), TRUE);
        }
    }
    y_grp += h_sim_evt_rows + S(10);

    if(g_lblSitlOut) MoveWindow(g_lblSitlOut, x + pad, y_grp, S(200), rowh, TRUE);

    const int column_gap = S(20);
    const int col_lbl_w = S(60);
    const int val_w = S(40);
    const int chk_w = S(35);
    const int col_pb_w  = (w - 2*pad - 2*(col_lbl_w + gap) - 2*chk_w - 2*val_w - column_gap - 4*gap) / 2;
    const int col_w     = col_lbl_w + gap + col_pb_w + gap + val_w + gap + chk_w;
    const int x_col0    = x + pad;
    const int x_col1    = x + pad + col_w + column_gap;

    if (GetDlgItem(h, IDC_SITL_OUT_REV_LBL1)) MoveWindow(GetDlgItem(h, IDC_SITL_OUT_REV_LBL1), x_col0 + col_lbl_w + gap + col_pb_w + gap + val_w + gap, y_grp, chk_w, rowh, TRUE);
    if (GetDlgItem(h, IDC_SITL_OUT_REV_LBL2)) MoveWindow(GetDlgItem(h, IDC_SITL_OUT_REV_LBL2), x_col1 + col_lbl_w + gap + col_pb_w + gap + val_w + gap, y_grp, chk_w, rowh, TRUE);

    y_grp += rowh + S(4);


    const int pb_h = S(18);

    for (int i = 0; i < 8; i++) {
        wchar_t lbl_text[16];

        int ch_idx_0 = i;
        int id_lbl_0 = IDC_SITL_OUT_LBL_BASE + ch_idx_0;
        int id_pb_0 = IDC_SITL_OUT_PB_BASE + ch_idx_0;
        int id_chk_0 = IDC_INVS_CH_BASE + ch_idx_0;
        int id_val_0 = IDC_SITL_OUT_VAL_BASE + ch_idx_0;
        swprintf(lbl_text, 16, L"Servo %d", ch_idx_0 + 1);

        if (!g_sitl_out_lbl[ch_idx_0]) {
            g_sitl_out_lbl[ch_idx_0] = CreateWindowExW(0,L"STATIC",lbl_text,WS_CHILD|WS_VISIBLE|SS_LEFT,
            0,0,0,0, h,(HMENU)(INT_PTR)id_lbl_0, GetModuleHandle(NULL),NULL);
            SendMessageW(g_sitl_out_lbl[ch_idx_0], WM_SETFONT, (WPARAM)g_uiFont, TRUE);
        }
        if (!g_sitl_out_pb[ch_idx_0]) {
            g_sitl_out_pb[ch_idx_0] = CreateWindowExW(0,PROGRESS_CLASS,L"",WS_CHILD|WS_VISIBLE|PBS_SMOOTH,
            0,0,0,0, h,(HMENU)(INT_PTR)id_pb_0, GetModuleHandle(NULL),NULL);
            SendMessageW(g_sitl_out_pb[ch_idx_0], PBM_SETRANGE, 0, MAKELPARAM(0, 100));
            SendMessageW(g_sitl_out_pb[ch_idx_0], PBM_SETBARCOLOR, 0, (LPARAM)RGB(200, 0, 0));
        }
        if (!g_sitl_out_val[ch_idx_0]) {
            g_sitl_out_val[ch_idx_0] = CreateWindowExW(0, L"STATIC", L"0%", WS_CHILD | WS_VISIBLE | SS_RIGHT,
            0, 0, 0, 0, h, (HMENU)(INT_PTR)id_val_0, GetModuleHandle(NULL), NULL);
            SendMessageW(g_sitl_out_val[ch_idx_0], WM_SETFONT, (WPARAM)g_uiFont, TRUE);
        }
        if (!g_sitl_out_inv_chk[ch_idx_0]) {
            g_sitl_out_inv_chk[ch_idx_0] = CreateWindowExW(0,L"BUTTON",L"",WS_CHILD|WS_VISIBLE|BS_AUTOCHECKBOX,
            0,0,0,0,h,(HMENU)(INT_PTR)id_chk_0, GetModuleHandle(NULL),NULL);
            SendMessageW(g_sitl_out_inv_chk[ch_idx_0], WM_SETFONT, (WPARAM)g_uiFont, TRUE);
            SendMessageW(g_sitl_out_inv_chk[ch_idx_0], BM_SETCHECK, G.invsim_ch[ch_idx_0]?BST_CHECKED:BST_UNCHECKED, 0);
        }
        MoveWindow(g_sitl_out_lbl[ch_idx_0], x_col0, y_grp + S(1), col_lbl_w, rowh, TRUE);
        MoveWindow(g_sitl_out_pb[ch_idx_0],  x_col0 + col_lbl_w + gap, y_grp, col_pb_w, pb_h, TRUE);
        MoveWindow(g_sitl_out_val[ch_idx_0], x_col0 + col_lbl_w + gap + col_pb_w + gap, y_grp + S(1), val_w, rowh, TRUE);
        MoveWindow(g_sitl_out_inv_chk[ch_idx_0], x_col0 + col_lbl_w + gap + col_pb_w + gap + val_w + gap, y_grp, chk_w, rowh, TRUE);

        int ch_idx_1 = i + 8;
        int id_lbl_1 = IDC_SITL_OUT_LBL_BASE + ch_idx_1;
        int id_pb_1 = IDC_SITL_OUT_PB_BASE + ch_idx_1;
        int id_chk_1 = IDC_INVS_CH_BASE + ch_idx_1;
        int id_val_1 = IDC_SITL_OUT_VAL_BASE + ch_idx_1;
        swprintf(lbl_text, 16, L"Servo %d", ch_idx_1 + 1);

        if (!g_sitl_out_lbl[ch_idx_1]) {
            g_sitl_out_lbl[ch_idx_1] = CreateWindowExW(0,L"STATIC",lbl_text,WS_CHILD|WS_VISIBLE|SS_LEFT,
            0,0,0,0, h,(HMENU)(INT_PTR)id_lbl_1, GetModuleHandle(NULL),NULL);
            SendMessageW(g_sitl_out_lbl[ch_idx_1], WM_SETFONT, (WPARAM)g_uiFont, TRUE);
        }
        if (!g_sitl_out_pb[ch_idx_1]) {
            g_sitl_out_pb[ch_idx_1] = CreateWindowExW(0,PROGRESS_CLASS,L"",WS_CHILD|WS_VISIBLE|PBS_SMOOTH,
            0,0,0,0, h,(HMENU)(INT_PTR)id_pb_1, GetModuleHandle(NULL),NULL);
            SendMessageW(g_sitl_out_pb[ch_idx_1], PBM_SETRANGE, 0, MAKELPARAM(0, 100));
            SendMessageW(g_sitl_out_pb[ch_idx_1], PBM_SETBARCOLOR, 0, (LPARAM)RGB(200, 0, 0));
        }
        if (!g_sitl_out_val[ch_idx_1]) {
            g_sitl_out_val[ch_idx_1] = CreateWindowExW(0, L"STATIC", L"0%", WS_CHILD | WS_VISIBLE | SS_RIGHT,
            0, 0, 0, 0, h, (HMENU)(INT_PTR)id_val_1, GetModuleHandle(NULL), NULL);
            SendMessageW(g_sitl_out_val[ch_idx_1], WM_SETFONT, (WPARAM)g_uiFont, TRUE);
        }
        if (!g_sitl_out_inv_chk[ch_idx_1]) {
            g_sitl_out_inv_chk[ch_idx_1] = CreateWindowExW(0,L"BUTTON",L"",WS_CHILD|WS_VISIBLE|BS_AUTOCHECKBOX,
            0,0,0,0,h,(HMENU)(INT_PTR)id_chk_1, GetModuleHandle(NULL),NULL);
            SendMessageW(g_sitl_out_inv_chk[ch_idx_1], WM_SETFONT, (WPARAM)g_uiFont, TRUE);
            SendMessageW(g_sitl_out_inv_chk[ch_idx_1], BM_SETCHECK, G.invsim_ch[ch_idx_1]?BST_CHECKED:BST_UNCHECKED, 0);
        }
        MoveWindow(g_sitl_out_lbl[ch_idx_1], x_col1, y_grp + S(1), col_lbl_w, rowh, TRUE);
        MoveWindow(g_sitl_out_pb[ch_idx_1],  x_col1 + col_lbl_w + gap, y_grp, col_pb_w, pb_h, TRUE);
        MoveWindow(g_sitl_out_val[ch_idx_1], x_col1 + col_lbl_w + gap + col_pb_w + gap, y_grp + S(1), val_w, rowh, TRUE);
        MoveWindow(g_sitl_out_inv_chk[ch_idx_1], x_col1 + col_lbl_w + gap + col_pb_w + gap + val_w + gap, y_grp, chk_w, rowh, TRUE);

        y_grp += pb_h + S(4);
    }
    y += grp_sitl_out_h + S(4);

    int h_map_hdr = rowh;
    int h_map_rows = NUM_JOY_AXES * (rowh + S(2));
    const int grp_di_in_h = h_map_hdr + h_map_rows + S(20);

    int di_w = S(500);
    int hud_w = w - di_w - gap;
    int hud_h = grp_di_in_h;
    if (hud_w < S(150)) {
        di_w = w;
        hud_w = w;
        hud_h = S(150);
        if(g_grp_di_in) MoveWindow(g_grp_di_in, x, y, di_w, grp_di_in_h, TRUE);
        y += grp_di_in_h + gap;
        if(g_hud) MoveWindow(g_hud, x, y, hud_w, hud_h, TRUE);
    } else {
        if(g_grp_di_in) MoveWindow(g_grp_di_in, x, y, di_w, grp_di_in_h, TRUE);
        if(g_hud) MoveWindow(g_hud, x + di_w + gap, y, hud_w, hud_h, TRUE);
    }

    y_grp = y + S(20);
    int x_grp = x + pad;

    const int col_ax_lbl_w = S(110);
    const int col_ax_pb_w = S(110);
    const int col_val_w = S(50);
    const int col_dst_w  = S(80);
    const int col_inv_w  = S(30);
    const int col_ovr_w  = S(70);
    int x_map = x_grp;

    if(g_lblMapping)   MoveWindow(g_lblMapping, x_map, y_grp, col_ax_lbl_w, rowh, TRUE);
    x_map += col_ax_lbl_w + gap;
    if(g_lblLivePreview) MoveWindow(g_lblLivePreview, x_map, y_grp, col_ax_pb_w + gap + col_val_w, rowh, TRUE);
    x_map += col_ax_pb_w + gap + col_val_w + gap;
    if(g_lblDest)      MoveWindow(g_lblDest,    x_map, y_grp, col_dst_w, rowh, TRUE);
    x_map += col_dst_w + gap;
    if(g_lblReverseIn) MoveWindow(g_lblReverseIn, x_map, y_grp, col_inv_w, rowh, TRUE);
    x_map += col_inv_w + gap;
    if(g_lblOverride)  MoveWindow(g_lblOverride, x_map, y_grp, col_ovr_w, rowh, TRUE);
    y_grp += rowh;

    for(int i=0; i < NUM_JOY_AXES; i++) {
        int yy = y_grp;
        int x_map = x_grp;

        if(g_map_lbl[i])     MoveWindow(g_map_lbl[i],     x_map, yy+S(4), col_ax_lbl_w, rowh, TRUE);
        x_map += col_ax_lbl_w + gap;

        if(g_ax_pb[i])       MoveWindow(g_ax_pb[i],       x_map, yy, col_ax_pb_w, pb_h, TRUE);
        x_map += col_ax_pb_w + gap;

        if(g_ax_val[i])      MoveWindow(g_ax_val[i],      x_map, yy+S(1), col_val_w, rowh, TRUE);
        x_map += col_val_w + gap;

        if(g_map_dst_cb[i])  MoveWindow(g_map_dst_cb[i],  x_map, yy, col_dst_w, S(200), TRUE);
        x_map += col_dst_w + gap;

        if(g_map_src_inv[i]) MoveWindow(g_map_src_inv[i], x_map, yy, col_inv_w, rowh, TRUE);
        x_map += col_inv_w + gap;

        if(g_map_ovr_cb[i])  MoveWindow(g_map_ovr_cb[i],  x_map, yy, col_ovr_w, S(200), TRUE);

        y_grp += rowh + S(2);
    }
}

static void UpdateUIFromGlobals() {
    if (g_joycb) ShowWindow(g_joycb, SW_SHOW);
    if (g_resample_cb) ShowWindow(g_resample_cb, SW_SHOW);
    if (g_pos_fmt_cb) ShowWindow(g_pos_fmt_cb, SW_SHOW);

    if (g_joycb) {
        int count = (int)SendMessageW(g_joycb, CB_GETCOUNT, 0, 0);
        if (G.joy_index>=0 && G.joy_index<count) SendMessageW(g_joycb, CB_SETCURSEL, (WPARAM)G.joy_index, 0);
    }

    wchar_t b[256];

    MultiByteToWideChar(CP_UTF8,0,G.dest.ip.c_str(),-1,b,256);
    SetWindowTextW(g_ip, b);
    wsprintfW(b,L"%u",G.dest.port_tx); SetWindowTextW(g_tx,b);
    wsprintfW(b,L"%u",G.dest.port_rx); SetWindowTextW(g_rx,b);
    wsprintfW(b,L"%d",G.rate_hz); SetWindowTextW(g_rate,b);
    if(g_resample_cb){ SendMessageW(g_resample_cb, CB_SETCURSEL, (WPARAM)G.resample_mode, 0); }
    if(g_pos_fmt_cb){ SendMessageW(g_pos_fmt_cb, CB_SETCURSEL, (WPARAM)G.json_pos_mode, 0); }

    bool time_sync_enabled = true;
    if(g_time_sync_cb) {
        SendMessageW(g_time_sync_cb, BM_SETCHECK, G.use_time_sync?BST_CHECKED:BST_UNCHECKED, 0);
        EnableWindow(g_time_sync_cb, time_sync_enabled?TRUE:FALSE);
        if(GetDlgItem(g_hwnd,ID_LBL_TSYNC)) EnableWindow(GetDlgItem(g_hwnd,ID_LBL_TSYNC), time_sync_enabled?TRUE:FALSE);
    }

    if(g_no_lockstep_cb) {
        SendMessageW(g_no_lockstep_cb, BM_SETCHECK, G.no_lockstep?BST_CHECKED:BST_UNCHECKED, 0);
    }

    if(g_match_sim) SendMessageW(g_match_sim, BM_SETCHECK, G.match_sim_rate?BST_CHECKED:BST_UNCHECKED, 0);
    EnableWindow(g_resample_cb, G.match_sim_rate?FALSE:TRUE);
    EnableWindow(g_rate, G.match_sim_rate?FALSE:TRUE);
    SendMessageW(g_match_sim, BM_SETCHECK, G.match_sim_rate?BST_CHECKED:BST_UNCHECKED, 0);
    EnableWindow(g_rate, G.match_sim_rate?FALSE:TRUE);

    for(int i=0; i<16; i++) {
        if(g_sitl_out_inv_chk[i]) {
            SendMessageW(g_sitl_out_inv_chk[i], BM_SETCHECK, G.invsim_ch[i]?BST_CHECKED:BST_UNCHECKED, 0);
        }
    }

    for(int i=0; i<16; i++) {
        if(g_sitl_evt_cb[i]) {
            SendMessageW(g_sitl_evt_cb[i], CB_SETCURSEL, (WPARAM)G_sim_evt_idx[i], 0);
        }
    }

    for(int i=0;i < NUM_JOY_AXES; i++){
        SendMessageW(g_map_dst_cb[i], CB_SETCURSEL, G.joy_map[i].rcDest, 0);
        SendMessageW(g_map_src_inv[i], BM_SETCHECK, (G.joy_map[i].srcInv == -1) ? BST_CHECKED : BST_UNCHECKED, 0);
        SendMessageW(g_map_ovr_cb[i], CB_SETCURSEL, iclamp(G.joy_map[i].overrideMode, 0, 3), 0);
    }

    int sel_idx = -1;
    for(int i = 0; i < (int)SendMessageW(g_joycb, CB_GETCOUNT, 0, 0); ++i) {

        if((int)SendMessageW(g_joycb, CB_GETITEMDATA, i, 0) == G.joy_index) {
            sel_idx = i;
            break;
        }
    }
    SendMessageW(g_joycb, CB_SETCURSEL, sel_idx, 0);

    if(sel_idx == -1 && g_joystickGUIDs.size() > 0) {
        SendMessageW(g_joycb, CB_SETCURSEL, 0, 0);
        G.joy_index = (int)SendMessageW(g_joycb, CB_GETITEMDATA, 0, 0);
    }
}

static void ApplyChanges(UINT code, UINT id, HWND hCtl) {

    if (code == EN_CHANGE && (hCtl == g_ip || hCtl == g_tx || hCtl == g_rx || hCtl == g_rate)) {
        std::lock_guard<std::mutex> lk(G.m_tx);
        wchar_t b[256];

        if (hCtl == g_ip) {
            GetWindowTextW(g_ip,b,256);
            char ip[256]; WideCharToMultiByte(CP_UTF8,0,b,-1,ip,256,NULL,NULL);
            G.dest.ip = ip;
        } else if (hCtl == g_tx) {
            GetWindowTextW(g_tx,b,256); G.dest.port_tx = (uint16_t)_wtoi(b);
        } else if (hCtl == g_rx) {
            GetWindowTextW(g_rx,b,256); G.dest.port_rx = (uint16_t)_wtoi(b);
        } else if (hCtl == g_rate) {
            GetWindowTextW(g_rate,b,256); G.rate_hz = iclamp(_wtoi(b),1,1000);
        }
    }

    else if (code == CBN_SELCHANGE) {

        if (hCtl == g_joycb) {
            std::lock_guard<std::mutex> lk(G.m_tx);
            int sel = (int)SendMessageW(g_joycb, CB_GETCURSEL, 0, 0);

            if (sel >= 0) {
                G.joy_index = (int)SendMessageW(g_joycb, CB_GETITEMDATA, sel, 0);
            }
        } else if (hCtl == g_resample_cb) {
            std::lock_guard<std::mutex> lk(G.m_tx);
            G.resample_mode = (int)SendMessageW(g_resample_cb, CB_GETCURSEL, 0, 0);
        } else if (hCtl == g_pos_fmt_cb) {
            std::lock_guard<std::mutex> lk(G.m_tx);
            G.json_pos_mode = (int)SendMessageW(g_pos_fmt_cb, CB_GETCURSEL, 0, 0);
        }

        else {
            bool found = false;
            std::lock_guard<std::mutex> lk(G.m_tx);
            for (int i = 0; i < NUM_JOY_AXES; i++) {

                if (hCtl == g_map_dst_cb[i]) {
                    G.joy_map[i].rcDest = (int)SendMessageW(g_map_dst_cb[i], CB_GETCURSEL, 0, 0);
                    found = true; break;
                }

                if (hCtl == g_map_ovr_cb[i]) {
                    G.joy_map[i].overrideMode = iclamp((int)SendMessageW(g_map_ovr_cb[i], CB_GETCURSEL, 0, 0), 0, 3);
                    found = true; break;
                }
            }

            if (!found && id >= IDC_SIMMAP_CB_BASE && id < IDC_SIMMAP_CB_BASE + 16) {
                int idx = id - IDC_SIMMAP_CB_BASE;
                int sel = (int)SendMessageW(hCtl, CB_GETCURSEL, 0, 0);

                if (sel >= 0) {
                    G_sim_evt_idx[idx] = sel;

                    if (gSim) {
                        SimConnect_MapClientEventToSimEvent(gSim, g_sim_evt_map[idx], get_sim_evt_by_idx(sel));
                    }
                }
            }
        }
    }

    else if (code == BN_CLICKED) {

        if (hCtl == g_match_sim) {
            std::lock_guard<std::mutex> lk(G.m_tx);
            G.match_sim_rate = (SendMessageW(g_match_sim, BM_GETCHECK,0,0)==BST_CHECKED);
            EnableWindow(g_rate, G.match_sim_rate?FALSE:TRUE);
            EnableWindow(g_resample_cb, G.match_sim_rate?FALSE:TRUE);
        }

        else if (id >= IDC_INVS_CH_BASE && id < IDC_INVS_CH_BASE + 16) {
            std::lock_guard<std::mutex> lk(G.m_tx);
            bool chk = (SendMessageW(hCtl, BM_GETCHECK,0,0)==BST_CHECKED);
            int ch_idx = id - IDC_INVS_CH_BASE;
            G.invsim_ch[ch_idx] = chk;
        }

        else if (id == IDC_TIME_SYNC_CB) {
            std::lock_guard<std::mutex> lk(G.m_tx);
            G.use_time_sync = (SendMessageW(hCtl, BM_GETCHECK, 0, 0) == BST_CHECKED);
        }

        else if (id == IDC_NO_LOCKSTEP_CB) {
            std::lock_guard<std::mutex> lk(G.m_tx);
            G.no_lockstep = (SendMessageW(hCtl, BM_GETCHECK, 0, 0) == BST_CHECKED);
        }

        else if (id >= IDC_MAP_SRC_INV && id < IDC_MAP_SRC_INV + NUM_JOY_AXES) {
            std::lock_guard<std::mutex> lk(G.m_tx);
            int idx = id - IDC_MAP_SRC_INV;
            G.joy_map[idx].srcInv = (SendMessageW(hCtl, BM_GETCHECK,0,0)==BST_CHECKED) ? -1 : +1;
        }
    }
}

static LRESULT CALLBACK WndProc(HWND h, UINT m, WPARAM w, LPARAM l){

    switch(m){
        case WM_GETMINMAXINFO: {
            LPMINMAXINFO pMMI = (LPMINMAXINFO)l;

            RECT r = {0,0, S(1240), S(950)};

            AdjustWindowRectEx(&r, WS_OVERLAPPEDWINDOW, FALSE, 0);

            int minW = r.right - r.left;
            int minH = r.bottom - r.top;

            if (pMMI) {
                pMMI->ptMinTrackSize.x = minW;
                pMMI->ptMinTrackSize.y = minH;
            }
            return 0;
        }

        case WM_CREATE:{
            g_hwnd = h;
            g_dpi = Dpi(h);
            CreateFonts(h);

            HMENU hMenuBar = CreateMenu();
            HMENU hFile = CreateMenu();
            HMENU hView = CreateMenu();
            HMENU hHelp = CreateMenu();

            AppendMenuW(hFile, MF_STRING, IDM_FILE_LOAD, L"&Load profile...\tCtrl+O");
            AppendMenuW(hFile, MF_STRING, IDM_FILE_SAVE, L"&Save Profile\tCtrl+S");
            AppendMenuW(hFile, MF_STRING, IDM_FILE_SAVEAS, L"Save Profile &As...");
            AppendMenuW(hFile, MF_SEPARATOR, 0, NULL);
            AppendMenuW(hFile, MF_STRING, IDM_FILE_EXIT, L"E&xit\tAlt+F4");

            AppendMenuW(hView, MF_STRING, IDM_VIEW_SIMCONNECT, L"&SimConnect Live Sensor\tCtrl+D");

            AppendMenuW(hHelp, MF_STRING | (g_logging_enabled.load()?MF_CHECKED:MF_UNCHECKED), IDM_HELP_LOGGING, L"&Enable logging");
            AppendMenuW(hHelp, MF_STRING, IDM_HELP_ABOUT, L"&About...");

            AppendMenuW(hMenuBar, MF_POPUP, (UINT_PTR)hFile, L"&File");
            AppendMenuW(hMenuBar, MF_POPUP, (UINT_PTR)hView, L"&View");
            AppendMenuW(hMenuBar, MF_POPUP, (UINT_PTR)hHelp, L"&Help");

            SetMenu(h, hMenuBar);
            if (g_logging_enabled.load()) { OpenLogFile(); CheckMenuItem(GetMenu(h), IDM_HELP_LOGGING, MF_BYCOMMAND | MF_CHECKED); }

            INITCOMMONCONTROLSEX icc{sizeof(icc), ICC_BAR_CLASSES|ICC_PROGRESS_CLASS|ICC_STANDARD_CLASSES|ICC_WIN95_CLASSES|ICC_LISTVIEW_CLASSES};
            InitCommonControlsEx(&icc);
            g_stat = CreateWindowExW(0,L"STATIC",APP_TITLE_W,
            WS_CHILD|WS_VISIBLE|SS_LEFT|SS_ENDELLIPSIS, S(10), S(10), S(800), S(24),h,(HMENU)(INT_PTR)IDC_STAT,GetModuleHandle(NULL),NULL);

            g_grp_conn = CreateWindowExW(0, L"BUTTON", L" SITL Connection and Status ", WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                S(10), S(40), S(700), S(90), h, (HMENU)IDC_GRP_CONN, GetModuleHandle(NULL), NULL);

            g_led_sim = CreateWindowExW(WS_EX_CLIENTEDGE, L"STATIC", L"", WS_CHILD | WS_VISIBLE,
                S(20), S(60), S(16), S(16), h, (HMENU)IDC_STATUS_LED_SIM, GetModuleHandle(NULL), NULL);
            g_lbl_sim_status = CreateWindowExW(0, L"STATIC", L"SimConnect: ---", WS_CHILD | WS_VISIBLE | SS_LEFT | SS_ENDELLIPSIS,
                S(42), S(60), S(200), S(24), h, (HMENU)IDC_STATUS_LBL_SIM, GetModuleHandle(NULL), NULL);
            g_led_tx = CreateWindowExW(WS_EX_CLIENTEDGE, L"STATIC", L"", WS_CHILD | WS_VISIBLE,
                S(250), S(60), S(16), S(16), h, (HMENU)IDC_STATUS_LED_TX, GetModuleHandle(NULL), NULL);
            g_lbl_tx_status = CreateWindowExW(0, L"STATIC", L"Sensors TX: ---", WS_CHILD | WS_VISIBLE | SS_LEFT | SS_ENDELLIPSIS,
                S(272), S(60), S(200), S(24), h, (HMENU)IDC_STATUS_LBL_TX, GetModuleHandle(NULL), NULL);
            g_led_rx = CreateWindowExW(WS_EX_CLIENTEDGE, L"STATIC", L"", WS_CHILD | WS_VISIBLE,
                S(480), S(60), S(16), S(16), h, (HMENU)IDC_STATUS_LED_RX, GetModuleHandle(NULL), NULL);
            g_lbl_rx_status = CreateWindowExW(0, L"STATIC", L"Servo RX: ---", WS_CHILD | WS_VISIBLE | SS_LEFT | SS_ENDELLIPSIS,
                S(502), S(60), S(200), S(24), h, (HMENU)IDC_STATUS_LBL_RX, GetModuleHandle(NULL), NULL);
            SetLedColor(g_led_sim, false);
            SetLedColor(g_led_tx, false);
            SetLedColor(g_led_rx, false);

            wchar_t wip[256]; MultiByteToWideChar(CP_UTF8,0,G.dest.ip.c_str(),-1,wip,256);
            CreateWindowExW(0,L"STATIC",L"IP:",WS_CHILD|WS_VISIBLE|SS_LEFT, S(20), S(125), S(30), S(24),h,(HMENU)(INT_PTR)ID_LBL_IP,GetModuleHandle(NULL),NULL);
            g_ip   = CreateWindowExW(WS_EX_CLIENTEDGE,L"EDIT",wip,WS_CHILD|WS_VISIBLE|ES_LEFT, S(55), S(125), S(120), S(24),h,(HMENU)(INT_PTR)IDC_IP,GetModuleHandle(NULL),NULL);

            CreateWindowExW(0,L"STATIC",L"SITL Servo Port (RX):",WS_CHILD|WS_VISIBLE|SS_LEFT, S(185), S(125), S(110), S(24),h,(HMENU)(INT_PTR)ID_LBL_RX,GetModuleHandle(NULL),NULL);
            g_rx   = CreateWindowExW(WS_EX_CLIENTEDGE,L"EDIT",L"9002",WS_CHILD|WS_VISIBLE|ES_LEFT, S(300), S(125), S(60), S(24),h,(HMENU)(INT_PTR)IDC_RX,GetModuleHandle(NULL),NULL);

            CreateWindowExW(0,L"STATIC",L"SITL Sensor Port (TX):",WS_CHILD|WS_VISIBLE|SS_LEFT, S(370), S(125), S(110), S(24),h,(HMENU)(INT_PTR)ID_LBL_TX,GetModuleHandle(NULL),NULL);
            g_tx   = CreateWindowExW(WS_EX_CLIENTEDGE,L"EDIT",L"9003",WS_CHILD|WS_VISIBLE|ES_LEFT, S(485), S(125), S(60), S(24),h,(HMENU)(INT_PTR)IDC_TX,GetModuleHandle(NULL),NULL);

            CreateWindowExW(0,L"STATIC",L"Hz:",WS_CHILD|WS_VISIBLE|SS_LEFT, S(555), S(125), S(30), S(24),h,(HMENU)(INT_PTR)ID_LBL_HZ,GetModuleHandle(NULL),NULL);
            g_rate = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"50", WS_CHILD|WS_VISIBLE|ES_LEFT, S(590), S(125), S(40), S(24), h, (HMENU)(INT_PTR)IDC_RATE, GetModuleHandle(NULL), NULL);
            g_match_sim = CreateWindowExW(0,L"BUTTON",L"= Sim fps",WS_CHILD|WS_VISIBLE|BS_AUTOCHECKBOX,
            S(640), S(125), S(90), S(24), h,(HMENU)(INT_PTR)IDC_MATCH_SIM,GetModuleHandle(NULL),NULL);

            CreateWindowExW(0,L"STATIC",L"Resample:",WS_CHILD|WS_VISIBLE,
            S(555), S(147), S(80), S(24), h,(HMENU)(INT_PTR)ID_LBL_RESAMP,GetModuleHandle(NULL),NULL);
            g_resample_cb = CreateWindowExW(0,L"COMBOBOX",L"",WS_CHILD|WS_VISIBLE|CBS_DROPDOWNLIST|WS_VSCROLL,
            S(640), S(145), S(90), S(200), h,(HMENU)(INT_PTR)IDC_RESAMPLE,GetModuleHandle(NULL),
            NULL);
            SendMessageW(g_resample_cb, CB_ADDSTRING, 0, (LPARAM)L"Off");
            SendMessageW(g_resample_cb, CB_ADDSTRING, 0, (LPARAM)L"Zoh");
            SendMessageW(g_resample_cb, CB_ADDSTRING, 0, (LPARAM)L"Linear");
            SendMessageW(g_resample_cb, CB_SETCURSEL, (WPARAM)G.resample_mode, 0);

            CreateWindowExW(0,L"STATIC",L"Pos. Format:",WS_CHILD|WS_VISIBLE,
            S(740), S(147), S(70), S(24), h,(HMENU)(INT_PTR)ID_LBL_POS_FMT,GetModuleHandle(NULL),NULL);
            g_pos_fmt_cb = CreateWindowExW(0,L"COMBOBOX",L"",WS_CHILD|WS_VISIBLE|CBS_DROPDOWNLIST|WS_VSCROLL,
            S(815), S(145), S(80), S(200), h,(HMENU)(INT_PTR)IDC_POS_FMT_CB,GetModuleHandle(NULL),NULL);
            SendMessageW(g_pos_fmt_cb, CB_ADDSTRING, 0, (LPARAM)L"MP SITL");
            SendMessageW(g_pos_fmt_cb, CB_ADDSTRING, 0, (LPARAM)L"Position");
            SendMessageW(g_pos_fmt_cb, CB_ADDSTRING, 0, (LPARAM)L"LLA");
            SendMessageW(g_pos_fmt_cb, CB_SETCURSEL, (WPARAM)G.json_pos_mode, 0);

            CreateWindowExW(0,L"STATIC",L"Use Time Sync:",WS_CHILD|WS_VISIBLE,
            S(900), S(147), S(90), S(24), h,(HMENU)(INT_PTR)ID_LBL_TSYNC,GetModuleHandle(NULL),NULL);
            g_time_sync_cb = CreateWindowExW(0,L"BUTTON",L"",WS_CHILD|WS_VISIBLE|BS_AUTOCHECKBOX,
            S(990), S(145), S(20), S(24), h,(HMENU)(INT_PTR)IDC_TIME_SYNC_CB,GetModuleHandle(NULL),
            NULL);
            SendMessageW(g_time_sync_cb, BM_SETCHECK, G.use_time_sync?BST_CHECKED:BST_UNCHECKED, 0);

            CreateWindowExW(0,L"STATIC",L"No Lockstep:",WS_CHILD|WS_VISIBLE,
            S(1020), S(147), S(80), S(24), h,(HMENU)(INT_PTR)ID_LBL_LOCKSTEP,GetModuleHandle(NULL),NULL);

            g_no_lockstep_cb = CreateWindowExW(0,L"BUTTON",L"",WS_CHILD|WS_VISIBLE|BS_AUTOCHECKBOX,
            S(1105), S(145), S(20), S(24), h,(HMENU)(INT_PTR)IDC_NO_LOCKSTEP_CB,GetModuleHandle(NULL), NULL);
            SendMessageW(g_no_lockstep_cb, BM_SETCHECK, G.no_lockstep?BST_CHECKED:BST_UNCHECKED, 0);

            g_grp_joy = CreateWindowExW(0, L"BUTTON", L" Joystick ", WS_CHILD|WS_VISIBLE|BS_GROUPBOX,
            S(10), S(170), S(700), S(55), h, (HMENU)IDC_GRP_JOY, GetModuleHandle(NULL), NULL);

            g_lblJoy = CreateWindowExW(0,L"STATIC",L"Joystick:",WS_CHILD|WS_VISIBLE|SS_LEFT, S(20), S(190), S(70), S(24),h,(HMENU)(INT_PTR)IDC_JOYLBL,GetModuleHandle(NULL),NULL);
            g_joycb = CreateWindowExW(0,L"COMBOBOX",L"",WS_CHILD|WS_VISIBLE|WS_TABSTOP|CBS_DROPDOWNLIST|WS_VSCROLL,
            S(120), S(185), S(300), S(200), h,(HMENU)(INT_PTR)IDC_JOYCB,GetModuleHandle(NULL),NULL);

            ShowWindow(g_joycb, SW_SHOW);
            g_btnJoyCal = CreateWindowExW(0, L"BUTTON", L"Calibrate...", WS_CHILD|WS_VISIBLE|WS_CLIPSIBLINGS, S(560), S(185), S(180), S(24), h, (HMENU)(INT_PTR)IDC_JOYCAL_BTN, GetModuleHandle(NULL), NULL);

            SendMessageW(g_btnJoyCal, WM_SETFONT, (WPARAM)g_uiFont, TRUE);
            ShowWindow(g_btnJoyCal, SW_SHOW);

            g_grp_sitl_out = CreateWindowExW(0, L"BUTTON", L" SITL -> MSFS (Outputs) ", WS_CHILD|WS_VISIBLE|BS_GROUPBOX,
            S(10), S(235), S(700), S(380), h, (HMENU)IDC_GRP_SITL_OUT, GetModuleHandle(NULL), NULL);

{
                const wchar_t* names[16] = {
                    L"Axis 1",L"Axis 2",L"Axis 3",L"Axis 4",
                    L"Aux 1",L"Aux 2",L"Aux 3",L"Aux 4",
                    L"Aux 5",L"Aux 6",L"Aux 7",L"Aux 8",
                    L"Aux 9",L"Aux 10",L"Aux 11",L"Aux 12"
                };
                int x0 = S(20), y0 = S(255);
                int colw = (S(700) - S(40)) / 4;
                int rowh_evt = S(24) + S(6);
                for(int i=0;i<16;i++){
                    int col = i % 4;
                    int row = i / 4;
                    int x = x0 + col * colw;
                    int y = y0 + row * rowh_evt;

                    wchar_t lbl_name[32];
                    if (i < 4) swprintf(lbl_name, 32, L"%s:", names[i]);
                    else swprintf(lbl_name, 32, L"S%d (Aux%d):", i + 1, i - 3);

                    HWND lbl = CreateWindowExW(0,L"STATIC",lbl_name,WS_CHILD|WS_VISIBLE|SS_LEFT,
                    x, y, S(90), S(20), h,(HMENU)(INT_PTR)(5100+i),GetModuleHandle(NULL),NULL);
                    SendMessageW(lbl, WM_SETFONT, (WPARAM)g_uiFont, TRUE);

                    g_sitl_evt_cb[i] = CreateWindowExW(0,L"COMBOBOX",L"",WS_CHILD|WS_VISIBLE|CBS_DROPDOWNLIST|WS_VSCROLL,
                    x+S(92), y-S(2), colw - S(100), S(200), h,(HMENU)(INT_PTR)(IDC_SIMMAP_CB_BASE+i),GetModuleHandle(NULL),NULL);
                    SendMessageW(g_sitl_evt_cb[i], WM_SETFONT, (WPARAM)g_uiFont, TRUE);
                    int nopt = (int)(sizeof(kEvtOpts)/sizeof(kEvtOpts[0]));
                    for(int k=0;k<nopt;k++) SendMessageW(g_sitl_evt_cb[i], CB_ADDSTRING, 0, (LPARAM)kEvtOpts[k].label);
                    SendMessageW(g_sitl_evt_cb[i], CB_SETCURSEL, (WPARAM)G_sim_evt_idx[i], 0);
                    SendMessageW(g_sitl_evt_cb[i], CB_SETDROPPEDWIDTH, (WPARAM)S(300), 0);
                }
            }
            int y_sitl_out = S(255) + (S(24) + S(6)) * 4 + S(10);

            g_lblSitlOut = CreateWindowExW(0,L"STATIC",L"SITL Output (live):",
            WS_CHILD|WS_VISIBLE|SS_LEFT|SS_ENDELLIPSIS, S(20), y_sitl_out, S(200), S(24),
            h,(HMENU)(INT_PTR)3009,GetModuleHandle(NULL),NULL);
            SendMessageW(g_lblSitlOut, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);

            CreateWindowExW(0, L"STATIC", L"Rev", WS_CHILD | WS_VISIBLE | SS_LEFT,
                0, 0, 0, 0, h, (HMENU)IDC_SITL_OUT_REV_LBL1, GetModuleHandle(NULL), NULL);
            CreateWindowExW(0, L"STATIC", L"Rev", WS_CHILD | WS_VISIBLE | SS_LEFT,
                0, 0, 0, 0, h, (HMENU)IDC_SITL_OUT_REV_LBL2, GetModuleHandle(NULL), NULL);


            for(int i=0; i<16; i++) {
                g_sitl_out_lbl[i] = NULL;
                g_sitl_out_pb[i] = NULL;
                g_sitl_out_inv_chk[i] = NULL;
                g_sitl_out_val[i] = NULL;
            }

            g_grp_di_in = CreateWindowExW(0, L"BUTTON", L" DirectInput -> SITL (Inputs) ", WS_CHILD|WS_VISIBLE|BS_GROUPBOX,
            S(10), S(625), S(700), S(300), h, (HMENU)IDC_GRP_DI_IN, GetModuleHandle(NULL), NULL);

            WNDCLASSEXW wc_hud{ sizeof(WNDCLASSEXW) };
            wc_hud.lpfnWndProc = HudWndProc;
            wc_hud.hInstance = GetModuleHandle(NULL);
            wc_hud.lpszClassName = kHudClass;
            wc_hud.hCursor = LoadCursor(NULL, IDC_ARROW);
            wc_hud.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
            RegisterClassExW(&wc_hud);

            g_hud = CreateWindowExW(WS_EX_CLIENTEDGE, kHudClass, L"HUD", WS_CHILD | WS_VISIBLE,
                S(520), S(625), S(200), S(300), h, (HMENU)IDC_HUD_DISPLAY, GetModuleHandle(NULL), NULL);
            SetTimer(g_hud, 3, 50, NULL);

            int baseY = S(645);

            g_lblMapping   = CreateWindowExW(0,L"STATIC",L"Joystick Axis:",WS_CHILD|WS_VISIBLE|SS_LEFT|SS_ENDELLIPSIS, S(20), baseY, S(110), S(24),h,(HMENU)(INT_PTR)3003,GetModuleHandle(NULL),NULL);
            SendMessageW(g_lblMapping, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);

            g_lblLivePreview = CreateWindowExW(0,L"STATIC",L"Live Preview",WS_CHILD|WS_VISIBLE|SS_LEFT|SS_ENDELLIPSIS, S(135), baseY, S(160), S(24),h,(HMENU)(INT_PTR)3004,GetModuleHandle(NULL),NULL);
            g_lblDest      = CreateWindowExW(0,L"STATIC",L"RC Dest",WS_CHILD|WS_VISIBLE|SS_LEFT|SS_ENDELLIPSIS, S(300), baseY, S(80), S(24),h,(HMENU)(INT_PTR)3006,GetModuleHandle(NULL),NULL);
            g_lblReverseIn = CreateWindowExW(0,L"STATIC",L"Rev",WS_CHILD|WS_VISIBLE|SS_LEFT|SS_ENDELLIPSIS, S(385), baseY, S(30), S(24),h,(HMENU)(INT_PTR)3005,GetModuleHandle(NULL),NULL);
            g_lblOverride = CreateWindowExW(0,L"STATIC",L"Override",WS_CHILD|WS_VISIBLE|SS_LEFT|SS_ENDELLIPSIS, S(420), baseY, S(70), S(24), h,(HMENU)(INT_PTR)IDC_MAP_OVR_LBL,GetModuleHandle(NULL),NULL);

            SendMessageW(g_lblLivePreview, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_lblDest, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_lblReverseIn, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_lblOverride, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(GetDlgItem(h, IDC_SITL_OUT_REV_LBL1), WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(GetDlgItem(h, IDC_SITL_OUT_REV_LBL2), WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);

            baseY += S(24);

            for(int i=0;i<NUM_JOY_AXES;i++){
                int yy = baseY + i*S(26);

                g_map_lbl[i] = CreateWindowExW(0,L"STATIC",AXIS_SRC_NAMES[i],WS_CHILD|WS_VISIBLE|SS_LEFT|SS_ENDELLIPSIS, S(20), yy+S(4), S(110), S(24),h,(HMENU)(INT_PTR)(IDC_MAP_LBL_BASE+i),GetModuleHandle(NULL),NULL);

                g_ax_pb[i]  = CreateWindowExW(0,PROGRESS_CLASS,L"",WS_CHILD|WS_VISIBLE|PBS_SMOOTH, S(135), yy, S(110), S(18), h,(HMENU)(INT_PTR)(IDC_AXPB_BASE+i),GetModuleHandle(NULL),NULL);
                SendMessageW(g_ax_pb[i], PBM_SETRANGE, 0, MAKELPARAM(0, 100));
                SendMessageW(g_ax_pb[i], PBM_SETPOS, 50, 0);
                SendMessageW(g_ax_pb[i], PBM_SETBARCOLOR, 0, (LPARAM)RGB(0, 180, 0));

                g_ax_val[i] = CreateWindowExW(0, L"STATIC", L"0%", WS_CHILD | WS_VISIBLE | SS_RIGHT,
                    S(250), yy + S(1), S(50), S(24), h, (HMENU)(INT_PTR)(IDC_AX_VAL_BASE + i), GetModuleHandle(NULL), NULL);
                SendMessageW(g_ax_val[i], WM_SETFONT, (WPARAM)g_uiFont, TRUE);

                g_map_dst_cb[i] = CreateWindowExW(0,L"COMBOBOX",L"",WS_CHILD|WS_VISIBLE|CBS_DROPDOWNLIST|WS_VSCROLL, S(300), yy, S(80), S(200),h,(HMENU)(INT_PTR)(IDC_MAP_DST_CB+i),GetModuleHandle(NULL),NULL);
                SendMessageW(g_map_dst_cb[i], CB_RESETCONTENT, 0, 0);
                for(int n=0; n < NUM_RC_DESTS; n++) {
                    SendMessageW(g_map_dst_cb[i], CB_ADDSTRING, 0, (LPARAM)RC_DEST_NAMES[n]);
                }

                g_map_src_inv[i] = CreateWindowExW(0,L"BUTTON",L"",WS_CHILD|WS_VISIBLE|BS_AUTOCHECKBOX, S(385), yy, S(30), S(24),h,(HMENU)(INT_PTR)(IDC_MAP_SRC_INV+i),GetModuleHandle(NULL),NULL);

                g_map_ovr_cb[i] = CreateWindowExW(0,L"COMBOBOX",L"",WS_CHILD|WS_VISIBLE|CBS_DROPDOWNLIST|WS_VSCROLL,
                S(420), yy, S(70), S(200), h,(HMENU)(INT_PTR)(IDC_MAP_OVR_CB+i),GetModuleHandle(NULL),NULL);
                SendMessageW(g_map_ovr_cb[i], CB_RESETCONTENT, 0, 0);
                SendMessageW(g_map_ovr_cb[i], CB_ADDSTRING, 0, (LPARAM)L"Off");
                SendMessageW(g_map_ovr_cb[i], CB_ADDSTRING, 0, (LPARAM)L"Min");
                SendMessageW(g_map_ovr_cb[i], CB_ADDSTRING, 0, (LPARAM)L"Center");
                SendMessageW(g_map_ovr_cb[i], CB_ADDSTRING, 0, (LPARAM)L"Max");
            }

            ApplyUIFont(h);

            SendMessageW(g_lblSitlOut, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_grp_conn, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_grp_joy, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_grp_sitl_out, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_grp_di_in, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_grp_status, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_lblSitlOut, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_lblMapping, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_lblLivePreview, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_lblDest, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_lblReverseIn, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_lblOverride, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(GetDlgItem(h, IDC_SITL_OUT_REV_LBL1), WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(GetDlgItem(h, IDC_SITL_OUT_REV_LBL2), WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);

            enumerate_joysticks(g_joycb);
            { int sel=(int)SendMessageW(g_joycb, CB_GETCURSEL, 0, 0); wchar_t name[256]=L""; if(sel>=0) SendMessageW(g_joycb, CB_GETLBTEXT, sel, (LPARAM)name); if(name[0]==0) lstrcpyW(name, L"Select joystick"); wchar_t cap[300]; lstrcpyW(cap, name); }
            UpdateUIFromGlobals();

            SetTimer(h,1,50,NULL);

            return 0;
        }

        case WM_APP_STATUSTEXT:{
            wchar_t* p = reinterpret_cast<wchar_t*>(l);
            if (g_stat && p) SetWindowTextW(g_stat, p);
            if (p) free(p);
            return 0;
        }
        case WM_APP_SIM_STATUS: {
            bool ok = (w == 1);

            double rate = *reinterpret_cast<double*>(l);

            G.status_sim_ok.store(ok);
            G.status_sim_rate.store(rate);
            SetLedColor(g_led_sim, ok);
            wchar_t buf[128];
            if (ok) swprintf(buf, 128, L"SimConnect: OK (%.0f Hz)", rate);
            else swprintf(buf, 128, L"SimConnect: KO");
            SetWindowTextW(g_lbl_sim_status, buf);
            return 0;
        }
        case WM_APP_TX_STATUS: {
            bool ok = (w == 1);

            double rate = *reinterpret_cast<double*>(l);

            G.status_tx_ok.store(ok);
            G.status_tx_rate.store(rate);
            SetLedColor(g_led_tx, ok);
            wchar_t buf[128];
            if (ok) swprintf(buf, 128, L"Sensors TX: OK (%.0f Hz)", rate);
            else swprintf(buf, 128, L"Sensors TX: ---");
            SetWindowTextW(g_lbl_tx_status, buf);
            return 0;
        }
        case WM_APP_RX_STATUS: {
            bool ok = (w == 1);

            double rate = *reinterpret_cast<double*>(l);

            G.status_rx_ok.store(ok);
            G.status_rx_rate.store(rate);
            SetLedColor(g_led_rx, ok);
            wchar_t buf[128];
            if (ok) swprintf(buf, 128, L"Servo RX: OK (%.0f Hz)", rate);
            else swprintf(buf, 128, L"Servo RX: ---");
            SetWindowTextW(g_lbl_rx_status, buf);
            return 0;
        }


        case WM_SIZE:
        UpdateLayout(h);
        return 0;

        case WM_DPICHANGED: {
            g_dpi = HIWORD(w);
            CreateFonts(h);
            ApplyUIFont(h);

            SendMessageW(g_lblSitlOut, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_grp_conn, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_grp_joy, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_grp_sitl_out, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_grp_di_in, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_grp_status, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_lblSitlOut, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_lblMapping, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_lblLivePreview, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_lblDest, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_lblReverseIn, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(g_lblOverride, WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(GetDlgItem(h, IDC_SITL_OUT_REV_LBL1), WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);
            SendMessageW(GetDlgItem(h, IDC_SITL_OUT_REV_LBL2), WM_SETFONT, (WPARAM)g_uiFontBold, TRUE);

            RECT* prc = (RECT*)l;
            if (prc) SetWindowPos(h, NULL, prc->left, prc->top, prc->right-prc->left, prc->bottom-prc->top, SWP_NOZORDER|SWP_NOACTIVATE);
            UpdateLayout(h);
            return 0;
        }

        case WM_ERASEBKGND: {
            HDC hdc = (HDC)w;
            RECT rc; GetClientRect(h, &rc);
            FillRect(hdc, &rc, (HBRUSH)(COLOR_WINDOW+1));
            return 1;
        }

        case WM_HSCROLL: {

            return 0;
        }

        case WM_TIMER: {
            if (w == 1) {

                double raw_ax[NUM_JOY_AXES];
                {
                    std::lock_guard<std::mutex> lk(G.m_gui);
                    for(int i=0;i<NUM_JOY_AXES;i++) raw_ax[i]=G.raw_axes[i];
                }
                for(int i=0;i<NUM_JOY_AXES;i++){
                    double val_0_100 = (raw_ax[i] * 0.5 + 0.5) * 100.0;
                    double val_pm_100 = raw_ax[i] * 100.0;
                    if (g_ax_pb[i]) {
                        SendMessageW(g_ax_pb[i], PBM_SETPOS, (WPARAM)(int)llround(val_0_100), 0);
                    }
                    if (g_ax_val[i]) {
                        wchar_t buf[16];
                        swprintf(buf, 16, L"%+4.0f%%", val_pm_100);
                        SetWindowTextW(g_ax_val[i], buf);
                    }
                }

                double s_pwm[16];
                bool inv_ch[16];
                {
                    std::lock_guard<std::mutex> lk(G.m_gui);
                    for(int i=0; i<16; i++) s_pwm[i] = G.sitl_out_pwm[i];
                }
                {
                    std::lock_guard<std::mutex> lk(G.m_tx);
                    for(int i=0; i<16; i++) inv_ch[i] = G.invsim_ch[i];
                }

                for(int i=0; i<16; i++) {
                    if (g_sitl_out_pb[i]) {
                        double v = s_pwm[i];

                        double pb_val_0_1;
                        if (i == 0 || i == 1 || i == 3) {
                            pb_val_0_1 = (v * 0.5) + 0.5;
                        } else {
                            pb_val_0_1 = v;
                        }
                        SendMessageW(g_sitl_out_pb[i], PBM_SETPOS, (WPARAM)(int)llround(pb_val_0_1 * 100.0), 0);

                        double val_pm_100;
                        if (i == 0 || i == 1 || i == 3) {
                            val_pm_100 = v * 100.0;
                        } else {
                            val_pm_100 = (v * 200.0) - 100.0;
                        }

                        if (g_sitl_out_val[i]) {
                            wchar_t buf[16];
                            swprintf(buf, 16, L"%+4.0f%%", val_pm_100);
                            SetWindowTextW(g_sitl_out_val[i], buf);
                        }
                    }
                }
            }
            return 0;
        }

case WM_COMMAND:
        {
            UINT code = HIWORD(w); UINT id = LOWORD(w);

            ApplyChanges(code, id, (HWND)l);

if (HIWORD(w) == BN_CLICKED) {

                switch (LOWORD(w)) {
                    case IDC_JOYCAL_BTN: { ShellExecuteW(NULL, L"open", L"control.exe", L"joy.cpl", NULL, SW_SHOWNORMAL); return 0; }

                    case IDC_MATCH_SIM:
                    {

                        BOOL chk = (SendMessageW(g_match_sim, BM_GETCHECK,0,0)==BST_CHECKED);
                        EnableWindow(g_rate, chk?FALSE:TRUE);
                        EnableWindow(g_resample_cb, chk?FALSE:TRUE);
                        return 0;
                    }
                }
            }
        }

        switch (LOWORD(w)) {
            case IDM_FILE_LOAD:
            DoFileLoad(h);
            return 0;

            case IDM_FILE_SAVE:
            {
                RECT rc;
                if (!IsIconic(h) && GetWindowRect(h, &rc) && rc.left > -10000 && rc.top > -10000) {

                    std::lock_guard<std::mutex> lk(G.m_tx);
                    G.win_x = rc.left;
                    G.win_y = rc.top;
                    int dpi = Dpi(h);
                    G.win_w = MulDiv(rc.right - rc.left, 96, dpi);
                    G.win_h = MulDiv(rc.bottom - rc.top, 96, dpi);
                }

                save_ini();
            }
            return 0;

            case IDM_FILE_SAVEAS:
            DoFileSaveAs(h);
            return 0;

            case IDM_FILE_EXIT:
            PostMessageW(h, WM_CLOSE, 0, 0);
            return 0;

            case IDM_VIEW_SIMCONNECT:
            ShowSimDbgPopup(h);
            return 0;

            case IDM_HELP_ABOUT:
            MessageBoxW(h, L"MSFS 202x <-> ArduPilot SITL Bridge v1.0.0\nAuthor: Marco Robustini (aka Marcopter)", L"About", MB_OK | MB_ICONINFORMATION);
            return 0;
            case IDM_HELP_LOGGING:
            {

                if (g_logging_enabled.load()) {
                    g_logging_enabled.store(false);
                    CloseLogFile();
                    CheckMenuItem(GetMenu(h), IDM_HELP_LOGGING, MF_BYCOMMAND | MF_UNCHECKED);
                    OutputDebugStringW(L"Logging disabled\r\n");
                } else {
                    g_logging_enabled.store(true);
                    OpenLogFile();
                    CheckMenuItem(GetMenu(h), IDM_HELP_LOGGING, MF_BYCOMMAND | MF_CHECKED);
                    OutputDebugStringW(L"Logging enabled\r\n");
                }
                return 0;
            }

        }

        return 0;

        case WM_PAINT:{
            PAINTSTRUCT ps;
            BeginPaint(h,&ps);
            SetLedColor(g_led_sim, G.status_sim_ok.load());
            SetLedColor(g_led_tx, G.status_tx_ok.load());
            SetLedColor(g_led_rx, G.status_rx_ok.load());
            EndPaint(h,&ps);
            return 0;
        }

        case WM_CLOSE: {

            if (g_simDbgPopup) {
                SendMessageW(g_simDbgPopup, WM_CLOSE, 0, 0);
            }
            if (g_hud) {
                KillTimer(g_hud, 3);
            }
            DestroyWindow(h);
            return 0;
        }

        case WM_DESTROY:
        g_logging_enabled.store(false);
        CloseLogFile();
        if (g_uiFont) DeleteObject(g_uiFont);
        if (g_uiFontBold) DeleteObject(g_uiFontBold);
        if (g_hudFont) DeleteObject(g_hudFont);
        PostQuitMessage(0);

        if ((HWND)l == g_joycb || (HWND)l == g_resample_cb || (HWND)l == g_pos_fmt_cb) {

            if (HIWORD(w) == CBN_DROPDOWN) {
                SetWindowPos((HWND)l, HWND_TOPMOST, 0,0,0,0, SWP_NOMOVE|SWP_NOSIZE);
                return 0;
            } else if (HIWORD(w) == CBN_CLOSEUP) {
                SetWindowPos((HWND)l, HWND_NOTOPMOST, 0,0,0,0, SWP_NOMOVE|SWP_NOSIZE);
                return 0;
            }
        }
        return 0;
    }
    return DefWindowProcW(h,m,w,l);
}

static void PostStatus(const wchar_t* fmt, ...){
    wchar_t buf[1024];
    va_list args;
    va_start(args, fmt);
    vswprintf(buf, _countof(buf), fmt, args);
    va_end(args);

    wchar_t* p = (wchar_t*)malloc( (wcslen(buf)+1)*sizeof(wchar_t) );

    if (p) {
        wcscpy(p, buf);
        PostMessageW(g_hwnd, WM_APP_STATUSTEXT, 0, (LPARAM)p);
    }
}

static void PostSimStatus(bool ok, double rate) {
    double* pRate = (double*)malloc(sizeof(double));
    if (pRate) {
        *pRate = rate;
        PostMessageW(g_hwnd, WM_APP_SIM_STATUS, (WPARAM)ok, (LPARAM)pRate);
    }
}
static void PostTxStatus(bool ok, double rate) {
    double* pRate = (double*)malloc(sizeof(double));
    if (pRate) {
        *pRate = rate;
        PostMessageW(g_hwnd, WM_APP_TX_STATUS, (WPARAM)ok, (LPARAM)pRate);
    }
}
static void PostRxStatus(bool ok, double rate) {
    double* pRate = (double*)malloc(sizeof(double));
    if (pRate) {
        *pRate = rate;
        PostMessageW(g_hwnd, WM_APP_RX_STATUS, (WPARAM)ok, (LPARAM)pRate);
    }
}

static void sim_thread(){
    setlocale(LC_NUMERIC, "C");
    UdpTx tx;

    static RawSensors R_receive_buffer{};
    static RawSensors R_prev_sample{};
    static uint64_t R_prev_ms = 0, R_last_ms = 0;
    auto next_try = std::chrono::steady_clock::now();
    int simconnect_attempts = 0;
    static auto last_status_update = std::chrono::steady_clock::now();

    double t_phys_acc = 0.0;
    double udp_send_acc = 0.0;
    auto t_prev = std::chrono::steady_clock::now();

    static uint64_t last_tx_time_ms = 0;
    static int tx_frame_count = 0;
    static double tx_rate_hz = 0.0;
    static uint64_t last_tx_calc_ms = _now_ms();

    bool origin_captured = false;
    int last_pos_mode = -1;

    auto sane_pos = [](double la, double lo){
        return std::isfinite(la) && std::isfinite(lo) &&
        fabs(la) <= 90 && fabs(lo) <= 180 &&
        !(fabs(la) < 1e-9 && fabs(lo) < 1e-9);
    };

    tx.open("", 0);

    while(RUN){

        auto now = std::chrono::steady_clock::now();
        double measured_dt = std::chrono::duration<double>(now - t_prev).count();
        if (measured_dt < 0) measured_dt = 0;
        if (measured_dt > 0.1) measured_dt = 0.1;
        t_prev = now;
        udp_send_acc += measured_dt;

        int rate_hz_snap;
        Dest d_now;
        bool match_sim_rate_snap;
        double sim_dt_ms_snap;
        int pos_mode_snap;

        {
            std::lock_guard<std::mutex> lk(G.m_tx);
            d_now = G.dest;
            match_sim_rate_snap = G.match_sim_rate;
            sim_dt_ms_snap = G.sim_dt_ms;
            rate_hz_snap = G.rate_hz;
            pos_mode_snap = G.json_pos_mode;
        }

        if (pos_mode_snap != last_pos_mode) {
            origin_captured = false;
            last_pos_mode = pos_mode_snap;
        }

        rate_hz_snap = match_sim_rate_snap ? iclamp((int)std::round(1000.0/std::max(5.0, sim_dt_ms_snap)), 10, 1000) : rate_hz_snap;
        const double target_dt = 1.0 / (double)iclamp(rate_hz_snap, 10, 1000);

        if (!g_sim_ok.load() && std::chrono::steady_clock::now() >= next_try){
            simconnect_attempts++;

            if (sim_open()) {
                g_sim_ok.store(true);
                simconnect_attempts = 0;
                PostStatus(L"SimConnect connected.");
            }
            else {
                next_try = std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
                if (simconnect_attempts % 3 == 0) {
                    PostStatus(L"SimConnect not found (attempt %d)...", simconnect_attempts);
                }
                PostSimStatus(false, 0.0);
            }
        }

        SIMCONNECT_RECV* p=nullptr; DWORD cb=0;

        if (g_sim_ok.load()){
            HRESULT hr = SimConnect_GetNextDispatch(gSim,&p,&cb);

            while(SUCCEEDED(hr) && p){

                switch(p->dwID){
                    case SIMCONNECT_RECV_ID_QUIT:
                    PostStatus(L"SimConnect disconnected.");
                    sim_close();
                    g_sim_ok.store(false);
                    origin_captured = false;
                    PostSimStatus(false, 0.0);
                    next_try = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
                    break;
                    case SIMCONNECT_RECV_ID_SIMOBJECT_DATA:{
                        static uint64_t last_ms = 0; uint64_t now_ms = GetTickCount64();
                        double dt = (now_ms > last_ms) ? (double)(now_ms - last_ms) : 0.0;
                        last_ms = now_ms;

                        if (dt>1 && dt<500) {
                            std::lock_guard<std::mutex> lk(G.m_tx);
                            G.sim_dt_ms = 0.8*G.sim_dt_ms + 0.2*dt;
                        }

                        auto* d=(SIMCONNECT_RECV_SIMOBJECT_DATA*)p;

                        if (d->dwRequestID==REQ_SENSORS){
                            const double* v=(const double*)&d->dwData;
                            R_receive_buffer.lat_deg=v[0]; R_receive_buffer.lon_deg=v[1];
                            R_receive_buffer.alt_msl_ft=v[2]; R_receive_buffer.alt_agl_ft=v[3];
                            R_receive_buffer.pitch_deg=v[4]; R_receive_buffer.bank_deg=v[5]; R_receive_buffer.hdg_true_deg=v[6];
                            R_receive_buffer.ias_kt=v[7];
                            R_receive_buffer.vel_e_fps = v[8]; R_receive_buffer.vel_n_fps = v[9]; R_receive_buffer.vel_u_fps = v[10];
                            R_receive_buffer.p_rads=v[13]; R_receive_buffer.q_rads=v[11]; R_receive_buffer.r_rads=v[12];
                            R_receive_buffer.accel_x_fps2=v[14]; R_receive_buffer.accel_y_fps2=v[15]; R_receive_buffer.accel_z_fps2=v[16];
                            R_receive_buffer.engine_rpm=v[17]; R_receive_buffer.prop_rpm=v[18]; R_receive_buffer.prop_pitch_rad=v[19];
                            R_receive_buffer.radio_height_ft=v[20]; R_receive_buffer.ground_alt_ft=v[21];

                            if (std::isfinite(R_receive_buffer.radio_height_ft) && R_receive_buffer.radio_height_ft>=0 && R_receive_buffer.radio_height_ft<=3000)
                            R_receive_buffer.alt_agl_ft=R_receive_buffer.radio_height_ft;
                            else if (std::isfinite(R_receive_buffer.ground_alt_ft))
                            R_receive_buffer.alt_agl_ft = std::max(R_receive_buffer.alt_msl_ft-R_receive_buffer.ground_alt_ft,0.0);

                            R_receive_buffer.valid = sane_pos(R_receive_buffer.lat_deg, R_receive_buffer.lon_deg);

                            {
                                std::lock_guard<std::mutex> lk(G.m_tx);

                                if (pos_mode_snap == 0 && !origin_captured && R_receive_buffer.valid) {
                                    G.sim_origin_lat = R_receive_buffer.lat_deg;
                                    G.sim_origin_lon = R_receive_buffer.lon_deg;
                                    G.sim_origin_alt_m = ft2m(R_receive_buffer.alt_msl_ft);
                                    G.sim_origin_set = true;
                                    origin_captured = true;
                                } else if (pos_mode_snap != 0 && !G.sim_origin_set && R_receive_buffer.valid) {
                                    G.sim_origin_lat = R_receive_buffer.lat_deg;
                                    G.sim_origin_lon = R_receive_buffer.lon_deg;
                                    G.sim_origin_alt_m = ft2m(R_receive_buffer.alt_msl_ft);
                                    G.sim_origin_set = true;
                                }

                            double Re = G.sim_earth_radius;
                            constexpr double DEG2RAD=0.01745329251994329577;

                            double dLat=(R_receive_buffer.lat_deg - G.sim_origin_lat) * DEG2RAD;
                            double dLon=(R_receive_buffer.lon_deg - G.sim_origin_lon) * DEG2RAD;
                            double latm=((R_receive_buffer.lat_deg + G.sim_origin_lat)/2.0)*DEG2RAD;

                            R_receive_buffer.N_m = dLat*Re;
                            R_receive_buffer.E_m = dLon*Re*std::cos(latm);
                            R_receive_buffer.U_m = ft2m(R_receive_buffer.alt_msl_ft) - G.sim_origin_alt_m;
                            }

                            {
                                std::lock_guard<std::mutex> lk(G.m_tx);
                                R_prev_sample = G.R;
                                R_prev_ms = R_last_ms;
                                G.R = R_receive_buffer;
                                R_last_ms = now_ms;
                            }
                            {
                                const int __hz = (rate_hz_snap > 0 ? rate_hz_snap : 50);
                                const uint64_t __period = (uint64_t)(1000 / __hz);
                                const uint64_t __now = GetTickCount64();
                                uint64_t __next = g_next_log_ms.load(std::memory_order_relaxed);

                                if (__now >= __next) {
                                    LogSensorsToFile(R_receive_buffer);
                                    g_next_log_ms.store(__now + __period, std::memory_order_relaxed);
                                }
                            }
                        }
                        break;
                    }
                    default: break;
                }

                if (g_sim_ok.load()) {
                    hr = SimConnect_GetNextDispatch(gSim,&p,&cb);
                } else {
                    p = nullptr;
                }
            }
        }

        PWMLast P;
        bool have_pwm=false;
        {
            std::lock_guard<std::mutex> lk(G.m_rx);
            have_pwm = (!G.pwm.pwm.empty()) && (std::chrono::duration<double>(std::chrono::steady_clock::now()-G.pwm.tlast).count() < 0.3);
            if (have_pwm) P = G.pwm;
        }

        static bool intercept_enabled = false;
        if (g_sim_ok.load()) {
            if (have_pwm && !intercept_enabled) {
                SimConnect_SetInputGroupPriority(gSim, GRP_INTERCEPT, SIMCONNECT_GROUP_PRIORITY_HIGHEST);
                intercept_enabled = true;
                PostStatus(L"HW axes: suppressed (SITL active)");
            } else if (!have_pwm && intercept_enabled) {
                SimConnect_SetInputGroupPriority(gSim, GRP_INTERCEPT, SIMCONNECT_GROUP_PRIORITY_STANDARD);
                intercept_enabled = false;
                PostStatus(L"HW axes: restored (SITL inactive)");
            }
        }

        if (g_sim_ok.load() && have_pwm && P.pwm.size() >= 16) {

            double norm_pwm[16];
            bool inv_ch[16];
            int sim_evt_idx_copy[16];

            {
                std::lock_guard<std::mutex> lk(G.m_tx);
                for(int i=0; i<16; i++) {
                    inv_ch[i] = G.invsim_ch[i];
                    sim_evt_idx_copy[i] = G_sim_evt_idx[i];
                }
            }
            {
                std::lock_guard<std::mutex> lk(G.m_gui);
                for(int i=0; i<16; i++) norm_pwm[i] = G.sitl_out_pwm[i];
            }

            for (int i = 0; i < 16; i++) {
                if (inv_ch[i]) {
                    if (i == 0 || i == 1 || i == 3) {
                        norm_pwm[i] = -norm_pwm[i];
                    } else {
                        norm_pwm[i] = 1.0 - norm_pwm[i];
                    }
                }
            }

            for (int i = 0; i < 16; i++) {
                if (sim_evt_idx_copy[i] != 0) {
                    LONG sim_val;
                    if (i == 0 || i == 1 || i == 3) {
                        sim_val = (LONG)llround(norm_pwm[i] * 16383.0);
                    } else {
                        sim_val = (LONG)llround((norm_pwm[i] * 2.0 - 1.0) * 16383.0);
                    }

                    SimConnect_TransmitClientEvent(gSim, 0, g_sim_evt_map[i], (DWORD)sim_val, SIMCONNECT_GROUP_PRIORITY_HIGHEST, SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);
                }
            }
        }

        if (tx.needs_reopen(d_now.ip, d_now.port_tx)) {
             tx.open(d_now.ip, d_now.port_tx);
        }

        while (udp_send_acc >= target_dt) {

            udp_send_acc -= target_dt;

            t_phys_acc += target_dt;
            const double t_sec = t_phys_acc;

            RawSensors R{};
            int resample_mode_snap;
            {
                std::lock_guard<std::mutex> lk(G.m_tx);
                R = G.R;
                resample_mode_snap = G.resample_mode;
            }

            if (!match_sim_rate_snap && resample_mode_snap == 2) {
                uint64_t now_ms = GetTickCount64();
                double sim_dt = (R_last_ms>0 && R_prev_ms>0) ? double(R_last_ms - R_prev_ms) : 0.0;
                double since  = (R_last_ms>0 && now_ms > R_last_ms) ? double(now_ms - R_last_ms) : 0.0;

                if (sim_dt > 0.0 && since >= 0.0 && since < 1000.0) {
                    double alpha = since / sim_dt;
                    alpha = clampd(alpha, 0.0, 1.0);

                    auto lerp  = [](double a,double b,double t){ return a + (b - a)*t; };
                    auto lerp_ang = [](double a,double b,double t){
                        double da = fmod(b - a + 540.0, 360.0) - 180.0;
                        return a + da*t;
                    };
                    R.lat_deg = lerp(R_prev_sample.lat_deg, R_receive_buffer.lat_deg, alpha);
                    R.lon_deg = lerp(R_prev_sample.lon_deg, R_receive_buffer.lon_deg, alpha);
                    R.alt_msl_ft = lerp(R_prev_sample.alt_msl_ft, R_receive_buffer.alt_msl_ft, alpha);
                    R.alt_agl_ft = lerp(R_prev_sample.alt_agl_ft, R_receive_buffer.alt_agl_ft, alpha);
                    R.pitch_deg = lerp(R_prev_sample.pitch_deg, R_receive_buffer.pitch_deg, alpha);
                    R.bank_deg  = lerp(R_prev_sample.bank_deg,  R_receive_buffer.bank_deg,  alpha);
                    R.hdg_true_deg = lerp_ang(R_prev_sample.hdg_true_deg, R_receive_buffer.hdg_true_deg, alpha);
                    R.vel_e_fps = lerp(R_prev_sample.vel_e_fps, R_receive_buffer.vel_e_fps, alpha);
                    R.vel_n_fps = lerp(R_prev_sample.vel_n_fps, R_receive_buffer.vel_n_fps, alpha);
                    R.vel_u_fps = lerp(R_prev_sample.vel_u_fps, R_receive_buffer.vel_u_fps, alpha);
                    R.p_rads = lerp(R_prev_sample.p_rads, R_receive_buffer.p_rads, alpha);
                    R.q_rads = lerp(R_prev_sample.q_rads, R_receive_buffer.q_rads, alpha);
                    R.r_rads = lerp(R_prev_sample.r_rads, R_receive_buffer.r_rads, alpha);
                    R.accel_x_fps2 = lerp(R_prev_sample.accel_x_fps2, R_receive_buffer.accel_x_fps2, alpha);
                    R.accel_y_fps2 = lerp(R_prev_sample.accel_y_fps2, R_receive_buffer.accel_y_fps2, alpha);
                    R.accel_z_fps2 = lerp(R_prev_sample.accel_z_fps2, R_receive_buffer.accel_z_fps2, alpha);
                    R.ias_kt = lerp(R_prev_sample.ias_kt, R_receive_buffer.ias_kt, alpha);
                    R.engine_rpm = lerp(R_prev_sample.engine_rpm, R_receive_buffer.engine_rpm, alpha);
                    R.prop_rpm   = lerp(R_prev_sample.prop_rpm,   R_receive_buffer.prop_rpm,   alpha);
                    R.prop_pitch_rad = lerp(R_prev_sample.prop_pitch_rad, R_receive_buffer.prop_pitch_rad, alpha);
                    R.radio_height_ft = lerp(R_prev_sample.radio_height_ft, R_receive_buffer.radio_height_ft, alpha);
                    R.ground_alt_ft   = lerp(R_prev_sample.ground_alt_ft,   R_receive_buffer.ground_alt_ft,   alpha);

                    R.N_m = lerp(R_prev_sample.N_m, R_receive_buffer.N_m, alpha);
                    R.E_m = lerp(R_prev_sample.E_m, R_receive_buffer.E_m, alpha);
                    R.U_m = lerp(R_prev_sample.U_m, R_receive_buffer.U_m, alpha);
                }
            }

            struct sockaddr_in dest_addr;
            bool dest_known;
            {
                std::lock_guard<std::mutex> lk(g_sitl_addr_mtx);
                dest_addr = g_sitl_addr;
                dest_known = g_sitl_addr_known;
            }

            if (R.valid && G.sim_origin_set) {

                double vel_n_ms = ft2m(R.vel_n_fps);
                double vel_e_ms = ft2m(R.vel_e_fps);
                double vel_d_ms = ft2m(-R.vel_u_fps);

                double accel_x_ms2 = ft2m(R.accel_x_fps2);
                double accel_y_ms2 = ft2m(R.accel_y_fps2);
                double accel_z_ms2 = ft2m(-R.accel_z_fps2);
                double alt_msl_m = ft2m(R.alt_msl_ft);
                double alt_agl_m = ft2m(R.alt_agl_ft);
                double airspeed_ms = kt2ms(R.ias_kt);

                double roll_rad = -deg2rad(R.bank_deg);
                double pitch_rad = -deg2rad(R.pitch_deg);
                double yaw_rad = deg2rad(R.hdg_true_deg);

                double cy = cos(yaw_rad * 0.5);
                double sy = sin(yaw_rad * 0.5);
                double cp = cos(pitch_rad * 0.5);
                double sp = sin(pitch_rad * 0.5);
                double cr = cos(roll_rad * 0.5);
                double sr = sin(roll_rad * 0.5);

                float q1 = cr * cp * cy + sr * sp * sy;
                float q2 = sr * cp * cy - cr * sp * sy;
                float q3 = cr * sp * cy + sr * cp * sy;
                float q4 = cr * cp * sy - sr * sp * cy;

                double rc_copy[12];
                {
                    std::lock_guard<std::mutex> lk(G.m_gui);
                    for(int i=0; i<12; i++) rc_copy[i] = G.rc_out[i];
                }

                if (dest_known) {
                    static char json_buf[4096];

                    bool use_time_sync_snap;
                    bool no_lockstep_snap;

                    {
                        std::lock_guard<std::mutex> lk(G.m_tx);
                        use_time_sync_snap = G.use_time_sync;
                        no_lockstep_snap = G.no_lockstep;
                    }

                    float rc_pwm[12];
                    for(int i=0; i<12; i++) {
                        rc_pwm[i] = (rc_copy[i] < 0.0) ? 1500.0f : (float)(rc_copy[i] * 1000.0 + 1000.0);
                    }

                    static char tsync_buf[64];
                    snprintf(tsync_buf, sizeof(tsync_buf), "\"no_time_sync\":%s, ", use_time_sync_snap ? "false" : "true");
                    const char* tsync_field = tsync_buf;


                    static char lockstep_buf[64];
                    snprintf(lockstep_buf, sizeof(lockstep_buf), "\"no_lockstep\": %s, ", no_lockstep_snap ? "true" : "false");
                    const char* lockstep_field = lockstep_buf;

                    static char geo_buf[256];
                    if (pos_mode_snap == 2) {
                        snprintf(geo_buf, sizeof(geo_buf),
                        "\"latitude\": %.10f, \"longitude\": %.10f, \"altitude\": %.4f, \"position\": [%.4f, %.4f, %.4f], ",
                        R.lat_deg, R.lon_deg, alt_msl_m, R.N_m, R.E_m, -R.U_m);
                    } else {
                        snprintf(geo_buf, sizeof(geo_buf),
                        "\"position\": [%.4f, %.4f, %.4f], ",
                        R.N_m, R.E_m, -R.U_m);
                    }

                    int len = snprintf(json_buf, sizeof(json_buf),
    "{"
      "\"timestamp\": %.6f, "
      "%s"
      "\"quaternion\": [%.6f, %.6f, %.6f, %.6f], "
      "\"velocity\": [%.6f, %.6f, %.6f], "
      "\"imu\": {"
        "\"gyro\": [%.6f, %.6f, %.6f], "
        "\"accel_body\": [%.6f, %.6f, %.6f]"
      "}, "
      "\"airspeed\": %.4f, "
      "\"rng_1\": %.4f, "
      "%s"
      "%s"
      "\"rc\": {"
        "\"rc_1\": %.1f, \"rc_2\": %.1f, \"rc_3\": %.1f, \"rc_4\": %.1f, "
        "\"rc_5\": %.1f, \"rc_6\": %.1f, \"rc_7\": %.1f, \"rc_8\": %.1f, "
        "\"rc_9\": %.1f, \"rc_10\": %.1f, \"rc_11\": %.1f, \"rc_12\": %.1f"
      "}"
    "}\n",
    t_sec,
    geo_buf,
    q1, q2, q3, q4,
    vel_n_ms, vel_e_ms, vel_d_ms,
    -R.p_rads, -R.q_rads, R.r_rads,
    accel_x_ms2, accel_y_ms2, accel_z_ms2,
    airspeed_ms,
    alt_agl_m,
    lockstep_field,
    tsync_field,
    rc_pwm[0], rc_pwm[1], rc_pwm[2], rc_pwm[3],
    rc_pwm[4], rc_pwm[5], rc_pwm[6], rc_pwm[7],
    rc_pwm[8], rc_pwm[9], rc_pwm[10], rc_pwm[11]
);

                    if (len > 0 && len < sizeof(json_buf)) {
                        tx.send_buffer(json_buf, len, &dest_addr);
                        tx_frame_count++;
                        last_tx_time_ms = _now_ms();
                    }
                }
            }
        }

        auto now_status = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now_status - last_status_update).count() >= 0.5){
            last_status_update = now_status;

            wchar_t wip[256];
            if (g_sitl_addr_known) {
                char sitl_ip_str[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &g_sitl_addr.sin_addr, sitl_ip_str, INET_ADDRSTRLEN);
                MultiByteToWideChar(CP_UTF8,0,sitl_ip_str,-1,wip,256);
            } else {
                wcscpy(wip, L"?.?.?.?");
            }

            double sim_dt_ms_now;
            {
                std::lock_guard<std::mutex> lk(G.m_tx);
                sim_dt_ms_now = G.sim_dt_ms;
            }
            double sim_fps = (sim_dt_ms_now > 0) ? (1000.0 / sim_dt_ms_now) : 0.0;

            PostSimStatus(g_sim_ok.load(), sim_fps);

            bool valid_data = false;
            { std::lock_guard<std::mutex> lk(G.m_tx); valid_data = G.R.valid; }
            const wchar_t* data_status = valid_data ? L"Data: VALID" : L"Data: NO";

            const wchar_t* joy_status = G.joy_ok.load() ? L"Joy: OK" : L"Joy: ---";

            bool sitl_is_alive;
            {
                std::lock_guard<std::mutex> lk(G.m_rx);
                sitl_is_alive = g_sitl_addr_known && (std::chrono::duration<double>(std::chrono::steady_clock::now() - G.pwm.tlast).count() < 2.0);
            }
            const wchar_t* sitl_rx_status = sitl_is_alive ? L"SITL RX: OK" : L"SITL RX: ---";

            bool tx_ok = g_sitl_addr_known && (_now_ms() - last_tx_time_ms < 2000);
            PostTxStatus(tx_ok, tx_rate_hz);

            wchar_t* buf=(wchar_t*)malloc(sizeof(wchar_t)*640);
            if (buf){
                swprintf(buf, 640, L"Sim fps: %.1f | %s | %s | %s (RX:%u) | TX: %s:%u | %dHz | JSON MODE",
                sim_fps,
                data_status,
                joy_status,
                sitl_rx_status,
                d_now.port_rx,
                wip, (g_sitl_addr_known ? ntohs(g_sitl_addr.sin_port) : 0),
                rate_hz_snap);
                PostMessageW(g_hwnd, WM_APP_STATUSTEXT, 0, (LPARAM)buf);
            }
        }

        uint64_t calc_now = _now_ms();
        if (calc_now - last_tx_calc_ms > 1000) {
            double dt_s = (calc_now - last_tx_calc_ms) / 1000.0;
            if (dt_s > 0) {
                tx_rate_hz = (double)tx_frame_count / dt_s;
            }
            tx_frame_count = 0;
            last_tx_calc_ms = calc_now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    tx.close();

    if(g_sim_ok.load()) {
        sim_close();
    }
    g_sim_ok.store(false);
    PostSimStatus(false, 0.0);
    PostTxStatus(false, 0.0);
}

static void joy_thread(){
    int joy_idx_last = -1;

    while(RUN){
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        int joy_idx_now;
        { std::lock_guard<std::mutex> lk(G.m_tx); joy_idx_now = G.joy_index; }

        if (joy_idx_now != joy_idx_last) {
            select_joystick(joy_idx_now);
            joy_idx_last = joy_idx_now;
            if (joy_idx_now >= 0) PostStatus(L"Joystick %d selected.", joy_idx_now);
            else PostStatus(L"No joystick selected.");
        }

        if (!g_pJoy) {
            G.joy_ok.store(false);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        HRESULT hr = g_pJoy->Poll();

        if (FAILED(hr)) {
            G.joy_ok.store(false);
            hr = g_pJoy->Acquire();

            if (FAILED(hr)) {
                G.joy_ok.store(false);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }

            G.joy_ok.store(true);
        }

        DIJOYSTATE2 js;
        hr = g_pJoy->GetDeviceState(sizeof(DIJOYSTATE2), &js);
        if (FAILED(hr)) {
            G.joy_ok.store(false);
            continue;
        }

        G.joy_ok.store(true);

        double raw_axes[NUM_JOY_AXES]{};
        raw_axes[0] = (double)js.lX / 1000.0;
        raw_axes[1] = (double)js.lY / 1000.0;
        raw_axes[2] = (double)js.lZ / 1000.0;
        raw_axes[3] = (double)js.lRx / 1000.0;
        raw_axes[4] = (double)js.lRy / 1000.0;
        raw_axes[5] = (double)js.lRz / 1000.0;
        raw_axes[6] = (double)js.rglSlider[0] / 1000.0;
        raw_axes[7] = (double)js.rglSlider[1] / 1000.0;

        if (js.rgdwPOV[0] == 0) raw_axes[8] = 1.0;
        else if (js.rgdwPOV[0] == 18000) raw_axes[8] = -1.0;
        else raw_axes[8] = 0.0;

        if (js.rgdwPOV[0] == 9000) raw_axes[9] = 1.0;
        else if (js.rgdwPOV[0] == 27000) raw_axes[9] = -1.0;
        else raw_axes[9] = 0.0;

        raw_axes[10] = (js.rgbButtons[0] & 0x80) ? 1.0 : -1.0;
        raw_axes[11] = (js.rgbButtons[1] & 0x80) ? 1.0 : -1.0;

        {
            std::lock_guard<std::mutex> lk(G.m_gui);
            for(int i=0;i<NUM_JOY_AXES;i++) G.raw_axes[i] = raw_axes[i];
        }

        JoyMapCfg map_copy[NUM_JOY_AXES];
        {
            std::lock_guard<std::mutex> lk(G.m_tx);
            for(int i=0; i < NUM_JOY_AXES; i++) map_copy[i] = G.joy_map[i];
        }

        double out_slots[12];
        for(int i=0; i<12; i++) out_slots[i] = -1.0;

        for (int i = 0; i < NUM_JOY_AXES; ++i) {
            const JoyMapCfg& m = map_copy[i];
            if (m.rcDest == 0) continue;

            double val_n1_1 = raw_axes[i] * m.srcInv;

            double val_0_1 = (val_n1_1 * 0.5) + 0.5;

            if (m.overrideMode == 1) val_0_1 = 0.0;
            if (m.overrideMode == 2) val_0_1 = 0.5;
            if (m.overrideMode == 3) val_0_1 = 1.0;

            val_0_1 = clampd(val_0_1, 0.0, 1.0);

            int slot_idx = m.rcDest - 1;
            if (slot_idx >= 0 && slot_idx < 12) {
                out_slots[slot_idx] = val_0_1;
            }
        }

        {
            std::lock_guard<std::mutex> lk(G.m_gui);
            for(int i=0;i<12;i++) G.rc_out[i] = out_slots[i];
        }
    }

    G.joy_ok.store(false);
    if(g_pJoy){ g_pJoy->Unacquire(); g_pJoy->Release(); g_pJoy = NULL; }
    if(g_pDI){ g_pDI->Release(); g_pDI = NULL; }
}

static void rx_thread(){
    UdpRxRaw rx;
    uint16_t port_last=0;
    std::vector<uint8_t> buf(8192);
    struct sockaddr_in from_addr = {};

    static uint64_t last_rx_time_ms = _now_ms();
    static int rx_frame_count = 0;
    static double rx_rate_hz = 0.0;
    static auto last_rx_status_post = std::chrono::steady_clock::now();


    auto normalize_pwm = [](uint16_t pwm, bool is_throttle_or_aux) -> double {
        if (is_throttle_or_aux) {
            return clampd(((double)pwm - 1000.0) / 1000.0, 0.0, 1.0);
        } else {
            return clampd(((double)pwm - 1500.0) / 500.0, -1.0, 1.0);
        }
    };

    while(RUN){
        uint16_t port_now;
        { std::lock_guard<std::mutex> lk(G.m_tx); port_now=G.dest.port_rx; }

        if(rx.needs_reopen(port_now)){
            rx.open(port_now);
            port_last = port_now;
            PostStatus(L"RX (Servo) settings updated: listening on port %u", port_now);
        }

        int len = rx.recv(buf.data(), (int)buf.size(), &from_addr);

        auto now_tp = std::chrono::steady_clock::now();
        if (len <= 0) {
            if (std::chrono::duration<double>(now_tp - last_rx_status_post).count() > 1.0) {
                if (G.status_rx_ok.load()) {
                    PostRxStatus(false, 0.0);
                }
                last_rx_status_post = now_tp;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        rx_frame_count++;
        uint64_t now_ms = _now_ms();
        uint64_t dt = now_ms - last_rx_time_ms;
        if (dt > 1000) {
            rx_rate_hz = (double)rx_frame_count / (dt / 1000.0);
            rx_frame_count = 0;
            last_rx_time_ms = now_ms;
        }
        if (std::chrono::duration<double>(now_tp - last_rx_status_post).count() > 0.5) {
             PostRxStatus(true, rx_rate_hz);
             last_rx_status_post = now_tp;
        }


        if (len >= sizeof(servo_packet_16)) {
            auto* pkt = reinterpret_cast<servo_packet_16*>(buf.data());
            if (pkt->magic == 18458) {
                {
                    std::lock_guard<std::mutex> lk(g_sitl_addr_mtx);
                    g_sitl_addr = from_addr;
                    g_sitl_addr_known = true;
                }
                {
                    std::lock_guard<std::mutex> lk(G.m_rx);
                    if (G.pwm.pwm.size() < 16) G.pwm.pwm.resize(16);
                    memcpy(G.pwm.pwm.data(), pkt->pwm, 16 * sizeof(uint16_t));
                    G.pwm.tlast = std::chrono::steady_clock::now();
                    G.pwm.rate_hz = pkt->frame_rate;
                }
                {
                    std::lock_guard<std::mutex> lk_gui(G.m_gui);
                    for(int i=0; i<16; i++) {
                        bool is_thr_aux = (i == 2 || i >= 4);
                        G.sitl_out_pwm[i] = normalize_pwm(pkt->pwm[i], is_thr_aux);
                        G.sitl_has_ch[i] = true;
                    }
                }
                continue;
            }
        }

        if (len >= sizeof(servo_packet_32)) {
            auto* pkt = reinterpret_cast<servo_packet_32*>(buf.data());
            if (pkt->magic == 29569) {
                {
                    std::lock_guard<std::mutex> lk(g_sitl_addr_mtx);
                    g_sitl_addr = from_addr;
                    g_sitl_addr_known = true;
                }
                {
                    std::lock_guard<std::mutex> lk(G.m_rx);
                    if (G.pwm.pwm.size() < 32) G.pwm.pwm.resize(32);
                    memcpy(G.pwm.pwm.data(), pkt->pwm, 32 * sizeof(uint16_t));
                    G.pwm.tlast = std::chrono::steady_clock::now();
                    G.pwm.rate_hz = pkt->frame_rate;
                }
                {
                    std::lock_guard<std::mutex> lk_gui(G.m_gui);
                    for(int i=0; i<16; i++) {
                        bool is_thr_aux = (i == 2 || i >= 4);
                        G.sitl_out_pwm[i] = normalize_pwm(pkt->pwm[i], is_thr_aux);
                        G.sitl_has_ch[i] = true;
                    }
                }
                continue;
            }
        }
    }
    rx.close();
    PostRxStatus(false, 0.0);
}

static void ShowCrashReport(const wchar_t* title, const wchar_t* format, ...) {
    wchar_t buf[2048];
    va_list args;
    va_start(args, format);
    vswprintf(buf, _countof(buf), format, args);
    va_end(args);

    MessageBoxW(NULL, buf, title, MB_OK | MB_ICONERROR | MB_TOPMOST);
}

static void sig_handler(int sig) {
    RUN.store(false);
    const char* sig_name = "UNKNOWN SIGNAL";
    if (sig == SIGSEGV) sig_name = "SIGSEGV (Access Violation)";
    if (sig == SIGFPE)  sig_name = "SIGFPE (Floating Point Exception)";
    if (sig == SIGILL)  sig_name = "SIGILL (Illegal Instruction)";
    if (sig == SIGABRT) sig_name = "SIGABRT (Abort)";

    ShowCrashReport(L"Fatal Error (Signal)", L"Caught signal: %hs (%d).\nThe application will close.", sig_name, sig);
    std::quick_exit(sig);
}

static void se_translator(unsigned int code, EXCEPTION_POINTERS* ep) {
    RUN.store(false);
    std::string err_type = "UNKNOWN SEH EXCEPTION";
    if (code == EXCEPTION_ACCESS_VIOLATION) err_type = "Access Violation";
    if (code == EXCEPTION_INT_DIVIDE_BY_ZERO) err_type = "Divide by Zero";
    if (code == EXCEPTION_STACK_OVERFLOW) err_type = "Stack Overflow";
    if (code == EXCEPTION_FLT_DIVIDE_BY_ZERO) err_type = "Float Divide by Zero";

    ShowCrashReport(L"Fatal Error (SEH)", L"Caught SEH Exception: %hs (Code: 0x%X)\nAddress: 0x%p\nThe application will close.",
    err_type.c_str(), code, ep ? ep->ExceptionRecord->ExceptionAddress : 0);
    std::quick_exit(code);
}

static void term_handler() {
    RUN.store(false);
    try {
        std::rethrow_exception(std::current_exception());

    } catch (const std::exception& e) {
        ShowCrashReport(L"Fatal Error (terminate)", L"std::terminate() called with exception:\n%hs\nThe application will close.", e.what());

    } catch (...) {
        ShowCrashReport(L"Fatal Error (terminate)", L"std::terminate() called with unknown exception.\nThe application will close.");
    }
    std::abort();
}

static void setup_crash_handlers() {
    signal(SIGSEGV, sig_handler);
    signal(SIGFPE, sig_handler);
    signal(SIGILL, sig_handler);
    signal(SIGABRT, sig_handler);
    _set_se_translator(se_translator);
    std::set_terminate(term_handler);
}

int WINAPI wWinMain(HINSTANCE hi, HINSTANCE, PWSTR, int nCmdShow)
{
    setup_crash_handlers();

    try
    {
        HMODULE hUser32 = LoadLibraryW(L"User32.dll");

        if(hUser32){
            typedef BOOL (WINAPI *PFN_SETTHREADDP)(DPI_AWARENESS_CONTEXT);
            PFN_SETTHREADDP pSetThreadDpiAwarenessContext = (PFN_SETTHREADDP)GetProcAddress(hUser32, "SetThreadDpiAwarenessContext");

            if(pSetThreadDpiAwarenessContext){
                pSetThreadDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
            }
            FreeLibrary(hUser32);
        }

        load_ini();

        WNDCLASSEXW wc{sizeof(WNDCLASSEXW)};
        wc.lpfnWndProc=WndProc;
        wc.hInstance=hi;
        wc.lpszClassName=L"MSFS_AP_BRIDGE_WNDCLASS";
        wc.hCursor=LoadCursor(NULL,IDC_ARROW);
        wc.hbrBackground=(HBRUSH)(COLOR_WINDOW+1);
        wc.hIcon = (HICON)LoadImageW(
        hi,
        MAKEINTRESOURCEW(IDI_APPICON),
        IMAGE_ICON,
        GetSystemMetrics(SM_CXICON),
        GetSystemMetrics(SM_CYICON),
        0
        );
        wc.hIconSm = (HICON)LoadImageW(
        hi,
        MAKEINTRESOURCEW(IDI_APPICON),
        IMAGE_ICON,
        GetSystemMetrics(SM_CXSMICON),
        GetSystemMetrics(SM_CYSMICON),
        0
        );
        RegisterClassExW(&wc);

        int win_x, win_y, win_w, win_h;
        {
            std::lock_guard<std::mutex> lk(G.m_tx);
            g_dpi = Dpi(NULL);
            win_x = G.win_x;
            win_y = G.win_y;
            win_w = G.win_w;
            win_h = G.win_h;
        }

        if (win_h < 920) win_h = 920;

        CreateWindowExW(0,wc.lpszClassName,APP_TITLE_W, WS_OVERLAPPEDWINDOW,
        win_x, win_y, S(win_w), S(win_h),
        NULL,NULL,hi,NULL);

        if(!g_hwnd) return 1;

        {

            if (win_x == CW_USEDEFAULT || win_y == CW_USEDEFAULT) {
                SetWindowPos(g_hwnd, NULL, 0, 0, S(win_w), S(win_h),
                    SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE);
            } else {
                SetWindowPos(g_hwnd, NULL, win_x, win_y, S(win_w), S(win_h),
                    SWP_NOZORDER | SWP_NOACTIVATE);
            }
        }

        int show_cmd = (win_x == CW_USEDEFAULT) ? nCmdShow : SW_SHOWNORMAL;
        ShowWindow(g_hwnd, show_cmd);
        UpdateWindow(g_hwnd);

        std::thread t_sim(sim_thread);
        std::thread t_joy(joy_thread);
        std::thread t_rx(rx_thread);

        MSG msg{};
        BOOL bRet;

        while((bRet = GetMessageW(&msg, NULL, 0, 0)) != 0){
            if (bRet == -1) break;
            if (msg.message >= WM_APP_SIM_STATUS && msg.message <= WM_APP_RX_STATUS) {
                if (msg.lParam) free(reinterpret_cast<void*>(msg.lParam));
            }
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }

        RUN = false;
        t_sim.join();
        t_joy.join();
        t_rx.join();

        return (int)msg.wParam;
    }

    catch (const std::exception& e)
    {
        char buf[2048];
        sprintf_s(buf, "Unhandled C++ exception in wWinMain:\n%hs", e.what());

        wchar_t wbuf[2048];
        MultiByteToWideChar(CP_UTF8, 0, buf, -1, wbuf, 2048);

        ShowCrashReport(L"Fatal Error (std::exception)", L"%s", wbuf);
        return 1;
    }

    catch (...)
    {
        ShowCrashReport(L"Fatal Error (Unknown)", L"Unhandled unknown C++ exception in wWinMain.");
        return 1;
    }
}

static void load_ini(){ g_ini_path = get_ini_path(); load_settings_from_path(g_ini_path); }

static void save_ini(){ if(g_ini_path.empty()) g_ini_path = get_ini_path(); save_settings_to_path(g_ini_path); }