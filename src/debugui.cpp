/* OpenUG debug overlay — Dear ImGui panel, compiled only into `make debug`.
 * Thin C-callable wrapper (declared in debug.h) so main.c stays plain C.
 * Backends: SDL2 + legacy OpenGL2 (matches the app's 2.1/compat GL context). */
#include <cstdio>
#include <SDL.h>
#ifdef __APPLE__
#  define GL_SILENCE_DEPRECATION 1
#  include <OpenGL/gl.h>
#else
#  include <SDL_opengl.h>
#endif
#include "imgui.h"
#include "backends/imgui_impl_sdl2.h"
#include "backends/imgui_impl_opengl2.h"
#include "debug.h"

extern "C" void dbgui_init(struct SDL_Window *win, void *glctx) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;          /* don't litter imgui.ini */
    ImGui::StyleColorsDark();
    ImGui_ImplSDL2_InitForOpenGL((SDL_Window *)win, glctx);
    ImGui_ImplOpenGL2_Init();
}

extern "C" void dbgui_event(const union SDL_Event *e) {
    ImGui_ImplSDL2_ProcessEvent((const SDL_Event *)e);
}
extern "C" int dbgui_want_mouse(void)    { return ImGui::GetIO().WantCaptureMouse; }
extern "C" int dbgui_want_keyboard(void) { return ImGui::GetIO().WantCaptureKeyboard; }

extern "C" void dbgui_frame(void) {
    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowSize(ImVec2(440, 0), ImGuiCond_FirstUseEver);
    ImGui::Begin("NFSU2 Master Inspector");
    g_dbg.fps = (int)ImGui::GetIO().Framerate;
    /* Momentary car/track switch requests: reset EVERY frame (not inside a tab,
       or an inactive tab would leave want_car at 0 = a valid index and main.c
       would relaunch the process every frame). The combos below set them on
       change only. */
    g_dbg.want_car = -1; g_dbg.want_track = -1;

    if (ImGui::BeginTabBar("MasterInspectorTabs")) {

    /* ---- Tab 1: Vehicle & Wheels ---- */
    if (ImGui::BeginTabItem("Vehicle & Wheels")) {
        if (ImGui::CollapsingHeader("Wheel & Rim Geometry", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SliderFloat("front offset", &g_dbg.wheel_frontf, 0.0f, 1.0f);
            ImGui::SliderFloat("rear offset",  &g_dbg.wheel_rearf,  0.0f, 1.0f);
            ImGui::SliderFloat("track width",  &g_dbg.wheel_trackf, 0.0f, 1.2f);
            ImGui::SliderFloat("z offset",     &g_dbg.wheel_z,    -1.0f, 1.0f);
            ImGui::SliderFloat("radius/scale", &g_dbg.wheel_scale, 0.3f, 2.0f);
            ImGui::Text("radius %.3f m   %.0f RPM   steer %+.1f deg",
                        g_dbg.wheel_radius, g_dbg.wheel_rpm, g_dbg.steer_deg);
        }
        if (ImGui::CollapsingHeader("Rim: brand / style / paint", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (g_dbg.wheel_brands && g_dbg.wheel_brand_n > 0) {
                static const char *brands[32];
                int nb = g_dbg.wheel_brand_n < 32 ? g_dbg.wheel_brand_n : 32;
                for (int i = 0; i < nb; i++) brands[i] = g_dbg.wheel_brands[i];
                if (ImGui::Combo("wheel brand", &g_dbg.wheel_brand, brands, nb)) g_dbg.wheel_reload = 1;
                if (ImGui::SliderInt("wheel style", &g_dbg.wheel_style, 1, 8)) g_dbg.wheel_reload = 1;
            } else ImGui::TextDisabled("wheel library not loaded");
            ImGui::Checkbox("paint rims (off = raw OEM texture)", (bool *)&g_dbg.rim_paint);
            ImGui::ColorEdit3("rim colour", g_dbg.rim_color);
            if (ImGui::Button("Chrome/Silver")) {
                g_dbg.rim_paint=1; g_dbg.rim_color[0]=0.85f; g_dbg.rim_color[1]=0.88f; g_dbg.rim_color[2]=0.92f;
            } ImGui::SameLine();
            if (ImGui::Button("OEM Gold")) g_dbg.rim_paint = 0;
            ImGui::SameLine();
            if (ImGui::Button("Gunmetal")) {
                g_dbg.rim_paint=1; g_dbg.rim_color[0]=0.30f; g_dbg.rim_color[1]=0.32f; g_dbg.rim_color[2]=0.36f;
            }
        }
        if (ImGui::CollapsingHeader("Vehicle Handling", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("live @ %.0f km/h", g_dbg.kmh);
            ImGui::SliderFloat("acceleration", &g_dbg.tune_accel, 0.2f, 3.0f, "%.2fx");
            ImGui::SliderFloat("braking",      &g_dbg.tune_brake, 0.2f, 3.0f, "%.2fx");
            ImGui::SliderFloat("steering",     &g_dbg.tune_turn,  0.2f, 2.5f, "%.2fx");
            ImGui::SliderFloat("top speed",    &g_dbg.tune_top,  60.0f, 320.0f, "%.0f km/h");
            if (ImGui::Button("reset handling")) {
                g_dbg.tune_accel=g_dbg.tune_brake=g_dbg.tune_turn=1.0f; g_dbg.tune_top=220.0f;
            }
        }
        if (ImGui::CollapsingHeader("Car parts")) {
            ImGui::Checkbox("body",   (bool *)&g_dbg.show_body);   ImGui::SameLine();
            ImGui::Checkbox("glass",  (bool *)&g_dbg.show_glass);  ImGui::SameLine();
            ImGui::Checkbox("lights", (bool *)&g_dbg.show_lights);
            ImGui::Checkbox("tires",  (bool *)&g_dbg.show_tires);  ImGui::SameLine();
            ImGui::Checkbox("misc",   (bool *)&g_dbg.show_misc);   ImGui::SameLine();
            ImGui::Checkbox("track",  (bool *)&g_dbg.show_track);
            ImGui::Checkbox("paint override", (bool *)&g_dbg.paint_override);
            ImGui::ColorEdit3("paint", g_dbg.paint);
            ImGui::TextDisabled("body kit: K cycles KIT00/01/02 (see console)");
        }
        if (ImGui::CollapsingHeader("Mesh Inspector")) {
            static const char *catn[] = {"ROAD","TERRAIN","OTHER","SKY","GLOW","?","?","?","?","?",
                                         "BODY","GLASS","LIGHT","TIRE","MISC","BRAKELIGHT","MECH"};
            ImGui::Checkbox("highlight", (bool *)&g_dbg.insp_highlight); ImGui::SameLine();
            ImGui::Checkbox("wireframe", (bool *)&g_dbg.insp_wire);
            if (ImGui::Button("Dump Selected Mesh Telemetry")) g_dbg.insp_dump = 1;
            ImGui::BeginChild("meshlist", ImVec2(0, 160), true);
            for (int i = 0; i < g_dbg.insp_count; i++) {
                int c = (g_dbg.insp_cat && i < g_dbg.insp_count) ? g_dbg.insp_cat[i] : 0;
                const char *cn = (c >= 0 && c <= 16) ? catn[c] : "?";
                char lbl[96];
                snprintf(lbl, sizeof lbl, "%3d  %-10s %5d v", i, cn,
                         g_dbg.insp_verts ? g_dbg.insp_verts[i] : 0);
                if (ImGui::Selectable(lbl, g_dbg.insp_sel == i)) g_dbg.insp_sel = i;
            }
            ImGui::EndChild();
            ImGui::Checkbox("[Debug] Flip Vertex Normals", (bool *)&g_dbg.insp_flipn);
            const char *cullm[] = { "no culling (engine default)", "cull BACK faces", "cull FRONT faces" };
            ImGui::Combo("[Debug] Face culling", &g_dbg.insp_cull, cullm, 3);
            ImGui::Checkbox("[Debug] Force Alpha Depth Write", (bool *)&g_dbg.insp_glass_depth);
            if (ImGui::Button("clear selection")) g_dbg.insp_sel = -1;
        }
        ImGui::EndTabItem();
    }

    /* ---- Tab 2: Lighting & Environment ---- */
    if (ImGui::BeginTabItem("Lighting & Environment")) {
        if (ImGui::CollapsingHeader("Lighting / Fog", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SliderFloat("ambient",   &g_dbg.ambient,   0.0f, 1.0f);
            ImGui::SliderFloat("diffuse",   &g_dbg.diffuse,   0.0f, 1.5f);
            ImGui::SliderFloat("body spec", &g_dbg.body_spec, 0.0f, 1.0f);
            ImGui::SliderFloat("fog density", &g_dbg.fog_density, 0.0f, 0.01f, "%.4f");
            ImGui::ColorEdit3("fog / sky colour", &g_dbg.fog_r);
        }
        if (ImGui::CollapsingHeader("Neon Underglow", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("neon on", (bool *)&g_dbg.neon_on);
            ImGui::ColorEdit3("neon colour", g_dbg.neon_col);
            ImGui::SliderFloat("intensity", &g_dbg.neon_str, 0.0f, 1.5f);
        }
        ImGui::EndTabItem();
    }

    /* ---- Tab 3: World & Entities ---- */
    if (ImGui::BeginTabItem("World & Entities")) {
        bool show_menu_hud = !g_dbg.hud_hide_menu;
        ImGui::Checkbox("show 3D HUD (menu + race)", &show_menu_hud);
        g_dbg.hud_hide_menu = !show_menu_hud;
        if (g_dbg.race_cars > 0)
            ImGui::Text("race: P%d/%d   lap %d/%d", g_dbg.race_pos, g_dbg.race_cars,
                        g_dbg.race_lap, g_dbg.race_laps);
        if (g_dbg.car_list && g_dbg.n_cars > 0) {
            static const char *items[64]; int n = g_dbg.n_cars < 64 ? g_dbg.n_cars : 64;
            for (int i = 0; i < n; i++) items[i] = g_dbg.car_list[i];
            int cur = g_dbg.sel_car;
            if (ImGui::Combo("car", &cur, items, n) && cur != g_dbg.sel_car) g_dbg.want_car = cur;
        } else ImGui::Text("car: %s (%d/%d)", g_dbg.car_name, g_dbg.sel_car+1, g_dbg.n_cars);
        if (g_dbg.track_list && g_dbg.n_tracks > 0) {
            static const char *items[64]; int n = g_dbg.n_tracks < 64 ? g_dbg.n_tracks : 64;
            for (int i = 0; i < n; i++) items[i] = g_dbg.track_list[i];
            int cur = g_dbg.sel_track;
            if (ImGui::Combo("track", &cur, items, n) && cur != g_dbg.sel_track) g_dbg.want_track = cur;
        } else ImGui::Text("track: %s (%d/%d)", g_dbg.track_name, g_dbg.sel_track+1, g_dbg.n_tracks);
        ImGui::Text("circuit: %d/%d   |   %d track meshes", g_dbg.sel_circuit+1, g_dbg.n_circuits, g_dbg.track_meshes);
        ImGui::Checkbox("Show UV Checker", (bool *)&g_dbg.show_uv_checker);
        ImGui::Separator();
        if (ImGui::CollapsingHeader("Scenery Semantics (asset names, 0x134011)", ImGuiTreeNodeFlags_DefaultOpen)) {
            static const char *scn[] = { "-","TERRAIN","BUILDING","PROP","TREE","WALL","STRUCT","OTHER" };
            int named = 0, tot = 0;
            for (int i = 0; i < 8; i++) { tot += g_dbg.scen_count[i]; if (i) named += g_dbg.scen_count[i]; }
            ImGui::Text("%d/%d meshes named (%.1f%%)", named, tot, tot ? 100.0f*named/tot : 0.0f);
            for (int i = 1; i < 8; i++) if (g_dbg.scen_count[i]) {
                ImGui::SameLine(); ImGui::TextDisabled("%s %d", scn[i], g_dbg.scen_count[i]); }
            ImGui::Separator();
            ImGui::Text("nearby world chunks (<=60 m):");
            ImGui::BeginChild("scnear", ImVec2(0, 150), true);
            for (int i = 0; i < g_dbg.scen_near_n; i++) ImGui::Text("%s", g_dbg.scen_near[i]);
            if (!g_dbg.scen_near_n) ImGui::TextDisabled("(none in range)");
            ImGui::EndChild();
        }
        if (ImGui::CollapsingHeader("Entity Definitions (0x39200, read-only)", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("%d ZCV_/ZCS_ defs from L4R*.BUN  [defs only, no placement]", g_dbg.scripted_count);
            ImGui::BeginChild("entdefs", ImVec2(0, 300), true);
            for (int i = 0; i < g_dbg.scripted_count; i++) {
                const ScriptedDef *e = &g_dbg.scripted[i];
                ImGui::Text("%-24s %08x  %5.1f x%5.1f x%5.1f", e->name, e->hash, e->w, e->l, e->h);
            }
            ImGui::EndChild();
        }
        ImGui::EndTabItem();
    }

    /* ---- Tab 4: Engine Telemetry ---- */
    if (ImGui::BeginTabItem("Engine Telemetry")) {
        ImGui::Text("%.1f FPS   %.2f ms/frame", ImGui::GetIO().Framerate,
                    1000.0f / (ImGui::GetIO().Framerate > 0 ? ImGui::GetIO().Framerate : 1));
        ImGui::Text("draw calls (meshes): %d   car %d   track %d",
                    g_dbg.drawn, g_dbg.car_meshes, g_dbg.track_meshes);
        ImGui::Separator();
        ImGui::Text("district: %-10s  (%d zones parsed)",
                    g_dbg.zone_name[0] ? g_dbg.zone_name : "-", g_dbg.zone_count);
        ImGui::Separator();
        ImGui::Text("camera XYZ  %.1f  %.1f  %.1f", g_dbg.cam[0], g_dbg.cam[1], g_dbg.cam[2]);
        ImGui::Text("car XYZ     %.1f  %.1f  %.1f", g_dbg.car[0], g_dbg.car[1], g_dbg.car[2]);
        ImGui::Text("heading %.2f rad   %.0f km/h", g_dbg.heading, g_dbg.kmh);
        ImGui::Separator();
        ImGui::Checkbox("Freecam (F)", (bool *)&g_dbg.freecam);
        ImGui::SetNextItemWidth(160);
        ImGui::SliderFloat("freecam speed", &g_dbg.speed, 0.05f, 3.0f);
        ImGui::EndTabItem();
    }

    ImGui::EndTabBar();
    }
    ImGui::End();

    /* ---- Minimap / Navigation Graph: the real drivable road network parsed
       from the per-region ROUTES path files (chunk 0x34148), drawn top-down
       in world XY. Independent of the 3D geometry viewer. ---- */
    ImGui::SetNextWindowSize(ImVec2(420, 460), ImGuiCond_FirstUseEver);
    ImGui::Begin("Minimap / Navigation Graph");
    ImGui::Text("%d nodes, %d edges", g_dbg.nnav, g_dbg.nnavedge);
    ImGui::SameLine();
    ImGui::TextDisabled("district %s", g_dbg.zone_name[0] ? g_dbg.zone_name : "-");
    if (g_dbg.nnav > 1) {
        ImVec2 p0 = ImGui::GetCursorScreenPos();
        ImVec2 avail = ImGui::GetContentRegionAvail();
        float side = avail.x < avail.y ? avail.x : avail.y;
        if (side < 80.0f) side = 80.0f;
        ImDrawList *dl = ImGui::GetWindowDrawList();
        dl->AddRectFilled(p0, ImVec2(p0.x+side, p0.y+side), IM_COL32(12,14,20,255));
        float x0=g_dbg.navbb[0], x1=g_dbg.navbb[1], y0=g_dbg.navbb[2], y1=g_dbg.navbb[3];
        float w = x1-x0, h = y1-y0, span = w > h ? w : h;
        if (span < 1.0f) span = 1.0f;
        /* world -> screen; world +Y is north, screen +Y is down, so flip Y */
        #define MAPX(X) (p0.x + ((X)-x0)/span*side)
        #define MAPY(Y) (p0.y + side - ((Y)-y0)/span*side)
        for (int e = 0; e < g_dbg.nnavedge; e++) {
            int a = g_dbg.navedge[e*2], b = g_dbg.navedge[e*2+1];
            dl->AddLine(ImVec2(MAPX(g_dbg.nav[a*2]), MAPY(g_dbg.nav[a*2+1])),
                        ImVec2(MAPX(g_dbg.nav[b*2]), MAPY(g_dbg.nav[b*2+1])),
                        IM_COL32(90,190,255,190), 1.0f);
        }
        /* player */
        float px = MAPX(g_dbg.car[0]), py = MAPY(g_dbg.car[1]);
        dl->AddCircleFilled(ImVec2(px, py), 4.0f, IM_COL32(255,80,60,255));
        float hx = px + cosf(g_dbg.heading)*11.0f;
        float hy = py - sinf(g_dbg.heading)*11.0f;   /* Y flipped */
        dl->AddLine(ImVec2(px,py), ImVec2(hx,hy), IM_COL32(255,220,90,255), 2.0f);
        ImGui::Dummy(ImVec2(side, side));
        ImGui::TextDisabled("X[%.0f..%.0f] Y[%.0f..%.0f]  car (%.0f, %.0f)",
                            x0, x1, y0, y1, g_dbg.car[0], g_dbg.car[1]);
        #undef MAPX
        #undef MAPY
    } else ImGui::TextDisabled("no navigation data loaded");
    ImGui::End();
}

extern "C" void dbgui_render(void) {
    ImGui::Render();
    /* the app binds its shader program once and never rebinds; the GL2 backend
       is fixed-function, so unbind the program around it and restore after. */
    GLint prev = 0; glGetIntegerv(GL_CURRENT_PROGRAM, &prev);
    glUseProgram(0);
    /* the GL2 backend draws with client-side arrays: any VBO the app left bound
       would make glVertexPointer read garbage, and attrib 0 aliases gl_Vertex. */
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glDisableVertexAttribArray(0);
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
    glUseProgram(prev);
}

extern "C" void dbgui_shutdown(void) {
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
}
