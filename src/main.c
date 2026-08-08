/* main.c — OpenUG: an open reimplementation of Need for Speed: Underground 2.
 * Loads the original game data directly (no Wine/box64), assembles a track
 * section, decodes its textures, and drives a textured car with AI opponents
 * around a circuit — SDL2 + OpenGL, portable across x86 and ARM.
 *
 * main.c is the orchestrator: setup, the game loop, input, race flow and HUD.
 * The engine proper lives in the modules it drives:
 *   nfsu2.h    — chunk parser (ground truth — do not "optimize")
 *   render.*   — Renderer: GL objects, shaders, matrices, font, screenshot
 *   physics.*  — car kinematics, wall + car-to-car collision
 *   ai.*       — racing-line opponents, circuit loading
 *   audio.*    — procedural engine/road/skid synth
 *   resource.* — file mapping + track/car/circuit discovery
 *
 * Desktop (x86/ARM, Linux/macOS/Windows): legacy GL 2.1 + GLSL 120.
 * Embedded / mobile (ARM): OpenGL ES 2.0 + GLSL 100 — build with -DN2_GLES
 * (links -lGLESv2). The shaders are written to compile on both.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <unistd.h>   /* execvp: menu track-switch re-launches the process */
#include <SDL.h>

#include "nfsu2.h"
#include "render.h"
#include "physics.h"
#include "ai.h"
#include "audio.h"
#include "resource.h"
#include "world.h"
#include "world_mesh.h"   /* F3 prelight/normal/wireframe debug pipeline */
#include "debug.h"

/* debug tunables — defaults match the previously hard-coded constants, so a
 * normal build behaves exactly as before; `make debug` adds an ImGui panel. */
DbgState g_dbg = {
    .freecam = 0, .speed = 0.6f,
    .race_maxlaps_want = 2,
    /* .wheel (VehicleWheelConfig) is filled per car at load by wheel_config_for()
       -- an explicit table entry or a body-box fallback -- so no default here. */
    .wheel_scale = 1.0f,
    .insp_sel = -1,
    .neon_on = 1, .neon_col = { 0.15f, 0.45f, 1.0f }, .neon_str = 0.85f,
    .rim_paint = 1, .rim_color = { 0.85f, 0.88f, 0.92f },   /* silver by default */
    .tune_accel = 1.0f, .tune_brake = 1.0f, .tune_turn = 1.0f, .tune_top = 220.0f,
    .ambient = 0.38f, .diffuse = 0.62f, .body_spec = 0.50f,   /* glossy paint */
    .body_env = 1.0f,                                          /* 1.0 = tuned clearcoat reflection */
    .vcolor = 1.0f,   /* full source prelight on world geometry */
    /* f(700m cull range) ~= 0.07 — far batches dissolve into the sky */
    .fog_density = 0.0023f, .fog_r = 0.06f, .fog_g = 0.07f, .fog_b = 0.11f,
    .paint_override = 0, .paint = { 0.68f, 0.09f, 0.08f },
    .show_body = 1, .show_glass = 1, .show_lights = 1, .show_tires = 1,
    .show_misc = 1, .show_track = 1,
    .hud_hide_menu = 1,   /* only consulted under DEBUG_UI; see debug.h */
};

/* ---- Per-car wheel stance ------------------------------------------------
 * Primary source is the GLOBAL AttribSys car table (n2_global_wheel_attr): the
 * exact factory axle X and track Y the game itself ships, decoded per car with
 * no hash and no hand-tuned table. Cars with no GLOBAL record (or when GLOBAL is
 * absent) fall back to a one-time seed from their own measured body box. Hub Z
 * stays the exact tyre-mesh measurement either way. The ImGui "Wheel Stance"
 * sliders still edit the active car's live copy. */
#define WHEEL_SEED_FRONTF 0.639f   /* fallback-only: fraction of front body extent */
#define WHEEL_SEED_REARF  0.597f   /* fallback-only: fraction of rear body extent  */
#define WHEEL_SEED_TRACKF 0.760f   /* fallback-only: fraction of half-width -> track */
static VehicleWheelConfig wheel_config_for(const char *name, const N2CarProfile *p,
                                           const unsigned char *gdata, long glen, int *from_global) {
    VehicleWheelConfig c;
    N2WheelAttr wa;
    if (n2_global_wheel_attr(gdata, glen, name, &wa)) {   /* exact, from GLOBAL AttribSys */
        c.front_axle = wa.front_axle; c.rear_axle = wa.rear_axle;
        c.front_track = wa.front_track; c.rear_track = wa.rear_track;
        c.ride_y = p->hub_z;
        if (from_global) *from_global = 1;
        return c;
    }
    c.front_axle  = p->body[1] * WHEEL_SEED_FRONTF;   /* fallback: this car's own body box */
    c.rear_axle   = p->body[0] * WHEEL_SEED_REARF;
    c.front_track = c.rear_track = 2.0f * p->body[3] * WHEEL_SEED_TRACKF;
    c.ride_y      = p->hub_z;
    if (from_global) *from_global = 0;
    return c;
}

/* Switching car or track means a whole new asset load (world batches, car
 * buffers/textures, physics obstacles, AI paths — ~30 pieces of long-lived
 * state with no teardown path), so both the menu's arrow keys and the
 * ImGui car/track combos below trigger the SAME clean re-exec rather than
 * an in-process hot-swap: the OS reclaims everything, so there is nothing
 * to leak or get left dangling, which an untested from-scratch teardown/
 * reload path could easily do quietly. */
/* Load one aftermarket rim style out of a wheel-brand archive and upload it.
 * The archive holds several size/width variants per style; the first mesh is
 * used. Frees any previously uploaded rim. Returns 1 on success. */
/* Drop triangles that weld two submesh runs together.
 *
 * What these triangles ACTUALLY are (Phase 49, re-measured -- two earlier
 * versions of this comment were wrong). They are NOT a grouping artifact:
 *   - The 0x134B02 index runs are clean. Every run's index count is a
 *     multiple of 3 and the runs are chained (start = prev start + count),
 *     so grouping the whole buffer in 3s from offset 0 never crosses a run
 *     boundary. There is no "tail bridges into the next run" bug.
 *   - The long triangles are genuine source geometry: a flat, axis-aligned
 *     quad (2 tris, 4 dedicated verts) at CONSTANT Y = 0.18, spanning the
 *     full rim diameter in X and Z. Verified identical across BBS/ENKEI/VOLK/
 *     OZ/ADVAN/LEXANI/WORK/RACINGHART/GIOVANNA/NFSU. It is the rim's flat
 *     backing/hub plane, hidden behind the spokes and occluded by the tyre
 *     and brake when mounted -- so dropping it is invisible, and keeping it
 *     would z-fight against the brake disc the engine draws separately.
 *     Measured: exactly 2 of 487 tris on BBS style 1 (LEXANI has 4 = two
 *     quads); the rest are clean spoke surface (mean edge 0.065 vs 0.85 diam).
 *
 * So the fix is a genuine-geometry cull, not a parser change: a run-boundary
 * splitter would keep these tris (they sit correctly inside one run), which
 * is why that approach was NOT taken. Dropping them here, before upload_scene,
 * means they never reach a VBO -- the VRAM is already optimal.
 *
 * Kept out of n2_add_pair on purpose: that path is shared by all 57 cars, and
 * this archive is the only place the condition arises. Purely local, and
 * geometric (edge > 0.25*diag) rather than name-based, so it is self-limiting
 * and cannot misfire on a legitimately large triangle in a small mesh. */
static void rim_drop_welded_mesh(N2Mesh *m) {
    float bb[6]; n2_mesh_bbox(m, bb);
    float dx = bb[1]-bb[0], dy = bb[3]-bb[2], dz = bb[5]-bb[4];
    float diag = sqrtf(dx*dx + dy*dy + dz*dz);
    if (diag <= 0.0f) return;
    float lim = 0.25f * diag, lim2 = lim * lim;
    int w = 0;
    for (int t = 0; t + 2 < m->nidx; t += 3) {
        int ok = 1;
        for (int k = 0; k < 3 && ok; k++) {
            const float *p = m->verts + m->idx[t+k]*5;
            const float *q = m->verts + m->idx[t+(k+1)%3]*5;
            float ex=p[0]-q[0], ey=p[1]-q[1], ez=p[2]-q[2];
            if (ex*ex+ey*ey+ez*ez > lim2) ok = 0;
        }
        if (ok) { m->idx[w]=m->idx[t]; m->idx[w+1]=m->idx[t+1]; m->idx[w+2]=m->idx[t+2]; w += 3; }
    }
    m->nidx = w;
}
static void rim_drop_welded_tris(N2Scene *s) {
    for (int i = 0; i < s->count; i++) rim_drop_welded_mesh(&s->meshes[i]);
}

static int load_rim_style(const unsigned char *wldata, long wllen,
                          const uint32_t *wkeys, int nwkeys, int style,
                          N2Scene *lib, GpuMesh **gm, int *ngm,
                          const unsigned char *wtdata, long wtlen,
                          GLuint *rimtex, float fitR) {
    if (!wldata) { (void)wllen; return 0; }
    for (int i = 0; i < *ngm; i++) {
        glDeleteBuffers(1, &(*gm)[i].vbo);
        glDeleteBuffers(1, &(*gm)[i].nbo);
        glDeleteBuffers(1, &(*gm)[i].ibo);
    }
    free(*gm); *gm = NULL; *ngm = 0;
    n2_free_scene(lib);
    N2CarConfig wcfg = { 0, style, 0, style };   /* STYLEnn selects the rim */
    int n = n2_load_car(wldata, wllen, lib, wkeys, nwkeys, &wcfg);
    if (n <= 0) return 0;
    rim_drop_welded_tris(lib);
    /* Fit the aftermarket rim to THIS car's wheel. The library rims are one
       fixed tuner size (~0.42 radius); a Hummer's arch is far bigger, so the
       rim floated tiny inside it. Scale every rim submesh (they share the
       origin) to the car's own stock-wheel radius fitR, which is measured from
       the car's N2_CAR_TIRE mesh -- the same size the procedural tyre uses, so
       the two stay consistent when the draw swaps between them at speed. */
    if (fitR > 0.0f && lib->count) {
        float bb[6]; n2_mesh_bbox(&lib->meshes[0], bb);
        float rimR = 0.25f * ((bb[1]-bb[0]) + (bb[5]-bb[4]));   /* mean X-Z radius */
        if (rimR > 1e-4f) {
            float sc = fitR / rimR;
            for (int mi = 0; mi < lib->count; mi++) {
                N2Mesh *mm = &lib->meshes[mi];
                for (int v = 0; v < mm->nverts; v++)
                    for (int a = 0; a < 3; a++) mm->verts[v*5+a] *= sc;
            }
            printf("  rim fit: rimR %.3f -> car wheel %.3f (x%.2f)\n", rimR, fitR, sc);
        }
    }
    *gm = upload_scene(lib); *ngm = n;

    /* Bind the rim's own diffuse out of WHEELS/TEXTURES.BIN. Every submesh of
       a given style carries the SAME key (measured across BBS/ENKEI/VOLK/OZ/
       LEXANI/WORK/ADVAN), so one texture covers the whole rim -- hence a
       single GLuint rather than a per-mesh map like the car body uses. */
    if (*rimtex) { glDeleteTextures(1, rimtex); *rimtex = 0; }
    uint32_t tk = lib->count ? lib->meshes[0].texkey : 0;
    N2Tex rt; memset(&rt, 0, sizeof rt);
    long ar=0, ag=0, ab=0, bmn=255, bmx=0;
    if (tk && wtdata && n2_load_car_tex_by_key(wtdata, wtlen, tk, &rt)) {
        *rimtex = upload_tpk_texture_to_gpu(&rt);
        /* rim sheets are atlases with UVs in [0,1]: clamp so REPEAT wrap +
           mip filtering cannot bleed the opposite border in (same reason as
           the car body atlases above). */
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        long np = (long)rt.w * rt.h;
        for (long p = 0; p < np; p++) { int b = rt.rgb[p*3+2];
            ar += rt.rgb[p*3]; ag += rt.rgb[p*3+1]; ab += b;
            if (b < bmn) bmn = b; if (b > bmx) bmx = b; }
        if (np) { ar/=np; ag/=np; ab/=np; }
        free(rt.rgb); free(rt.alpha); free(rt.dxt);
    }
    /* channel telemetry: B min/max spanning 0..255 confirms the decode is NOT
       truncating blue -- the low average is an authentic gold/bronze rim. */
    printf("  rim diffuse: texkey %08x -> %s (%dx%d) avg RGB %ld,%ld,%ld  B[min %ld..max %ld]\n",
           tk, *rimtex ? "bound" : "UNRESOLVED, procedural fallback",
           *rimtex ? rt.w : 0, *rimtex ? rt.h : 0, ar, ag, ab, bmn, bmx);
    return 1;
}

static void relaunch(const char *selfexe, const char *dataroot,
                     const char *car, const char *track) {
    char *na[8]; int a = 0;
    na[a++] = (char *)selfexe; na[a++] = (char *)dataroot;
    na[a++] = "--car";   na[a++] = (char *)car;
    na[a++] = "--track"; na[a++] = (char *)track; na[a] = NULL;
    SDL_Quit(); execvp(selfexe, na);
    _exit(1);   /* only reached if execvp itself failed */
}

/* ---- --carinfo <CAR>: GL-free vehicle geometry + texture inspection ---- */
static const char *car_cat_name(int c) {
    switch (c) {
        case N2_CAR_BODY: return "BODY";  case N2_CAR_GLASS: return "GLASS";
        case N2_CAR_LIGHT: return "LIGHT"; case N2_CAR_TIRE: return "TIRE";
        case N2_CAR_MISC: return "MISC";  case N2_CAR_BRAKELIGHT: return "BRAKELIGHT";
        case N2_CAR_MECH: return "MECH";  default: return "OTHER";
    }
}
static void car_info_walk(const unsigned char *d, long beg, long end,
                          long *nobj, long *cat) {
    long o = beg;
    while (o + 8 <= end) {
        uint32_t m = n2_u32(d + o), s = n2_u32(d + o + 4); long ds = o + 8;
        if (m == 0x80134010u) {
            char nm[48]; n2_mesh_name(d, ds, ds + s, nm, sizeof nm);
            int c = n2_car_category(d, ds, ds + s);
            N2Leaf vtx[64], idx[64]; int nv = 0, ni = 0;
            n2_find_leaves(d, ds, ds + s, 0x00134B01u, vtx, &nv, 64);
            n2_find_leaves(d, ds, ds + s, 0x00134B03u, idx, &ni, 64);
            long verts = 0, tris = 0;
            for (int k = 0; k < nv; k++) {
                int pad = n2_skip_filler(d + vtx[k].off, (int)vtx[k].size);
                int b = (int)vtx[k].size - pad; if (b > 0 && b % 36 == 0) verts += b / 36;
            }
            for (int k = 0; k < ni; k++) {
                const unsigned char *ib = d + idx[k].off; int ip = 0, ib2 = (int)idx[k].size;
                while (ip + 2 <= ib2 && ib[ip] == 0x11 && ib[ip+1] == 0x11) ip += 2;
                tris += (ib2 - ip) / 2 / 3;
            }
            float M[16]; int hasM = n2_obj_matrix(d, ds, ds + s, M);
            printf("  %-34s %-10s verts=%-5ld tris=%-5ld  mat=%d t=(%+.3f %+.3f %+.3f)\n",
                   nm[0] ? nm : "(noname)", car_cat_name(c), verts, tris,
                   hasM, M[12], M[13], M[14]);
            /* Running-gear parts (wheel + brake disc) carry the axle position IF
               they are modelled in place. Print their vertex bbox centre to show
               they are NOT: every one sits at the model origin, so GEOMETRY.BIN
               holds no per-axle wheelbase/track -- see the N2CarProfile audit. */
            if ((strstr(nm, "BRAKE") && !strstr(nm, "LIGHT")) || strstr(nm, "WHEEL")) {
                float b0[3] = {1e30f,1e30f,1e30f}, b1[3] = {-1e30f,-1e30f,-1e30f};
                for (int k = 0; k < nv; k++) {
                    int pad = n2_skip_filler(d + vtx[k].off, (int)vtx[k].size);
                    int nvv = ((int)vtx[k].size - pad) / 36;
                    for (int q = 0; q < nvv; q++) {
                        const unsigned char *vp = d + vtx[k].off + pad + q*36;
                        for (int a = 0; a < 3; a++) { float f; memcpy(&f, vp + a*4, 4);
                            if (f < b0[a]) b0[a] = f; if (f > b1[a]) b1[a] = f; } } }
                printf("      bbox centre (%+.3f %+.3f %+.3f)  <- at origin: no axle offset stored\n",
                       0.5f*(b0[0]+b1[0]), 0.5f*(b0[1]+b1[1]), 0.5f*(b0[2]+b1[2]));
            }
            (*nobj)++; if (c >= 0 && c < 24) cat[c]++;
        } else if (m != 0 && (m >> 28) == 8) {
            car_info_walk(d, ds, ds + s, nobj, cat);
        }
        o = ds + s;
    }
}
static int dump_car_info(const char *dataroot, const char *car) {
    char gp[1024], tp[1024];
    snprintf(gp, sizeof gp, "%s/CARS/%s/GEOMETRY.BIN", dataroot, car);
    snprintf(tp, sizeof tp, "%s/CARS/%s/TEXTURES.BIN", dataroot, car);
    long gn = 0; unsigned char *g = n2_read_file(gp, &gn);
    if (!g) { fprintf(stderr, "--carinfo: cannot read %s\n", gp); return 1; }
    long cat[24] = {0}, nobj = 0;
    printf("=== %s (%ld KB) : vehicle parts ===\n", gp, gn >> 10);
    car_info_walk(g, 0, gn, &nobj, cat);
    printf("  %ld part objects.  by category:", nobj);
    for (int i = 10; i <= 16; i++) if (cat[i]) printf(" %s=%ld", car_cat_name(i), cat[i]);
    printf("\n\n");
    long tn = 0; unsigned char *t = n2_read_file(tp, &tn);
    if (!t) { fprintf(stderr, "--carinfo: cannot read %s\n", tp); free(g); return 0; }
    uint32_t keys[512]; int nk = n2_car_tex_keys(t, tn, keys, 512);
    printf("=== %s (%ld KB) : %d car textures ===\n", tp, tn >> 10, nk);
    for (int i = 0; i < nk; i++) {
        N2Tex tex;
        if (n2_load_car_tex_by_key(t, tn, keys[i], &tex)) {
            printf("  key=0x%08X  %4dx%-4d  %s\n", keys[i], tex.w, tex.h,
                   tex.alpha ? "DXT3 (alpha)" : "DXT1");
            free(tex.rgb); free(tex.alpha); free(tex.dxt);
        } else printf("  key=0x%08X  (decode failed)\n", keys[i]);
    }
    free(g); free(t);
    return 0;
}

int main(int argc, char **argv) {
    collide_walls_selftest();
    phys_selftest();
    audio_selftest();
    /* Point at your own NFSU2 install/data directory (contains TRACKS/, CARS/).
       Usage: nfsu2 [DATA_DIR] [options]
         --car NAME       car folder under CARS/ (default HUMMER)
         --track NAME     STREAM .BUN under TRACKS/, or ALL = whole city (default).
                          The 8 region bundles share ONE coord space; world_dedup
                          fuses the byte-identical re-ships into one seamless map.
                          (There is no STREAMALL.BUN; ALL = every STREAM<region>.)
         --circuit PATH   circuit Paths .bin under TRACKS/ (default ROUTESL4RF/Paths4602.bin)
         --shot out.png   render one frame and exit
         --carinfo CAR    dump CAR's part list + texture catalog and exit (GL-free) */
    const char *selfexe = argv[0];   /* for the menu's track-switch re-exec */
    const char *dataroot = ".", *shot = NULL, *objdump = NULL, *carinfo = NULL;
    const char *carname = "HUMMER", *trackname = "ALL";
    const char *circuit = "ROUTESL4RF/Paths4602.bin"; int explicit_circuit = 0;
    int want_event_id = 0;   /* --event <id>: boot straight into a race event */
    int shotframes = 40;     /* --frames N: how long --shot drives before the grab */
    int want_laps = 2;       /* --laps N: race distance for --event */
    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--shot")    && i+1 < argc) shot      = argv[++i];
        else if (!strcmp(argv[i], "--car")     && i+1 < argc) carname   = argv[++i];
        else if (!strcmp(argv[i], "--event")   && i+1 < argc) want_event_id = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--frames")  && i+1 < argc) shotframes = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--laps")    && i+1 < argc) want_laps = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--track")   && i+1 < argc) trackname = argv[++i];
        else if (!strcmp(argv[i], "--circuit") && i+1 < argc) { circuit = argv[++i]; explicit_circuit = 1; }
        else if (!strcmp(argv[i], "--objdump") && i+1 < argc) objdump = argv[++i];
        else if (!strcmp(argv[i], "--carinfo") && i+1 < argc) carinfo = argv[++i];
        else dataroot = argv[i];
    }
    if (carinfo) return dump_car_info(dataroot, carinfo);   /* inspect one car, GL-free, exit */
    char carp[1024], cartexp[1024], pathp[1024], troot[1024];
    snprintf(troot,   sizeof troot,   "%s/TRACKS", dataroot);
    snprintf(carp,    sizeof carp,    "%s/CARS/%s/GEOMETRY.BIN", dataroot, carname);
    snprintf(cartexp, sizeof cartexp, "%s/CARS/%s/TEXTURES.BIN", dataroot, carname);
    snprintf(pathp,   sizeof pathp,   "%s/TRACKS/%s", dataroot, circuit);

    static World world;
    int nm = world_load(&world, troot, trackname);

    /* --objdump: write the exact post-dedup, world-space scene the GPU batches
       are built from to a .obj, then exit. Independent parser-vs-renderer check:
       if this opens coherent in Blender/MeshLab, the assembly is sound and any
       on-screen mangling is strictly the GL path or camera. Runs before any GL. */
    if (objdump) {
        FILE *of = fopen(objdump, "w");
        if (!of) { fprintf(stderr, "objdump: cannot write %s\n", objdump); return 1; }
        const N2Scene *s = &world.scene;
        fprintf(of, "# OpenUG scene export  track=%s  meshes=%d (post-dedup, world space)\n",
                trackname, s->count);
        long base = 1, tv = 0, tf = 0;   /* OBJ is 1-indexed */
        for (int mi = 0; mi < s->count; mi++) {
            const N2Mesh *m = &world.scene.meshes[mi];
            fprintf(of, "o m%d_%.31s\n", mi, m->sname[0] ? m->sname : "mesh");
            for (int v = 0; v < m->nverts; v++) {
                const float *p = m->verts + v*5;
                fprintf(of, "v %.3f %.3f %.3f\n", p[0], p[1], p[2]);
                fprintf(of, "vt %.4f %.4f\n", p[3], p[4]);
            }
            for (int t = 0; t + 2 < m->nidx; t += 3) {
                long a = base + m->idx[t], b = base + m->idx[t+1], c = base + m->idx[t+2];
                fprintf(of, "f %ld/%ld %ld/%ld %ld/%ld\n", a,a, b,b, c,c);
            }
            base += m->nverts; tv += m->nverts; tf += m->nidx/3;
        }
        fclose(of);
        printf("objdump: wrote %s  (%ld verts, %ld tris, %d meshes)\n",
               objdump, tv, tf, s->count);
        return 0;
    }
    if (!nm) { fprintf(stderr, "no track data\n  (pass your NFSU2 data dir: nfsu2 /path/to/data)\n"); return 1; }
    N2Scene scene = world.scene;   /* shares world.scene.meshes */
    printf("loaded %d submeshes from %d region(s)\n", nm, world.nreg);
    printf("terrain texture TRN_GRASSC: %s\n", world.have_grass ? "ok" : "-");
    int nroad = 0, nterr = 0;
    for (int i = 0; i < nm; i++) {
        if (scene.meshes[i].cat == N2_ROAD) nroad++;
        else if (scene.meshes[i].cat == N2_TERRAIN) nterr++;
    }
    printf("categories: road=%d terrain=%d other=%d\n", nroad, nterr, nm-nroad-nterr);
    /* scenery semantics from each mesh's own asset name (0x134011) */
    {   int sc[8] = {0}, named = 0;
        for (int i = 0; i < nm; i++) { int c2 = scene.meshes[i].scen;
            if (c2 < 8) sc[c2]++; if (c2) named++; }
        printf("scenery: %d/%d named (%.1f%%)", named, nm, nm ? 100.0*named/nm : 0.0);
        for (int c2 = 1; c2 <= N2_SC_OTHER; c2++)
            if (sc[c2]) printf("  %s=%d", n2_scen_name(c2), sc[c2]);
        printf("\n");
        for (int i = 0, shown = 0; i < nm && shown < 3; i++)
            if (scene.meshes[i].scen == N2_SC_PROP) { float bb[6];
                n2_mesh_bbox(&scene.meshes[i], bb); shown++;
                printf("  join sample: %-26s [%s]  bbox X[%.1f..%.1f] Y[%.1f..%.1f] Z[%.1f..%.1f]\n",
                       scene.meshes[i].sname, n2_scen_name(scene.meshes[i].scen),
                       bb[0],bb[1],bb[2],bb[3],bb[4],bb[5]); }
    }

    /* scene centroid + extent for the camera */
    float cx=0,cy=0,cz=0; long cnt=0;
    float mn[3]={1e30f,1e30f,1e30f}, mx[3]={-1e30f,-1e30f,-1e30f};
    for (int i=0;i<nm;i++) for (int v=0;v<scene.meshes[i].nverts;v++) {
        float *p = scene.meshes[i].verts + v*5;
        cx+=p[0]; cy+=p[1]; cz+=p[2]; cnt++;
        for (int c=0;c<3;c++){ if(p[c]<mn[c])mn[c]=p[c]; if(p[c]>mx[c])mx[c]=p[c]; }
    }
    if (cnt){ cx/=cnt; cy/=cnt; cz/=cnt; }
    float maxr = 1;
    for (int c=0;c<3;c++) if ((mx[c]-mn[c])/2 > maxr) maxr = (mx[c]-mn[c])/2;

    /* densest built-up spot: bin building (N2_OTHER) mesh positions into a grid
       and take the peak cell. The geometry centroid is often an open hole (the
       city spreads in a ring), so this is where to start the car for a view
       that actually frames buildings. Falls back to the centroid if no props. */
    float densx = cx, densy = cy;
    {
        static int hist[40][40]; memset(hist, 0, sizeof hist);
        float gw = (mx[0]-mn[0])/40.0f, gh = (mx[1]-mn[1])/40.0f;
        if (gw > 1e-3f && gh > 1e-3f) {
            for (int i=0;i<nm;i++) if (scene.meshes[i].cat == N2_OTHER && scene.meshes[i].nverts) {
                float *p = scene.meshes[i].verts;          /* first vertex ~ mesh location */
                int gx=(int)((p[0]-mn[0])/gw), gy=(int)((p[1]-mn[1])/gh);
                if (gx<0)gx=0; if (gx>39)gx=39; if (gy<0)gy=0; if (gy>39)gy=39;
                hist[gx][gy]++;
            }
            int best=0, bx=20, by=20;
            for (int gx=0;gx<40;gx++) for (int gy=0;gy<40;gy++)
                if (hist[gx][gy] > best) { best=hist[gx][gy]; bx=gx; by=gy; }
            if (best > 0) { densx = mn[0]+(bx+0.5f)*gw; densy = mn[1]+(by+0.5f)*gh; }
        }
    }
    printf("dense build-up centre: (%.0f,%.0f)\n", densx, densy);

    /* building collision footprints — the car is kept out of these */
    #define MAXOBST 32768   /* whole city worth of building footprints */
    static float obst[MAXOBST][4];
    int nobst = phys_collect_walls(&scene, obst, MAXOBST);
    printf("collision obstacles: %d buildings\n", nobst);

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) != 0) { fprintf(stderr, "SDL: %s\n", SDL_GetError()); return 1; }

    /* engine sound, most to least authentic: the car's own Gnsu20 sweep
       recordings (matched by name), else an .abk sample bank (name-hash
       pick), else the procedural synth. */
    int nsweep = audio_load_ginsu_sweeps(dataroot, carname);
    int engbank = -1, nloops = 0;
    if (!nsweep) {
        engbank = audio_bank_for_car(carname);
        nloops = audio_load_engine_bank(dataroot, engbank);
        if (!nloops && engbank != 0) { engbank = 0; nloops = audio_load_engine_bank(dataroot, 0); }
    }
    SDL_AudioDeviceID adev = audio_init();
    printf("engine audio: %s (%s)\n", adev ? "on" : "unavailable",
           nsweep ? "Gnsu20 sweeps" : nloops ? "game sample bank" : "procedural synth");
    if (nsweep) printf("engine sweeps: %d for %s (%.0f-%.0f rpm)\n", nsweep, carname,
                       g_engine.gin.accel_sweep.rpm_min, g_engine.gin.accel_sweep.rpm_max);
    else if (nloops) printf("engine bank: CAR_%02d for %s (%d loops incl idle)\n",
                            engbank, carname, nloops);
#ifdef N2_GLES
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
#endif
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_Window *win = SDL_CreateWindow("OpenUG",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 960, 600,
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    SDL_GLContext ctx = SDL_GL_CreateContext(win);
    if (!ctx) { fprintf(stderr, "GL ctx: %s\n", SDL_GetError()); return 1; }
    SDL_GL_SetSwapInterval(shot ? 0 : 1);   /* raw frame times in shot mode */
    /* Detect S3TC so car/rim TPK textures can upload their DXT blocks directly
       (glCompressedTexImage2D) instead of the CPU-decoded RGBA. Legacy GL 2.1
       and GLES2 both return a valid GL_EXTENSIONS string here. */
    { const char *ext = (const char *)glGetString(GL_EXTENSIONS);
      g_tex_s3tc = ext && (strstr(ext, "GL_EXT_texture_compression_s3tc") ||
                           strstr(ext, "texture_compression_dxt1")); }
    printf("texture path: %s\n", g_tex_s3tc ? "S3TC direct (glCompressedTexImage2D)"
                                            : "CPU DXT decode (no s3tc extension)");
#ifdef DEBUG_UI
    dbgui_init(win, ctx);
#endif

    RProg rp = render_program();
    GLint uMVP = rp.uMVP, uUseTex = rp.uUseTex, uColor = rp.uColor,
          uUnlit = rp.uUnlit, uAlpha = rp.uAlpha, uSoft = rp.uSoft,
          uSpec = rp.uSpec, uAmbient = rp.uAmbient, uDiffuse = rp.uDiffuse,
          uLight = rp.uLight, uGloss = rp.uGloss;

    /* Per-mesh track textures: each mesh's diffuse key (0x134012) resolves to a
       texture in the region's own STREAM TPK (grass/road/props) or the shared
       LOC4DYNTEX pack (sky/facades). Decode each distinct key once, reject any
       that decode to noise (wrong format / swizzled surface), map key->GL tex.
       Generalises the old hard-coded TRN_GRASSC/RDP_PARKING lookup across
       regions (L4RR uses ORG_GRASS_001 etc.). */
    static uint32_t tmapkey[2048]; static GLuint tmaptex[2048];
    int ntmap = world_bind_textures(&world, tmapkey, tmaptex, 2048);
    printf("track textures bound: %d distinct\n", ntmap);

    /* GPS self-check: route across the city so a broken graph is loud at load. */
    if (world.nnav > 0 && world.ndist >= 2) {
        int a = -1, b = -1;
        for (int i = 0; i < world.ndist; i++) {
            if (!strcmp(world.dist[i].tok, "UC")) a = i;
            if (!strcmp(world.dist[i].tok, "IP")) b = i;
        }
        if (a < 0) a = 0;
        if (b < 0) b = world.ndist - 1;
        int s0 = world_nav_nearest(&world, world.dist[a].cx, world.dist[a].cy);
        int g0 = world_nav_nearest(&world, world.dist[b].cx, world.dist[b].cy);
        static int path[8192]; float dist = 0;
        uint32_t t0 = SDL_GetTicks();
        int n = world_route(&world, s0, g0, path, 8192, &dist);
        uint32_t t1 = SDL_GetTicks();
        printf("GPS test %s (%s) -> %s (%s): %d nodes, %.0f m, %u ms\n",
               world.dist[a].tok, world_district_name(world.dist[a].tok),
               world.dist[b].tok, world_district_name(world.dist[b].tok),
               n, dist, t1 - t0);
    }

    /* Boot straight into a race event if asked (--event <id>). The Phase 71/72
       load-time A-star and checkpoint self-checks that used to run here were
       removed: they asserted on load and aborted for small events (e.g. 4301,
       where the on-circuit sample finds no reroutable pair) and flooded the
       console -- exactly the automated race telemetry the manual-testing
       protocol drops. The barrier/checkpoint logic in world.c is unchanged. */
    if (want_event_id && world.nev > 0) {
        int wi = -1;
        for (int i = 0; i < world.nev; i++) if (world.ev[i].id == want_event_id) wi = i;
        if (wi < 0) fprintf(stderr, "--event %d: no such race event\n", want_event_id);
        else world_race_start(&world, troot, wi, want_laps);
    }

    /* Scripted-object entity DEFINITIONS from the district companion L4R*.BUN.
       Read-only decode for the inspector; the data has no world placement or
       mesh (Phase 50), so these are logged as a table, not instantiated. */
    static ScriptedDef sdefs[256];
    int nsd = world_scripted_defs(&world, troot, sdefs, 256);
    printf("scripted entity defs: %d  [definitions only - no placement in data]\n", nsd);
    for (int i = 0; i < nsd; i++)
        printf("  %-24s %08x  %6.1f x%6.1f x%6.1f\n",
               sdefs[i].name, sdefs[i].hash, sdefs[i].w, sdefs[i].l, sdefs[i].h);
    /* resolve each mesh's texture once — the per-frame key scan was fine for
       one region, not for a whole city of meshes */
    GLuint *mtex = (GLuint *)calloc(nm, sizeof *mtex);
    for (int i = 0; i < nm; i++)
        for (int j = 0; j < ntmap; j++)
            if (tmapkey[j] == scene.meshes[i].texkey) { mtex[i] = tmaptex[j]; break; }
    /* load a car and drop it on the track (36-byte vertex format) */
    long clen; unsigned char *cdata = n2_read_file(carp, &clen);
    long ctlen; unsigned char *ctdata = n2_read_file(cartexp, &ctlen);
    /* per-mesh car textures: get the TPK keys, then decode each key referenced
       by a mesh into its own GL texture (body, wheel, brake, ... bound by UVs). */
    uint32_t ckeys[64]; int nck = ctdata ? n2_car_tex_keys(ctdata, ctlen, ckeys, 64) : 0;
    uint32_t mapkey[32]; GLuint maptex[32]; char mapalpha[32]; int nmap = 0;
    N2Scene car; int ncar = 0; GpuMesh *cgm = NULL;
    int stock_wheel = -1;   /* cgm[] index of the car's own highest-LOD stock wheel mesh */
    float wheelT[4][16];                         /* 4 wheel placements (car-local) */
    float wheelTAI[4][16];                       /* same, minus the player's steer (AI cars) */
    GpuMesh wheelmesh; int have_wheel = 0;       /* procedural tyre, built after GL init */
    for (int k=0;k<4;k++){ float I[16]={1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1}; memcpy(wheelT[k],I,sizeof(I)); }
    float carbb[6] = {0,0,0,0,0,0};              /* body AABB min/max, for wheel placement */
    float carWheelR = 0.0f;                       /* car's stock wheel radius (rim fit) */
    N2CarProfile carprof; memset(&carprof, 0, sizeof carprof);   /* per-car dimensions */
    N2CarConfig carcfg = { 0, 0, 0, 0 };         /* active customization profile (K cycles kits) */
    float spawn[3] = { cx, cy, cz }, heading0 = 0.0f;
    if (cdata) {
        ncar = n2_load_car(cdata, clen, &car, ckeys, nck, &carcfg);
        /* Wheels are modelled once at the origin (the SolidObject transform is
           identity for every part — verified), so a lone wheel mesh renders
           buried in the body. Place it at 4 arch positions derived from the body
           AABB. ponytail: fractions, since no explicit wheel data exists in
           GEOMETRY.BIN / PARTS_ANIMATIONS.bin — tune here if a car sits wrong. */
        float bb0[3]={1e30f,1e30f,1e30f}, bb1[3]={-1e30f,-1e30f,-1e30f};
        float tb0[3]={1e30f,1e30f,1e30f}, tb1[3]={-1e30f,-1e30f,-1e30f}; int ntv=0;
        for (int i=0;i<ncar;i++) {
            int tire = car.meshes[i].cat==N2_CAR_TIRE;
            for (int v=0; v<car.meshes[i].nverts; v++) {
                float *p=car.meshes[i].verts+v*5;
                if (tire) { ntv++; for(int a=0;a<3;a++){ if(p[a]<tb0[a])tb0[a]=p[a]; if(p[a]>tb1[a])tb1[a]=p[a]; } }
                else for (int a=0;a<3;a++){ if(p[a]<bb0[a])bb0[a]=p[a]; if(p[a]>bb1[a])bb1[a]=p[a]; }
            }
        }
        carbb[0]=bb0[0];carbb[1]=bb0[1];carbb[2]=bb0[2];
        carbb[3]=bb1[0];carbb[4]=bb1[1];carbb[5]=bb1[2];  /* wheelT built per-frame from g_dbg */
        /* Prep each stock wheel mesh: (1) the tyre is modelled with its solid
           spoke/cap face inboard (low Y) and the hollow barrel mouth outboard,
           so rotate it 180 deg about the vertical (Z) through its own Y-centre
           -- (x,y)->(-x,-y) -- which flips the cap outward and centres the tyre
           on the AttribSys track line; a rotation (not a mirror) keeps winding
           and normals valid. (2) drop the flat backing/hub-plane quad so the
           rim renders as clean spokes. Then remember the highest-LOD tier. */
        for (int i=0;i<ncar;i++) if (car.meshes[i].cat==N2_CAR_TIRE) {
            N2Mesh *m=&car.meshes[i];
            float ty0=1e30f,ty1=-1e30f;
            for(int v=0;v<m->nverts;v++){ float y=m->verts[v*5+1]; if(y<ty0)ty0=y; if(y>ty1)ty1=y; }
            float ymid=0.5f*(ty0+ty1);
            for(int v=0;v<m->nverts;v++){ m->verts[v*5]=-m->verts[v*5]; m->verts[v*5+1]=ymid-m->verts[v*5+1]; }
            rim_drop_welded_mesh(m);
            if (stock_wheel<0 || m->nverts>car.meshes[stock_wheel].nverts) stock_wheel=i;
        }
        cgm = upload_scene(&car);
        /* size the procedural tyre from the car's own wheel mesh (radius in X-Z,
           width in Y) so a Hummer gets big tyres and a compact gets small ones. */
        float wR = 0.33f, wHW = 0.11f;
        if (ntv) { wR = 0.25f*((tb1[0]-tb0[0])+(tb1[2]-tb0[2]));   /* mean disc radius */
                   wHW = 0.5f*(tb1[1]-tb0[1]); if (wHW<0.04f) wHW=0.04f; }
        /* Build this car's dimension profile from its OWN geometry, and drive
           the wheel/ride numbers from it instead of any absolute constant. */
        n2_car_profile(&car, carname, WHEEL_SEED_FRONTF, WHEEL_SEED_REARF,
                       WHEEL_SEED_TRACKF, n2_car_brake_radius(cdata, 0, clen),
                       &carprof);
        /* GLOBAL AttribSys holds each car's exact factory wheel positions.
           Prefer the pre-decompressed GLOBALB.BUN; if it is absent, decompress
           the shipped GlobalB.lzc in place (it is a JDLZ stream) so the pipeline
           is fully self-contained from the retail files. */
        long globlen = 0; char gp[1024];
        snprintf(gp, sizeof gp, "%s/GLOBAL/GLOBALB.BUN", dataroot);
        unsigned char *globdata = n2_read_file(gp, &globlen);
        if (!globdata) {
            snprintf(gp, sizeof gp, "%s/GLOBAL/GlobalB.lzc", dataroot);
            long clen2 = 0; unsigned char *cz = n2_read_file(gp, &clen2);
            if (cz && clen2 >= 16 && memcmp(cz, "JDLZ", 4) == 0) {
                uint32_t usize = n2_u32(cz + 8);
                if (usize > 0 && usize < (1u << 28)) {
                    globdata = (unsigned char *)malloc(usize);
                    if (globdata) {
                        globlen = n2_jdlz(cz, (int)clen2, globdata, (int)usize);
                        printf("GLOBAL: decompressed GlobalB.lzc (JDLZ) -> %ld bytes\n", globlen);
                    }
                }
            }
            free(cz);
        }
        int wheel_from_global = 0;
        g_dbg.wheel = wheel_config_for(carname, &carprof, globdata, globlen, &wheel_from_global);
        free(globdata);
        wR = carprof.wheel_r; wHW = 0.5f * carprof.wheel_w;
        if (wHW < 0.04f) wHW = 0.04f;
        carWheelR = carprof.wheel_r;               /* aftermarket rims fit to this */
        printf("wheel stance %-12s axle F%+.3f R%+.3f (wb %.3f)  track F%.3f R%.3f  ride %+.3f  [%s]\n",
               carprof.name, g_dbg.wheel.front_axle, g_dbg.wheel.rear_axle,
               g_dbg.wheel.front_axle - g_dbg.wheel.rear_axle,
               g_dbg.wheel.front_track, g_dbg.wheel.rear_track, g_dbg.wheel.ride_y,
               wheel_from_global ? "GLOBAL AttribSys" : "body-box fallback");
        printf("car profile %-12s wheel R %.3f W %.3f%s  hubZ %+.3f  ride %.3f  "
               "wheelbase %.2f  track %.2f  clearance %.3f\n",
               carprof.name, carprof.wheel_r, carprof.wheel_w,
               carprof.has_tire ? "" : " (no tyre mesh: derived from body length)",
               carprof.hub_z, carprof.ride, carprof.wheelbase, carprof.track_f,
               carprof.clearance);
        wheelmesh = make_wheel(wR, wHW); have_wheel = 1;
        /* decode + upload each distinct texture actually bound by a mesh */
        for (int i = 0; i < ncar; i++) {
            uint32_t tk = car.meshes[i].texkey; if (!tk) continue;
            int seen = 0; for (int j = 0; j < nmap; j++) if (mapkey[j]==tk) seen = 1;
            if (seen || nmap >= 32) continue;
            N2Tex ct;
            if (n2_load_car_tex_by_key(ctdata, ctlen, tk, &ct)) {
                mapkey[nmap] = tk; maptex[nmap] = upload_tpk_texture_to_gpu(&ct);
                mapalpha[nmap] = ct.alpha != NULL;   /* DXT3 = decal mask */
                nmap++;
                /* car textures are atlases (UVs in [0,1]): clamp so REPEAT
                   wrap + mip filtering can't bleed the opposite border in. */
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
                free(ct.rgb); free(ct.alpha); free(ct.dxt);
            }
        }
        printf("car textures bound: %d distinct\n", nmap);
        /* sponsor vinyl layer: VINYLS.BIN is one big TPK whose offset-slots
           point at EA "HUFF" (Huffman) blobs, not JDLZ — no open decoder
           exists (Nikki et al call EA's closed LZCompressLib.dll), so every
           decode below fails cleanly today and the car keeps its badge
           atlas. The compositing (paint -> vinyl -> badges, one texture)
           lights up as soon as a HUFF decoder lands in nfsu2.h.
           ponytail: first-fit vinyl choice — a vinyl-select menu can replace
           the pick without touching the compositing. */
        {
            char vpath[512];
            snprintf(vpath, sizeof vpath, "%s/CARS/%s/VINYLS.BIN", dataroot, carname);
            long vlen; unsigned char *vdata = n2_read_file(vpath, &vlen);
            uint32_t bodykey = 0;
            for (int i = 0; i < ncar && !bodykey; i++) {
                int c = car.meshes[i].cat;
                if ((c == N2_CAR_BODY || c == N2_CAR_MISC) && car.meshes[i].texkey)
                    for (int j = 0; j < nmap; j++)
                        if (mapkey[j] == car.meshes[i].texkey && mapalpha[j])
                            bodykey = mapkey[j];
            }
            static uint32_t vkeys[512];
            int nvk = vdata ? n2_car_tex_keys(vdata, vlen, vkeys, 512) : 0;
            N2Tex vt; int got = 0; uint32_t gotkey = 0;
            for (int k = 0; k < nvk && !got; k++) {
                if (!n2_load_car_tex_by_key(vdata, vlen, vkeys[k], &vt)) continue;
                if (vt.alpha) {
                    long n = (long)vt.w * vt.h, op = 0;
                    for (long p = 0; p < n; p++) if (vt.alpha[p] > 128) op++;
                    float f = (float)op / (float)n;
                    if (f > 0.04f && f < 0.55f) { got = 1; gotkey = vkeys[k]; break; }
                }
                free(vt.rgb); free(vt.alpha); free(vt.dxt);
            }
            N2Tex bt;
            if (got && bodykey && n2_load_car_tex_by_key(ctdata, ctlen, bodykey, &bt)) {
                int W = bt.w > vt.w ? bt.w : vt.w, H = bt.h > vt.h ? bt.h : vt.h;
                N2Tex out = { W, H, (unsigned char *)malloc((long)W*H*3),
                                    (unsigned char *)malloc((long)W*H), NULL, 0, 0 };
                for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) {
                    long o  = (long)y*W + x;
                    long bo = (long)(y*bt.h/H)*bt.w + x*bt.w/W;   /* nearest */
                    long vo = (long)(y*vt.h/H)*vt.w + x*vt.w/W;
                    int ba = bt.alpha[bo];                        /* badge over vinyl */
                    for (int ch = 0; ch < 3; ch++)
                        out.rgb[o*3+ch] = (unsigned char)
                            ((bt.rgb[bo*3+ch]*ba + vt.rgb[vo*3+ch]*(255-ba)) / 255);
                    out.alpha[o] = ba > vt.alpha[vo] ? (unsigned char)ba : vt.alpha[vo];
                }
                for (int j = 0; j < nmap; j++) if (mapkey[j] == bodykey) {
                    glDeleteTextures(1, &maptex[j]);
                    maptex[j] = upload_tex(&out);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
                    /* updates the badge atlas's OWN GL texture in place, so
                       any mesh that references this key directly (its own
                       0x134012 slot list, e.g. GOLF's BASE_A) picks up the
                       vinyl-under-badge composite automatically in the draw
                       loop — no separate fallback texture needed. */
                    break;
                }
                printf("vinyl layer: key %08x (%dx%d) composited under the badge atlas\n",
                       gotkey, vt.w, vt.h);
                free(out.rgb); free(out.alpha); free(bt.rgb); free(bt.alpha); free(bt.dxt);
            }
            if (got) { free(vt.rgb); free(vt.alpha); free(vt.dxt); }
            free(vdata);
        }
        /* spawn on the road mesh nearest the track centre; aim inward so a
           straight run stays on the populated track (user steers in play). */
        float bestd = 1e30f;
        for (int i = 0; i < nm; i++) if (scene.meshes[i].cat == N2_ROAD) {
            float vx = scene.meshes[i].verts[0], vy = scene.meshes[i].verts[1];
            float d = (vx-cx)*(vx-cx) + (vy-cy)*(vy-cy);
            if (d < bestd) { bestd = d;
                spawn[0]=vx; spawn[1]=vy; spawn[2]=scene.meshes[i].verts[2]; }
        }
        heading0 = atan2f(cy - spawn[1], cx - spawn[0]);
        printf("car: %d meshes, spawn (%.1f,%.1f) heading %.2f\n",
               ncar, spawn[0], spawn[1], heading0);
    }

    /* Enumerate the selectable assets for the pre-race menu. Switching car or
       track means a whole new load, so those re-launch the process (below);
       circuit is a cheap in-place reload. */
    #define MAXTRACK WORLD_MAXREG
    #define MAXCARS  64
    #define MAXCIRC  24
    char tracklist[MAXTRACK][64]; int seltrack = 0;
    int ntrack = res_list_tracks(troot, tracklist, MAXTRACK, trackname, &seltrack);
    if (ntrack < MAXTRACK) {   /* city mode is selectable from the menu too */
        snprintf(tracklist[ntrack], sizeof tracklist[0], "ALL");
        if (!strcmp(trackname, "ALL")) seltrack = ntrack;
        ntrack++;
    }
    char carlist[MAXCARS][64]; int selcar = 0;
    int ncars = res_list_cars(dataroot, carlist, MAXCARS, carname, &selcar);
    char circlist[MAXCIRC][256]; int selcirc = 0;
    int ncirc = res_list_circuits(troot, circlist, MAXCIRC, mn, mx);
    if (explicit_circuit)  /* honor an explicit --circuit if it's valid on this track */
        for (int i=0;i<ncirc;i++) if(!strcmp(circlist[i], circuit)) selcirc=i;
    printf("circuits available: %d (menu: Left/Right)\n", ncirc);

    N2Path aipath = {0};
    AiCar ais[N_AI]; int start_idx = 0;
    int nai = ncirc ? load_circuit(dataroot, circlist[selcirc], &scene, &aipath,
                                   ais, spawn, &heading0, &start_idx, densx, densy) : 0;
    if (nai) printf("circuit: %d-waypoint loop; %d AI racers, lap system on\n",
                    aipath.n, nai);

    GLuint texTerr = world.have_grass ? upload_tex(&world.grass) : 0;
    GLuint texWheel = make_wheel_tex();   /* radial alloy-rim look for the tyres */
    /* Above ~40 km/h the 5-spoke pattern is turning faster than the frame rate
       can sample, so crisp spokes alias into a strobing mess. Swap to the
       angular-averaged rim then — a stand-in for the pre-baked blur asset
       retail used, which is NOT present in this data: no _BLUR key exists in
       any per-car TEXTURES.BIN. */
    GLuint texWheelBlur = make_wheel_blur_tex();
    #define WHEEL_BLUR_KMH 40.0f

    /* Aftermarket rim library. NOT CARS/WHEELS/GEOMETRY.BIN -- that file is a
       64-byte stub; the real geometry is per brand (GEOMETRY_BBS.BIN,
       GEOMETRY_ENKEI.BIN, ...) in the same chunk format as a car, so the
       ordinary car walker reads it. Each brand file holds its styles under
       STYLEnn tokens, which is the SAME token hoods use inside a car, so the
       style selector here is wheel_style_id rather than hood_style. */
    /* NFSU first: GEOMETRY_NFSU.BIN is the game's STOCK rim set (the wheels a
       car ships with), so it is the authentic default; the rest are the
       aftermarket brands, still selectable in the debug panel. */
    static const char wheel_brands[][24] = {
        "NFSU",
        "BBS","ENKEI","VOLK","OZ","MOMO","ADVAN","AVUS","KONIG",
        "LOWENHART","RACINGHART","ROTA","WORK","5ZIGEN","LEXANI"
    };
    const int n_wheel_brands = (int)(sizeof wheel_brands / sizeof wheel_brands[0]);
    (void)n_wheel_brands;   /* only read by the ImGui panel (debug builds) */
    N2Scene wheellib; memset(&wheellib, 0, sizeof wheellib);
    GpuMesh *wheelgm = NULL; int nwheelgm = 0, wheel_style = 1;
    long wllen = 0; unsigned char *wldata = NULL;
    int wheel_brand = 0;
    {   char wlp[1024];
        snprintf(wlp, sizeof wlp, "%s/CARS/WHEELS/GEOMETRY_%s.BIN",
                 dataroot, wheel_brands[wheel_brand]);
        wldata = n2_read_file(wlp, &wllen);
    }
    /* WHEELS/TEXTURES.BIN holds 274 texture slots (measured). This array used
       to be 64 entries, which silently truncated the table at slot 63: only a
       rim whose diffuse key happened to land in that first 64 ever resolved
       (LEXANI did, by luck), and every other brand fell back to flat colour.
       Sized well clear of 274 so a larger table still fits; n2_car_tex_keys
       stops at maxk, so an undersized array fails quietly rather than loudly. */
    uint32_t wkeys[512]; int nwkeys = 0;
    long wtlen = 0; unsigned char *wtdata = NULL;   /* kept resident: the rim
                       diffuse is decoded out of it on every style/brand swap */
    {   char wtp[1024];
        snprintf(wtp, sizeof wtp, "%s/CARS/WHEELS/TEXTURES.BIN", dataroot);
        wtdata = n2_read_file(wtp, &wtlen);
        if (wtdata) nwkeys = n2_car_tex_keys(wtdata, wtlen, wkeys,
                                             (int)(sizeof wkeys / sizeof wkeys[0]));
    }
    GLuint rimtex = 0;    /* real rim diffuse; 0 => fall back to procedural */
    if (load_rim_style(wldata, wllen, wkeys, nwkeys, wheel_style,
                       &wheellib, &wheelgm, &nwheelgm,
                       wtdata, wtlen, &rimtex, carWheelR))
        printf("rims: %s style %d, %d mesh(es)\n", wheel_brands[wheel_brand], wheel_style, nwheelgm);
    else
        printf("rims: library unavailable, using procedural wheels\n");


    /* static world -> per-(cell,texture) interleaved batches; the CPU-side
       scene stays alive for the physics/ground queries */
    /* Phase 21: skybox + neon/glow are pulled into their own batch lists so
       they can get their own draw pass (camera-locked/depth-off for the
       sky, additive-blended at frame end for neon) instead of blending into
       the ordinary opaque city batches. Print a note if a region genuinely
       has no SKYDOME mesh — the shader just falls back to the flat fog
       clear colour, which is correct but worth knowing about. */
    N2Batch *skybatch = NULL; int nsky = upload_cat_batches(&scene, N2_SKY, mtex, &skybatch);
    N2Batch *glowbatch = NULL; int nglow = upload_cat_batches(&scene, N2_GLOW, mtex, &glowbatch);
    printf("sky: %d batch(es)%s, neon/glow: %d batch(es)\n", nsky,
           nsky ? "" : " (no SKYDOME mesh found in this region set)", nglow);

    N2Batch *wbatch = NULL;
    int nbatch = upload_world_batches(&scene, (const float (*)[4])world.mbb,
                                      mtex, texTerr, &wbatch);
    free(mtex);
    printf("world batched: %d meshes -> %d batches\n", nm, nbatch);

    /* F3 debug pipeline: wrap the existing world VBOs/IBOs in WorldMeshBatch so
       render_world_map can draw them with the prelight/normal/wireframe shader.
       Shared GL handles (owned by wbatch) -> no extra VRAM, no separate upload. */
    GLuint dbgprog = world_debug_program_load("src/world_debug_120.vert",
                                              "src/world_debug_120.frag");
    WorldMeshBatch *wmbatch = (WorldMeshBatch *)calloc((size_t)(nbatch > 0 ? nbatch : 1),
                                                       sizeof *wmbatch);
    for (int k = 0; k < nbatch; k++) {
        wmbatch[k].vbo = wbatch[k].vbo; wmbatch[k].ibo = wbatch[k].ibo;
        wmbatch[k].index_count = wbatch[k].index_count; wmbatch[k].chunk_id = (uint32_t)k;
    }
    int g_debug_mode = 0;   /* F3 cycles: 0 default, 1 prelight, 2 normals, 3 wireframe */
    if (!dbgprog) fprintf(stderr, "world_debug shaders failed to load; F3 disabled\n");

    /* unit-quad for the 2D HUD (drawn in NDC via uMVP) */
    GpuMesh quad = make_quad();

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);   /* coplanar re-draws (last write wins) pass cleanly */
    /* Stock showroom paint: metallic silver (the files carry no chosen
       colour; badges/vinyls overlay as decals). The debug pane's paint
       override still allows any colour live. */
    float paint[3] = { 0.70f, 0.70f, 0.75f };
    if (!strcmp(carname, "MIATA")) { paint[0]=0.55f; paint[1]=0.10f; paint[2]=0.09f; }  /* reference showroom red */
    float carpos[3] = { spawn[0], spawn[1], spawn[2] };
    float car_up[3] = { 0, 0, 1 };   /* chassis up, lerped toward the ground normal */
    if (shot && aipath.n > 0) {
        /* --shot skips the menu (and its Enter-key start-line snap), so the
           showcase density-spawn would leave the car parked off-circuit in
           the void on proxy regions. Snap to the start line like a race. */
        carpos[0] = aipath.xy[start_idx*2]; carpos[1] = aipath.xy[start_idx*2+1];
        carpos[2] = world_ground_z(&scene, carpos[0], carpos[1], carpos[2]);
        int nx = (start_idx+1) % aipath.n;
        heading0 = atan2f(aipath.xy[nx*2+1]-carpos[1], aipath.xy[nx*2]-carpos[0]);
    }
    /* an armed race wins: start on its own grid, facing through the start line */
    if (world.race.active && world.race.ngrid > 0) {
        /* Keep the grid slot's ACROSS-track offset (that part is shipped data)
           but force the along-track position behind the start line — the raw
           slot can sit a couple of metres past gate 0 once the gate is snapped
           to the nearest racing-line node, and a car that starts past the line
           can never cross it. */
        const WGate *g0 = &world.race.gate[0];
        float rx = world.race.grid[0][0] - g0->x, ry = world.race.grid[0][1] - g0->y;
        float lat = -rx*g0->dy + ry*g0->dx;
        carpos[0] = g0->x + (-g0->dy)*lat - g0->dx*15.0f;
        carpos[1] = g0->y + ( g0->dx)*lat - g0->dy*15.0f;
        carpos[2] = world_ground_z(&scene, carpos[0], carpos[1], carpos[2]);
        heading0 = atan2f(g0->dy, g0->dx);
        printf("race start: grid slot 0 (%.1f, %.1f) -> lined up at (%.1f, %.1f), "
               "15 m behind the start line\n", world.race.grid[0][0],
               world.race.grid[0][1], carpos[0], carpos[1]);
    }
    float heading = heading0, speed = 0.0f, vel[2] = {0,0};
    float cam[3] = { spawn[0], spawn[1], spawn[2]+5 };
    float fc[3] = { spawn[0]-6, spawn[1], spawn[2]+3 };   /* freecam eye */
    float fyaw = 0.0f, fpitch = -0.25f;                   /* freecam look angles */
    int   mlook = 0;                                      /* right-drag mouse look active */
    int p_lap = 0, p_prev = 0;   /* player lap + previous loop-progress */
    /* race flow: 3 = pre-race menu, 0 = countdown, 1 = racing, 2 = finished */
    const int COUNTDOWN = 180, LAP_TARGET = 2;
    int race_state = shot ? 1 : 3, racetimer = 0, finish_place = 0;
    int gear = 1; float shift_t = 0.0f;   /* virtual gearbox (engine audio) */
    float menuspin = 0.0f;   /* orbit-camera angle on the menu screen */
    int running = 1, shotframe = 0;
    uint32_t t0 = SDL_GetTicks();   /* --shot prints avg ms/frame at exit */
    /* tyre skid marks: a ring buffer of oriented ground SEGMENTS (ax,ay,az,
       bx,by,bz, life) — life starts at 1 and decays so marks fade over time.
       bx,by,bz) forming continuous ribbons; plus drift smoke particles. */
    #define MAXSKID 700
    static float skid[MAXSKID][7]; int skidn = 0, skidhead = 0;
    float prevL[3] = {0,0,0}, prevR[3] = {0,0,0}; int skidpen = 0;  /* pen-down state */
    #define MAXSMOKE 512
    static struct { float p[3], v[3], life, size; } smoke[MAXSMOKE]; int smoken = 0;
    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
#ifdef DEBUG_UI
            dbgui_event(&e);
            if ((e.type==SDL_KEYDOWN||e.type==SDL_KEYUP) && dbgui_want_keyboard()) continue;
#endif
            if (e.type == SDL_QUIT) running = 0;
            /* freecam mouse-look: hold right button to rotate (keeps the cursor
               free for the ImGui panel the rest of the time). */
            else if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_RIGHT && g_dbg.freecam) {
                SDL_SetRelativeMouseMode(SDL_TRUE); mlook = 1;
            }
            else if (e.type == SDL_MOUSEBUTTONUP && e.button.button == SDL_BUTTON_RIGHT) {
                SDL_SetRelativeMouseMode(SDL_FALSE); mlook = 0;
            }
            else if (e.type == SDL_MOUSEMOTION && mlook) {
                fyaw   -= e.motion.xrel * 0.005f;
                fpitch -= e.motion.yrel * 0.005f;
                if (fpitch> 1.5f) fpitch= 1.5f; if (fpitch<-1.5f) fpitch=-1.5f;
            }
            else if (e.type == SDL_KEYDOWN) {
                SDL_Keycode k = e.key.keysym.sym;
                if (k == SDLK_ESCAPE) running = 0;
                else if (k == SDLK_f) {       /* toggle freecam; seed it from the current view */
                    g_dbg.freecam ^= 1;
                    if (g_dbg.freecam) {
                        fc[0]=cam[0]; fc[1]=cam[1]; fc[2]=cam[2];
                        fyaw = atan2f(carpos[1]-cam[1], carpos[0]-cam[0]);
                        fpitch = -0.2f;
                    }
                }
                else if (k == SDLK_F3) {   /* cycle world render debug view */
                    if (dbgprog) {
                        g_debug_mode = (g_debug_mode + 1) & 3;
                        static const char *dm[4] = { "default", "prelight",
                                                     "normals", "wireframe" };
                        printf("render mode %d: %s\n", g_debug_mode, dm[g_debug_mode]);
                    }
                }
                else if (k == SDLK_w && wldata) {
                    wheel_style = wheel_style % 8 + 1;   /* 1..8 */
                    if (load_rim_style(wldata, wllen, wkeys, nwkeys, wheel_style,
                                       &wheellib, &wheelgm, &nwheelgm,
                                       wtdata, wtlen, &rimtex, carWheelR))
                        printf("rims -> BBS style %d (%d mesh(es))\n", wheel_style, nwheelgm);
                }
                else if (k == SDLK_k && cdata) {
                    /* cycle body kit 0 -> 1 -> 2 -> 0 and re-stream the car in
                       place. Unlike car/track (a whole new world + audio load,
                       hence relaunch()), this only touches the car's own
                       buffers, so it is cheap and safe to do live. */
                    carcfg.body_kit = (carcfg.body_kit + 1) % 3;
                    for (int i = 0; i < ncar; i++) {
                        glDeleteBuffers(1, &cgm[i].vbo);
                        glDeleteBuffers(1, &cgm[i].nbo);
                        glDeleteBuffers(1, &cgm[i].ibo);
                    }
                    free(cgm); cgm = NULL;
                    n2_free_scene(&car);
                    ncar = n2_load_car(cdata, clen, &car, ckeys, nck, &carcfg);
                    stock_wheel = -1;   /* re-orient + re-cull wheels, re-find stock index */
                    for (int i=0;i<ncar;i++) if (car.meshes[i].cat==N2_CAR_TIRE) {
                        N2Mesh *m=&car.meshes[i];
                        float ty0=1e30f,ty1=-1e30f;
                        for(int v=0;v<m->nverts;v++){ float y=m->verts[v*5+1]; if(y<ty0)ty0=y; if(y>ty1)ty1=y; }
                        float ymid=0.5f*(ty0+ty1);
                        for(int v=0;v<m->nverts;v++){ m->verts[v*5]=-m->verts[v*5]; m->verts[v*5+1]=ymid-m->verts[v*5+1]; }
                        rim_drop_welded_mesh(m);
                        if (stock_wheel<0 || m->nverts>car.meshes[stock_wheel].nverts) stock_wheel=i;
                    }
                    cgm = upload_scene(&car);
                    printf("body kit -> KIT%02d (%d meshes)\n", carcfg.body_kit, ncar);
                }
                else if (race_state == 3) {   /* pre-race menu navigation */
                    /* car and track each mean a whole new load, so they re-launch
                       the process (see relaunch()); circuit is a cheap in-place
                       reload. */
                    if ((k==SDLK_LEFT || k==SDLK_RIGHT) && ncars > 1) {
                        selcar = (selcar + (k==SDLK_RIGHT?1:ncars-1)) % ncars;
                        relaunch(selfexe, dataroot, carlist[selcar], trackname);
                    } else if ((k==SDLK_UP || k==SDLK_DOWN) && ntrack > 1) {
                        seltrack = (seltrack + (k==SDLK_DOWN?1:ntrack-1)) % ntrack;
                        relaunch(selfexe, dataroot, carname, tracklist[seltrack]);
                    } else if ((k==SDLK_LEFTBRACKET || k==SDLK_RIGHTBRACKET) && ncirc > 1) {
                        selcirc = (selcirc + (k==SDLK_RIGHTBRACKET?1:ncirc-1)) % ncirc;
                        nai = load_circuit(dataroot, circlist[selcirc], &scene,
                                           &aipath, ais, spawn, &heading0, &start_idx, densx, densy);
                        carpos[0]=spawn[0]; carpos[1]=spawn[1]; carpos[2]=spawn[2];
                        heading=heading0; vel[0]=vel[1]=0; speed=0; p_lap=p_prev=0;
                    } else if (k==SDLK_RETURN || k==SDLK_SPACE) {
                        /* snap the player from the showcase spot to the circuit
                           start line so the race itself is fair. */
                        if (aipath.n > 0) {
                            carpos[0]=aipath.xy[start_idx*2]; carpos[1]=aipath.xy[start_idx*2+1];
                            carpos[2]=world_ground_z(&scene, carpos[0], carpos[1], carpos[2]);
                            int nx=(start_idx+1)%aipath.n;
                            heading=atan2f(aipath.xy[nx*2+1]-carpos[1], aipath.xy[nx*2]-carpos[0]);
                            vel[0]=vel[1]=0; speed=0;
                        }
                        race_state = 0; racetimer = 0;   /* -> 3-2-1 countdown */
                    }
                }
            }
        }
        const Uint8 *ks = SDL_GetKeyboardState(NULL);
        if (g_dbg.freecam) {                 /* fly the camera; WASD move, arrows look */
            float sp = g_dbg.speed * (ks[SDL_SCANCODE_LSHIFT]?4.0f:1.0f);
            float cp=cosf(fpitch);
            float dir[3]={cp*cosf(fyaw), cp*sinf(fyaw), sinf(fpitch)};
            float rt[3]={sinf(fyaw), -cosf(fyaw), 0};   /* strafe axis */
            if (ks[SDL_SCANCODE_W]){ fc[0]+=dir[0]*sp; fc[1]+=dir[1]*sp; fc[2]+=dir[2]*sp; }
            if (ks[SDL_SCANCODE_S]){ fc[0]-=dir[0]*sp; fc[1]-=dir[1]*sp; fc[2]-=dir[2]*sp; }
            if (ks[SDL_SCANCODE_D]){ fc[0]+=rt[0]*sp;  fc[1]+=rt[1]*sp; }
            if (ks[SDL_SCANCODE_A]){ fc[0]-=rt[0]*sp;  fc[1]-=rt[1]*sp; }
            if (ks[SDL_SCANCODE_E]||ks[SDL_SCANCODE_SPACE]) fc[2]+=sp;
            if (ks[SDL_SCANCODE_Q]||ks[SDL_SCANCODE_LCTRL]) fc[2]-=sp;
            if (ks[SDL_SCANCODE_LEFT])  fyaw   += 0.03f;
            if (ks[SDL_SCANCODE_RIGHT]) fyaw   -= 0.03f;
            if (ks[SDL_SCANCODE_UP])    fpitch += 0.02f;
            if (ks[SDL_SCANCODE_DOWN])  fpitch -= 0.02f;
            if (fpitch> 1.5f) fpitch= 1.5f; if (fpitch<-1.5f) fpitch=-1.5f;
        }
        /* driving inputs -> Physics module (velocity-vector arcade kinematics:
           throttle along the heading, steering rotates it, tyres scrub the
           sideways component so hard cornering at speed slides/drifts). */
        float throttle = 0.0f;
        if (!g_dbg.freecam && race_state == 1) {
            if      (ks[SDL_SCANCODE_W] || shot) throttle =  1.0f;
            else if (ks[SDL_SCANCODE_S])         throttle = -1.0f;
        }
        float steer = g_dbg.freecam ? 0.f
                    : (ks[SDL_SCANCODE_A]?1.f:0.f) - (ks[SDL_SCANCODE_D]?1.f:0.f);
        int handbrake = (race_state==1 && ks[SDL_SCANCODE_SPACE]);
        /* push the ImGui handling sliders into the physics tune (stock when untouched) */
        g_phys_tune.accel = g_dbg.tune_accel; g_phys_tune.brake = g_dbg.tune_brake;
        g_phys_tune.turn = g_dbg.tune_turn;   g_phys_tune.top_kmh = g_dbg.tune_top;
        /* Race autopilot (--shot only): walk the car along the corridor-masked
           A* route to the armed gate so a headless run drives the real course
           and the checkpoint logic sees real positions. Kinematic on purpose —
           the same role as the --circuit demo autopilot below, and the physics
           drive is separately pinned on this track (the default --circuit start
           snap lands the car inside geometry, so collide_walls holds it at
           0 km/h; pre-existing, not the race system). */
        static int rpath[8192]; static int rpn = 0, rp_gate = -1, rp_at = 0;
        int race_auto = shot && world.race.active && !world.race.finished;
        if (race_auto) {
            if (rp_gate != world.race.next) {
                int s = world_nav_nearest(&world, carpos[0], carpos[1]);
                int gnode = world.race.gate[world.race.next].node;
                rpn = world_route(&world, s, gnode, rpath, 8192, NULL);
                if (rpn <= 1 && world.mode == MODE_RACE_EVENT) {
                    /* the corridor mask can disconnect the spawn/previous node
                       from this gate; without a route the car would beeline
                       straight through the city. Re-route on the UNMASKED road
                       graph (fully connected) so it always stays on asphalt. */
                    int mode = world.mode; world.mode = MODE_FREEROAM;
                    rpn = world_route(&world, s, gnode, rpath, 8192, NULL);
                    world.mode = mode;
                }
                rp_gate = world.race.next; rp_at = 0;
            }
            const WGate *ng = &world.race.gate[world.race.next];
            float gdx = ng->x - carpos[0], gdy = ng->y - carpos[1];
            float dx, dy;
            if (gdx*gdx + gdy*gdy < 9.0f*9.0f) {
                /* almost on the gate: drive straight through on the current
                   heading so the crossing always registers (no node stalling). */
                dx = cosf(heading); dy = sinf(heading);
            } else {
                float tx = ng->x, ty = ng->y;
                if (rpn > 1) {                     /* follow the A* route in */
                    /* hug each node (advance at 3 m) so the aim tracks the road
                       polyline rather than cutting toward the distant gate */
                    if (rp_at < rpn - 1) {
                        float ax = world.nav[rpath[rp_at]*2]   - carpos[0];
                        float ay = world.nav[rpath[rp_at]*2+1] - carpos[1];
                        if (ax*ax + ay*ay < 9.0f) rp_at++;
                    }
                    tx = world.nav[rpath[rp_at]*2]; ty = world.nav[rpath[rp_at]*2+1];
                }
                dx = tx - carpos[0]; dy = ty - carpos[1];
            }
            float d = sqrtf(dx*dx + dy*dy);
            if (d > 1e-3f) {
                float step = 1.2f;                 /* ~72 m/s at 60 Hz */
                carpos[0] += dx/d*step; carpos[1] += dy/d*step;
                heading = atan2f(dy, dx);
                speed = step * 60.0f;
            }
            /* clamp Z to the nearest road/terrain surface, eased so a gap in an
               elevated deck descends smoothly instead of teleporting */
            { float gz = world_ground_z(&scene, carpos[0], carpos[1], carpos[2]);
              float dz = gz - carpos[2];
              if (dz >  1.5f) dz =  1.5f; if (dz < -1.5f) dz = -1.5f;
              carpos[2] += dz; }
            world_race_update(&world, carpos[0], carpos[1]);
        }
        else if (shot && aipath.n > 0) {   /* screenshot autopilot: follow the racing
               line (chasing an AI used to drift off small proxy regions into
               the empty void — black screenshots) */
            int nearest = 0; float bd = 1e30f;
            for (int i = 0; i < aipath.n; i++) {
                float dx = aipath.xy[i*2]-carpos[0], dy = aipath.xy[i*2+1]-carpos[1];
                float d2 = dx*dx + dy*dy;
                if (d2 < bd) { bd = d2; nearest = i; }
            }
            int tgt = (nearest + 3) % aipath.n;      /* aim a few waypoints ahead */
            float da = atan2f(aipath.xy[tgt*2+1]-carpos[1],
                              aipath.xy[tgt*2]-carpos[0]) - heading;
            while (da> 3.14159f) da-=6.28318f;
            while (da<-3.14159f) da+=6.28318f;
            if (da> 0.06f) da= 0.06f; if (da<-0.06f) da=-0.06f;
            heading += da;
        }
        float dmag = race_auto ? speed/60.0f
                   : phys_car_step(carpos, vel, &heading, &speed,
                                   throttle, steer, handbrake);
        float nf[2] = { cosf(heading), sinf(heading) }, nr[2] = { nf[1], -nf[0] };
        /* engine note: 6-speed virtual gearbox drives RPM + load; shifts cut
           the throttle for 150ms and let the revs sag (idles during the
           countdown, since throttle is locked out until GO). */
        { float sp = (speed < 0 ? -speed : speed) / PHYS_MAXSPD;
          eng_gearbox_step(sp, throttle, 1.0f/60.0f, &gear, &shift_t);
          g_engine.master_volume = (race_state==0 ? 0.16f : 0.16f + sp*0.5f);
          g_road_vol = sp*sp*0.35f; }   /* tyre/wind roar rises with speed */
        float fwd[3] = { nf[0], nf[1], 0 };
        /* building collision: push the car out of any wall footprint it entered.*/
        if (race_state == 1 && !race_auto && collide_walls(carpos, vel, obst, nobst, 1.3f)) g_hit = 0.5f;
        /* guardrail/fence collision: push out of near-vertical road/terrain faces */
        if (race_state == 1 && !race_auto && world_wall_push(&scene, carpos, 1.3f)) {
            vel[0]*=0.3f; vel[1]*=0.3f; g_hit = 0.5f;   /* rebound: bleed speed */
        }
        /* race blockades: only solid while a race event is active (Phase 71) */
        if (race_state == 1 && !race_auto && world_barrier_push(&world, carpos, 1.3f)) {
            vel[0]*=0.2f; vel[1]*=0.2f; g_hit = 0.5f;
        }
        /* checkpoint / lap tracking, after the pushes so it sees the final XY */
        if (race_state == 1 && !race_auto) world_race_update(&world, carpos[0], carpos[1]);
        /* sit on the road/terrain surface (smoothed to avoid jitter) */
        float gz = world_ground_z(&scene, carpos[0], carpos[1], carpos[2]);
        carpos[2] += (gz - carpos[2]) * 0.35f;
        /* chassis pitch/roll: ease the body up-vector toward the ground normal
           (slow lerp so sharp mesh seams don't snap the car). */
        { float gn[3]; world_ground_normal(&scene, carpos[0], carpos[1], gn);
          for (int a = 0; a < 3; a++) car_up[a] += (gn[a] - car_up[a]) * 0.10f;
          float l = sqrtf(car_up[0]*car_up[0]+car_up[1]*car_up[1]+car_up[2]*car_up[2]);
          if (l > 1e-6f) { car_up[0]/=l; car_up[1]/=l; car_up[2]/=l; } }

        /* drifting? extend the tyre ribbons + puff smoke at the rear wheels */
        int drifting = (dmag > PHYS_MAXSPD*0.2f && speed > PHYS_MAXSPD*0.22f);
        /* tyre screech scales with how hard the back is sliding */
        g_skid = (race_state==1 && drifting)
               ? (0.12f + 0.45f*(dmag - PHYS_MAXSPD*0.2f)/PHYS_MAXSPD) : 0.0f;
        if (g_skid > 0.35f) g_skid = 0.35f;
        if (drifting) {
            float rx = carpos[0]-nf[0]*1.6f, ry = carpos[1]-nf[1]*1.6f, rz = carpos[2]+0.05f;
            float curL[3]={rx+nr[0], ry+nr[1], rz}, curR[3]={rx-nr[0], ry-nr[1], rz};
            if (skidpen) {   /* connect last frame's wheel points into ribbon segments */
                float *seg;
                seg=skid[skidhead]; seg[0]=prevL[0];seg[1]=prevL[1];seg[2]=prevL[2];
                seg[3]=curL[0];seg[4]=curL[1];seg[5]=curL[2];seg[6]=1.0f;
                skidhead=(skidhead+1)%MAXSKID; if(skidn<MAXSKID)skidn++;
                seg=skid[skidhead]; seg[0]=prevR[0];seg[1]=prevR[1];seg[2]=prevR[2];
                seg[3]=curR[0];seg[4]=curR[1];seg[5]=curR[2];seg[6]=1.0f;
                skidhead=(skidhead+1)%MAXSKID; if(skidn<MAXSKID)skidn++;
            }
            memcpy(prevL,curL,sizeof curL); memcpy(prevR,curR,sizeof curR); skidpen=1;
            /* dense puffs at both wheels every frame; soft = many low-alpha billboards */
            float wheels[2][2] = {{curL[0],curL[1]},{curR[0],curR[1]}};
            for (int wi = 0; wi < 2; wi++)
                for (int pc = 0; pc < 3 && smoken < MAXSMOKE; pc++) {
                    int si = smoken++;
                    smoke[si].p[0]=wheels[wi][0]+(rand()%100-50)*0.012f;
                    smoke[si].p[1]=wheels[wi][1]+(rand()%100-50)*0.012f;
                    smoke[si].p[2]=rz+0.2f+(rand()%30)*0.01f;
                    smoke[si].v[0]=(rand()%100-50)*0.0025f; smoke[si].v[1]=(rand()%100-50)*0.0025f;
                    smoke[si].v[2]=0.045f+(rand()%40)*0.0015f;
                    smoke[si].life=1.0f; smoke[si].size=0.55f+(rand()%40)*0.01f;
                }
        } else skidpen = 0;   /* lift the pen so the next drift starts a fresh ribbon */
        /* advance smoke particles (rise, expand, fade) */
        for (int i = 0; i < smoken; i++) {
            smoke[i].p[0]+=smoke[i].v[0]; smoke[i].p[1]+=smoke[i].v[1]; smoke[i].p[2]+=smoke[i].v[2];
            smoke[i].v[2]*=0.98f; smoke[i].size+=0.05f; smoke[i].life-=0.012f;
            if (smoke[i].life <= 0) smoke[i]=smoke[--smoken], i--;
        }

        /* AI opponents: each steers toward its next racing-line waypoint */
        for (int k = 0; race_state == 1 && k < nai; k++)
            ai_step(&ais[k], k, &aipath, &scene, start_idx,
                    p_lap*aipath.n + p_prev);
        if (race_state == 1 && aipath.n > 1) {   /* same lap logic for the player */
            int prel = (n2_nearest_wp(&aipath, carpos[0], carpos[1]) - start_idx
                        + aipath.n) % aipath.n;
            if (p_prev > aipath.n*3/4 && prel < aipath.n/4) p_lap++;
            p_prev = prel;
        }

        /* race state machine: countdown -> racing -> finished */
        racetimer++;
        if (race_state == 0 && racetimer >= COUNTDOWN) { race_state = 1; racetimer = 0; }
        if (race_state == 1 && p_lap >= LAP_TARGET) {
            int ahead = 0, pp = p_lap*aipath.n + p_prev;
            for (int k = 0; k < nai; k++)
                if (ais[k].lap*aipath.n + ais[k].prevrel > pp) ahead++;
            finish_place = ahead + 1;
            race_state = 2;
        }

        /* car-to-car collision: push overlapping cars apart (circle test) */
        { float thud = phys_car_contacts(carpos, vel, speed, ais, nai);
          if (thud > g_hit) g_hit = thud; }

        /* camera: menu = slow orbit around the parked car; else chase cam */
        float want[3];
        if (race_state == 3) {              /* orbit the parked car, framing the city around it */
            menuspin += 0.006f;
            want[0] = carpos[0] + cosf(menuspin)*16.0f;
            want[1] = carpos[1] + sinf(menuspin)*16.0f;
            want[2] = carpos[2] + 8.0f;
        } else {
            want[0] = carpos[0]-fwd[0]*10; want[1] = carpos[1]-fwd[1]*10;
            want[2] = carpos[2]+4.5f;
        }
        for (int c=0;c<3;c++) cam[c] += (want[c]-cam[c])*0.22f; /* smooth follow */
        float look[3] = { carpos[0]-cam[0], carpos[1]-cam[1], (carpos[2]+1.5f)-cam[2] };
        if (g_dbg.freecam) {                 /* fly-through overrides the chase/orbit cam */
            cam[0]=fc[0]; cam[1]=fc[1]; cam[2]=fc[2];
            float cp=cosf(fpitch);
            look[0]=cp*cosf(fyaw); look[1]=cp*sinf(fyaw); look[2]=sinf(fpitch);
        }

        int W, H; SDL_GL_GetDrawableSize(win, &W, &H);
        glViewport(0, 0, W, H);
        /* the sky is cleared to the fog colour: distant geometry dissolves
           into exactly what the horizon shows */
        glClearColor(g_dbg.fog_r, g_dbg.fog_g, g_dbg.fog_b, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glUniform3f(rp.uFogColor, g_dbg.fog_r, g_dbg.fog_g, g_dbg.fog_b);
        glUniform1f(rp.uFogDensity, g_dbg.fog_density);
        glUniform1f(rp.uUVCheck, (float)g_dbg.show_uv_checker);

        float P[16], V[16], MVP[16];
        /* menu orbits ~9m from the car, so use a close near-plane there; the
           driving near scales with region size but is clamped so a whole-city
           maxr can't push it past the car and clip it. */
        float znear = race_state==3 ? 0.2f : (maxr*0.01f < 1.0f ? maxr*0.01f : 1.0f);
        /* zfar used to be maxr*30 (~290000) — a ~300000:1 range that destroys
           24-bit depth precision and z-fights coplanar surfaces (worse on GPUs
           with a shallower depth buffer). Nothing past VIEW_DIST (700 m) is drawn,
           so clamp the world far plane to a realistic 2000 m: znear/zfar ~= 2000:1
           gives ample resolution. The skydome shell spans ~16 km and would clip
           at 2000, so it gets its own deep frustum (Psky) below. */
        float zfar = 2000.0f;
        mat_persp(0.9f, (float)W/H, znear, zfar, P);
        mat_lookat(cam, look, V);
        mat_mul(P, V, MVP);
        glUniformMatrix4fv(uMVP, 1, GL_FALSE, MVP);
        glUniform1f(uUnlit, 0.0f); glUniform1f(uSpec, 0.0f);   /* world is matte */
        glUniform1f(uAmbient, g_dbg.ambient); glUniform1f(uDiffuse, g_dbg.diffuse);
        glUniform3f(uLight, N2_SUN_X, N2_SUN_Y, N2_SUN_Z);   /* track = world space */

        /* skybox: drawn first, camera-locked (view built from a zero eye so
           translation drops out — the classic "at infinity" trick) and with
           depth-write off so every real batch below still overdraws it via
           the depth test alone. Falls back to nothing (flat fog clear
           colour shows through) when the region has no SKYDOME mesh. */
        if (nsky) {
            float zero[3] = {0,0,0}, Vsky[16], MVPsky[16], Psky[16];
            mat_lookat(zero, look, Vsky);
            mat_persp(0.9f, (float)W/H, znear, maxr*30, Psky);  /* deep: dome ~16 km */
            mat_mul(Psky, Vsky, MVPsky);
            glUniformMatrix4fv(uMVP, 1, GL_FALSE, MVPsky);
            glUniform1f(uUnlit, 1.0f);
            glDepthMask(GL_FALSE);
            /* The unlit path outputs mix(uFogColor,uColor,fog) and never
               samples the texture, so every sky batch is flat-shaded to
               uColor. It MUST be set here: the loop used to set it only for
               untextured batches, so a textured sky mesh (region D has one)
               inherited whatever uColor the previous frame last left -- the
               red brake-light/neon colour -- and, being camera-locked, drew a
               fixed red block at the top of the frame. Pin it to the fog
               colour so the dome always dissolves into the horizon. */
            glUniform1f(uUseTex, 0.0f);
            glUniform3f(uColor, g_dbg.fog_r, g_dbg.fog_g, g_dbg.fog_b);
            for (int k = 0; k < nsky; k++) {
                draw_batch(&skybatch[k]);
                g_dbg.drawn++;
            }
            glDepthMask(GL_TRUE); glUniform1f(uUnlit, 0.0f);
            glUniformMatrix4fv(uMVP, 1, GL_FALSE, MVP);   /* restore the real camera */
        }

        /* track: one draw per visible (cell,texture) batch, texture-sorted so
           binds are rare; terrain fallback + untextured-gray are baked into
           the batch key at build time.
           cull: XY distance only — the old per-mesh size cull is gone, a
           batch's tiny meshes are ~free once merged. ponytail: no frustum
           test — add one if the batch count becomes the bottleneck. */
        #define VIEW_DIST 700.0f
        int ndrawn = 0;
        g_dbg.drawn = 0;   /* per-frame draw-call tally (text glyphs excluded) */
        if (g_debug_mode == 0 || !dbgprog) {   /* --- default textured world pass --- */
        GLuint lasttex = (GLuint)-1;
        glUniform1f(rp.uVColor, g_dbg.vcolor);   /* apply source prelight to world geom */
        for (int k = 0; g_dbg.show_track && k < nbatch; k++) {
            N2Batch *b = &wbatch[k];
            float dx = cam[0] < b->bbox_min[0] ? b->bbox_min[0]-cam[0]
                     : (cam[0] > b->bbox_max[0] ? cam[0]-b->bbox_max[0] : 0);
            float dy = cam[1] < b->bbox_min[1] ? b->bbox_min[1]-cam[1]
                     : (cam[1] > b->bbox_max[1] ? cam[1]-b->bbox_max[1] : 0);
            if (dx*dx + dy*dy > VIEW_DIST*VIEW_DIST) continue;
            ndrawn += b->nmesh;
            if (b->tex != lasttex) {
                if (b->tex) { glUniform1f(uUseTex, 1.0f); glBindTexture(GL_TEXTURE_2D, b->tex); }
                else { glUniform1f(uUseTex, 0.0f); glUniform3f(uColor, 0.28f, 0.29f, 0.31f); }
                lasttex = b->tex;
            }
            draw_batch(b);
            g_dbg.drawn++;
        }
        glUniform1f(rp.uVColor, 0.0f);   /* off for everything else (cars carry no
                                            prelight; their attrib-3 default is black) */
        } else if (g_dbg.show_track) {   /* --- F3 debug view: prelight/normals/wire --- */
            /* draw every world batch with the debug shader (no distance cull),
               then restore the main program for the car/glow/HUD passes below. */
            render_world_map(wmbatch, (uint32_t)nbatch, dbgprog, MVP, g_debug_mode - 1);
            glUseProgram(rp.prog);
            glUniform1f(rp.uVColor, 0.0f);
            ndrawn += nbatch; g_dbg.drawn += nbatch;
        }

        /* car shadows: a soft dark blob on the ground under each car, so they
           sit on the road instead of floating (darkens, so it reads on any
           surface unlike the additive glows). */
        if (ncar) {
            glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDepthMask(GL_FALSE);
            glUniform1f(uUnlit,1.0f); glUniform1f(uUseTex,0.0f); glUniform1f(uSoft,1.0f);
            glUniform3f(uColor, 0.0f, 0.0f, 0.0f); glUniform1f(uAlpha, 0.5f);
            for (int c=0; c<=nai; c++) {
                float *cp = c==0 ? carpos : ais[c-1].pos;
                float gz2 = world_ground_z(&scene, cp[0], cp[1], cp[2]) + 0.03f;
                /* footprint from the loaded body AABB rather than a fixed
                   square, so a Miata and a Hummer do not cast the same blob.
                   Falls back to the old 3.2 if no car geometry is loaded. */
                float sl = carbb[3]-carbb[0], sw = carbb[4]-carbb[1];
                float sx = sl > 0.5f ? sl*1.10f : 3.2f;
                float sy = sw > 0.5f ? sw*1.35f : 3.2f;
                float M[16]={sx,0,0,0, 0,sy,0,0, 0,0,1,0, cp[0]-sx*0.5f, cp[1]-sy*0.5f, gz2, 1};
                float MV[16]; mat_mul(MVP,M,MV);
                glUniformMatrix4fv(uMVP,1,GL_FALSE,MV); draw_gpumesh(&quad);
                g_dbg.drawn++;
            }
            glUniform1f(uAlpha,1.0f); glUniform1f(uSoft,0.0f);
            glDepthMask(GL_TRUE); glDisable(GL_BLEND);

            /* neon underglow: an additive coloured pool on the asphalt under
               the chassis. Additive (not alpha) so it reads as emitted light
               on dark tarmac rather than paint, uSoft so it falls off round
               the edges, and sized from the same body AABB the shadow uses so
               it tracks the car's real footprint. Drawn after the shadow so it
               lights the ground the shadow just darkened. */
            if (g_dbg.neon_on && g_dbg.neon_str > 0.001f) {
                glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE);
                glDepthMask(GL_FALSE);
                glUniform1f(uUnlit,1.0f); glUniform1f(uUseTex,0.0f); glUniform1f(uSoft,1.0f);
                glUniform3f(uColor, g_dbg.neon_col[0], g_dbg.neon_col[1], g_dbg.neon_col[2]);
                float nl = carbb[3]-carbb[0], nw = carbb[4]-carbb[1];
                float nx = nl > 0.5f ? nl*1.45f : 4.2f, ny = nw > 0.5f ? nw*1.9f : 4.2f;
                float nz = world_ground_z(&scene, carpos[0], carpos[1], carpos[2]) + 0.02f;
                float M[16]={nx,0,0,0, 0,ny,0,0, 0,0,1,0,
                             carpos[0]-nx*0.5f, carpos[1]-ny*0.5f, nz, 1};
                float MV[16]; mat_mul(MVP,M,MV);
                glUniformMatrix4fv(uMVP,1,GL_FALSE,MV);
                glUniform1f(uAlpha, g_dbg.neon_str);
                draw_gpumesh(&quad); g_dbg.drawn++;
                glUniform1f(uAlpha,1.0f); glUniform1f(uSoft,0.0f);
                glUniformMatrix4fv(uMVP,1,GL_FALSE,MVP);
                glDepthMask(GL_TRUE); glDisable(GL_BLEND);
            }
        }

        /* headlights: warm soft light-pools cast on the road ahead (it's night),
           three overlapping ground glows that widen + fade with distance to read
           as a headlight cone. Additive blend. */
        if (race_state != 3) {
            glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE);
            glDepthMask(GL_FALSE);
            glUniform1f(uUnlit,1.0f); glUniform1f(uUseTex,0.0f); glUniform1f(uSoft,1.0f);
            glUniform3f(uColor, 0.60f, 0.56f, 0.42f);
            float fx=cosf(heading), fy=sinf(heading);
            for (int s=0;s<3;s++){
                float dist=3.0f+s*3.5f, size=3.0f+s*2.4f;
                float px=carpos[0]+fx*dist, py=carpos[1]+fy*dist;
                float pz=world_ground_z(&scene,px,py,carpos[2])+0.06f;
                float M[16]={size,0,0,0, 0,size,0,0, 0,0,1,0, px-size*0.5f, py-size*0.5f, pz, 1};
                float MV[16]; mat_mul(MVP, M, MV);
                glUniformMatrix4fv(uMVP,1,GL_FALSE,MV);
                glUniform1f(uAlpha, 0.5f - s*0.13f);
                draw_gpumesh(&quad);
                g_dbg.drawn++;
            }
            glUniform1f(uAlpha,1.0f); glUniform1f(uSoft,0.0f);
            glDepthMask(GL_TRUE); glDisable(GL_BLEND);
        }

        /* tyre skid marks: flat dark quads on the ground, alpha-blended */
        if (skidn > 0) {
            glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDepthMask(GL_FALSE);
            glUniform1f(uUnlit, 1.0f); glUniform1f(uUseTex, 0.0f);
            glUniform3f(uColor, 0.04f, 0.04f, 0.05f);
            for (int i = 0; i < skidn; i++) {
                float *s = skid[i];                          /* ax,ay,az, bx,by,bz, life */
                if (race_state != 3) s[6] -= 0.0018f;        /* ~9s to fade (frozen on menu) */
                if (s[6] <= 0.0f) continue;
                float dx=s[3]-s[0], dy=s[4]-s[1];            /* segment direction */
                float len2=sqrtf(dx*dx+dy*dy); if(len2<1e-4f) continue;
                glUniform1f(uAlpha, 0.4f*s[6]);              /* fade with age */
                float ux=dx, uy=dy;                          /* along-travel axis (u) */
                const float SW=0.45f; float px=-dy/len2*SW, py=dx/len2*SW;  /* width axis (v) */
                /* unit quad (u,v) -> A + u*(B-A) + v*perp*W, centred across width */
                float M[16]={ ux,uy,0,0,  px,py,0,0,  0,0,1,0,
                              s[0]-px*0.5f, s[1]-py*0.5f, s[2], 1 };
                float MV[16]; mat_mul(MVP, M, MV);
                glUniformMatrix4fv(uMVP,1,GL_FALSE,MV); draw_gpumesh(&quad);
                g_dbg.drawn++;
            }
            glUniform1f(uAlpha, 1.0f); glDepthMask(GL_TRUE); glDisable(GL_BLEND);
        }

        /* car: solid-shaded, positioned + banked to the road (up = car_up) */
        if (ncar) {
            float Model[16], MVPc[16];
            /* Ride height = this car's OWN wheel radius. Measured across every
               car: the stock N2_CAR_TIRE hub centre sits at model Z = 0.000, so
               the body origin must ride exactly one wheel radius above the road
               for the tyre to touch and the hub to land inside the arch. The old
               fixed 0.43 was wrong for every vehicle (TT floated +0.053 m, 350Z
               +0.042, Hummer sank -0.049) -- that mismatch is what read as the
               chassis hovering above dislocated wheels.
               The wheel mesh is additionally scaled by wheel_scale in its own
               matrix, so the effective radius is carWheelR*wheel_scale -- ride
               tracks that product, keeping the car flush at any slider value. */
            float ride = carprof.ride
                       * (g_dbg.wheel_scale > 0.05f ? g_dbg.wheel_scale : 1.0f);
            mat_car(carpos, heading, car_up, ride, Model);
            mat_mul(MVP, Model, MVPc);
            glUniformMatrix4fv(uMVP, 1, GL_FALSE, MVPc);
            /* normals stay model-space, so counter-rotate the sun into car
               space — otherwise the lit side turns with the car's heading.
               The camera goes into model space the same way, for the
               shader's reflection vector. */
            { float ch=cosf(heading), sh=sinf(heading);
              float dx=cam[0]-carpos[0], dy=cam[1]-carpos[1], dz=cam[2]-(carpos[2]+ride);
              glUniform3f(uLight, ch*N2_SUN_X + sh*N2_SUN_Y,
                                 -sh*N2_SUN_X + ch*N2_SUN_Y, N2_SUN_Z);
              glUniform3f(rp.uCamPos, ch*dx + sh*dy, -sh*dx + ch*dy, dz); }
            /* rebuild the 4 wheel placements from the (live-tunable) fractions;
               the discs spin with road speed (visible via the radial rim tex) */
            { static float wang = 0.0f;
              /* rolling radius = this car's profile radius times the user scale,
                 so w=v/r and the RPM readout are correct per vehicle (a Hummer
                 rolls slower than a compact at the same speed). */
              float wR = carprof.wheel_r
                       * (g_dbg.wheel_scale > 0.05f ? g_dbg.wheel_scale : 1.0f);
              /* w = v/r per tick: speed is m/tick and this runs once per tick, so
                 the angle step is exactly speed/wR rad (the earlier *1/60 made the
                 tread crawl ~60x too slow -- ~3 rad/s instead of ~205 at 220 km/h). */
              wang += speed / wR;
              wang = fmodf(wang, 6.2831853f);
              float c = cosf(wang), sn = sinf(wang);
              /* visual steer: ease the raw -1/0/1 key state toward a ~28 deg
                 lock so the front wheels swing smoothly instead of snapping.
                 Rotation is about the wheel's own centre (the mesh is modelled
                 at the origin and the arch position lives in the translation
                 column), so steering can't swing it out of the arch. */
              static float vsteer = 0.0f;
              vsteer += (steer*0.50f - vsteer) * 0.25f;
              float sc = cosf(vsteer), ss = sinf(vsteer);
              /* telemetry: wheel RPM from w=v/r (rad/s -> RPM) and steer angle */
              g_dbg.wheel_rpm = fabsf(speed / wR) * 60.0f * 9.5493f;
              g_dbg.steer_deg = vsteer * 57.2958f;
              g_dbg.wheel_radius = wR;
              /* Explicit per-car stance (g_dbg.wheel): absolute axle X, per-axle
                 track and hub height, set at load from the table/fallback and
                 live-edited by the ImGui stance sliders. No fraction math here. */
              float fht = g_dbg.wheel.front_track*0.5f, rht = g_dbg.wheel.rear_track*0.5f,
                    s=g_dbg.wheel_scale, wz=g_dbg.wheel.ride_y;
              float wp[4][2]={{g_dbg.wheel.front_axle, fht},{g_dbg.wheel.front_axle,-fht},
                              {g_dbg.wheel.rear_axle,  rht},{g_dbg.wheel.rear_axle, -rht}};
              for(int k=0;k<4;k++){ float sy=(k&1)?-s:s;
                  /* rear axle: plain scale * rotY(wang) */
                  float M[16]={s*c,0,-s*sn,0, 0,sy,0,0, s*sn,0,s*c,0,
                               wp[k][0],wp[k][1],wz,1};
                  memcpy(wheelTAI[k],M,sizeof(M));   /* unsteered: AI reuse these */
                  if (k < 2) {   /* front axle: rotZ(steer) * that, per column */
                      float m2[16]={ s*c*sc,  s*c*ss,  -s*sn, 0,
                                     -sy*ss,  sy*sc,    0,    0,
                                     s*sn*sc, s*sn*ss,  s*c,  0,
                                     wp[k][0], wp[k][1], wz,  1 };
                      memcpy(M, m2, sizeof M);
                  }
                  memcpy(wheelT[k],M,sizeof(M)); } }
            float *pnt = g_dbg.paint_override ? g_dbg.paint : paint;
            /* uUnlit/uSoft were left ON (1.0) by the shadow/headlight-glow/
               skid-mark billboards drawn just above (all three intentionally
               use the unlit flat-colour shader path for those FX quads) —
               without this reset every car mesh below hits the shader's
               unlit early-return and skips texture sampling, decal blending,
               specular AND environment mapping entirely. This one leaked
               uniform is the actual root cause behind "the potato car" seen
               since Phase 6: none of the lit-path work (gloss, badges,
               reflections) was ever reaching the screen. */
            glUniform1f(uUnlit, 0.0f); glUniform1f(uSoft, 0.0f);
            /* per-mesh: each part wears its own bound texture (body/wheel/...);
               parts with no in-TPK texture get a sensible flat colour by class. */
            for (int i = 0; i < ncar; i++) {
                int c = cgm[i].cat;
                int is_light = (c==N2_CAR_LIGHT || c==N2_CAR_BRAKELIGHT);
                if ((c==N2_CAR_BODY && !g_dbg.show_body) ||
                    (is_light       && !g_dbg.show_lights)|| (c==N2_CAR_TIRE && !g_dbg.show_tires) ||
                    ((c==N2_CAR_MISC||c==N2_CAR_MECH) && !g_dbg.show_misc)) continue;
                if (c == N2_CAR_GLASS) continue;   /* translucent: blended pass below */
                /* plastic trim (bumpers/skirts, real per-part name tokens —
                   see n2_car_is_trim): duller and broader than the metallic
                   paint around it, so it doesn't read as the same "sticker"
                   material as the door/hood/fender panels. */
                float specv = (c==N2_CAR_BODY||c==N2_CAR_MISC)?g_dbg.body_spec
                            : is_light?0.45f : c==N2_CAR_MECH?0.05f : 0.0f;
                if (cgm[i].trim) specv *= 0.4f;
                if (cgm[i].roof) specv = 0.05f;   /* canvas soft-top: near-matte */
                glUniform1f(uSpec, specv);
                glUniform1f(uGloss, cgm[i].trim || cgm[i].roof ? 6.0f : 20.0f);
                /* no diffuse texture exists for any light part (verified
                   exhaustively against the data, see n2_car_category) — chrome
                   housing + coloured lens read entirely through reflection.
                   Mechanical compartment parts (engine/exhaust) are unpainted
                   metal/plastic when they have no texture of their own — no
                   body-paint gloss or reflection either. Soft-top canvas
                   doesn't reflect the environment like painted metal either. */
                glUniform1f(rp.uEnv, cgm[i].roof ? 0.02f
                                   : (c==N2_CAR_BODY||c==N2_CAR_MISC)?0.50f*g_dbg.body_env
                                   : is_light?0.55f : c==N2_CAR_MECH?0.0f : 0.15f);
                glUniform1f(rp.uDecal, 0.0f);   /* body branch may re-enable */
                GLuint tex = 0; int hasalpha = 0;
                for (int j = 0; j < nmap; j++) if (mapkey[j]==cgm[i].texkey) {
                    tex = maptex[j]; hasalpha = mapalpha[j]; break; }
                if (c == N2_CAR_BODY || c == N2_CAR_MISC) {
                    /* glossy paint; a mesh that references the badge/vinyl
                       atlas in its OWN 0x134012 slot list (a real per-mesh
                       data reference, e.g. GOLF's BASE_A) still decal-blends
                       it — that's verified correct (renders the actual VW
                       roundel). Panels with NO texture reference of their
                       own render as pure metallic paint, full stop: no
                       vinyl/badge fallback. (Removed: substituting the
                       shared composite onto any texture-less panel "because
                       it probably shares the same UV sheet" — false for
                       most of them; on the Miata almost every body/misc
                       mesh has no texkey at all, so the fallback painted
                       the composite's stretched hook-shape/checker pattern
                       across large panels like the engine bay, reading as
                       a solid mismatched block. TODO: the data actually
                       supports per-submesh materials via the 0x134B02
                       submesh table (mat_id -> its own 0x134011/0x134012) —
                       n2_walk_car currently assigns ONE texkey per whole
                       mesh object from the first material found. Modeling
                       submesh-level materials would let genuinely-textured
                       sub-regions (if any exist) resolve correctly instead
                       of an all-or-nothing per-object key. Not implemented. */
                    /* soft-top canvas ignores the player's paint choice —
                       real convertible tops don't get body-coloured, and
                       this mesh has no texture of its own to show fabric
                       weave instead. */
                    if (cgm[i].roof) glUniform3f(uColor, 0.035f, 0.032f, 0.03f);
                    else             glUniform3f(uColor, pnt[0], pnt[1], pnt[2]);
                    if (tex && !hasalpha) {
                        glUniform1f(uUseTex, 1.0f);
                        glBindTexture(GL_TEXTURE_2D, tex);
                    } else {
                        if (tex) {
                            glUniform1f(uUseTex, 1.0f); glUniform1f(rp.uDecal, 1.0f);
                            glBindTexture(GL_TEXTURE_2D, tex);
                        } else glUniform1f(uUseTex, 0.0f);
                    }
                } else if (tex) {
                    glUniform1f(uUseTex, 1.0f); glBindTexture(GL_TEXTURE_2D, tex);  /* wheel/brake */
                } else {
                    glUniform1f(uUseTex, 0.0f);
                    if      (c == N2_CAR_LIGHT)      glUniform3f(uColor, 0.92f, 0.90f, 0.85f);  /* clear/chrome lens */
                    else if (c == N2_CAR_BRAKELIGHT) {
                        /* lens glows hot while braking (S, rolling forward) or the
                           handbrake (SPACE) is pulled; otherwise its unlit red. */
                        int braking = (throttle < -0.1f && speed > 0.01f) || handbrake;
                        if (braking) glUniform3f(uColor, 1.0f, 0.16f, 0.10f);
                        else         glUniform3f(uColor, 0.60f, 0.02f, 0.02f);
                    }
                    else if (c == N2_CAR_TIRE)       glUniform3f(uColor, 0.05f, 0.05f, 0.06f);
                    else if (c == N2_CAR_MECH)       glUniform3f(uColor, 0.05f, 0.05f, 0.05f);  /* unpainted metal/plastic */
                    else                              glUniform3f(uColor, pnt[0], pnt[1], pnt[2]);
                }
#ifdef DEBUG_UI
                /* Mesh Inspector overlay: purely a draw-state override on the
                   selected mesh -- no asset, parser or transform is touched. */
                int insp_on = (g_dbg.insp_sel == i);
                if (insp_on && g_dbg.insp_highlight) {
                    glUniform1f(uUseTex, 0.0f); glUniform1f(rp.uDecal, 0.0f);
                    glUniform1f(uSpec, 0.0f); glUniform1f(rp.uEnv, 0.0f);
                    glUniform1f(uUnlit, 1.0f);
                    glUniform3f(uColor, 1.0f, 0.08f, 0.85f);   /* neon magenta */
                }
                if (insp_on && g_dbg.insp_wire)
                    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                if (insp_on && g_dbg.insp_flipn) glUniform1f(rp.uFlipN, 1.0f);
                if (insp_on && g_dbg.insp_cull) {
                    /* culling is OFF engine-wide, so glFrontFace alone would be
                       inert; enable it here so winding actually has an effect
                       and the inversion question becomes testable. */
                    glEnable(GL_CULL_FACE);
                    glCullFace(g_dbg.insp_cull == 1 ? GL_BACK : GL_FRONT);
                }
#endif
                if (c != N2_CAR_TIRE) { draw_gpumesh(&cgm[i]); g_dbg.drawn++; }   /* tyres = procedural, below */
#ifdef DEBUG_UI
                if (insp_on && g_dbg.insp_wire) glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                if (insp_on && g_dbg.insp_highlight) glUniform1f(uUnlit, 0.0f);
                if (insp_on && g_dbg.insp_flipn) glUniform1f(rp.uFlipN, 0.0f);
                if (insp_on && g_dbg.insp_cull) glDisable(GL_CULL_FACE);
#endif
            }
            /* glass pass: translucent tint, blended over the finished body,
               depth-write off (no self-occlusion), spec kept by the shader's
               uAlpha output. State restored before anything else draws. */
            if (g_dbg.show_glass) {
                glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
#ifdef DEBUG_UI
                glDepthMask(g_dbg.insp_glass_depth ? GL_TRUE : GL_FALSE);
#else
                glDepthMask(GL_FALSE);
#endif
                glUniform1f(rp.uDecal, 0.0f); glUniform1f(uUseTex, 0.0f);
                glUniform1f(uSpec, 0.6f); glUniform1f(uGloss, 20.0f); glUniform1f(uAlpha, 0.55f);
                glUniform1f(rp.uEnv, 0.8f);   /* glass reflects hardest */
                glUniform3f(uColor, 0.10f, 0.13f, 0.17f);
                for (int i = 0; i < ncar; i++)
                    if (cgm[i].cat == N2_CAR_GLASS) {
#ifdef DEBUG_UI
                        /* the opaque loop skips glass, so the inspector overlay
                           has to be applied here too or selecting a window did
                           nothing at all. */
                        int gi = (g_dbg.insp_sel == i);
                        if (gi && g_dbg.insp_highlight) {
                            glUniform1f(uUnlit, 1.0f); glUniform1f(uAlpha, 1.0f);
                            glUniform3f(uColor, 1.0f, 0.08f, 0.85f);
                        }
                        if (gi && g_dbg.insp_wire) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
#endif
                        draw_gpumesh(&cgm[i]); g_dbg.drawn++;
#ifdef DEBUG_UI
                        if (gi && g_dbg.insp_wire) glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                        if (gi && g_dbg.insp_highlight) {
                            glUniform1f(uUnlit, 0.0f); glUniform1f(uAlpha, 0.55f);
                            glUniform3f(uColor, 0.10f, 0.13f, 0.17f);
                        }
#endif
                    }
                glUniform1f(uAlpha, 1.0f);
                glDepthMask(GL_TRUE); glDisable(GL_BLEND);
            }
            /* procedural tyres at the 4 arches (the game rims render as urchins);
               the radial rim texture gives them a hub + spokes instead of a void */
            if (have_wheel && g_dbg.show_tires) {
                glUniform1f(rp.uDecal, 0.0f);
                /* Authentic stock wheel: the car's OWN FRONT_WHEEL mesh (rim +
                   tyre) from its GEOMETRY.BIN, with the flat backing-plane quad
                   culled at load, instanced at all four AttribSys corners. This
                   is the real factory wheel, so it wins over the shared rim
                   library below whenever the car ships one (all but the 2 tyre-
                   less cars). At blur speed the procedural disc still takes over. */
                if (stock_wheel >= 0 && PHYS_KMH(speed) <= WHEEL_BLUR_KMH) {
                    GLuint stex=0; for(int j=0;j<nmap;j++) if(mapkey[j]==cgm[stock_wheel].texkey){stex=maptex[j];break;}
                    glUniform1f(uUseTex, stex?1.0f:0.0f); glUniform1f(rp.uEnv, 0.25f);
                    glUniform1f(uSpec, 0.5f); glUniform3f(uColor,0.6f,0.6f,0.62f);
                    if (stex) glBindTexture(GL_TEXTURE_2D, stex);
                    for (int k=0;k<4;k++){ float MVPw[16]; mat_mul(MVPc, wheelT[k], MVPw);
                        glUniformMatrix4fv(uMVP,1,GL_FALSE,MVPw); draw_gpumesh(&cgm[stock_wheel]); }
                    g_dbg.drawn+=4; glUniformMatrix4fv(uMVP,1,GL_FALSE,MVPc);
                    goto wheels_drawn;
                }
                /* Geometric rim below the blur threshold, else the procedural
                   disc (its angular-averaged sheet sells the motion blur).
                   The WHEELS/TEXTURES.BIN rim diffuse is AUTHENTIC (blue spans
                   the full 0..255; the low average is just a gold/bronze rim,
                   not a decode bug), so it is bound directly and shows the real
                   manufacturer colour. uEnv=0 keeps the warm env sphere off it
                   (that, not the texture, was the old green cast) and a strong
                   specular adds the chrome highlight over the diffuse.
                   uColor is ignored on the textured path (base = t.rgb).
                   PHYS_KMH(speed), not g_dbg.kmh, which lags a frame here. */
                int geo = nwheelgm > 0 && PHYS_KMH(speed) <= WHEEL_BLUR_KMH;
                if (geo) {
                    /* Rim paint: bind the authentic OEM diffuse and recolor it
                       toward the chosen rim colour (silver default) via uRimTint,
                       keeping the spoke detail. rim_paint=0 shows the raw gold. */
                    glUniform1f(uUseTex, rimtex ? 1.0f : 0.0f);
                    glUniform1f(rp.uEnv, 0.0f); glUniform1f(uSpec, 0.85f);
                    glUniform3fv(uColor, 1, g_dbg.rim_color);
                    glUniform1f(rp.uRimTint, rimtex && g_dbg.rim_paint ? 1.0f : 0.0f);
                    if (rimtex) glBindTexture(GL_TEXTURE_2D, rimtex);
                } else {
                    glUniform1f(uUseTex, 1.0f); glUniform1f(rp.uEnv, 0.3f);
                    glUniform1f(uSpec, 0.4f);
                    glBindTexture(GL_TEXTURE_2D,
                        PHYS_KMH(speed) > WHEEL_BLUR_KMH ? texWheelBlur : texWheel);
                }
                for (int k=0;k<4;k++){ float MVPw[16]; mat_mul(MVPc, wheelT[k], MVPw);
                    glUniformMatrix4fv(uMVP,1,GL_FALSE,MVPw);
                    draw_gpumesh(geo ? &wheelgm[0] : &wheelmesh); }
                g_dbg.drawn += 4;
                glUniform1f(rp.uRimTint, 0.0f);   /* rim paint is rim-only */
                glUniformMatrix4fv(uMVP,1,GL_FALSE,MVPc);
                wheels_drawn: ;
            }
            glUniform1f(uUseTex, 0.0f);
            glUniform1f(uSpec, 0.3f);     /* AIs: flat colour but glossy paint */
            glUniform1f(rp.uEnv, 0.35f);
            /* AI opponents — same body, each in its own colour */
            for (int k = 0; k < nai; k++) {
                float aup[3]; world_ground_normal(&scene, ais[k].pos[0], ais[k].pos[1], aup);
                mat_car(ais[k].pos, ais[k].head, aup, ride, Model);   /* AI: same per-car ride */
                mat_mul(MVP, Model, MVPc);
                glUniformMatrix4fv(uMVP, 1, GL_FALSE, MVPc);
                { float ch=cosf(ais[k].head), sh=sinf(ais[k].head);
                  float dx=cam[0]-ais[k].pos[0], dy=cam[1]-ais[k].pos[1];
                  float dz=cam[2]-(ais[k].pos[2]+ride);
                  glUniform3f(uLight, ch*N2_SUN_X + sh*N2_SUN_Y,
                                     -sh*N2_SUN_X + ch*N2_SUN_Y, N2_SUN_Z);
                  glUniform3f(rp.uCamPos, ch*dx + sh*dy, -sh*dx + ch*dy, dz); }
                glUniform3f(uColor, ais[k].col[0], ais[k].col[1], ais[k].col[2]);
                for (int i = 0; i < ncar; i++)
                    if (cgm[i].cat != N2_CAR_TIRE) { draw_gpumesh(&cgm[i]); g_dbg.drawn++; }
                if (have_wheel && g_dbg.show_tires) {     /* procedural tyres */
                    glUniform1f(uUseTex, 1.0f); glBindTexture(GL_TEXTURE_2D, texWheel);
                    for (int w=0;w<4;w++){ float MVPw[16]; mat_mul(MVPc, wheelTAI[w], MVPw);
                        glUniformMatrix4fv(uMVP,1,GL_FALSE,MVPw); draw_gpumesh(&wheelmesh); }
                    g_dbg.drawn += 4;
                    glUniformMatrix4fv(uMVP,1,GL_FALSE,MVPc);
                    glUniform1f(uUseTex, 0.0f);
                    glUniform3f(uColor, ais[k].col[0], ais[k].col[1], ais[k].col[2]);
                }
            }
            glUniformMatrix4fv(uMVP, 1, GL_FALSE, MVP);
            glUniform3f(uLight, N2_SUN_X, N2_SUN_Y, N2_SUN_Z);   /* back to world */
            glUniform1f(rp.uEnv, 0.0f);   /* reflections are cars-only */
        }

        /* tail lights: red camera-facing glows at each car's rear (night) */
        if (race_state != 3 && ncar) {
            float lz=sqrtf(look[0]*look[0]+look[1]*look[1]+look[2]*look[2]); if(lz<1e-4f)lz=1;
            float ld[3]={look[0]/lz,look[1]/lz,look[2]/lz};
            float rt[3]={ld[1],-ld[0],0}; float rl=sqrtf(rt[0]*rt[0]+rt[1]*rt[1]); if(rl<1e-4f)rl=1;
            rt[0]/=rl; rt[1]/=rl;
            float up[3]={rt[1]*ld[2], -rt[0]*ld[2], rt[0]*ld[1]-rt[1]*ld[0]};
            glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE);
            glDepthMask(GL_FALSE); glDisable(GL_DEPTH_TEST);   /* glow overlay on the rear */
            glUniform1f(uUnlit,1.0f); glUniform1f(uUseTex,0.0f); glUniform1f(uSoft,1.0f);
            glUniform3f(uColor, 1.0f, 0.12f, 0.06f); glUniform1f(uAlpha, 0.9f);
            for (int c=0; c<=nai; c++) {
                float *cp = c==0 ? carpos : ais[c-1].pos;
                float hd = c==0 ? heading : ais[c-1].head;
                float fx=cosf(hd), fy=sinf(hd), rx=-fy, ry=fx;
                for (int side=-1; side<=1; side+=2) {
                    float bx=cp[0]-fx*1.9f+rx*0.6f*side, by=cp[1]-fy*1.9f+ry*0.6f*side, bz=cp[2]+0.5f;
                    float s=1.1f;
                    float cxp=bx-(rt[0]+up[0])*s*0.5f, cyp=by-(rt[1]+up[1])*s*0.5f, czp=bz-(rt[2]+up[2])*s*0.5f;
                    float M[16]={rt[0]*s,rt[1]*s,rt[2]*s,0, up[0]*s,up[1]*s,up[2]*s,0, 0,0,1,0, cxp,cyp,czp,1};
                    float MV[16]; mat_mul(MVP,M,MV);
                    glUniformMatrix4fv(uMVP,1,GL_FALSE,MV); draw_gpumesh(&quad);
                    g_dbg.drawn++;
                }
            }
            glUniform1f(uAlpha,1.0f); glUniform1f(uSoft,0.0f);
            glDepthMask(GL_TRUE); glEnable(GL_DEPTH_TEST); glDisable(GL_BLEND);
        }

        /* drift smoke: camera-facing billboards, light + fading, additive-ish */
        if (smoken > 0) {
            float lz = sqrtf(look[0]*look[0]+look[1]*look[1]+look[2]*look[2]); if(lz<1e-4f)lz=1;
            float ld[3]={look[0]/lz,look[1]/lz,look[2]/lz};
            float rt[3];                                   /* look x up(0,0,1) */
            rt[0]=ld[1]; rt[1]=-ld[0]; rt[2]=0;
            float rl=sqrtf(rt[0]*rt[0]+rt[1]*rt[1]); if(rl<1e-4f)rl=1; rt[0]/=rl;rt[1]/=rl;
            float up[3]={rt[1]*ld[2]-0, 0-rt[0]*ld[2], rt[0]*ld[1]-rt[1]*ld[0]}; /* right x look */
            glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDepthMask(GL_FALSE);
            glUniform1f(uUnlit, 1.0f); glUniform1f(uUseTex, 0.0f); glUniform1f(uSoft, 1.0f);
            glUniform3f(uColor, 0.78f, 0.78f, 0.80f);
            for (int i = 0; i < smoken; i++) {
                float s = smoke[i].size, *p = smoke[i].p;
                float cxp = p[0]-(rt[0]+up[0])*s*0.5f, cyp = p[1]-(rt[1]+up[1])*s*0.5f,
                      czp = p[2]-(rt[2]+up[2])*s*0.5f;
                float M[16]={ rt[0]*s,rt[1]*s,rt[2]*s,0,  up[0]*s,up[1]*s,up[2]*s,0,
                              0,0,1,0,  cxp,cyp,czp,1 };
                float MV[16]; mat_mul(MVP, M, MV);
                glUniformMatrix4fv(uMVP,1,GL_FALSE,MV);
                glUniform1f(uAlpha, smoke[i].life*smoke[i].life*0.22f);
                draw_gpumesh(&quad);
                g_dbg.drawn++;
            }
            glUniform1f(uAlpha,1.0f); glUniform1f(uSoft,0.0f); glDepthMask(GL_TRUE); glDisable(GL_BLEND);
        }

        /* neon signs / bulbs / lens flares: additive pass at the very end of
           the 3D frame, using each mesh's own texture as its emissive colour
           (these never receive diffuse lighting in-game — they ARE the
           light). Depth test stays on (a sign behind a building must still
           be occluded); only the write is off, so overlapping glows blend
           into each other instead of fighting on depth. */
        if (nglow && g_dbg.show_track) {
            glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE);
            glDepthMask(GL_FALSE);
            glUniform1f(uUnlit, 1.0f);
            GLuint lastglowtex = (GLuint)-1;
            for (int k = 0; k < nglow; k++) {
                N2Batch *b = &glowbatch[k];
                if (b->tex != lastglowtex) {
                    if (b->tex) { glUniform1f(uUseTex, 1.0f); glBindTexture(GL_TEXTURE_2D, b->tex); }
                    else { glUniform1f(uUseTex, 0.0f); glUniform3f(uColor, 1.0f, 0.85f, 0.5f); }
                    lastglowtex = b->tex;
                }
                draw_batch(b);
                g_dbg.drawn++;
            }
            glUniform1f(uUnlit, 0.0f); glDepthMask(GL_TRUE); glDisable(GL_BLEND);
        }

        /* HUD: race-position leaderboard — one colour bar per car, ordered by
           progress along the racing line (leader on top); player bar wider. */
        /* race telemetry (position/lap/speed) always mirrored to g_dbg for
           the ImGui panel; the viewport HUD drawing itself is additionally
           gated so debug builds can hide it (same pattern as the menu HUD
           above — plain builds have no ImGui, so they always draw it). */
        if (nai > 0 && race_state != 3) {
            int myprog = p_lap*aipath.n + p_prev, ppos_mirror = 1;
            for (int k = 0; k < nai; k++)
                if (ais[k].lap*aipath.n + ais[k].prevrel > myprog) ppos_mirror++;
            g_dbg.race_pos = ppos_mirror; g_dbg.race_cars = nai + 1;
            g_dbg.race_lap = p_lap<LAP_TARGET?p_lap+1:LAP_TARGET; g_dbg.race_laps = LAP_TARGET;
        } else g_dbg.race_cars = 0;   /* not racing: ImGui readout hides itself */
#ifdef DEBUG_UI
        int draw_race_hud = !g_dbg.hud_hide_menu;
#else
        int draw_race_hud = 1;
#endif
        if (nai > 0 && race_state != 3 && draw_race_hud) {
            glDisable(GL_DEPTH_TEST);
            glUniform1f(uUnlit, 1.0f); glUniform1f(uUseTex, 0.0f);
            /* rank by monotonic progress = lap*loop + progress-along-loop */
            int nc = nai + 1, ord[N_AI+1], prog[N_AI+1], pl[N_AI+1];
            float col[N_AI+1][3];
            prog[0] = p_lap*aipath.n + p_prev; pl[0]=1;
            col[0][0]=0.85f; col[0][1]=0.12f; col[0][2]=0.12f;
            for (int k=0;k<nai;k++){
                prog[k+1]=ais[k].lap*aipath.n + ais[k].prevrel;
                memcpy(col[k+1], ais[k].col, sizeof col[0]); pl[k+1]=0;
            }
            for (int i=0;i<nc;i++) ord[i]=i;
            for (int i=0;i<nc;i++) for (int j=i+1;j<nc;j++)
                if (prog[ord[j]] > prog[ord[i]]) { int t=ord[i]; ord[i]=ord[j]; ord[j]=t; }
            for (int i=0;i<nc;i++){
                int c=ord[i];
                float w = pl[c]?0.10f:0.06f, x=-0.97f, y=0.90f-i*0.10f, h=0.075f;
                float M[16]={ w,0,0,0, 0,h,0,0, 0,0,1,0, x,y,0,1 };
                glUniformMatrix4fv(uMVP, 1, GL_FALSE, M);
                glUniform3f(uColor, col[c][0], col[c][1], col[c][2]);
                draw_gpumesh(&quad);
                g_dbg.drawn++;
            }
            /* lap counter (green pips, one per completed lap) + lap-progress bar */
            for (int i=0;i<p_lap && i<8;i++){
                float x=-0.97f+i*0.05f, y=-0.93f;
                float M[16]={0.035f,0,0,0, 0,0.05f,0,0, 0,0,1,0, x,y,0,1};
                glUniformMatrix4fv(uMVP,1,GL_FALSE,M);
                glUniform3f(uColor,0.2f,0.9f,0.3f); draw_gpumesh(&quad);
                g_dbg.drawn++;
            }
            {   /* thin bar: fraction of the current lap completed */
                float frac = (float)p_prev / (float)aipath.n;
                float bg[16]={1.5f,0,0,0, 0,0.03f,0,0, 0,0,1,0, -0.75f,-0.9f,0,1};
                glUniformMatrix4fv(uMVP,1,GL_FALSE,bg);
                glUniform3f(uColor,0.15f,0.15f,0.18f); draw_gpumesh(&quad);
                g_dbg.drawn++;
                float fg[16]={1.5f*frac,0,0,0, 0,0.03f,0,0, 0,0,1,0, -0.75f,-0.9f,0,1};
                glUniformMatrix4fv(uMVP,1,GL_FALSE,fg);
                glUniform3f(uColor,0.9f,0.8f,0.2f); draw_gpumesh(&quad);
                g_dbg.drawn++;
            }
            {   /* speedometer: fraction of top speed (bottom-right) */
                float sf = speed / PHYS_MAXSPD; if (sf < 0) sf = -sf; if (sf > 1) sf = 1;
                float bg[16]={0.35f,0,0,0, 0,0.03f,0,0, 0,0,1,0, 0.58f,-0.9f,0,1};
                glUniformMatrix4fv(uMVP,1,GL_FALSE,bg);
                glUniform3f(uColor,0.15f,0.15f,0.18f); draw_gpumesh(&quad);
                g_dbg.drawn++;
                float fg[16]={0.35f*sf,0,0,0, 0,0.03f,0,0, 0,0,1,0, 0.58f,-0.9f,0,1};
                glUniformMatrix4fv(uMVP,1,GL_FALSE,fg);
                glUniform3f(uColor,0.3f,0.85f,1.0f); draw_gpumesh(&quad);
                g_dbg.drawn++;
            }
            {   /* text labels: position (top-right), lap (bottom-left), speed (kph) */
                char buf[32];
                int ppos = 1; for (int i=0;i<nc;i++) if (ord[i]==0) { ppos=i+1; break; }
                snprintf(buf,sizeof buf,"P%d/%d", ppos, nc);
                glUniform3f(uColor,1,1,1);
                draw_text(&quad, uMVP, buf, -text_width(buf,0.024f)/2, 0.96f, 0.024f, 0.036f);
                snprintf(buf,sizeof buf,"LAP %d/%d", p_lap<LAP_TARGET?p_lap+1:LAP_TARGET, LAP_TARGET);
                draw_text(&quad, uMVP, buf, -0.97f, -0.80f, 0.018f, 0.028f);
                int kph = (int)PHYS_KMH(speed<0?-speed:speed);
                snprintf(buf,sizeof buf,"%d", kph);
                glUniform3f(uColor,0.3f,0.85f,1.0f);
                draw_text(&quad, uMVP, buf, 0.58f, -0.80f, 0.02f, 0.03f);
            }
            if (aipath.n > 1) {   /* minimap (top-right): racing line + car dots */
                float bx0=0.66f, by0=0.52f, bw=0.30f, bh=0.36f;
                float px0=1e30f,py0=1e30f,px1=-1e30f,py1=-1e30f;
                for (int i=0;i<aipath.n;i++){ float x=aipath.xy[i*2],y=aipath.xy[i*2+1];
                    if(x<px0)px0=x; if(x>px1)px1=x; if(y<py0)py0=y; if(y>py1)py1=y; }
                float asp=(float)W/H, span=(px1-px0)>(py1-py0)?(px1-px0):(py1-py0);
                if (span < 1e-3f) span = 1e-3f;
                float scy = bh*0.9f/span, scx = scy/asp;   /* uniform on screen */
                float ccx=(px0+px1)*0.5f, cyw=(py0+py1)*0.5f;
                float mmx0 = bx0+bw*0.5f, mmy0 = by0+bh*0.5f;
                /* dark panel */
                float PM[16]={bw,0,0,0, 0,bh,0,0, 0,0,1,0, bx0,by0,0,1};
                glUniformMatrix4fv(uMVP,1,GL_FALSE,PM); glUniform3f(uColor,0.04f,0.05f,0.08f); draw_gpumesh(&quad);
                g_dbg.drawn++;
                /* draw a centred dot of half-size hs at world (wx,wy) */
                #define MMDOT(wx,wy,hs,r,g,b) do{ \
                    float mmx=mmx0+((wx)-ccx)*scx, mmy=mmy0+((wy)-cyw)*scy; \
                    float M[16]={(hs)*2/asp,0,0,0, 0,(hs)*2,0,0, 0,0,1,0, mmx-(hs)/asp,mmy-(hs),0,1}; \
                    glUniformMatrix4fv(uMVP,1,GL_FALSE,M); glUniform3f(uColor,r,g,b); draw_gpumesh(&quad); \
                    g_dbg.drawn++; }while(0)
                for (int i=0;i<aipath.n;i++) MMDOT(aipath.xy[i*2],aipath.xy[i*2+1], 0.004f, 0.4f,0.4f,0.46f);
                for (int k=0;k<nai;k++) MMDOT(ais[k].pos[0],ais[k].pos[1], 0.010f, ais[k].col[0],ais[k].col[1],ais[k].col[2]);
                MMDOT(carpos[0],carpos[1], 0.013f, 0.95f,0.15f,0.15f);
                #undef MMDOT
            }
            glEnable(GL_DEPTH_TEST);
        }

        /* race banners: 3-2-1 countdown, then a finish banner + place pips */
        glDisable(GL_DEPTH_TEST);
        glUniform1f(uUnlit, 1.0f); glUniform1f(uUseTex, 0.0f);
#ifdef DEBUG_UI
        /* the retro pixel-font menu overlay is off by default under the debug
           build (session info lives in the ImGui panel instead, see below) —
           re-enable it live with the panel's "show 3D menu HUD" checkbox.
           Plain (non-debug) builds always draw it: it is their ONLY UI. */
        int draw_menu_hud = !g_dbg.hud_hide_menu;
#else
        int draw_menu_hud = 1;
#endif
        if (race_state == 3 && draw_menu_hud) {
            /* car selector (Left/Right): a tight row of pips, chosen lit white */
            float cw = 0.02f, cx0 = -((ncars-1)*cw)/2.0f;
            for (int i = 0; i < ncars; i++) {
                int s = (i == selcar);
                float M[16]={0.014f,0,0,0, 0,s?0.05f:0.028f,0,0, 0,0,1,0, cx0+i*cw,0.44f,0,1};
                glUniformMatrix4fv(uMVP,1,GL_FALSE,M);
                if (s) glUniform3f(uColor,0.95f,0.95f,0.98f);
                else   glUniform3f(uColor,0.30f,0.32f,0.38f);
                draw_gpumesh(&quad);
                g_dbg.drawn++;
            }
            /* track selector (Up/Down): one blue pip per STREAM*.BUN found */
            float tx0 = -((ntrack-1)*0.05f)/2.0f;
            for (int i = 0; i < ntrack; i++) {
                int s = (i == seltrack);
                float M[16]={0.035f,0,0,0, 0,s?0.05f:0.03f,0,0, 0,0,1,0, tx0+i*0.05f,0.36f,0,1};
                glUniformMatrix4fv(uMVP,1,GL_FALSE,M);
                if (s) glUniform3f(uColor,0.3f,0.7f,1.0f);
                else   glUniform3f(uColor,0.25f,0.3f,0.4f);
                draw_gpumesh(&quad);
                g_dbg.drawn++;
            }
            /* circuit selector ([ / ]): one yellow pip per circuit, chosen lit + tall */
            float x0 = -((ncirc-1)*0.05f)/2.0f;
            for (int i = 0; i < ncirc; i++) {
                int s = (i == selcirc);
                float h = s?0.05f:0.03f, y = 0.29f - (s?0.01f:0);
                float M[16]={0.035f,0,0,0, 0,h,0,0, 0,0,1,0, x0+i*0.05f,y,0,1};
                glUniformMatrix4fv(uMVP,1,GL_FALSE,M);
                if (s) glUniform3f(uColor,0.95f,0.8f,0.2f);
                else   glUniform3f(uColor,0.3f,0.3f,0.35f);
                draw_gpumesh(&quad);
                g_dbg.drawn++;
            }
            /* labels: car name (big, white) above its pips; track name below */
            glUniform3f(uColor, 0.95f, 0.95f, 0.98f);
            draw_text(&quad, uMVP, carname, -text_width(carname,0.017f)/2, 0.56f, 0.017f, 0.026f);
            glUniform3f(uColor, 0.45f, 0.7f, 1.0f);
            draw_text(&quad, uMVP, trackname, -text_width(trackname,0.012f)/2, 0.23f, 0.012f, 0.02f);
            /* "press ENTER" prompt: a gently pulsing green bar */
            float pulse = 0.55f + 0.45f*sinf(menuspin*6.0f);
            float M[16]={0.5f,0,0,0, 0,0.06f,0,0, 0,0,1,0, -0.25f,-0.25f,0,1};
            glUniformMatrix4fv(uMVP,1,GL_FALSE,M);
            glUniform3f(uColor,0.15f*pulse,0.85f*pulse,0.3f*pulse);
            draw_gpumesh(&quad);
            g_dbg.drawn++;
        } else if (race_state == 0) {         /* 3-2-1 / GO, big and centred */
            if (racetimer >= COUNTDOWN-24) {
                glUniform3f(uColor,0.2f,0.95f,0.3f);
                draw_text(&quad, uMVP, "GO", -text_width("GO",0.13f)/2, 0.44f, 0.13f, 0.17f);
            } else {
                char b[4]; snprintf(b,sizeof b,"%d", 3 - racetimer/60);
                glUniform3f(uColor,0.95f,0.25f,0.15f);
                draw_text(&quad, uMVP, b, -text_width(b,0.13f)/2, 0.44f, 0.13f, 0.17f);
            }
        } else if (race_state == 2) {         /* finished: FINISH + place */
            glUniform3f(uColor,0.95f,0.8f,0.2f);
            draw_text(&quad, uMVP, "FINISH", -text_width("FINISH",0.07f)/2, 0.36f, 0.07f, 0.10f);
            char b[16]; snprintf(b,sizeof b,"P%d/%d", finish_place, nai+1);
            glUniform3f(uColor,1,1,1);
            draw_text(&quad, uMVP, b, -text_width(b,0.05f)/2, 0.14f, 0.05f, 0.075f);
        }
        glEnable(GL_DEPTH_TEST);

        g_dbg.nav = world.nav; g_dbg.nnav = world.nnav;
        g_dbg.navedge = world.navedge; g_dbg.nnavedge = world.nnavedge;
        g_dbg.navcomp = world.navcomp; g_dbg.ndist = world.ndist;
        for (int i = 0; i < world.ndist && i < 8; i++) {
            snprintf(g_dbg.dist_tok[i], 4, "%s", world.dist[i].tok);
            snprintf(g_dbg.dist_name[i], 24, "%s", world_district_name(world.dist[i].tok));
        }
        /* Race & Track Manager: mirror the catalog, apply panel mode switches */
        g_dbg.ev = world.ev;  g_dbg.ev_count  = world.nev;
        g_dbg.bar = world.bar; g_dbg.bar_count = world.nbar;
        g_dbg.mode = world.mode; g_dbg.active_ev = world.active_ev;
        g_dbg.masked_links = world.nmasked;
        g_dbg.navopen = world.mode == MODE_RACE_EVENT ? world.navopen : NULL;
        g_dbg.race = &world.race;
        if (g_dbg.mode_request) {
            g_dbg.mode_request = 0;
            world_race_stop(&world);
            world_set_mode(&world, g_dbg.want_mode, g_dbg.want_event);
            g_dbg.gps_n = 0;             /* the old route may cross a new barrier */
        }
        if (g_dbg.race_start_request) {
            g_dbg.race_start_request = 0;
            if (world.active_ev >= 0)
                world_race_start(&world, troot, world.active_ev, g_dbg.race_maxlaps_want);
        }
        if (g_dbg.race_stop_request) { g_dbg.race_stop_request = 0; world_race_stop(&world); }
        /* GPS: solve a fresh route whenever the panel asks for a destination */
        {   static int gpath[8192];
            if (g_dbg.gps_request) {
                g_dbg.gps_request = 0;
                int s0 = world_nav_nearest(&world, carpos[0], carpos[1]);
                int g0 = world_nav_nearest(&world, g_dbg.gps_want_x, g_dbg.gps_want_y);
                float gd = 0; uint32_t ta = SDL_GetTicks();
                int gn = world_route(&world, s0, g0, gpath, 8192, &gd);
                g_dbg.gps_ms = (int)(SDL_GetTicks() - ta);
                g_dbg.gps_path = gpath; g_dbg.gps_n = gn; g_dbg.gps_dist = gd;
                printf("GPS route: %d nodes, %.0f m, %d ms\n", gn, gd, g_dbg.gps_ms);
            }
        }
        for (int i = 0; i < 4; i++) g_dbg.navbb[i] = world.navbb[i];
        /* live district tracking: log every boundary crossing */
        {   static int lastzone = -2;
            int zi = world_district_at(&world, carpos[0], carpos[1], 150.0f);
            static char zbuf[24];
            if (zi >= 0 && zi < world.ndist) snprintf(zbuf, sizeof zbuf, "%s", world.dist[zi].tok);
            else snprintf(zbuf, sizeof zbuf, "-");
            const char *zn = zbuf;
            if (zi != lastzone) {
                if (lastzone != -2)
                    printf("Transition: %s -> %s   at (%.0f, %.0f)\n",
                           (lastzone >= 0 && lastzone < world.ndist) ? world.dist[lastzone].tok : "-", zn,
                           carpos[0], carpos[1]);
                lastzone = zi;
            }
            snprintf(g_dbg.zone_name, sizeof g_dbg.zone_name, "%s", zn);
            g_dbg.zone_count = world.ndist;
        }
#ifdef DEBUG_UI     /* debug readouts + ImGui overlay, drawn on top of everything */
        g_dbg.cam[0]=cam[0]; g_dbg.cam[1]=cam[1]; g_dbg.cam[2]=cam[2];
        g_dbg.car[0]=carpos[0]; g_dbg.car[1]=carpos[1]; g_dbg.car[2]=carpos[2];
        g_dbg.heading=heading; g_dbg.kmh=PHYS_KMH(speed);
        g_dbg.car_meshes=ncar; g_dbg.track_meshes=nm;
        snprintf(g_dbg.car_name, sizeof g_dbg.car_name, "%s", carname);
        snprintf(g_dbg.track_name, sizeof g_dbg.track_name, "%s", trackname);
        g_dbg.sel_car=selcar; g_dbg.n_cars=ncars;
        g_dbg.sel_track=seltrack; g_dbg.n_tracks=ntrack;
        g_dbg.sel_circuit=selcirc; g_dbg.n_circuits=ncirc;
        {   static int icat[512], ivts[512];
            int ni = ncar < 512 ? ncar : 512;
            for (int i = 0; i < ni; i++) { icat[i]=cgm[i].cat; ivts[i]=car.meshes[i].nverts; }
            g_dbg.insp_count = ni; g_dbg.insp_cat = icat; g_dbg.insp_verts = ivts;
        }
        g_dbg.scripted = sdefs; g_dbg.scripted_count = nsd;
        /* scenery semantics: class census + the named chunks nearest the car */
        {   for (int i = 0; i < 8; i++) g_dbg.scen_count[i] = 0;
            for (int i = 0; i < nm; i++) {
                int sc = scene.meshes[i].scen; if (sc < 8) g_dbg.scen_count[sc]++; }
            int nn = 0;
            for (int i = 0; i < nm && nn < 12; i++) {
                if (!scene.meshes[i].scen || !scene.meshes[i].sname[0]) continue;
                float *bb = world.mbb[i];
                float dx = carpos[0]<bb[0]?bb[0]-carpos[0]:(carpos[0]>bb[2]?carpos[0]-bb[2]:0);
                float dy = carpos[1]<bb[1]?bb[1]-carpos[1]:(carpos[1]>bb[3]?carpos[1]-bb[3]:0);
                float dd = sqrtf(dx*dx+dy*dy);
                if (dd > 60.0f) continue;
                snprintf(g_dbg.scen_near[nn], sizeof g_dbg.scen_near[0], "%-24s %-8s %4.0fm",
                         scene.meshes[i].sname, n2_scen_name(scene.meshes[i].scen), dd);
                nn++;
            }
            g_dbg.scen_near_n = nn;
        }
        g_dbg.wheel_brands = wheel_brands;
        g_dbg.wheel_brand_n = n_wheel_brands;
        if (g_dbg.wheel_style < 1) g_dbg.wheel_style = wheel_style;
        g_dbg.car_list = (const char (*)[64])carlist;
        g_dbg.track_list = (const char (*)[64])tracklist;
        dbgui_frame();
        dbgui_render();
        /* the Session panel's car/track combos ask for a switch the same
           way the arrow keys do: a clean process relaunch (see relaunch()
           above) — there's no in-place teardown/reload path for the ~30
           pieces of long-lived world/car state, and building one blind
           (no way to interactively test repeated swaps right now) risks
           a leak or dangling handle that a single screenshot can't catch. */
        if (g_dbg.insp_dump) {
            g_dbg.insp_dump = 0;
            int i = g_dbg.insp_sel;
            if (i >= 0 && i < ncar) {
                float bb[6]; n2_mesh_bbox(&car.meshes[i], bb);
                printf("\n[inspector] car mesh %d  cat=%d  verts=%d  tris=%d\n",
                       i, cgm[i].cat, car.meshes[i].nverts, car.meshes[i].nidx/3);
                printf("  local bbox  x[%.4f,%.4f] y[%.4f,%.4f] z[%.4f,%.4f]\n",
                       bb[0],bb[1],bb[2],bb[3],bb[4],bb[5]);
                printf("  car world pos (%.2f,%.2f,%.2f) heading %.3f rad  %.1f km/h\n",
                       carpos[0],carpos[1],carpos[2], heading, PHYS_KMH(speed));
                printf("  wheel scale %.3f  stance: axle F%+.3f R%+.3f  track F%.3f R%.3f  ride %+.3f\n",
                       g_dbg.wheel_scale, g_dbg.wheel.front_axle, g_dbg.wheel.rear_axle,
                       g_dbg.wheel.front_track, g_dbg.wheel.rear_track, g_dbg.wheel.ride_y);
                for (int w = 0; w < 4; w++) {
                    printf("  wheelT[%d] (column-major):\n", w);
                    for (int r = 0; r < 4; r++)
                        printf("    % .4f % .4f % .4f % .4f\n",
                               wheelT[w][r*4+0], wheelT[w][r*4+1],
                               wheelT[w][r*4+2], wheelT[w][r*4+3]);
                }
                if (nwheelgm > 0) {
                    float rb[6]; n2_mesh_bbox(&wheellib.meshes[0], rb);
                    printf("  active rim: %d meshes, mesh0 %d verts, bbox x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]\n",
                           nwheelgm, wheellib.meshes[0].nverts, rb[0],rb[1],rb[2],rb[3],rb[4],rb[5]);
                }
            } else printf("[inspector] no mesh selected\n");
        }
        if (g_dbg.wheel_reload) {          /* panel picked a brand/style */
            g_dbg.wheel_reload = 0;
            if (g_dbg.wheel_brand >= 0 && g_dbg.wheel_brand < n_wheel_brands &&
                g_dbg.wheel_brand != wheel_brand) {
                char wlp[1024];
                snprintf(wlp, sizeof wlp, "%s/CARS/WHEELS/GEOMETRY_%s.BIN",
                         dataroot, wheel_brands[g_dbg.wheel_brand]);
                unsigned char *nd = n2_read_file(wlp, &wllen);
                if (nd) { free(wldata); wldata = nd; wheel_brand = g_dbg.wheel_brand; }
            }
            wheel_style = g_dbg.wheel_style < 1 ? 1 : g_dbg.wheel_style;
            if (load_rim_style(wldata, wllen, wkeys, nwkeys, wheel_style,
                               &wheellib, &wheelgm, &nwheelgm,
                               wtdata, wtlen, &rimtex, carWheelR))
                printf("rims -> %s style %d (%d mesh(es))\n",
                       wheel_brands[wheel_brand], wheel_style, nwheelgm);
        }
        if (g_dbg.want_car >= 0 && g_dbg.want_car < ncars)
            relaunch(selfexe, dataroot, carlist[g_dbg.want_car], trackname);
        if (g_dbg.want_track >= 0 && g_dbg.want_track < ntrack)
            relaunch(selfexe, dataroot, carname, tracklist[g_dbg.want_track]);
#endif
        if (shot && ++shotframe >= shotframes) {
            unsigned char *px = malloc((size_t)W*H*3), *fl = malloc((size_t)W*H*3);
            glReadPixels(0, 0, W, H, GL_RGB, GL_UNSIGNED_BYTE, px);
            for (int y = 0; y < H; y++) memcpy(fl+(size_t)y*W*3, px+(size_t)(H-1-y)*W*3, W*3);
            write_png(shot, W, H, fl);
            free(px); free(fl);
            printf("frame avg: %.1f ms (vsync on), %d/%d world meshes drawn, %d draw calls\n",
                   (SDL_GetTicks()-t0)/(float)shotframe, ndrawn, nm, g_dbg.drawn);
            printf("camera XYZ: (%.1f, %.1f, %.1f)  looking at car (%.1f, %.1f, %.1f)\n",
                   cam[0], cam[1], cam[2], carpos[0], carpos[1], carpos[2]);
            { float f[3]={cosf(heading),sinf(heading),0};   /* pitch/roll from car_up */
              float d=f[0]*car_up[0]+f[1]*car_up[1]+f[2]*car_up[2];
              f[0]-=d*car_up[0]; f[1]-=d*car_up[1]; f[2]-=d*car_up[2];
              float fl2=sqrtf(f[0]*f[0]+f[1]*f[1]+f[2]*f[2]); if(fl2>1e-6f){f[0]/=fl2;f[1]/=fl2;f[2]/=fl2;}
              float lft[3]={car_up[1]*f[2]-car_up[2]*f[1], car_up[2]*f[0]-car_up[0]*f[2], car_up[0]*f[1]-car_up[1]*f[0]};
              printf("chassis: up(%.3f,%.3f,%.3f) pitch=%+.1fdeg roll=%+.1fdeg  (ground grade %.1fdeg)\n",
                     car_up[0],car_up[1],car_up[2], asinf(f[2])*57.2958f, asinf(lft[2])*57.2958f,
                     acosf(car_up[2]>1?1:car_up[2])*57.2958f); }
            printf("wheels: radius %.3f m, %.0f RPM, steer %+.1f deg, %.0f km/h\n",
                   g_dbg.wheel_radius, g_dbg.wheel_rpm, g_dbg.steer_deg, g_dbg.kmh);
            {   float sc = g_dbg.wheel_scale > 0.05f ? g_dbg.wheel_scale : 1.0f;
                float rd = carprof.ride * sc;
                printf("profile %-11s R %.3f%s  ride %.3f  hubZ %+.3f  wheelbase %.2f"
                       "  track %.2f  tyre bottom %+.3f (0=flush)  clearance %.3f\n",
                       carprof.name, carprof.wheel_r, carprof.has_tire ? "" : "*",
                       rd, carprof.hub_z, carprof.wheelbase, carprof.track_f,
                       rd + g_dbg.wheel.ride_y - g_dbg.wheel_radius,
                       rd + carprof.body[4]); }
            printf("wrote %s (%dx%d) after driving to (%.0f,%.0f)\n", shot, W, H, carpos[0], carpos[1]);
            running = 0;
        }
        SDL_GL_SwapWindow(win);
    }

#ifdef DEBUG_UI
    dbgui_shutdown();
#endif
    n2_free_scene(&scene);   /* region buffers already freed after texture upload */
    free(wmbatch);           /* wraps wbatch's GL handles; frees the array only */
    if (dbgprog) glDeleteProgram(dbgprog);
    if (adev) SDL_CloseAudioDevice(adev);
    SDL_GL_DeleteContext(ctx); SDL_DestroyWindow(win); SDL_Quit();
    return 0;
}
