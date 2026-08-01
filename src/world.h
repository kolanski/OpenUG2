/* world.h — OpenUG World module: multi-region city loading (STREAM .BUN
 * stitching into one scene), per-region texture binding, per-mesh bounds
 * for draw culling, and a grid-accelerated ground query. Owns the region
 * file buffers between load and texture upload. */
#ifndef OPENUG_WORLD_H
#define OPENUG_WORLD_H

#include "nfsu2.h"
#include "render.h"
#include "debug.h"   /* ScriptedDef, for world_scripted_defs */

#define WORLD_MAXREG 16
#define WORLD_MAXZONE 48

/* A logical city zone, derived entirely from shipped data (Phase 66).
 *
 * DATA AUDIT (do not repeat it): the five district names the game shows --
 * "City Core", "Beacon Hill", "Jackson Heights", "Coal Harbor West/East" --
 * exist ONLY as UI strings in the LANGUAGES .bin files (by "World Map").
 * They are NOT bound to any coordinate anywhere: searched every .BUN/.BIN in
 * TRACKS, GLOBAL, FRONTEND and SDATA, plus all three JDLZ archives after
 * decompressing them (FrontB, GlobalB, InGameCommon .lzc) -- zero hits.
 * GLOBALB's only "Jackson" is the parts brand "Jackson Racing". So the district
 * bounds live in speed2.exe, which is SafeDisc-encrypted (see docs/FORMATS.md).
 *
 * What the data DOES carry is the artists' own area code in every mesh's asset
 * name: TRN_[AREA]_... / PAN_[AREA]_... (SH, UC, CN, CS, IP, ...). Those codes
 * are real, and the meshes carrying them have real world coordinates, so zone
 * bounds are measured from the geometry rather than invented. */
typedef struct {
    char  tok[24];      /* area code straight out of the asset names */
    float bb[4];        /* measured XY bounds: x0, x1, y0, y1 */
    int   n;            /* meshes contributing */
} WZone;

typedef struct {
    char name[64];
    unsigned char *data; long len;   /* file buffer, freed after texture bind */
    N2Tpk tpk;
    int mesh0, mesh1;                /* this region's mesh range in the scene */
} WRegion;

typedef struct {
    N2Scene scene;
    WRegion rgn[WORLD_MAXREG]; int nreg;
    unsigned char *loc4; long loc4len;                       /* shared tex library */
    unsigned char *master; long masterlen; N2Tpk mastertpk;  /* single-region mode */
    N2Tex grass; int have_grass;                             /* terrain fallback */
    float (*mbb)[4];   /* per-mesh XY bbox (x0,y0,x1,y1) for culling + ground grid */
    WZone zone[WORLD_MAXZONE]; int nzone;   /* logical city zones (see WZone) */
    /* AI/GPS navigation graph: the real drivable road network (see world_load_nav) */
    float *nav;        /* nnav * 2 floats: world X,Y of each node */
    int    nnav;
    int   *navedge;    /* nnavedge * 2 node indices */
    int    nnavedge;
    float  navbb[4];   /* x0,x1,y0,y1 over all nodes, for map framing */
} World;

/* Load the navigation graph for the loaded regions from TRACKS/ROUTES<REGION>/
 * Paths*.bin. Each file's 0x34148 leaf is an array of 24-byte records:
 *   +0 float X, +4 float Y, +8 flags, +12/+14/+16 u16 neighbour indices
 *   (0xffff = none), +20 float cumulative distance along the segment.
 * Files concatenate several segments, so consecutive records are only joined
 * when they are close enough to be one road (see NAV_LINK_MAX in world.c).
 * Returns the node count. */
int world_load_nav(World *w, const char *troot);

/* Build the zone table from the loaded scene's asset names. Called by
 * world_load; safe to call again. Returns the zone count. */
int world_build_zones(World *w);

/* Index of the zone containing (x,y), or -1. Smallest matching zone wins so a
 * nested area beats the sprawling one it sits inside. */
int world_zone_at(const World *w, float x, float y);

/* Load trackname ("ALL" = every STREAM*.BUN under troot, else one region)
 * into w->scene. Builds per-mesh bounds and the ground grid. Returns the
 * mesh count (0 = nothing readable). */
int world_load(World *w, const char *troot, const char *trackname);

/* Decode + upload every distinct mesh texture (own TPK -> LOC4 -> master),
 * writing the key->GL map, then free the region buffers. Needs a GL context.
 * Returns the number of textures bound. */
int world_bind_textures(World *w, uint32_t *keys, GLuint *texs, int cap);

/* Ground height at (x,y): same contract as n2_ground_z but only tests the
 * road/terrain meshes whose bbox covers the point (grid lookup). */
float world_ground_z(const N2Scene *s, float x, float y, float fallback);

/* Unit up-normal of the ground triangle under (x,y); (0,0,1) if off-track. */
void world_ground_normal(const N2Scene *s, float x, float y, float n[3]);

/* Push the car circle (centre pos[3], radius r) out of any near-vertical
   guardrail/fence face baked into the road/terrain. Returns 1 if it pushed. */
int world_wall_push(const N2Scene *s, float *pos, float r);

/* Decode the scripted-object entity DEFINITIONS (name + FNV-32 hash + local
 * OBB extents) from each loaded district's companion L4R*.BUN, deduped by
 * hash. Read-only inspector data — the companion carries no world placement
 * or mesh (see docs/FORMATS.md). Returns the number written (<= cap). */
int world_scripted_defs(const World *w, const char *troot,
                        ScriptedDef *out, int cap);

#endif
