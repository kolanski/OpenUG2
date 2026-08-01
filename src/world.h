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
#define WORLD_MAXDIST 24

/* A district = one connected component of the drivable nav graph (Phase 68).
 *
 * Replaces the old bbox zones, which were unusable: 3D mesh bounds overlap, so
 * one building could span several "districts". Districts are now derived from
 * road connectivity, which is what a player actually experiences.
 *
 * DATA AUDIT first (Data-First): the nav node's +8 u32 is NOT a region code.
 * It is two u16s (bits 7..15 never set; the high half is usually 0xffff, the
 * same no-link sentinel as the other slots). Over all 18064 nodes the low u16
 * takes 113 values and EVERY value spans nearly the whole city (value 0:
 * X[-2986..2520] Y[-2274..3116]), where a geographic id would be compact. It
 * is constant in runs and flips at segment boundaries, i.e. a per-segment road
 * id/class. And the five district NAMES the game shows are UI-only strings
 * (the LANGUAGES files), bound to no coordinates anywhere. Hence topology, not lookup. */
typedef struct {
    char  tok[4];   /* 2-letter area code straight out of the TRN_ asset names */
    int   n;        /* nav nodes fused into this district */
    float bb[4];    /* x0, x1, y0, y1 of those nodes */
    float cx, cy;   /* node centroid */
    float medz;     /* median terrain elevation (separates the hill districts) */
} WDistrict;

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
    WDistrict dist[WORLD_MAXDIST]; int ndist;  /* connected road components */
    int *navcomp;                             /* district index per nav node, -1 = none */
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

/* Flood-fill the nav graph into connected drivable components, largest first.
 * Returns the district count. */
int world_build_districts(World *w);

/* District of the nav node nearest (x,y) within maxdist, else -1. */
int world_district_at(const World *w, float x, float y, float maxdist);

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
