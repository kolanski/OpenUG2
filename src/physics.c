/* physics.c — OpenUG2 Physics module implementation. */
#include <math.h>
#include <assert.h>

#include "physics.h"

/* handbrake: rear grip lets go, the lateral velocity survives → drift */
#define HANDBRAKE_GRIP   0.985f
#define REVERSE_SPD_FRAC 0.2f    /* reverse cap ~45 km/h */
#define REVERSE_ACCEL    0.7f    /* reverse thrust vs forward */
#define BRAKE_ACCEL      1.5f    /* braking vs forward thrust: ~10 m/s^2 */
#define COAST_DRAG       0.9994f /* off-throttle engine braking on top of drag */
/* turn authority ramps in by ~35% of top speed then holds — responsive from
 * low speed without getting twitchy/spinny flat out. */
#define TURN_RAMP_FRAC   0.35f
/* ...and eases off toward top speed so 200+ km/h stays stable (NFSU2 cars
 * corner tight at city speed, wide at full tilt) */
#define TURN_HISPD_DROP  0.55f

PhysTune g_phys_tune = { 1.0f, 1.0f, 1.0f, 220.0f };   /* stock defaults */

float phys_car_step(float pos[3], float vel[2], float *heading, float *speed,
                    float throttle, float steer, int handbrake) {
    float top = g_phys_tune.top_kmh / 3.6f / PHYS_TICKRATE;   /* cap in m/tick */
    float hf[2] = { cosf(*heading), sinf(*heading) };
    float fwd = vel[0]*hf[0] + vel[1]*hf[1];   /* signed forward speed */
    if (throttle > 0) {
        /* throttle tapers as speed builds: punchy off the line, eases near top */
        float sp = *speed < 0 ? -*speed : *speed;
        float a = PHYS_ACCEL * g_phys_tune.accel * (1.15f - 0.55f*sp/PHYS_MAXSPD) * throttle;
        vel[0] += hf[0]*a; vel[1] += hf[1]*a;
    } else if (throttle < 0) {
        /* moving forward = brakes (strong); at rest / rolling back = reverse */
        float a = PHYS_ACCEL * (fwd > 0.01f ? BRAKE_ACCEL*g_phys_tune.brake : REVERSE_ACCEL);
        vel[0] += hf[0]*a*throttle; vel[1] += hf[1]*a*throttle;
    } else {
        vel[0] *= COAST_DRAG; vel[1] *= COAST_DRAG;
    }
    vel[0] *= PHYS_FRICTION; vel[1] *= PHYS_FRICTION;
    float spd = sqrtf(vel[0]*vel[0]+vel[1]*vel[1]);
    float dir = (vel[0]*hf[0]+vel[1]*hf[1]) < 0 ? -1.f : 1.f;  /* fwd vs reverse */
    float sfac = spd/(PHYS_MAXSPD*TURN_RAMP_FRAC); if (sfac > 1) sfac = 1;
    float hifrac = spd/PHYS_MAXSPD; if (hifrac > 1) hifrac = 1;
    *heading += steer * PHYS_TURN * g_phys_tune.turn * sfac * (1.0f - TURN_HISPD_DROP*hifrac) * dir;
    /* decompose velocity in the new heading frame, clamp forward, scrub side */
    float nf[2] = { cosf(*heading), sinf(*heading) }, nr[2] = { nf[1], -nf[0] };
    float vf = vel[0]*nf[0]+vel[1]*nf[1], vl = vel[0]*nr[0]+vel[1]*nr[1];
    if (vf >  top) vf =  top;
    if (vf < -top*REVERSE_SPD_FRAC) vf = -top*REVERSE_SPD_FRAC;
    vl *= handbrake ? HANDBRAKE_GRIP : PHYS_GRIP;
    vel[0] = nf[0]*vf + nr[0]*vl; vel[1] = nf[1]*vf + nr[1]*vl;
    *speed = vf;                      /* forward speed, for HUD/collision */
    pos[0] += vel[0]; pos[1] += vel[1];
    return vl < 0 ? -vl : vl;         /* drift magnitude */
}

void phys_selftest(void) {
    /* the NFSU2 tuning targets, asserted: 0-100 in 3-6 s, top ~220 km/h,
       100-0 braking well under 5 s */
    float pos[3]={0,0,0}, vel[2]={0,0}, h=0, spd=0;
    int t100 = -1;
    for (int t = 1; t <= 60*60; t++) {
        phys_car_step(pos, vel, &h, &spd, 1.0f, 0, 0);
        if (t100 < 0 && PHYS_KMH(spd) >= 100.0f) t100 = t;
    }
    assert(t100 > 2*60 && t100 < 6*60);
    assert(PHYS_KMH(spd) > 200.0f && PHYS_KMH(spd) < 232.0f);
    /* brake from ~100 km/h */
    vel[0] = cosf(h)*100.0f/3.6f/PHYS_TICKRATE; vel[1] = sinf(h)*100.0f/3.6f/PHYS_TICKRATE;
    int tstop = -1;
    for (int t = 1; t <= 8*60 && tstop < 0; t++) {
        phys_car_step(pos, vel, &h, &spd, -1.0f, 0, 0);
        if (spd <= 0.0f) tstop = t;
    }
    assert(tstop > 0 && tstop < 5*60);
}

/* Does this mesh present an actual wall to the car here? Near-vertical face,
 * height span overlapping the car, XY projection within r. */
int cw_probe_contact(const N2Scene *s, int mi, float px, float py,
                     float r, float cz0, float cz1);
static int cw_mesh_contact(const N2Scene *s, int mi, float px, float py,
                           float r, float cz0, float cz1) {
    if (mi < 0 || mi >= s->count) return 1;          /* no source: keep the rect */
    const N2Mesh *m = &s->meshes[mi];
    float r2 = r*r;
    for (int t = 0; t + 2 < m->nidx; t += 3) {
        const float *A = m->verts + m->idx[t]*5;
        const float *B = m->verts + m->idx[t+1]*5;
        const float *C = m->verts + m->idx[t+2]*5;
        float e1[3], e2[3], n[3];
        for (int a = 0; a < 3; a++) { e1[a] = B[a]-A[a]; e2[a] = C[a]-A[a]; }
        n[0]=e1[1]*e2[2]-e1[2]*e2[1]; n[1]=e1[2]*e2[0]-e1[0]*e2[2];
        n[2]=e1[0]*e2[1]-e1[1]*e2[0];
        float L = sqrtf(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]); if (L < 1e-9f) continue;
        if (fabsf(n[2]/L) >= 0.30f) continue;                 /* not a wall face */
        float zlo = A[2], zhi = A[2];
        if (B[2]<zlo) zlo=B[2]; if (C[2]<zlo) zlo=C[2];
        if (B[2]>zhi) zhi=B[2]; if (C[2]>zhi) zhi=C[2];
        if (zhi < cz0 || zlo > cz1) continue;                 /* not at car height */
        const float *P[3] = { A, B, C };
        for (int e = 0; e < 3; e++) {
            const float *p0 = P[e], *p1 = P[(e+1)%3];
            float dx = p1[0]-p0[0], dy = p1[1]-p0[1], l2 = dx*dx+dy*dy;
            float u = l2 > 1e-9f ? ((px-p0[0])*dx + (py-p0[1])*dy) / l2 : 0.0f;
            if (u < 0) u = 0; if (u > 1) u = 1;
            float qx = px - (p0[0]+dx*u), qy = py - (p0[1]+dy*u);
            if (qx*qx + qy*qy <= r2) return 1;
        }
    }
    return 0;
}

int cw_probe_contact(const N2Scene *s, int mi, float px, float py,
                     float r, float cz0, float cz1) {
    return cw_mesh_contact(s, mi, px, py, r, cz0, cz1);
}

int collide_walls(float *pos, float *vel, const float obst[][4],
                  const float obz[][2], int nobst, float r, float cz0, float cz1,
                  const N2Scene *scene, const int *src) {
    int hits = 0;
    for (int o = 0; o < nobst; o++) {
        float x0=obst[o][0]-r, y0=obst[o][1]-r, x1=obst[o][2]+r, y1=obst[o][3]+r;
        if (pos[0]<=x0 || pos[0]>=x1 || pos[1]<=y0 || pos[1]>=y1) continue;
        /* vertical volumes must actually overlap for this to be a collision */
        if (obz && (obz[o][1] < cz0 || obz[o][0] > cz1)) continue;
        /* the rect was only broad phase: confirm against the mesh's own faces */
        if (scene && src && !cw_mesh_contact(scene, src[o], pos[0], pos[1], r, cz0, cz1))
            continue;
        float pl=pos[0]-x0, pr=x1-pos[0], pd=pos[1]-y0, pu=y1-pos[1], m=pl; int ax=0;
        if (pr<m){m=pr;ax=1;} if (pd<m){m=pd;ax=2;} if (pu<m){m=pu;ax=3;}
        if      (ax==0){ pos[0]=x0; if(vel[0]>0)vel[0]=0; }
        else if (ax==1){ pos[0]=x1; if(vel[0]<0)vel[0]=0; }
        else if (ax==2){ pos[1]=y0; if(vel[1]>0)vel[1]=0; }
        else           { pos[1]=y1; if(vel[1]<0)vel[1]=0; }
        hits++;
    }
    return hits;
}
void collide_walls_selftest(void) {
    float obst[1][4] = {{0,0,10,10}};
    float p[3]={5,5,0}, v[2]={1,1};
    assert(collide_walls(p, v, obst, NULL, 1, 1.0f, 0, 0, NULL, NULL) == 1); /* deep inside -> resolved */
    assert(p[0]<=0 || p[0]>=10 || p[1]<=0 || p[1]>=10);    /* ...and now outside the box */
    float p2[3]={0.5f,5,0}, v2[2]={2,0};                   /* near left face, moving +x */
    collide_walls(p2, v2, obst, NULL, 1, 1.0f, 0, 0, NULL, NULL);
    assert(p2[0] <= -1.0f + 1e-4f && v2[0] == 0.0f);       /* pushed left, +x vel killed */
    float p3[3]={100,100,0}, v3[2]={1,0};
    assert(collide_walls(p3, v3, obst, NULL, 1, 1.0f, 0, 0, NULL, NULL) == 0); /* far outside */

    /* --- Z overlap gate (M95) --- */
    float obz[1][2] = {{204.4f, 210.5f}};                  /* the M94 slab */
    float q[3]={5,5,199.08f}, qv[2]={1,1};                 /* car 5.3 m BELOW it */
    assert(collide_walls(q, qv, obst, obz, 1, 1.0f, 199.08f, 200.55f, NULL, NULL) == 0);
    assert(q[0]==5.0f && q[1]==5.0f && qv[0]==1.0f && qv[1]==1.0f);  /* untouched */
    float u[3]={5,5,215.0f}, uv[2]={1,1};                  /* car ABOVE it */
    assert(collide_walls(u, uv, obst, obz, 1, 1.0f, 215.0f, 216.5f, NULL, NULL) == 0);
    /* XY + Z overlap still resolves exactly as the XY-only path does */
    float w1[3]={5,5,206.0f}, wv1[2]={1,1};
    float w2[3]={5,5,206.0f}, wv2[2]={1,1};
    int h1 = collide_walls(w1, wv1, obst, obz,  1, 1.0f, 206.0f, 207.5f, NULL, NULL);
    int h2 = collide_walls(w2, wv2, obst, NULL, 1, 1.0f, 0, 0, NULL, NULL);
    assert(h1 == 1 && h1 == h2);
    assert(w1[0]==w2[0] && w1[1]==w2[1] && wv1[0]==wv2[0] && wv1[1]==wv2[1]);
    /* touching spans count as overlapping (no gap) */
    float t1[3]={5,5,204.4f}, tv1[2]={1,1};
    assert(collide_walls(t1, tv1, obst, obz, 1, 1.0f, 202.9f, 204.4f, NULL, NULL) == 1);
}

#define WALL_MIN_HEIGHT 2.5f    /* z-extent below this = flat prop, not a wall */
#define WALL_MAX_SPAN   300.0f  /* skip oversized shells (sky domes etc.) */

/* Which scenery stops a car (Phase 65). Each mesh now carries its asset-name
 * class, so the decision is semantic instead of a pure height guess:
 *   BUILDING / WALL / STRUCT -> always a solid, immovable boundary
 *   TREE / TERRAIN           -> never a wall here (ground is the query's job)
 *   PROP  -> MEASURED, not assumed: the XO_ prefix mixes 1x1x1.5 m boxes
 *            (XO_IP_WBOX) with 24x29x36 m office blocks (XO_INDUSTRIALOFFICESA)
 *            and 7x8x26 m tower containers, so the prefix alone cannot decide.
 *            Height bands over L4RA's 939 props: <2 m 254, 2-4 274, 4-8 353,
 *            8-16 359, 16-32 166, >32 60. Street furniture is distinguished by a
 *            SMALL FOOTPRINT (poles/cans/boxes are thin), not by being short --
 *            a streetlight is 8 m tall but ~1 m wide. So a prop is solid when
 *            its shorter horizontal span reaches PROP_SOLID_SPAN; thinner props
 *            stay out of the AABB set for a future knock-down/rebound pass.
 * Unnamed meshes keep the old N2_OTHER height heuristic. */
#define PROP_SOLID_SPAN 3.0f
static int scen_is_wall(int sc) {
    return sc == N2_SC_BUILDING || sc == N2_SC_WALL || sc == N2_SC_STRUCT;
}
int phys_collect_walls(const N2Scene *s, float (*obst)[4], int *src,
                       float (*obz)[2], int max) {
    int nobst = 0;
    for (int i = 0; i < s->count && nobst < max; i++) {
        int sc = s->meshes[i].scen;
        int prop_check = 0;
        if (sc != N2_SC_NONE) {                 /* named: decide semantically */
            if (sc == N2_SC_TERRAIN) continue;              /* ground, never a wall */
            /* props, trees and unclassified: let measured size decide, so a tree
               cluster or a big container still blocks but a trunk/pole does not */
            if (!scen_is_wall(sc)) prop_check = 1;
        } else if (s->meshes[i].cat != N2_OTHER) continue;   /* unnamed fallback */
        if (s->meshes[i].nverts < 3) continue;
        float ox0=1e30f,oy0=1e30f,oz0=1e30f, ox1=-1e30f,oy1=-1e30f,oz1=-1e30f;
        for (int v=0;v<s->meshes[i].nverts;v++){ float *p=s->meshes[i].verts+v*5;
            if(p[0]<ox0)ox0=p[0]; if(p[0]>ox1)ox1=p[0];
            if(p[1]<oy0)oy0=p[1]; if(p[1]>oy1)oy1=p[1];
            if(p[2]<oz0)oz0=p[2]; if(p[2]>oz1)oz1=p[2]; }
        if (oz1-oz0 < WALL_MIN_HEIGHT) continue;             /* flat: not a wall */
        if (ox1-ox0 > WALL_MAX_SPAN || oy1-oy0 > WALL_MAX_SPAN) continue;
        if (prop_check) {   /* thin street furniture: leave it drivable-through */
            float sx = ox1-ox0, sy = oy1-oy0, smin = sx < sy ? sx : sy;
            if (smin < PROP_SOLID_SPAN) continue;
        }
        obst[nobst][0]=ox0; obst[nobst][1]=oy0; obst[nobst][2]=ox1; obst[nobst][3]=oy1;
        if (obz) { obz[nobst][0]=oz0; obz[nobst][1]=oz1; }   /* same pass, already measured */
        if (src) src[nobst] = i;
        nobst++;
    }
    return nobst;
}

#define CAR_RADIUS 2.6f   /* car-to-car collision circle */

float phys_car_contacts(float carpos[3], float vel[2], float speed,
                        AiCar *ais, int nai) {
    const float MIN = CAR_RADIUS*2.0f;
    float thud = 0.0f;
    /* player vs AI: player pushed at full weight; AIs share the rest so they
       don't get shoved off their line too hard. */
    for (int k = 0; k < nai; k++) {
        float dx = ais[k].pos[0]-carpos[0], dy = ais[k].pos[1]-carpos[1];
        float d2 = dx*dx+dy*dy;
        if (d2 > 1e-4f && d2 < MIN*MIN) {
            float d = sqrtf(d2), push = (MIN - d);
            float ux = dx/d, uy = dy/d;
            carpos[0]    -= ux*push*0.5f; carpos[1]    -= uy*push*0.5f;
            ais[k].pos[0]+= ux*push*0.5f; ais[k].pos[1]+= uy*push*0.5f;
            vel[0]*=0.85f; vel[1]*=0.85f;   /* bump scrubs a little speed */
            float s = (speed<0?-speed:speed)/PHYS_MAXSPD;
            if (0.3f + s*0.5f > thud) thud = 0.3f + s*0.5f;
        }
    }
    for (int a = 0; a < nai; a++) for (int b = a+1; b < nai; b++) {
        float dx = ais[b].pos[0]-ais[a].pos[0], dy = ais[b].pos[1]-ais[a].pos[1];
        float d2 = dx*dx+dy*dy;
        if (d2 > 1e-4f && d2 < MIN*MIN) {
            float d = sqrtf(d2), push = (MIN - d)*0.5f, ux = dx/d, uy = dy/d;
            ais[a].pos[0]-=ux*push; ais[a].pos[1]-=uy*push;
            ais[b].pos[0]+=ux*push; ais[b].pos[1]+=uy*push;
        }
    }
    return thud;
}
