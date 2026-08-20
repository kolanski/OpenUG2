/* physics.h — OpenUG2 Physics module: arcade car kinematics (heading-frame
 * velocity with lateral tyre scrub → drift), wall AABB collision, and
 * car-to-car circle separation. No GL, no SDL, no file IO. */
#ifndef OPENUG2_PHYSICS_H
#define OPENUG2_PHYSICS_H

#include "nfsu2.h"
#include "ai.h"

/* driving constants — car length axis = local X; world +Z up.
 * Real units: world coordinates are metres, physics ticks at 60 Hz, so
 * speeds are metres/tick. Tuned to NFSU2 driving: ~220 km/h top speed,
 * 0-100 km/h in ~4 s, ~100-0 braking in ~3 s, long pull to top speed. */
#define PHYS_TICKRATE 60.0f
#define PHYS_MAXSPD   (61.0f/PHYS_TICKRATE)   /* 220 km/h cap (m/tick) */
#define PHYS_ACCEL    (7.0f/(PHYS_TICKRATE*PHYS_TICKRATE)) /* 7 m/s^2 peak thrust */
#define PHYS_FRICTION 0.99886f /* rolling+air drag; equilibrium lands at MAXSPD */
#define PHYS_TURN     0.045f   /* steering rate at full authority (rad/tick) */
#define PHYS_GRIP     0.86f    /* lateral scrub per tick (lower = grippier) */
/* km/h for the HUD from a m/tick forward speed */
#define PHYS_KMH(v)   ((v) * PHYS_TICKRATE * 3.6f)

/* Live handling tuning (ImGui sliders). accel/brake/turn are multipliers on the
 * constants above (1.0 = stock); top_kmh is a hard forward-speed cap. Defaults
 * reproduce the tuned NFSU2 feel exactly, so phys_selftest still holds. */
typedef struct { float accel, brake, turn, top_kmh; } PhysTune;
extern PhysTune g_phys_tune;

/* One tick of car kinematics: throttle in [-1..1] along the heading, steering
 * in [-1..1] rotates it, tyres scrub the sideways velocity (handbrake keeps
 * it, so the car slides), position integrates. Returns the post-grip lateral
 * speed magnitude — the drift signal for skid marks / smoke / screech. */
float phys_car_step(float pos[3], float vel[2], float *heading, float *speed,
                    float throttle, float steer, int handbrake);

/* Push (pos.xy) out of any wall AABB (expanded by r) it penetrates, along the
 * least-penetration axis; zero the into-wall velocity so the car slides along
 * the face. Returns the number of walls resolved.
 *
 * obz is the per-obstacle {z0,z1} span from the same phys_collect_walls pass,
 * and [cz0,cz1] is the car's own world-space vertical extent: an obstacle whose
 * Z span does not overlap the car's is skipped, so a deck or overhang the car
 * is under (or over) no longer blocks it in XY (M94: a 6.1 m slab 5.3 m ABOVE
 * the car produced 68 responses). Pass obz = NULL to disable the Z test and get
 * the original XY-only behaviour. */
int collide_walls(float *pos, float *vel, const float obst[][4],
                  const float obz[][2], int nobst, float r, float cz0, float cz1);
void collide_walls_selftest(void);
void phys_selftest(void);   /* asserts the NFSU2 velocity tuning targets */

/* Collect building collision footprints: the 2D (XY) bounding box of every
 * tall N2_OTHER mesh. Flat props/signs/road paint are skipped so they don't
 * block the road. Returns the number of AABBs written. */
/* Same selection pass, plus an optional parallel array of the SOURCE mesh index
 * each rect came from (pass NULL to ignore). Selection semantics are unchanged;
 * src exists so a diagnostic can name the mesh behind a collision response
 * without re-implementing the predicate. */
int phys_collect_walls(const N2Scene *s, float (*obst)[4], int *src,
                       float (*obz)[2], int max);

/* Circle-separate the player from each AI and the AIs from each other.
 * Player is pushed at half weight each way; a bump scrubs a little player
 * speed. Returns the collision thud amplitude for the audio (0 = no hit). */
float phys_car_contacts(float carpos[3], float vel[2], float speed,
                        AiCar *ais, int nai);

#endif
