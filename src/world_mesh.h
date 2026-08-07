/* world_mesh.h — pure-C99 world mesh batch uploader + prelight debug pipeline.
 *
 * Fits the existing engine natively: OpenGL 2.1 / GLSL 1.20, no VAO (GL 2.1 core
 * has none), and the shipping 36-byte BatchedVertex layout (render.h):
 *     float pos[3];   @0     float uv[2];   @12
 *     float normal[3];@20    unsigned char col[4]; @32  (RGBA8 prelight, normalized)
 * Attribute state is (re)specified per draw, exactly like render.c's draw_batch.
 * Indices are u16 to match the engine's 65535-vertex batch ceiling. */
#ifndef OPENUG_WORLD_MESH_H
#define OPENUG_WORLD_MESH_H

#include <stdint.h>
#include "render.h"   /* BatchedVertex + the GL headers (single home) */

typedef struct {
    GLuint   vao;          /* reserved: 0 on GL 2.1 core (no VAO); attribs bound per draw */
    GLuint   vbo, ibo;
    uint32_t index_count;
    uint32_t chunk_id;
    char     section_name[64];
} WorldMeshBatch;

/* Upload one section's interleaved BatchedVertex array + u16 index array into a
   static VBO/IBO. Returns a batch with vbo==0 (skipped at draw) on empty input. */
WorldMeshBatch upload_world_mesh_to_gpu(const BatchedVertex *verts, uint32_t vcount,
                                        const uint16_t *indices, uint32_t icount,
                                        const char *section_name, uint32_t chunk_id);

/* Draw every non-empty batch through `program`, setting u_MVP (column-major
   float[16]) and u_DebugMode (0 prelight, 1 normals, 2 solid/wireframe). */
void render_world_map(const WorldMeshBatch *batches, uint32_t n,
                      GLuint program, const float *mvp16, int debug_mode);

/* Free the batch's GL objects and zero it. */
void destroy_world_mesh_batch(WorldMeshBatch *b);

/* Compile world_debug_120.vert/.frag into a linked program (0 on failure).
   Binds a_Position/a_Normal/a_UV0/a_Color to locations 0..3. */
GLuint world_debug_program_load(const char *vert_path, const char *frag_path);

#endif /* OPENUG_WORLD_MESH_H */
