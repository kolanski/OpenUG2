/* world_mesh.c — see world_mesh.h. Pure C99, GL 2.1, no VAO. */
#include "world_mesh.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>   /* offsetof */

WorldMeshBatch upload_world_mesh_to_gpu(const BatchedVertex *verts, uint32_t vcount,
                                        const uint16_t *indices, uint32_t icount,
                                        const char *section_name, uint32_t chunk_id) {
    WorldMeshBatch b;
    memset(&b, 0, sizeof b);
    b.index_count = icount;
    b.chunk_id    = chunk_id;
    snprintf(b.section_name, sizeof b.section_name, "%s", section_name ? section_name : "");
    if (!verts || !indices || !vcount || !icount) return b;   /* vbo stays 0 -> skipped */

    glGenBuffers(1, &b.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, b.vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 (GLsizeiptr)vcount * (GLsizeiptr)sizeof(BatchedVertex),
                 verts, GL_STATIC_DRAW);

    glGenBuffers(1, &b.ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, b.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 (GLsizeiptr)icount * (GLsizeiptr)sizeof(uint16_t),
                 indices, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    return b;   /* GL 2.1: no VAO, so attribute pointers are set at draw time */
}

void render_world_map(const WorldMeshBatch *batches, uint32_t n,
                      GLuint program, const float *mvp16, int debug_mode) {
    if (!batches || !program) return;
    glUseProgram(program);

    GLint locMVP  = glGetUniformLocation(program, "u_MVP");
    GLint locMode = glGetUniformLocation(program, "u_DebugMode");
    if (locMVP  >= 0 && mvp16) glUniformMatrix4fv(locMVP, 1, GL_FALSE, mvp16);
    if (locMode >= 0)         glUniform1i(locMode, debug_mode);

    /* Attribute locations (bound to 0..3 by world_debug_program_load; queried so
       an externally-built program still works). Fetched once, not per batch. */
    GLint aPos = glGetAttribLocation(program, "a_Position");
    GLint aNor = glGetAttribLocation(program, "a_Normal");
    GLint aUV  = glGetAttribLocation(program, "a_UV0");
    GLint aCol = glGetAttribLocation(program, "a_Color");
    const GLsizei stride = (GLsizei)sizeof(BatchedVertex);   /* 36 B */

#ifndef N2_GLES
    if (debug_mode == 2) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);   /* desktop only */
#endif

    for (uint32_t k = 0; k < n; k++) {
        const WorldMeshBatch *b = &batches[k];
        if (!b->vbo || !b->index_count) continue;   /* skip empty/disabled sections */

        glBindBuffer(GL_ARRAY_BUFFER, b->vbo);
        if (aPos >= 0) { glEnableVertexAttribArray((GLuint)aPos);
            glVertexAttribPointer((GLuint)aPos, 3, GL_FLOAT, GL_FALSE, stride,
                                  (const void *)offsetof(BatchedVertex, pos)); }
        if (aNor >= 0) { glEnableVertexAttribArray((GLuint)aNor);
            glVertexAttribPointer((GLuint)aNor, 3, GL_FLOAT, GL_FALSE, stride,
                                  (const void *)offsetof(BatchedVertex, normal)); }
        if (aUV  >= 0) { glEnableVertexAttribArray((GLuint)aUV);
            glVertexAttribPointer((GLuint)aUV, 2, GL_FLOAT, GL_FALSE, stride,
                                  (const void *)offsetof(BatchedVertex, uv)); }
        if (aCol >= 0) { glEnableVertexAttribArray((GLuint)aCol);
            glVertexAttribPointer((GLuint)aCol, 4, GL_UNSIGNED_BYTE, GL_TRUE, stride,
                                  (const void *)offsetof(BatchedVertex, col)); }

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, b->ibo);
        glDrawElements(GL_TRIANGLES, (GLsizei)b->index_count, GL_UNSIGNED_SHORT, 0);
    }

#ifndef N2_GLES
    if (debug_mode == 2) glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
#endif
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void destroy_world_mesh_batch(WorldMeshBatch *b) {
    if (!b) return;
    if (b->ibo) glDeleteBuffers(1, &b->ibo);
    if (b->vbo) glDeleteBuffers(1, &b->vbo);
    /* b->vao intentionally not deleted: never created on GL 2.1 core */
    memset(b, 0, sizeof *b);
}

/* ---- GLSL 1.20 program loader (file-based) ---- */
static char *wm_read_text(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    if (n < 0) { fclose(f); return NULL; }
    char *s = (char *)malloc((size_t)n + 1);
    if (!s) { fclose(f); return NULL; }
    if (fread(s, 1, (size_t)n, f) != (size_t)n) { fclose(f); free(s); return NULL; }
    s[n] = 0; fclose(f);
    return s;
}
static GLuint wm_compile(GLenum type, const char *src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);
    GLint ok = 0; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024]; glGetShaderInfoLog(s, (GLsizei)sizeof log, NULL, log);
        fprintf(stderr, "world_debug shader compile: %s\n", log);
        glDeleteShader(s); return 0;
    }
    return s;
}
/* Embedded fallbacks — kept byte-for-byte in sync with world_debug_120.vert/.frag
   so the binary runs from any CWD when the source files aren't found on disk. */
static const char *WORLD_DEBUG_VS_120 =
    "#version 120\n"
    "attribute vec3 a_Position;\n"
    "attribute vec3 a_Normal;\n"
    "attribute vec2 a_UV0;\n"
    "attribute vec4 a_Color;\n"
    "uniform mat4 u_MVP;\n"
    "varying vec3 v_Normal;\n"
    "varying vec2 v_UV0;\n"
    "varying vec4 v_Color;\n"
    "void main() {\n"
    "    v_Normal    = a_Normal;\n"
    "    v_UV0       = a_UV0;\n"
    "    v_Color     = a_Color;\n"
    "    gl_Position = u_MVP * vec4(a_Position, 1.0);\n"
    "}\n";
static const char *WORLD_DEBUG_FS_120 =
    "#version 120\n"
    "varying vec3 v_Normal;\n"
    "varying vec2 v_UV0;\n"
    "varying vec4 v_Color;\n"
    "uniform int u_DebugMode;\n"
    "void main() {\n"
    "    if (u_DebugMode == 1) {\n"
    "        gl_FragColor = vec4(normalize(v_Normal) * 0.5 + 0.5, 1.0);\n"
    "    } else if (u_DebugMode == 2) {\n"
    "        gl_FragColor = vec4(0.85, 0.85, 0.90, 1.0);\n"
    "    } else {\n"
    "        gl_FragColor = vec4(v_Color.rgb, 1.0);\n"
    "    }\n"
    "}\n";

GLuint world_debug_program_load(const char *vert_path, const char *frag_path) {
    /* Prefer on-disk files (live-editable during dev); fall back per-shader to
       the embedded source so the binary works from any working directory. */
    char *vfile = wm_read_text(vert_path), *ffile = wm_read_text(frag_path);
    const char *vsrc = vfile ? vfile : WORLD_DEBUG_VS_120;
    const char *fsrc = ffile ? ffile : WORLD_DEBUG_FS_120;
    if (!vfile) fprintf(stderr, "world_debug: %s not found, using embedded vertex shader\n", vert_path);
    if (!ffile) fprintf(stderr, "world_debug: %s not found, using embedded fragment shader\n", frag_path);

    GLuint v = wm_compile(GL_VERTEX_SHADER, vsrc);
    GLuint f = wm_compile(GL_FRAGMENT_SHADER, fsrc);
    free(vfile); free(ffile);
    if (!v || !f) return 0;

    GLuint p = glCreateProgram();
    glAttachShader(p, v); glAttachShader(p, f);
    glBindAttribLocation(p, 0, "a_Position");
    glBindAttribLocation(p, 1, "a_Normal");
    glBindAttribLocation(p, 2, "a_UV0");
    glBindAttribLocation(p, 3, "a_Color");
    glLinkProgram(p);
    GLint ok = 0; glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024]; glGetProgramInfoLog(p, (GLsizei)sizeof log, NULL, log);
        fprintf(stderr, "world_debug link: %s\n", log);
        glDeleteProgram(p); p = 0;
    }
    glDeleteShader(v); glDeleteShader(f);
    return p;
}
