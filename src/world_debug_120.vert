#version 120
/* World prelight debug — vertex stage (GLSL 1.20 / GL 2.1).
   Attribute locations bound to 0..3 by world_debug_program_load. */
attribute vec3 a_Position;
attribute vec3 a_Normal;
attribute vec2 a_UV0;
attribute vec4 a_Color;   /* RGBA8 prelight, normalized to [0,1] */

uniform mat4 u_MVP;

varying vec3 v_Normal;
varying vec2 v_UV0;
varying vec4 v_Color;

void main() {
    /* World meshes are pre-transformed to world space, so the baked normal is
       already world-space — no normal matrix needed. Pass attributes through. */
    v_Normal    = a_Normal;
    v_UV0       = a_UV0;
    v_Color     = a_Color;
    gl_Position = u_MVP * vec4(a_Position, 1.0);
}
