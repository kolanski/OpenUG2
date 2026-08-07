#version 120
/* World prelight debug — fragment stage (GLSL 1.20 / GL 2.1). */
varying vec3 v_Normal;
varying vec2 v_UV0;
varying vec4 v_Color;

uniform int u_DebugMode;   /* 0 prelight, 1 normals, 2 solid/wireframe overlay */

void main() {
    if (u_DebugMode == 1) {
        /* RGB surface normals */
        gl_FragColor = vec4(normalize(v_Normal) * 0.5 + 0.5, 1.0);
    } else if (u_DebugMode == 2) {
        /* solid overlay; wireframe comes from glPolygonMode(GL_LINE) on the host */
        gl_FragColor = vec4(0.85, 0.85, 0.90, 1.0);
    } else {
        /* baked vertex prelight */
        gl_FragColor = vec4(v_Color.rgb, 1.0);
    }
}
