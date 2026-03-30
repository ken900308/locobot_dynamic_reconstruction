#version 330

#if defined VERTEX_SHADER

in vec3 in_position;
in vec3 in_color;

uniform mat4 m_camera;
uniform mat4 m_proj;
uniform mat4 m_model;

out vec3 v_color;

void main() {
    gl_Position = m_proj * m_camera * m_model * vec4(in_position, 1.0);
    v_color = in_color;
}

#elif defined FRAGMENT_SHADER

in vec3 v_color;
out vec4 fragColor;

void main() {
    fragColor = vec4(v_color, 1.0);
}

#endif
