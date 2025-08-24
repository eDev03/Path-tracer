#version 330 core
out vec4 final_color;

in vec2 uv;

void main() {
    final_color = vec4(uv.x ,uv.y, 0.0, 1.0);
}