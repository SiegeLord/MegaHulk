#version 330 core
out vec4 color;

uniform sampler2D al_tex;

varying vec3 uv_depth;
varying vec3 visible2;

void main()
{
	color = vec4(vec3(visible2), 1.0);
}
