#version 330 core
out vec4 color;

uniform sampler2D al_tex;

varying vec3 visible;

void main()
{
	//color = vec4(vec3(float(visible == 1.0)), 1.0);
	//color = vec4(vec3(visible), 1.0);
	color = vec4(vec3(float(visible[0] > 0.99)), 1.0);
}
