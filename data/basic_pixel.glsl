#version 330 core
in vec4 varying_color;
in vec2 varying_texcoord;
out vec4 color;

uniform sampler2D al_tex;
uniform bool al_usetex;

void main()
{
	if (al_usetex)
	{
		color = varying_color * texture2D(al_tex, varying_texcoord);
	}
	else
	{
		color = varying_color;
	}
}
