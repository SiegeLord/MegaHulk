#version 330 core
layout(location = 0) in vec4 al_pos;
layout(location = 1) in vec4 al_color;
layout(location = 2) in vec2 al_texcoord;
varying vec4 varying_color;
varying vec2 varying_texcoord;
uniform mat4 al_projview_matrix;

void main()
{
   varying_color = al_color;
   varying_texcoord = al_texcoord;
   gl_Position = al_projview_matrix * al_pos;
}
