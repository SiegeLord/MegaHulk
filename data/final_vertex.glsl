#version 330 core
layout(location = 0) in vec4 al_pos;
layout(location = 2) in vec2 al_texcoord;
uniform mat4 al_projview_matrix;
varying vec2 varying_texcoord;

void main()
{
   varying_texcoord = al_texcoord;
   gl_Position = al_projview_matrix * al_pos;
}
