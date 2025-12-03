#version 330 core
layout(location = 0) in vec4 al_pos;
uniform mat4 al_projview_matrix;

void main()
{
   gl_Position = al_projview_matrix * al_pos;
}
