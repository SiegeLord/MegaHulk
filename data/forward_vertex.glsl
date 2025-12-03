#version 330 core
attribute vec4 al_pos;
attribute vec4 al_color;
attribute vec2 al_texcoord;
attribute vec3 al_user_attr_0;  // normal
attribute vec2 al_user_attr_1;  // uv2

uniform mat4 al_projview_matrix;
uniform mat4 model_matrix;

varying vec3 varying_pos;
varying vec3 varying_normal;
varying vec2 varying_texcoord;
varying vec2 varying_texcoord2;
varying vec4 varying_color;

void main()
{
   varying_color = al_color;
   varying_texcoord = al_texcoord;
   varying_texcoord2 = al_user_attr_1;
   varying_pos = (model_matrix * al_pos).xyz;
   varying_normal = normalize(mat3(model_matrix) * al_user_attr_0);
   gl_Position = al_projview_matrix * al_pos;
}
