#version 330 core
layout(location = 0) in vec4 al_pos;
layout(location = 1) in vec4 al_color;
layout(location = 2) in vec2 al_texcoord;
attribute vec2 al_user_attr_1;  // uv2

uniform sampler2D al_tex;

uniform mat4 al_projview_matrix;

varying vec3 uv_depth;
varying vec3 visible2;

float linear_to_depth(float linear)
{
	float near = 0.1;
	float far = 10.;
	return (1.0 / (linear * far) - 1.0 / near) / (1.0 / far - 1.0 / near);
}

float decode_depth(vec3 v)
{
	return linear_to_depth(v.r + v.g / 255. + v.b / (255. * 255.));
}

void main()
{
   // We need to put this into clip space, which is -1..1
   gl_Position = vec4(2. * al_user_attr_1 - 1., 0., 1.);

   vec4 clip_pos = al_projview_matrix * al_pos;
   vec3 projected = clip_pos.xyz / clip_pos.w;
   // Back to 0..1
   uv_depth = vec3(0.5 * projected.xy + 0.5, 0.5 * projected.z + 0.5);

   float valid = float(uv_depth.x >= 0.) * float(uv_depth.x < 1.) * float(uv_depth.y >= 0.) * float(uv_depth.y < 1.) * float(clip_pos.w > 0.);
   float depth = decode_depth(texture(al_tex, uv_depth.xy).rgb);

   visible2 = valid * vec3(uv_depth.z < depth + 1e-4);
   //visible2 = valid * vec3(uv_depth.z);
   //visible2 = valid * vec3(depth);
   //visible2 = valid * vec3(uv_depth.xy, 0.);
}
