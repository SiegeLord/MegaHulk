#version 330 core
layout(location = 0) in vec4 al_pos;
layout(location = 1) in vec4 al_color;
layout(location = 2) in vec2 al_texcoord;
attribute vec2 al_user_attr_1;  // uv2

uniform sampler2D al_tex;

uniform mat4 al_projview_matrix;

varying vec3 visible;

float depth_to_linear(float depth)
{
    float near = 1.;
    float far = 50.;
    // Convert the non-linear [0,1] depth back to linear [0,1]
    float z = depth * 2.0 - 1.0; // Back to NDC (-1 to 1)
    return (2.0 * near * far) / (far + near - z * (far - near)) / far;
}

float linear_to_depth(float linear)
{
    float near = 1.;
    float far = 50.;
    return (1.0 / (linear * far) - 1.0 / near) / (1.0 / far - 1.0 / near);
}

float decode_depth(vec3 v)
{
    return v.r;// + v.g / 255. + v.b / (255. * 255.);
}

void main()
{
   // We need to put this into clip space, which is -1..1
   gl_Position = vec4(2. * al_user_attr_1 - 1., 0., 1.);

   vec4 clip_pos = al_projview_matrix * al_pos;
   vec3 projected = clip_pos.xyz / clip_pos.w;
   // Back to 0..1
   vec3 uv_depth = vec3(0.5 * projected.xy + 0.5, 0.5 * projected.z + 0.5);

   float valid = float(uv_depth.x >= 0.) * float(uv_depth.x < 1.) * float(uv_depth.y >= 0.) * float(uv_depth.y < 1.) * float(clip_pos.w > 0.);
   float depth = decode_depth(texture(al_tex, uv_depth.xy - vec2(0.5 / 255.)).rgb);
   float cur_depth = depth_to_linear(uv_depth.z);

   visible = valid * vec3(float(abs(cur_depth - depth) < 1 / 256.));
   //float eps = 0.1;
   //visible = valid * float(cur_depth / depth < (1. + eps) && cur_depth / depth > (1. - eps));
   //visible = valid * vec3(5. * max(depth - cur_depth, 0), 5. * max(cur_depth - depth, 0.), 5. * cur_depth);
   //visible = valid * vec3(float(cur_depth / depth < (1. + eps) && cur_depth / depth > (1. - eps)));
   //visible = valid * vec3(cur_depth);
   //visible = valid * vec3(depth, 0., 0.);
   //visible = valid * vec3(uv_depth.xy, 0.);
}
