#version 330 core
in vec3 varying_pos;
in vec3 varying_normal;
in vec2 varying_texcoord;
in vec2 varying_texcoord2;
in vec4 varying_color;

layout(location = 0) out vec3 position_buffer;
layout(location = 1) out vec4 normal_buffer;
layout(location = 2) out vec4 albedo_buffer;
layout(location = 3) out vec4 light_buffer;

uniform sampler2D al_tex;
uniform sampler2D lightmap;

uniform int material;

void main()
{
    vec4 tex_color = texture(al_tex, varying_texcoord);
    vec4 lightmap_color = texture(lightmap, varying_texcoord2);
    if (tex_color.a == 0.0) discard;
    position_buffer = varying_pos;
    normal_buffer = vec4(normalize(varying_normal), float(material));
    albedo_buffer = varying_color * tex_color;

    vec4 light = vec4(vec3(0.05), 0.);
    if (material == STATIC_MATERIAL)
	light += vec4(lightmap_color.rgb, 0.);
    light_buffer = light;
}
