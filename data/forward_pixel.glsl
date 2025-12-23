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

uniform vec2 tex_size;
uniform int num_frames;
uniform float frame_ms;
uniform vec2 frame_dxy;

uniform sampler2D al_tex;
uniform sampler2D lightmap;

uniform int material;
uniform vec4 base_color;
uniform vec4 base_light;

uniform float time;
uniform vec3 player_pos;

void main()
{
    float frame = int(floor(time / (frame_ms / 1000.))) % num_frames;
    vec2 duv = frame * frame_dxy / tex_size;

    vec4 tex_color = texture(al_tex, varying_texcoord + duv);
    vec4 lightmap_color = texture(lightmap, varying_texcoord2 + duv);
    if (tex_color.a == 0.0) discard;
    position_buffer = varying_pos;
    normal_buffer = vec4(normalize(varying_normal), float(material));
    albedo_buffer = base_color * varying_color * tex_color;
    if (albedo_buffer.a == 0.0) discard;

    vec4 light = base_light;
    if (material == STATIC_MATERIAL || material == DYNAMIC_WITH_LIGHTMAP_MATERIAL || material == DYNAMIC_WITH_ADDITIVE_LIGHTMAP_MATERIAL)
	light = vec4(lightmap_color.rgb, 0.);
    else if (material == FULLBRIGHT_MATERIAL)
	light = vec4(vec3(1.), 0.);
    else if (material == LEVEL_MAP_MATERIAL) {
	float f = pow(0.5 + 0.5 * sin(length(varying_pos.xyz - player_pos) - 10. * time), 15.);
	vec3 c1 = (0.8 + 0.2 * normal_buffer.xyz);
	vec3 c2 = vec3(1., 0., 0.);
	vec3 pulsating = mix(c1, c2, f);
	light = vec4(mix(c1, pulsating, 1. - varying_color.a), 0.);
    }
    light_buffer = clamp(light, 0., 1.);
}
