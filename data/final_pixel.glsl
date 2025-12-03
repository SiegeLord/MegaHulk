#version 330 core
in vec2 varying_texcoord;
layout(location = 0) out vec4 out_color;

uniform sampler2D al_tex; // Unused.
uniform sampler2D position_buffer;
uniform sampler2D normal_buffer;
uniform sampler2D albedo_buffer;
uniform sampler2D light_buffer;

void main()
{
    vec3 pos = texture(position_buffer, varying_texcoord).xyz;
    vec4 normal_mat = texture(normal_buffer, varying_texcoord);
    vec3 normal = normal_mat.xyz;
    float material = normal_mat.w;
    vec4 color = vec4(texture(albedo_buffer, varying_texcoord).rgb, 1);
    vec4 light_color = texture(light_buffer, varying_texcoord);

    float fullbright = float(material == FULLBRIGHT_MATERIAL);
    vec3 final_light_color = (1 - fullbright) * light_color.xyz + fullbright;

    float specular = light_color.w;
    out_color = vec4(final_light_color * color.xyz + 1. * vec3(specular), 1.);
    //out_color = vec4(final_light_color, 1.);
	//out_color = vec4(mod(pos.xyz, 1), 1);
    //out_color = vec4(normal, 1);
    //out_color = vec4(1., material, 0., 1.);
}
