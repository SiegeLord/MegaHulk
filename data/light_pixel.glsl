#version 330 core
out vec4 color;

uniform vec4 light_color;
uniform vec3 light_pos;
uniform float light_intensity;
uniform vec2 buffer_size;
uniform vec3 camera_pos;
uniform bool is_static;

uniform sampler2D position_buffer;
uniform sampler2D normal_buffer;

void main()
{
    vec2 texcoord = gl_FragCoord.xy / buffer_size;
    vec3 pos = texture(position_buffer, texcoord).xyz;
    vec4 normal_mat = texture(normal_buffer, texcoord);
    vec3 normal = normal_mat.xyz;
    float material = normal_mat.w;

    vec3 ray_dir = normalize(light_pos - pos);
    vec3 camera_dir = normalize(camera_pos - pos);
    //if (dot(ray_dir, normal) > 0.)
    //    discard;

    float diffuse_dot = abs(max(dot(ray_dir, normal), 0.));
    float specular_dot = pow(max(dot(reflect(-ray_dir, normal), camera_dir), 0.), 20.);

    vec3 diff = (light_pos - pos) / light_intensity;
    float dist_sq = 10000. * dot(diff, diff);
    float dist_frac = 1 / (1 + dist_sq * dist_sq);
    //color = vec4(light_color.xyz, 0.);
    
    if (is_static && material == STATIC_MATERIAL)
	dist_frac = 0.;

    color = dist_frac * vec4((light_color * diffuse_dot).xyz, specular_dot * 0.);
}
