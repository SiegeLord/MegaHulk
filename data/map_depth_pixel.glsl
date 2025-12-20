#version 330 core
out vec4 color;

uniform float visible;

float depth_to_linear(float depth)
{
    float near = 1.;
    float far = 50.;
    // Convert the non-linear [0,1] depth back to linear [0,1]
    float z = depth * 2.0 - 1.0; // Back to NDC (-1 to 1)
    return (2.0 * near * far) / (far + near - z * (far - near)) / far;
}

vec3 encode_depth(float depth)
{
    // depth = r + g / 255. + b / (255. * 255.);
    depth = depth_to_linear(depth);
    float r = depth;
    float g = 0. * mod(depth * 255., 1);
    float b = 0. * mod(depth * 255. * 255., 1.);
    return vec3(r, g, b);
}

void main()
{
    color = vec4(visible * encode_depth(gl_FragCoord.z), 1.0);
}
