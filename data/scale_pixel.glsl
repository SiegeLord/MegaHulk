#version 330 core
uniform sampler2D al_tex;

uniform float bitmap_width;
uniform float bitmap_height;
uniform float scale;

varying vec4 varying_color;
varying vec2 varying_texcoord;

void main()
{
	vec2 sz = vec2(bitmap_width, bitmap_height);
	vec3 step = vec3(1.0 / scale, 1.0 / scale, 0);
	vec2 tex_pixel = sz * varying_texcoord - step.xy / 2;
	
	vec2 corner = floor(tex_pixel) + 1;
	vec2 frac = min((corner - tex_pixel) * scale, vec2(1.0, 1.0));
	
	vec4 c1 = texture2D(al_tex, (floor(tex_pixel + step.zz) + 0.5) / sz);
	vec4 c2 = texture2D(al_tex, (floor(tex_pixel + step.xz) + 0.5) / sz);
	vec4 c3 = texture2D(al_tex, (floor(tex_pixel + step.zy) + 0.5) / sz);
	vec4 c4 = texture2D(al_tex, (floor(tex_pixel + step.xy) + 0.5) / sz);
	
	c1 *=        frac.x  *        frac.y;
	c2 *= (1.0 - frac.x) *        frac.y;
	c3 *=        frac.x  * (1.0 - frac.y);
	c4 *= (1.0 - frac.x) * (1.0 - frac.y);
	
	gl_FragColor = varying_color * (c1 + c2 + c3 + c4);
}

