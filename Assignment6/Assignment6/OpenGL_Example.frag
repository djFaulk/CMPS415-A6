#version 330
in vec3 ex_color;
in vec4 ex_normal;
in vec4 ex_position;

//in vec3 ex_Od;

in vec4 light_pos_v;
in vec4 WorldtoEye;

in vec2 UV;

//in bool ex_Ihdraw;
//in bool ex_Iddraw;

uniform sampler2D texture_Colors;

out vec4 gl_FragColor;

//vec4 Od = vec4(0.2, 0.6, 0.2, 1.0);
vec4 Id = vec4(0.5, 0.0, 0.5, 1.0);
vec4 Ih = vec4(0.0, 0.5, 0.5, 1.0);

void main(void) {

    vec4 lightDir = light_pos_v - ex_position;
	float lightDist = length(lightDir);

	lightDir = lightDir / lightDir;

	float diffuse = max(0.0, dot(ex_normal, lightDir));

	vec4 U = normalize(ex_position - WorldtoEye);
	
	gl_FragColor = (Ih * vec4(ex_color,1.0) * (0.5 + (0.5 * ex_normal * U))) + (Id * vec4(ex_color,1.0) * diffuse);
	//gl_FragColor = vec4(texture2D(texture_Colors, UV).rgb, 1.0) * ((Ih * vec4(ex_color,1.0) * (0.5 + (0.5 * ex_normal * U))) + (Id * vec4(ex_color,1.0) * diffuse));

	// Pass through the interpolated color with full opacity.
	//gl_FragColor = vec4(ex_color,1.0);
}