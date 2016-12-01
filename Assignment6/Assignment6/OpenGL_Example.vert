#version 330
uniform mat4 M;
uniform mat4 PVM;
uniform mat4 light_pos;
uniform mat4 in_WtE;

in  vec3 in_position;
in  vec3 in_color;
in 	vec3 in_normal;
in	vec3 in_Od;

in vec2 vertexUV;

//in bool in_Ihdraw;
//in bool in_Iddraw;

// We output the ex_color variable to the next shader in the chain
out vec3 ex_color;
out vec4 ex_normal;
out vec4 ex_position;

//out	vec3 ex_Od;

out vec4 light_pos_v;
out vec4 WorldtoEye;

//out bool ex_Ihdraw;
//out bool ex_Iddraw;

out vec2 UV;

void main(void) {

    gl_Position = PVM * vec4(in_position.x, in_position.y, in_position.z, 1.0);

    // Since we aren't using projection matrices yet, 
    // the following adjusts for depth buffer "direction".
    // Remove this once projection is added.
    //gl_Position.z = -gl_Position.z;

    // We're passing the color to the rasterizer for interpolation
    ex_color = in_color;
    ex_normal = normalize(M * vec4(in_normal, 0.0)); //Implicit cast from Vec4 to Vec3
    ex_position = M * vec4(in_position, 1.0);

	light_pos_v = vec4(light_pos[3][0], light_pos[3][1], light_pos[3][2], light_pos[3][3]);
	WorldtoEye = vec4(in_WtE[3][0], in_WtE[3][1], in_WtE[3][2], in_WtE[3][3]);

	UV = vertexUV;

	//ex_Ihdraw = in_Ihdraw;
	//ex_Iddraw = in_Iddraw;

	//ex_Od = in_Od;
	//ex_L = normalize(light_pos_v - vec4(in_position, 1.0)); //sending in matrix, using Vector?
	//ex_U = normalize(ex_position - WorldtoEye); //eye space coord of pixel - eye space coord of World origin
}