#include <stdlib.h>
#include <stdio.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <gmtl/gmtl.h> 
#include <stdlib.h>
#include <stdio.h>

#define _USE_MATH_DEFINES
#include <math.h>

#pragma comment (lib, "opengl32.lib")
#pragma comment (lib, "glew32.lib")
#pragma comment (lib, "glfw3.lib")

//Class decalaration
class Bird {

	gmtl::Matrix44f rotation;
	float omega;
	float vertVel;
	float radius;

public:
	Bird()
	{
		gmtl::Matrix44f temp;
		gmtl::identity(temp);

		rotation = temp;
		omega = 1;
		vertVel = 1.5;
		radius = 1.6;
	}
	Bird(gmtl::Matrix44f rot)
	{
		rotation = rot;
		omega = 1;
		vertVel = 1.5;
		radius = 1.6;
	}


	Bird(gmtl::Matrix44f rot, float om)
	{
		rotation = rot;
		omega = om;
		vertVel = 1.5;
		radius = 1.6;
	}

	Bird(float Vv, bool Vel)
	{
		gmtl::Matrix44f temp;
		gmtl::identity(temp);

		rotation = temp;
		if (Vel)
		{
			vertVel = Vv;
			omega = 1;
		}
		else
		{
			vertVel = 1.5;
			omega = Vv;
		}

		radius = 1.6;
	}

	//Getters and Setters

	gmtl::Matrix44f getRot()
	{
		return rotation;
	}

	float getRot(int row, int column)
	{
		return rotation[row][column];
	}

	void setRot(gmtl::Matrix44f rot)
	{
		rotation = rot;
	}

	void setRot(int row, int column, float n)
	{
		rotation[row][column] = n;
	}

	float getOmega()
	{
		return omega;
	}

	void setOmega(float om)
	{
		omega = om;
	}

	float getVertVel()
	{
		return vertVel;
	}

	void setVertVel(float Vv)
	{
		vertVel = Vv;
	}

	float getRadius()
	{
		return radius;
	}

	void setRadius(float rad)
	{
		radius = rad;
	}
};

//Global Variables NP, NM, and Radius
const int np = 10;
const int nm = 10;

//int representing number of birds
const int nb = 4;
const int no = 4;
int num = 0;
float meanAccMag, dispAccMag, centAccMag, vmatchAccMag, totalAccMag, meanDispMag, meanCentMag, meanVmatchMag, iterations = 0;
float maxAccMag, maxDispMag, maxCentMag, maxVmatchMag = -100;
float maxAccel = 0.3;

const float cRadius = 0.01;
const int height = 10;
const int cnm = 5;

gmtl::Matrix44f M;

//W - World 
gmtl::Matrix44f W;

//R - Point of Rotation
gmtl::Matrix44f R;

//O - Offset from point of rotation
gmtl::Matrix44f O;

//B - Base of the Bird object
gmtl::Matrix44f B;

//j1 - Offset of Bird for Joint 1
gmtl::Matrix44f joint1;

//j2 - Offset of Bird for Joint 2
gmtl::Matrix44f joint2;

//prim1 - Offset for Primitive from Joint 1
gmtl::Matrix44f prim1;

//prim2 - Offset for Primitive from Joint 2
gmtl::Matrix44f prim2;

//cam1 - Position for Camera 1
gmtl::Matrix44f cam1;

//cam2 - Position for Camera 2
gmtl::Matrix44f cam2;

//activeCam - bool describing active camera. 0(false) for camera 1, 1(true) for camera 2
bool activeCam = 0;

//Q - {R} w.r.t. {W}
gmtl::Matrix44f WtR;

//q - Quaternion for {R} w.r.t. {W}
gmtl::Quatf q (0, 0, 0, 1);

gmtl::Matrix44f Q;

gmtl::Quatf move_bird_forward( 0, 0, 0, 1);

gmtl::Quatf turn_bird_right( 0, 0, 0, 1);

gmtl::Quatf turn_bird_left( 0, 0, 0, 1);

//RtO - {O} w.r.t. {R}
gmtl::Matrix44f RtO;

//OtB - {B} w.r.t. {O}
gmtl::Matrix44f OtB;

//BtJ1 - {J1}, or joint 1, w.r.t. {B}
gmtl::Matrix44f BtJ1;

//BtJ2 - {J2} w.r.t. {B}
gmtl::Matrix44f BtJ2;

//J1tP - {J1} to Primitive
gmtl::Matrix44f J1tP;

//J2tP - {J2} to Primitive
gmtl::Matrix44f J2tP;

//WtC1 - relation from {W} to Camera 1
gmtl::Matrix44f WtC1;
 
//OtC2 - relation from {O} to Camera 2
gmtl::Matrix44f OtC2;

gmtl::Matrix44f V;

gmtl::Matrix44f camChange;

gmtl::Matrix44f R_lat;

gmtl::Matrix44f R_long;

gmtl::Matrix44f R_azi;

gmtl::Matrix44f R_ele;

gmtl::Matrix44f R_z;

//Z-Translation for Camera 1
gmtl::Matrix44f T1;

//Z-Translation for Camera 2
gmtl::Matrix44f T2;

//toggle for IH and ID
bool IH, ID = true;

gmtl::Matrix44f Wy;
gmtl::Matrix44f Wx;
gmtl::Matrix44f Wz;
bool showW = false;

gmtl::Matrix44f Ry;
gmtl::Matrix44f Rx;
gmtl::Matrix44f Rz;
bool showR = false;

gmtl::Matrix44f Oy;
gmtl::Matrix44f Ox;
gmtl::Matrix44f Oz;
bool showO = false;

bool toggleMouse = true;

//Mouse Call objects
double x_Beg = 0.0;
double y_Beg = 0.0;
double x_End = 0.0;
double y_End = 0.0;
double x_Diff = 0.0;
double y_Diff = 0.0;

bool captureMouse = false;

float toRadians(float angle);

GLfloat * setColor(int type);

Bird bird1;
Bird bird2;
Bird bird3;
Bird bird4;

Bird * birdArray = new Bird[nb];

//Vector arrays for keeping track of forces on Bird
gmtl::Vec4f * BirdDisp = new gmtl::Vec4f[nb];
gmtl::Vec4f * BirdCenter = new gmtl::Vec4f[nb];
gmtl::Vec4f * BirdVelMatch = new gmtl::Vec4f[nb];

bool showDisp, showCenter, showMatch = false;

gmtl::Matrix44f * obstacleList = new gmtl::Matrix44f[no];

/* A simple function that will read a file into an allocated char pointer buffer */
char* filetobuf(char *file)
{
	FILE *fptr;
	long length;
	char *buf;

	fopen_s(&fptr, file, "rb"); /* Open file for reading */
	if (!fptr) /* Return NULL on failure */
		return NULL;
	fseek(fptr, 0, SEEK_END); /* Seek to the end of the file */
	length = ftell(fptr); /* Find out how many bytes into the file we are */
	buf = (char*)malloc(length + 1); /* Allocate a buffer for the entire length of the file and a null terminator */
	fseek(fptr, 0, SEEK_SET); /* Go back to the beginning of the file */
	fread(buf, length, 1, fptr); /* Read the contents of the file in to the buffer */
	fclose(fptr); /* Close the file */
	buf[length] = 0; /* Null terminator */

	return buf; /* Return the buffer */
}

void error_callback(int error, const char* description)
{
	fprintf(stderr, "Error: %s\n", description);
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	gmtl::Matrix44f change;
	gmtl::identity(change);

	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GLFW_TRUE);

	//Move the bird forward by incrementing Q
	else if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
	{
		//q = q * move_bird_forward;
		
		showDisp = !showDisp;
		showCenter = !showCenter;
		showMatch = !showMatch;

		printf("ShowDisp: %d\nShowCenter: %d\nShowMatch: %d\n\n", showDisp, showCenter, showMatch);

		/*change[0][0] = cos(toRadians(10));
		change[0][1] = sin(toRadians(10)) * -1;
		change[1][0] = sin(toRadians(10));
		change[1][1] = cos(toRadians(10));
		change.setState(gmtl::Matrix44f::ORTHOGONAL);

		WtR = WtR * change;*/
	}
	//Allow the bird to Yaw by small x-rotation
	else if ( key == GLFW_KEY_D && action == GLFW_PRESS ) 
	{
		showDisp = !showDisp;
		printf("ShowDisp: %d\nShowCenter: %d\nShowMatch: %d\n\n", showDisp, showCenter, showMatch);
		/*change[1][1] = cos(toRadians(22));
		change[1][2] = sin(toRadians(22)) * -1;
		change[2][1] = sin(toRadians(22));
		change[2][2] = cos(toRadians(22));
		change.setState(gmtl::Matrix44f::ORTHOGONAL);

		WtR = WtR * change;*/

		//q = q * turn_bird_right;
	}
	else if (key == GLFW_KEY_S && action  == GLFW_PRESS)
	{
		showMatch = !showMatch;
		printf("ShowDisp: %d\nShowCenter: %d\nShowMatch: %d\n\n", showDisp, showCenter, showMatch);

	}
	else if (key == GLFW_KEY_A && action == GLFW_PRESS )
	{
		showCenter = !showCenter;
		printf("ShowDisp: %d\nShowCenter: %d\nShowMatch: %d\n\n", showDisp, showCenter, showMatch);

		/*change[1][1] = cos(toRadians(-22));
		change[1][2] = sin(toRadians(-22)) * -1;
		change[2][1] = sin(toRadians(-22));
		change[2][2] = cos(toRadians(-22));
		change.setState(gmtl::Matrix44f::ORTHOGONAL);
		
		WtR = WtR * change;*/
	
		//q = q * turn_bird_left;
	}
	//Q - positive Banking around {O}'s Y-axis
	else if (key == GLFW_KEY_Q && GLFW_PRESS)
	{
		
		change[0][0] = cos(toRadians(15));
		change[0][2] = sin(toRadians(15)) * -1;
		change[2][0] = sin(toRadians(15));
		change[2][2] = cos(toRadians(15));
		change.setState(gmtl::Matrix44f::ORTHOGONAL);
		
		OtB = OtB * change;
	}
	//E - negative banking around {O}'s Y-axis
	else if (key == GLFW_KEY_E && GLFW_PRESS)
	{
		change[0][0] = cos(toRadians(-15));
		change[0][2] = sin(toRadians(-15)) * -1;
		change[2][0] = sin(toRadians(-15));
		change[2][2] = cos(toRadians(-15));
		change.setState(gmtl::Matrix44f::ORTHOGONAL);

		OtB = OtB * change;
	}
	//Z - Joint 1 rotation
	else if (key == GLFW_KEY_Z && GLFW_PRESS)
	{
		change[1][1] = cos(toRadians(15));
		change[1][2] = sin(toRadians(15)) * -1;
		change[2][1] = sin(toRadians(15));
		change[2][2] = cos(toRadians(15));
		change.setState(gmtl::Matrix44f::ORTHOGONAL);
		
		BtJ1 = BtJ1 * change;
	}
	//C - Joint 2 rotation
	else if (key == GLFW_KEY_C && GLFW_PRESS)
	{
		change[1][1] = cos(toRadians(15));
		change[1][2] = sin(toRadians(15)) * -1;
		change[2][1] = sin(toRadians(15));
		change[2][2] = cos(toRadians(15));
		change.setState(gmtl::Matrix44f::ORTHOGONAL);

		BtJ2 = BtJ2 * change;
	}
	//F - Switch Active Camera
	else if (key == GLFW_KEY_F && action == GLFW_PRESS)
	{
		activeCam = !activeCam;
	}
	//1 - Activate World Coord for W
	else if (key == GLFW_KEY_1 && action == GLFW_PRESS)
	{
		showW = !showW;
	}
	//2 - Activate World Coord for R
	else if (key == GLFW_KEY_2 && action == GLFW_PRESS)
	{
		showR = !showR;
	}
	//3 - Activate World Coord for O
	else if (key == GLFW_KEY_3 && action == GLFW_PRESS)
	{
		showO = !showO;
	}
	//L - Toggle Mouse options
	else if (key == GLFW_KEY_L && action == GLFW_PRESS)
	{
		toggleMouse = !toggleMouse;
	}
	//Arrow key up - Adjust T to zoom in
	else if (key == GLFW_KEY_UP && action == GLFW_PRESS)
	{
		//printf("UP PRESSED!\n");
		if (!activeCam)
		{
			//printf("UP PRESSED1!\n");
			T1[2][3] -= .15;
			T1.setState(gmtl::Matrix44f::TRANS);
		}
		else
		{
			//printf("UP PRESSED2!\n");
			T2[2][3] -= .15;
			T2.setState(gmtl::Matrix44f::TRANS);
		}
	}
	//Arrow key down - Adjust T to zoom out
	else if (key == GLFW_KEY_DOWN && action == GLFW_PRESS)
	{
		printf("DOWN PRESSED!\n");
		if (!activeCam)
		{
			T1[2][3] += .15;
			T1.setState(gmtl::Matrix44f::TRANS);
		}
		else
		{
			T2[2][3] += .15;
			T2.setState(gmtl::Matrix44f::TRANS);
		}
	}
	/*else if (key == GLFW_KEY_I && action == GLFW_PRESS)
	{
		ID = !ID;
	}
	else if (key == GLFW_KEY_U && action == GLFW_PRESS)
	{
		IH = !IH;
	}*/
}

//Called when mouse is pressed. Format taken from example 1b provided for Assignment 1
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	gmtl::identity(camChange);

	gmtl::identity(R_lat);

	gmtl::identity(R_long);

	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS && toggleMouse) 
	{
		captureMouse = true;
		//printf("Entering Press\n");
		glfwGetCursorPos(window, &x_Beg, &y_Beg);
		//printf("mouse position captured at:\t %f\t %f\n", x_Beg, y_Beg);
	}
	else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE)
	{
		captureMouse = false;
	}
}

/** This function sets up our window, OpenGL context, etc. For assignments, you don't need to know how it works. */
GLFWwindow* setupWindow()
{
	GLFWwindow *window;
	glfwSetErrorCallback(error_callback);

	if (!glfwInit())
		exit(EXIT_FAILURE);

	//glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); //Update later
	//glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	window = glfwCreateWindow(1024, 576, "Simple example", NULL, NULL);
	if (!window)
	{
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
	glfwSetKeyCallback(window, key_callback);
	glfwMakeContextCurrent(window);
	//gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
	glfwSwapInterval(1);

	if (glewInit() != GLEW_OK)
		exit(EXIT_FAILURE);

	glEnable(GL_DEPTH_TEST);

	return window;
}

/** This function sets up shaders on the graphics card. For assignments, you don't need to know how it works. */
GLuint setupShaderProgram()
{
	GLuint vertex_shader, fragment_shader, shader_program;
	int IsCompiled_VS, IsCompiled_FS, IsLinked, max_length;
	char *vertex_shader_log;
	char *fragment_shader_log;
	char *shader_program_log;

	/* Read our shaders into the appropriate buffers */
	char* vertex_source = filetobuf("OpenGL_Example.vert");
	char* fragment_source = filetobuf("OpenGL_Example.frag");

	vertex_shader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertex_shader, 1, &vertex_source, NULL);
	glCompileShader(vertex_shader);

	glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &IsCompiled_VS);
	if (IsCompiled_VS == GL_FALSE)
	{
		glGetShaderiv(vertex_shader, GL_INFO_LOG_LENGTH, &max_length);

		/* The max_length includes the NULL character */
		vertex_shader_log = (char *)malloc(max_length);

		glGetShaderInfoLog(vertex_shader, max_length, &max_length, vertex_shader_log);
		printf("Error: %s", vertex_shader_log);
		/* Handle the error in an appropriate way such as displaying a message or writing to a log file. */
		/* In this simple program, we'll just leave */
		free(vertex_shader_log);
		free(vertex_source);
		return 0;
	}

	fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragment_shader, 1, &fragment_source, NULL);
	glCompileShader(fragment_shader);
	glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &IsCompiled_FS);
	if (IsCompiled_FS == GL_FALSE)
	{
		glGetShaderiv(fragment_shader, GL_INFO_LOG_LENGTH, &max_length);

		/* The max_length includes the NULL character */
		fragment_shader_log = (char *)malloc(max_length);

		glGetShaderInfoLog(fragment_shader, max_length, &max_length, fragment_shader_log);
		printf("Error: %s", fragment_shader_log);
		/* Handle the error in an appropriate way such as displaying a message or writing to a log file. */
		/* In this simple program, we'll just leave */
		free(fragment_shader_log);
		free(vertex_source);
		free(fragment_source);
		return 0;
	}

	/* If we reached this point it means the vertex and fragment shaders compiled and are syntax error free. */
	/* We must link them together to make a GL shader program */
	/* GL shader programs are monolithic. It is a single piece made of 1 vertex shader and 1 fragment shader. */
	/* Assign our program handle a "name" */
	shader_program = glCreateProgram();

	/* Attach our shaders to our program */
	glAttachShader(shader_program, vertex_shader);
	glAttachShader(shader_program, fragment_shader);

	/* Link our program */
	/* At this stage, the vertex and fragment programs are inspected, optimized and a binary code is generated for the shader. */
	/* The binary code is uploaded to the GPU, if there is no error. */
	glLinkProgram(shader_program);

	/* Again, we must check and make sure that it linked. If it fails, it would mean either there is a mismatch between the vertex */
	/* and fragment shaders. It might be that you have surpassed your GPU's abilities. Perhaps too many ALU operations or */
	/* too many texel fetch instructions or too many interpolators or dynamic loops. */

	glGetProgramiv(shader_program, GL_LINK_STATUS, (int *)&IsLinked);
	if (IsLinked == GL_FALSE)
	{
		/* Noticed that glGetProgramiv is used to get the length for a shader program, not glGetShaderiv. */
		glGetProgramiv(shader_program, GL_INFO_LOG_LENGTH, &max_length);

		/* The max_length includes the NULL character */
		shader_program_log = (char *)malloc(max_length);

		/* Notice that glGetProgramInfoLog, not glGetShaderInfoLog. */
		glGetProgramInfoLog(shader_program, max_length, &max_length, shader_program_log);
		printf("Error: %s", shader_program_log);
		/* Handle the error in an appropriate way such as displaying a message or writing to a log file. */
		/* In this simple program, we'll just leave */
		free(shader_program_log);
		free(vertex_source);
		free(fragment_source);
		return 0;
	}
	//	glBindAttribLocation(shader_program, SHADER_POSITION_INDEX, "in_position");
	//	glBindAttribLocation(shader_program, SHADER_COLOR_INDEX, "in_color");

	free(vertex_source);
	free(fragment_source);

	return shader_program;

}

/**
This function gets attribute and uniform locations from shaders. For _this_ assignment (A2),
you don't need to know how it works.
*/
void init(GLuint shader_program, GLuint &pos_loc_out, GLuint &color_loc_out, GLuint &vm_loc_out, GLuint &pvm_loc_out, GLuint &norm_loc_out, GLuint &light_pos_out, GLuint &WtE_loc_out, GLuint &Od_loc_out, GLuint &UV_loc_out)//, GLuint &norm_Mat_loc_out
{
	pos_loc_out = glGetAttribLocation(shader_program, "in_position");
	color_loc_out = glGetAttribLocation(shader_program, "in_color");
	norm_loc_out = glGetAttribLocation(shader_program, "in_normal");
	light_pos_out = glGetAttribLocation(shader_program, "light_pos");
	WtE_loc_out = glGetAttribLocation(shader_program, "in_WtE");
	Od_loc_out = glGetAttribLocation(shader_program, "in_Od");
	UV_loc_out = glGetAttribLocation(shader_program, "vertexUV");

	//vm_loc_out = glGetUniformLocation(shader_program, "VM");
	vm_loc_out = glGetUniformLocation(shader_program, "M");
	pvm_loc_out = glGetUniformLocation(shader_program, "PVM");
	//norm_Mat_loc_out = glGetUniformLocation(shader_program, "normalMatrix");

}

void initMatrices()
{
	gmtl::identity(V);
	V.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(M);
	M.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(WtR);
	WtR.setState(gmtl::Matrix44f::AFFINE);

	//RtO is the relation from {R} to {O} which is merely an X-transform of the Radius + Hovering Distance
	//This means we can simply set the matrix here and we dont need to adjust it again
	gmtl::identity(RtO);
	RtO[0][3] = 1.6;
	RtO.setState(gmtl::Matrix44f::TRANS);

	gmtl::identity(OtB);
	OtB.setState(gmtl::Matrix44f::ORTHOGONAL);

	gmtl::identity(W);
	W.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(B);
	B.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(joint1);
	joint1.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(joint2);
	joint2.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(BtJ1);
	BtJ1[0][3] = 0.07;
	BtJ1[1][3] = -0.07;
	BtJ1[2][3] = -0.07;
	BtJ1.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(BtJ2);
	BtJ2[0][3] = 0.07;
	BtJ2[1][3] = -0.07;
	BtJ2[2][3] = 0.07;
	BtJ2.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(J1tP);
	J1tP[1][3] = 0.05;
	J1tP.setState(gmtl::Matrix44f::TRANS);

	gmtl::identity(J2tP);
	J2tP[1][3] = 0.05;
	J1tP.setState(gmtl::Matrix44f::TRANS);

	gmtl::identity(WtC1);
	WtC1.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(camChange);
	camChange.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(R_lat);
	R_lat.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(R_long);
	R_long.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(R_azi);
	R_azi.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(R_ele);
	R_ele.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(R_z);
	R_z[0][0] = cos(toRadians(-90));
	R_z[0][1] = sin(toRadians(-90)) * -1;
	R_z[1][0] = sin(toRadians(-90));
	R_z[1][1] = cos(toRadians(-90));
	R_z.setState(gmtl::Matrix44f::ORTHOGONAL);

	gmtl::identity(Wx);
	gmtl::identity(Wy);
	gmtl::identity(Wz);
	Wx.setState(gmtl::Matrix44f::AFFINE);
	Wy.setState(gmtl::Matrix44f::AFFINE);
	Wz.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(Rx);
	gmtl::identity(Ry);
	gmtl::identity(Rz);
	Rx.setState(gmtl::Matrix44f::AFFINE);
	Ry.setState(gmtl::Matrix44f::AFFINE);
	Rz.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(Ox);
	gmtl::identity(Oy);
	gmtl::identity(Oz);
	Ox.setState(gmtl::Matrix44f::AFFINE);
	Oy.setState(gmtl::Matrix44f::AFFINE);
	Oz.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(T1);
	T1[2][3] = 1.75;
	T1.setState(gmtl::Matrix44f::TRANS);
	gmtl::identity(T2);
}

void initBird(bool print)
{
	gmtl::Matrix44f change;
	gmtl::identity(change);
	change[0][0] = cos(toRadians(30));
	change[0][1] = sin(toRadians(30)) * -1;
	change[1][0] = sin(toRadians(30));
	change[1][1] = cos(toRadians(30));
	change.setState(gmtl::Matrix44f::ORTHOGONAL);

	bird2.setRot(bird2.getRot() * change * change);

	gmtl::identity(change);
	change[0][0] = cos(toRadians(40));
	change[0][2] = sin(toRadians(40)) * -1;
	change[2][0] = sin(toRadians(40));
	change[2][2] = cos(toRadians(40));
	change.setState(gmtl::Matrix44f::ORTHOGONAL);
	bird3.setRot(bird3.getRot() * change * change);


	gmtl::identity(change);
	change[0][0] = cos(toRadians(180));
	change[0][2] = sin(toRadians(180)) * -1;
	change[2][0] = sin(toRadians(180));
	change[2][2] = cos(toRadians(180));
	change.setState(gmtl::Matrix44f::ORTHOGONAL);
	bird4.setRot(bird4.getRot() * change);


	bird1.setOmega(1);
	bird2.setOmega(2);
	bird3.setOmega(3);
	bird4.setOmega(2.5);

	birdArray[0] = bird1;
	birdArray[1] = bird2;
	birdArray[2] = bird3;
	birdArray[3] = bird4;

	if (print) {
		for (int b = 0; b < nb; b++)
		{
		for (int o = 0; o < 4; o++)
		{
		for (int i = 0; i < 4; i++)
		{
		printf("%f\t", birdArray[b].getRot(o, i));
		}

		printf("\n");

		}
		printf("\n\n");
		}
	}
}

float toRadians(float angle)
{
	return angle * (M_PI / 180);
}

GLfloat * generateVertexList(float radius, float zRestrict, float xRestrict)
{
	//loop variables should be integers, so we should use degrees
	//We subtract 1 from np and nm + 1 so that we avoid an extra loop

	if (np == 1 || nm == 0)
	{
		printf("You cant divide by Zero or you get points taken off!\n");
		return 0;
	}

	int Ostep = 180 / (np - 1);
	int Istep = 360 / nm;

	float x, y, z, r;

	int numVert = np * (nm + 1);

	static GLfloat * verticesList = new GLfloat[3 * numVert];

	int p = 0;

	for (int o = 0; o <= 180; o += Ostep)
	{
		r = sin(toRadians(o));

		y = cos(toRadians(o)) * -1 * radius;

		for (int i = 0; i <= 360; i += Istep)
		{

			x = r * cos(toRadians(i)) * radius * xRestrict;
			z = r * sin(toRadians(i)) * -1 * radius * zRestrict;

			verticesList[p] = x;
			p++;

			verticesList[p] = y;
			p++;

			verticesList[p] = z;
			p++;
		}
	}

	return verticesList;

}

GLfloat * generateTrenchVertexList(float radius)
{
	if (np == 1 || nm == 0)
	{
		printf("You cant divide by Zero or you get points taken off!\n");
		return 0;
	}

	int Ostep = 180 / (np - 1);
	int Istep = 360 / nm;

	float x, y, z, r;

	int numVert = ((np + 6) * (nm + 1));
	
	//int numVert = np * (nm + 1);
	
	static GLfloat * verticesList = new GLfloat[3 * numVert];

	int p = 0;
	//Bottom Half of Sphere
	for (int o = 0; o <= 80; o += Ostep) {
		r = sin(toRadians(o));
		y = cos(toRadians(o)) * -1 * radius;
		for (int i = 0; i <= 360; i += Istep)
		{
			x = r * cos(toRadians(i)) * radius;
			z = r * sin(toRadians(i)) * -1 * radius;

			verticesList[p] = x;
			p++;
			verticesList[p] = y;
			p++;
			verticesList[p] = z;
			p++;
		}
	}

	//Outer Ring of Bottom Wall
	for (int i = 0; i <= 360; i += Istep)
	{
		x = r * cos(toRadians(i)) * radius;
		z = r * sin(toRadians(i)) * -1 * radius;

		verticesList[p] = x;
		p++;
		verticesList[p] = y;
		p++;
		verticesList[p] = z;
		p++;
	}

	//Inner Ring of Bottom Wall
	for (int i = 0; i <= 360; i += Istep)
	{
		x = r * cos(toRadians(i)) * radius * 0.75;
		z = r * sin(toRadians(i)) * -1 * radius * 0.75;

		verticesList[p] = x;
		p++;
		verticesList[p] = y;
		p++;
		verticesList[p] = z;
		p++;
	}

	//Cylinder Middle
	for (int o = 80; o <= 100; o += Ostep) {
		r = sin(toRadians(o));
		y = cos(toRadians(o)) * -1 * radius;
		for (int i = 0; i <= 360; i += Istep)
		{
			x = r * cos(toRadians(i)) * radius * 0.75;
			z = r * sin(toRadians(i)) * -1 * radius * 0.75;

			verticesList[p] = x;
			p++;
			verticesList[p] = y;
			p++;
			verticesList[p] = z;
			p++;
		}
	}

	//Inner Ring of Upper Wall
	for (int i = 0; i <= 360; i += Istep)
	{
		x = r * cos(toRadians(i)) * radius * 0.75;
		z = r * sin(toRadians(i)) * -1 * radius * 0.75;

		verticesList[p] = x;
		p++;
		verticesList[p] = y;
		p++;
		verticesList[p] = z;
		p++;
	}

	//Outter Ring of Upper Wall
	for (int i = 0; i <= 360; i += Istep)
	{
		x = r * cos(toRadians(i)) * radius;
		z = r * sin(toRadians(i)) * -1 * radius;

		verticesList[p] = x;
		p++;
		verticesList[p] = y;
		p++;
		verticesList[p] = z;
		p++;
	}

	//Upper Half of 
	for (int o = 100; o <= 180; o += Ostep) {
	r = sin(toRadians(o));
	y = cos(toRadians(o)) * -1 * radius;
	for (int i = 0; i <= 360; i += Istep)
	{
		x = r * cos(toRadians(i)) * radius;
		z = r * sin(toRadians(i)) * -1 * radius;

		verticesList[p] = x;
		p++;
		verticesList[p] = y;
		p++;
		verticesList[p] = z;
		p++;
	}
}

	return verticesList;

}

GLfloat * generateTrenchNormals(float radius)
{
	if (np == 1 || nm == 0)
	{
		printf("You cant divide by Zero or you get points taken off!\n");
		return 0;
	}

	int Ostep = 180 / (np - 1);
	int Istep = 360 / nm;

	float x, y, z, r;

	int numVert = ((np + 6) * (nm + 1));

	static GLfloat * normalsList = new GLfloat[3 * numVert];

	int p = 0;

	//Bottom Half of Sphere
	for (int o = 0; o <= 80; o += Ostep) {
		r = sin(toRadians(o));
		y = cos(toRadians(o)) * -1 * radius;
		for (int i = 0; i <= 360; i += Istep)
		{
			x = r * cos(toRadians(i)) * radius;
			z = r * sin(toRadians(i)) * -1 * radius;

			normalsList[p] = x;
			p++;
			normalsList[p] = y;
			p++;
			normalsList[p] = z;
			p++;
		}
	}

	//Outer Ring of Bottom Wall
	for (int i = 0; i <= 360; i += Istep)
	{
		normalsList[p] = 0.0;
		p++;
		normalsList[p] = 1.0;
		p++;
		normalsList[p] = 0.0;
		p++;
	}

	//Inner Ring of Bottom Wall
	for (int i = 0; i <= 360; i += Istep)
	{
		normalsList[p] = 0;
		p++;
		normalsList[p] = 1.0;
		p++;
		normalsList[p] = 0;
		p++;
	}

	//Cylinder Middle
	for (int o = 80; o <= 100; o += Ostep) {
		r = sin(toRadians(o));
		y = cos(toRadians(o)) * -1 * radius;
		for (int i = 0; i <= 360; i += Istep)
		{
			x = r * cos(toRadians(i)) * radius * 0.75;
			z = r * sin(toRadians(i)) * -1 * radius * 0.75;

			normalsList[p] = x;
			p++;
			normalsList[p] = y;
			p++;
			normalsList[p] = z;
			p++;
		}
	}

	//Inner Ring of Upper Wall
	for (int i = 0; i <= 360; i += Istep)
	{
		normalsList[p] = 0;
		p++;
		normalsList[p] = -1;
		p++;
		normalsList[p] = 0;
		p++;
	}

	//Outter Ring of Upper Wall
	for (int i = 0; i <= 360; i += Istep)
	{
		normalsList[p] = 0;
		p++;
		normalsList[p] = -1;
		p++;
		normalsList[p] = 0;
		p++;
	}

	//Upper Half of 
	for (int o = 100; o <= 180; o += Ostep) {
		r = sin(toRadians(o));
		y = cos(toRadians(o)) * -1 * radius;
		for (int i = 0; i <= 360; i += Istep)
		{
			x = r * cos(toRadians(i)) * radius;
			z = r * sin(toRadians(i)) * -1 * radius;

			normalsList[p] = x;
			p++;
			normalsList[p] = y;
			p++;
			normalsList[p] = z;
			p++;
		}

		return normalsList;

	}
}

GLfloat * generateSphereNormals(float radius, float zRestrict, float xRestrict)
{
	if (np == 1 || nm == 0)
	{
		printf("You cant divide by Zero or you get points taken off!\n");
		return 0;
	}

	int Ostep = 180 / (np - 1);
	int Istep = 360 / nm;

	float x, y, z, r;

	int numVert = np * (nm + 1);

	static GLfloat * normalsList = new GLfloat[3 * numVert];

	int p = 0;

	for (int o = 0; o <= 180; o += Ostep)
	{
		r = sin(toRadians(o));

		y = cos(toRadians(o)) * -1 * radius;

		for (int i = 0; i <= 360; i += Istep)
		{

			x = r * cos(toRadians(i)) * radius * xRestrict;
			z = r * sin(toRadians(i)) * -1 * radius * zRestrict;

			normalsList[p] = x;
			p++;

			normalsList[p] = y;
			p++;

			normalsList[p] = z;
			p++;
		}
	}

	return normalsList;
}

GLfloat * generateCylinderVertex()
{
	if (height == 1 || cnm == 0)
	{
		printf("You cant divide by Zero or you get points taken off!\n");
		return 0;
	}

	int oStep = 180 / (height - 1);
	int iStep = 360 / cnm;

	float x, y, z, r;

	int numVert = height * (cnm + 1);

	static GLfloat * vertList = new GLfloat[3 * numVert];

	int p = 0;

	for (int o = 0; o <= 180; o += oStep)
	{
		//y = cos(toRadians(o)) * -1 * cRadius;
		y = o;
		for (int i = 0; i <= 360; i += iStep)
		{
			x = cos(toRadians(i)) *cRadius;
			z = sin(toRadians(i)) *-1 * cRadius;

			vertList[p] = x;
			p++;

			vertList[p] = y;
			p++;

			vertList[p] = z;
			p++;
		}

	}

	return vertList;
}

GLuint * generateCylinderIndexList()
{

	int numIndex = (2 * (cnm + 1) + 1) * (height - 1);

	static GLuint * indexList = new GLuint[numIndex];

	int p = 0;

	for (int o = 1; o <= height - 1; o++)
	{
		for (int i = o * (cnm + 1); i < o * (cnm + 1) + (cnm + 1); i++)
		{
			indexList[p] = i;
			p++;

			indexList[p] = i - (cnm + 1);
			p++;
		}

		indexList[p] = 0xFFFF;
		p++;

	}

	return indexList;

}

GLuint * generateIndexList(bool trench)
{
	int numIndices;
	if (trench)
	{
		numIndices = (2 * (nm + 1) + 1 ) * (np + 6);
	}
	else
	{
		numIndices = (2 * (nm + 1) + 1) * (np - 1);
	}

	//printf("NumInd is %i\n", numIndices);

	//We add np - 1 to the number of indices for the number of primitive restart indices
	static GLuint * indices = new GLuint[numIndices];

	//used to keep track of place in index list
	int p = 0;

	//printf("The number of indices is: \t%i\n", numIndices);
	//printf("Size of index list is: \t%i\n", numIndices);

	int outerMax;
	if (trench)
	{
		outerMax = np + 6;
	}
	else
	{
		outerMax = np - 1;
	}

	for (int o = 1; o <= outerMax; o++)
	{
		//o is the triangle strip number
		for (int i = o * (nm + 1); i < o * (nm + 1) + (nm + 1); i++)
		{
			indices[p] = i;
			p++;

			indices[p] = i - (nm + 1);
			p++;
		}
		indices[p] = 0xFFFF;
		p++;
	}

	/*printf("After nested loop, p is: %i\n", p);

	for (int c = 0; c < numIndices; c++)
	{
	printf("%i\t", indices[c]);
	}*/
	//printf("NumIndices = %i\nP = %i\n", numIndices, p);
	return indices;
}

GLfloat * generateTextureUV()
{
	int numIndices;

	numIndices = ((np + 6) * (nm + 1)) - 1;

	static GLfloat * indices = new GLfloat[numIndices];

	int p = 0;

	for (int o = 0; o <= np + 5; o++)
	{
		for (int i = 0; i < nm; i++)
		{
			indices[p] = i / nm;
			p++;

			indices[p] = o;
			p++;
		}
		
	}

	return indices;
}

/**
This function sets up a Vertex Array Object and buffers vertex data on the graphics card.
You should change the vertex and index lists to generate sphere geometry as described in
the assignment handout and in class.
*/
GLuint setupSphereVAO(GLuint position_loc, GLuint color_loc, GLuint  norm_loc, GLuint Od_loc, GLuint UV_loc, float radius, float zRestrict, float xRestrict, int colorType, bool trench)
{
	//printf("Radius is %f\n", radius);

	//GLfloat * vertices = generateVertexList(radius, zRestrict, xRestrict);
	//GLfloat * vertices = generateTrenchVertexList(radius);

	int numVerts, indexLsize;

	GLfloat * vertices;
	GLfloat * normals;
	/*GLfloat * UV;
	if (trench) {

		UV = generateTextureUV();

	}	*/
	if (trench) 
	{
		indexLsize = (2 * (nm + 1) + 1) * (np + 6);
		numVerts = (np + 6) * (nm + 1);
		vertices = generateTrenchVertexList(radius);
		normals = generateTrenchNormals(radius);
	}
	else
	{
		indexLsize = (2 * (nm + 1) + 1) * (np - 1);
		numVerts = (np) * (nm + 1);
		vertices = generateVertexList(radius, zRestrict, xRestrict);
		normals = generateSphereNormals(radius, zRestrict, xRestrict);
	}

	GLuint * index_list = generateIndexList(trench);

	GLfloat * colorsT = new GLfloat[numVerts * 3];

	float redLevel = 1.0;
	float blueLevel = -1.0;
	float greenLevel = 0.0;
	float colorLevel = 0.0;

	float cvStep = 1.0 / np - 1;
	float chStep = 1.0 / nm;
	int p = 0;

	//gmtl::Vec3f Od[3];

	if (colorType == 1)
	{
		//Od[0] = 0.1f;
		//Od[1] = 0.6f;
		//Od[2] = 0.3f;

		for (int o = 0; o < np + 6 ; o++)
		{
			for (int i = 0; i < nm + 1; i++)
			{
				colorsT[p] = redLevel;
				redLevel *= -1;
				p++;

				colorsT[p] = 0.0;
				greenLevel += chStep;
				p++;

				colorsT[p] = blueLevel;
				blueLevel *= -1;
				p++;
			}
		}
	}
	else if (colorType == 2)
	{
		//Od[0] = 0.6f;
		//Od[1] = 0.0f;
		//Od[2] = 0.3f;

		blueLevel = 0.0;

		for (int o = 0; o < np; o++)
		{
			for (int i = 0; i < nm + 1; i++)
			{
				colorsT[p] = redLevel * colorLevel;
				redLevel -= 0.15;
				p++;

				colorsT[p] = 0.0;
				p++;

				colorsT[p] = blueLevel * colorLevel;
				blueLevel += 0.15;
				p++;
			}

			redLevel = 1.0;
			blueLevel = 0.0;
			colorLevel += 0.15;
		}
	}
	else
	{
		//Od[0] = 0.2;
		//Od[1] = 0.2;
		//Od[2] = 0.6;

		colorLevel = 1.0;
		for (int o = 0; o < np; o++)
		{
			for (int i = 0; i < nm + 1; i++)
			{
				colorsT[p] = colorLevel;
				p++;

				colorsT[p] = colorLevel;
				p++;

				colorsT[p] = colorLevel;
				p++;

			}

			colorLevel -= 0.25;
		}

	}

	GLuint VAO_out, VBO[4], EAB;  /* Create handles for our Vertex Array Object and two Vertex Buffer Objects */

								  /* Assign ID for a Vertex Array Object to our handle */
	glGenVertexArrays(1, &VAO_out);
	/* Assign IDs for two Vertex Buffer Objects to our handle */
	glGenBuffers(4, VBO); //need to add one for normals
	/* Assign ID for an index buffer to our handle */
	glGenBuffers(1, &EAB);

	/* Bind our Vertex Array Object as the current used object */
	glBindVertexArray(VAO_out);

	/* Bind our first VBO as being the active buffer and storing vertex attributes (coordinates) */
	glBindBuffer(GL_ARRAY_BUFFER, VBO[0]);
	/* Copy the vertex data from diamond to our buffer */
	/* num_verts * 3 * sizeof(GLfloat) is the size of the diamond array, since */
	/* it contains num_verts * 3 GLfloat values */
	glBufferData(GL_ARRAY_BUFFER, numVerts * 3 * sizeof(GLfloat), vertices, GL_STATIC_DRAW);
	/* Specify that our coordinate data is going into attribute index 0 (SHADER_POSITION_INDEX), and contains three floats per vertex */
	glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);
	/* Enable attribute index 0 (SHADER_POSITION_INDEX) as being used */
	glEnableVertexAttribArray(position_loc);

	/* Bind our second VBO as being the active buffer and storing vertex attributes (colors) */
	glBindBuffer(GL_ARRAY_BUFFER, VBO[1]);
	/* Copy the color data from colors to our buffer */
	/* num_verts * 3 * sizeof(GLfloat) is the size of the diamond array, since */
	/* it contains num_verts * 3 GLfloat values */
	glBufferData(GL_ARRAY_BUFFER, numVerts * 3 * sizeof(GLfloat), colorsT, GL_STATIC_DRAW);
	/* Specify that our color data is going into attribute index 1 (SHADER_COLOR_INDEX), and contains three floats per vertex */
	glVertexAttribPointer(color_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);
	/* Enable attribute index 1 (SHADER_COLOR_INDEX) as being used */
	glEnableVertexAttribArray(color_loc);

	
	//Loading in Normals to Vertex Shader
	glBindBuffer(GL_ARRAY_BUFFER, VBO[2]);
	glBufferData(GL_ARRAY_BUFFER, numVerts * 3 * sizeof(GLfloat), normals, GL_STATIC_DRAW);
	glVertexAttribPointer(norm_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(norm_loc);

	/*if (trench) 
	{
		glBindBuffer(GL_ARRAY_BUFFER, VBO[3]);
		glBufferData(GL_ARRAY_BUFFER, 3 * numVerts * sizeof(GLfloat), UV, GL_STATIC_DRAW);
		glVertexAttribPointer(UV_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(UV_loc);
	}	*/


	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EAB);
	/* Copy the index data from indices to our buffer */
	/* 4 * sizeof(GLuint) is the size of the indices array, since it contains 4 GLuint values */
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexLsize * sizeof(GLuint), index_list, GL_STATIC_DRAW);

	/* Clear array bindings for to avoid later problems */
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	/*
	These calls just remove the names/IDs from use. The buffered data are still associated
	with the vertex array object. Since they are only scoped to this function, however,
	we would normally remove them here or GL will never use them again within the application.
	This probably wouldn't cause errors for the assignment, so we will omit it here.
	*/
	// glDeleteBuffers(2, VBO);
	// glDeleteBuffers(1, &EAB);

	return VAO_out;
}

GLuint setupCylinder(GLuint position_loc, GLuint color_loc)
{
	int numVerts = height * (nm + 1);
	int numIndex = (2 * (nm + 1) + 1) * (height - 1);

	GLfloat * cylinderVertices = generateCylinderVertex();
	GLuint * cylinderIndexList = generateCylinderIndexList();
	GLfloat * cylinderColors = new GLfloat[numVerts * 3];

	for (int i = 0; i < (height - 1) * (nm + 1); i++)
	{
		cylinderColors[i] = 1.0;
	}

	GLuint VAO_cylinder, VBO_cylinder[2], EABc;

	glGenVertexArrays(1, &VAO_cylinder);
	glGenBuffers(2, VBO_cylinder);
	glGenBuffers(1, &EABc);

	glBindVertexArray(VAO_cylinder);

	glBindBuffer(GL_ARRAY_BUFFER, VBO_cylinder[0]);
	glBufferData(GL_ARRAY_BUFFER, numVerts * 3 * sizeof(GLfloat), cylinderVertices, GL_STATIC_DRAW);
	glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(position_loc);

	glBindBuffer(GL_ARRAY_BUFFER, VBO_cylinder[1]);
	glBufferData(GL_ARRAY_BUFFER, numVerts * 3 * sizeof(GLfloat), cylinderColors, GL_STATIC_DRAW);
	glVertexAttribPointer(color_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(color_loc);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EABc);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, numIndex * sizeof(GLuint), cylinderIndexList, GL_STATIC_DRAW);

	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	return VAO_cylinder;
}

float influence(float distance, float a, float b, float c)
{

	if (distance > 1.7f)
	{
		//printf("Distance became too large!\n\n");
		return 0.0;
	}

	return 1 / (a + (b * distance) + (c * distance * distance));

}

void simulationStep()
{

	iterations++;

	//1 - Convert Bird's omega to world-referenced velocity vector
	//Array of all bird velocities
	gmtl::Vec4f * birdVelocity = new gmtl::Vec4f[nb];
	gmtl::Vec4f * birdAccel = new gmtl::Vec4f[nb];
	
	//Container for Y_axis
	gmtl::Vec4f get_Y = { 0, 1, 0, 0 };
	gmtl::Vec4f bird_Y;
	
	//flight radius
	float flightRadius = 1.5;

	//delta
	float delta = 0.1;

	for (int n = 0; n < nb; n++)
	{
		//get Y-axis as a vector
		bird_Y = birdArray[n].getRot() * get_Y;
		birdVelocity[n] = bird_Y * birdArray[n].getRadius() * birdArray[n].getOmega();
	}

	//Step 2 - Compute bird's world referenced acceleration vector based on intrabird forces
	//	2.1 - Compute distance between bird and all other birds
	//		for each bird
	float theta, lateralDist, accelMagAccum, avgRadius, vertDist, distance;

	gmtl::Vec3f U, X_i, X_j;

	float influenceLevel;

	gmtl::Vec3f centering, dispersion, velocityMatching, forcesOnBird, forceOfJ, dispF1, centF1, dispcentX, centX;
	gmtl::Vec4f forceForAccel;

	gmtl::Vec3f get_X = {1, 0, 0};
	gmtl::Vec3f rotVj, subVi;
	gmtl::AxisAnglef rot_about_U;
	gmtl::Matrix44f U_rot_mat, birdR;
	gmtl::identity(U_rot_mat);
	gmtl::identity(birdR);

	gmtl::Quatf U_rot_quat;
	for (int n = 0; n < nb; n++)
	{
		X_i = birdArray[n].getRot() * get_X;
		subVi = { birdVelocity[n][0], birdVelocity[n][1], birdVelocity[n][2] };
		
		birdR = W * birdArray[n].getRot();
		dispcentX = birdR * get_X;
		
		subVi = subVi + (dispcentX * birdArray[n].getVertVel());
		
		forcesOnBird = { 0,0,0 };
		accelMagAccum = 0;
		forceOfJ = { 0, 0, 0 };

		//Add way to calculate dispersion on obsticles from obsticleList
		for (int o = 0; o < no; o++)
		{
			X_j = obstacleList[o] * get_X;

			gmtl::cross(U, X_i, X_j);
			gmtl::normalize(U);

			theta = acos(gmtl::dot(X_i, X_j));

			lateralDist = theta * flightRadius * 1.2;

			gmtl::cross(dispersion, X_i, U);
			dispersion = dispersion * influence(lateralDist, 1, 7, 3);
			accelMagAccum += gmtl::length(dispersion);
			if (accelMagAccum < maxAccel)
			{
				forceOfJ += dispersion;
				BirdDisp[n] = BirdDisp[n] + gmtl::Vec4f(dispersion[0], dispersion[1], dispersion[2], 0.0f);
				//printf("Dispersion magnitude: \t%f\n", gmtl::length(dispersion));
			}

			//BirdDisp[n] = BirdDisp[n] + gmtl::Vec4f(dispersion[0], dispersion[1], dispersion[2], 0.0f);

			//forcesOnBird += dispersion;
		}

		//compute the distance between it, and each other bird
		for (int i = 0; i < nb; i++)
		{
			if (i != n)
			{
				X_j = birdArray[i].getRot() * get_X;
				
				gmtl::cross(U, X_i, X_j);
				gmtl::normalize(U);

				theta = acos(gmtl::dot(X_i, X_j));

				avgRadius = (birdArray[n].getRadius() + birdArray[i].getRadius()) * 0.5;

				lateralDist = theta * avgRadius;

				vertDist = birdArray[n].getRadius() - birdArray[i].getRadius();

				//printf("Vertical distance from %i to %i: \t%f\n\n\n", n, i, vertDist);

				distance = sqrtf((lateralDist * lateralDist) + (vertDist * vertDist));

				//printf("Lateral distance / Distance from %i to %i: \t%f / %f\n\n\n", n, i, lateralDist, distance);
				
				//	2.2 - Develop an influence function to weight the influence of other birds
				
				//	2.3 - Calculate Inter Bird forces
				//		2.3.1 - Dispersion
				gmtl::cross(dispF1, X_i, U);

				dispersion = (lateralDist * dispF1) + (vertDist * dispcentX);
				gmtl::normalize(dispersion);
				
				dispersion = dispersion * influence(distance, 1, 4, 10);

				//		2.3.2 - Centering
				gmtl::cross(centF1, U, X_i);

				centering = (lateralDist * centF1) + (vertDist * dispcentX);
				gmtl::normalize(centering);

				centering = centering * influence(distance, 1.1, 8, 4);

				//		2.3.3 - Velocity matching
				rotVj = { birdVelocity[i][0], birdVelocity[i][1], birdVelocity[i][2] };

				//gmtl::AxisAnglef rot_about_U(theta * -1, U);
				U_rot_quat[0] = sin(toRadians(theta * -0.5)) * U[0];
				U_rot_quat[1] = sin(toRadians(theta * -0.5)) * U[1];
				U_rot_quat[2] = sin(toRadians(theta * -0.5)) * U[2];
				U_rot_quat[3] = cos(toRadians(theta * -0.5));
				
				U_rot_mat = gmtl::make<gmtl::Matrix44f>(U_rot_quat);

				//		rotate vector birdVelocity[i] around U
				rotVj = U_rot_mat * rotVj;

				//		subtract vector birdVelocity[n] from above
				velocityMatching = rotVj - subVi;

				//		scale by custom influence level
				velocityMatching *= influence(distance, 1, 7, 12);

				accelMagAccum += gmtl::length(dispersion);
				if (accelMagAccum < maxAccel) 
				{
					forceOfJ += dispersion;
					BirdDisp[n] = BirdDisp[n] + gmtl::Vec4f(dispersion[0], dispersion[1], dispersion[2], 0.0f);
					printf("Dispersion magnitude bird %i: \t%f\n",n, gmtl::length(dispersion));
				}
				else
				{
					printf("Dispersion magnitude bird %i: \t%f\tWas not added\n",n, gmtl::length(dispersion));
				}
				
				accelMagAccum += gmtl::length(velocityMatching);
				if (accelMagAccum < maxAccel) 
				{
					forceOfJ += velocityMatching;
					BirdVelMatch[n] = BirdDisp[n] + gmtl::Vec4f(velocityMatching[0], velocityMatching[1], velocityMatching[2], 0.0f);
					printf("VelMatching magnitude bird %i: \t%f\n",n ,gmtl::length(velocityMatching));
				}
				else
				{
					printf("VelMatching magnitude bird %i: \t%f\tWas not added!\n",n, gmtl::length(velocityMatching));
				}
				
				accelMagAccum += gmtl::length(centering);
				if (accelMagAccum < maxAccel) 
				{
					forceOfJ += centering;
					BirdCenter[n] = BirdDisp[n] + gmtl::Vec4f(centering[0], centering[1], centering[2], 0.0f);
					printf("Centering magnitude bird %i: \t%f\n",n, gmtl::length(centering));
				}
				else
				{
					printf("Centering was not added to calculations!\n");
				}

				printf("\n\n\n");
				//Force J on bird I = dispersion + centering + velocity matching + flight speed control
				//forceOfJ = centering + dispersion + velocityMatching;
				//BirdDisp[n] = BirdDisp[n] + gmtl::Vec4f( dispersion[0], dispersion[1], dispersion[2], 0.0f );
				//BirdCenter[n] = BirdDisp[n] + gmtl::Vec4f(centering[0], centering[1], centering[2], 0.0f);
				//BirdVelMatch[n] = BirdDisp[n] + gmtl::Vec4f(velocityMatching[0], velocityMatching[1], velocityMatching[2], 0.0f);


				//Forces acting on Bird I = forces + ForceJonI
				forcesOnBird += forceOfJ;

				accelMagAccum = 0.0;

			}

		}

		forceForAccel = { forcesOnBird[0], forcesOnBird[1], forcesOnBird[2], 0.0f };
		
		//gmtl::normalize(forceForAccel);

		if (gmtl::length(BirdDisp[n]) > maxDispMag)
		{
			maxDispMag = gmtl::length(BirdDisp[n]);
		}

		if (gmtl::length(BirdCenter[n]) > maxCentMag)
		{
			maxCentMag = gmtl::length(BirdCenter[n]);
		}
		if (gmtl::length(BirdVelMatch[n]) > maxVmatchMag)
		{
			maxVmatchMag = gmtl::length(BirdVelMatch[n]);
		}
	
		dispAccMag += gmtl::length(BirdDisp[n]);
		centAccMag += gmtl::length(BirdCenter[n]);
		vmatchAccMag += gmtl::length(BirdVelMatch[n]);

		meanCentMag = centAccMag / (iterations * nb) ;
		meanDispMag = dispAccMag / (iterations * nb);
		meanVmatchMag = vmatchAccMag / (iterations * nb);
		
		//printf("Iterations\t: %f\n\n", iterations);
		//
		//printf("Actual Dispersion Magnitude: %f\n", gmtl::length(BirdDisp[n]));
		//printf("Mean Dispersion Magnitude: %f\n", meanDispMag);
		//printf("Largest Dispersion Magnitude: %f\n\n", maxDispMag);
		//
		//printf("Actual Centering Magnitude: %f\n", gmtl::length(BirdCenter[n]));
		//printf("Mean Centering Magnitude: %f\n", meanCentMag);
		//printf("Largest Centering Magnitude: %f\n\n", maxCentMag);
		//
		//printf("Actual VelMatching Magnitude: %f\n", gmtl::length(BirdVelMatch[n]));
		//printf("Mean VelMatching Magnitude: %f\n", meanVmatchMag);
		//printf("Largest VelMatching Magnitude: %f\n\n", maxVmatchMag);
		//
		gmtl::normalize(BirdDisp[n]);
		BirdDisp[n] *= 0.5;

		gmtl::normalize(BirdCenter[n]);
		BirdCenter[n] *= 0.5;

		gmtl::normalize(BirdVelMatch[n]);
		BirdVelMatch[n] *= 0.5;

		birdAccel[n] = { forceForAccel[0] / (nb - 1), forceForAccel[1] / (nb - 1), forceForAccel[2] / (nb - 1), forceForAccel[3] / (nb - 1) };

		if (gmtl::length(birdAccel[n]) > maxAccMag)
		{
			maxAccMag = gmtl::length(birdAccel[n]);
		}

		totalAccMag += gmtl::length(birdAccel[n]);

		meanAccMag = totalAccMag / iterations;

		maxAccel = meanAccMag;
		
		//(meanDispMag + meanCentMag + meanVmatchMag) / (iterations);

		//printf("Max Accel is now: \t%f\n\n\n", maxAccel);

		//printf("Mean Accelerations Magnitude is: %f\n", meanAccMag);
		//
		//printf("Largest Acceleration Magnitude: %f\n\n", maxAccMag);
		//
		//printf("Accel is: \t%f\t%f\t%f\t%f\n\n", birdAccel[n][0] * delta, birdAccel[n][1] * delta, birdAccel[n][2] * delta, birdAccel[n][3] * delta);

		//printf("Bird %i Radius is: \t%f\n\n\n", n, birdArray[n].getRadius());
		//printf("Bird %i VertVel is: \t%f\n\n\n", n, birdArray[n].getVertVel());

	}

	gmtl::Vec4f plus_X = {1.1f, 0, 0, 0};
	gmtl::Vec4f plus_Y = { 0, 1.5f, 0, 0 };

	//Step 3 - Update bird's world-referenced velocity by adding the product of step 2 and delta
	for (int n = 0; n < nb; n++)
	{
		//gmtl::normalize(birdVelocity[n]);
		//printf("Velocity is: \t%f\t%f\t%f\t%f\n\n", birdVelocity[n][0], birdVelocity[n][1], birdVelocity[n][2], birdVelocity[n][3]);

		birdVelocity[n] += (delta * birdAccel[n]);

		//printf("Velocity magn for bird %i is: \t%f\n\n\n", n, gmtl::length(birdVelocity[n]));


		//Flight Speed Control
		if (gmtl::length(birdVelocity[n]) < 0.75)
		{
			birdVelocity[n] += plus_Y;
		}
		else if (gmtl::length(birdVelocity[n]) > 1.2)
		{
			birdVelocity[n] -= plus_Y;
			//printf("magntiude of vel is: %f\n\n", gmtl::length(birdVelocity[n]));
			//gmtl::normalize(birdVelocity[n]);
			//birdVelocity[n][1] -= 1.5;
		}

	}

	//Step 4 - Rotate bird to point in updated velocity and convert velocity into omega
	gmtl::Matrix44f newRot;
	gmtl::Vec3f newZ, newY, oldZ, oldY, oldX, newV_i;
	float newOmega, newVertVel;
	gmtl::Vec3f get_Z = { 0, 0, 1 };
	gmtl::Vec3f get_Y3 = { 0, 1 ,0 };
	gmtl::Vec4f get_X4 = {1, 0, 0, 0};
	gmtl::Vec4f temp;
	
	for (int n = 0; n < nb; n++)
	{
		gmtl::identity(newRot);

		oldX = birdArray[n].getRot() * get_X;
		oldY = birdArray[n].getRot() * get_Y3;
		oldZ = birdArray[n].getRot() * get_Z;
		
		newV_i = { birdVelocity[n][0], birdVelocity[n][1], birdVelocity[n][2] };

		//New Z is normalized cross product of X and Vi from Step 3
		gmtl::cross(newZ, oldX, newV_i);
		//gmtl::cross(newZ, newV_i, oldX);
		gmtl::normalize(newZ);
		
		gmtl::cross(newY, newZ, oldX);

		//printf("New Y for bird %i  is: \t%f\t%f\t%f\n", n, newY[0], newY[1], newY[2]);
		//printf("New V for bird %i  is: \t%f\t%f\t%f\n", n, newV_i[0], newV_i[1], newV_i[2]);

		//Set X
		newRot[0][0] = oldX[0];
		newRot[1][0] = oldX[1];
		newRot[2][0] = oldX[2];

		//Set Y
		newRot[0][1] = newY[0];
		newRot[1][1] = newY[1];
		newRot[2][1] = newY[2];

		//Set Z
		newRot[0][2] = newZ[0];
		newRot[1][2] = newZ[1];
		newRot[2][2] = newZ[2];
		
		//Compute new Omega
		newOmega = gmtl::dot(newY, newV_i) / birdArray[n].getRadius();
		//printf("New O for bird %i  is: \t%f\n\n\n", n, newOmega);
		temp = birdArray[n].getRot() * get_X4;

		newVertVel = gmtl::dot(birdVelocity[n], temp);

		//printf("Current Velocity for bird %i is:\t%f\t%f\t%f\n\n\n", n, birdVelocity[n][0], birdVelocity[n][1], birdVelocity[n][2]);

		/*printf("For bird %i, rotation is:\n");
		for (int o = 0; o < 4; o++)
		{
			for (int i = 0; i < 4; i++)
			{
				printf("%f\t", birdArray[n].getRot(o, i));
			}

			printf("\n");

		}
		printf("\n\n");
		

		printf("For bird %i x is: \t%f\t%f\t%f\n\n\n", n, temp[0], temp[1], temp[2]);

		printf("Current VertVel for bird %i is: %f\n\n\n", n, newVertVel);*/

		//Update State
		birdArray[n].setOmega(newOmega);
		birdArray[n].setRot(newRot);
		birdArray[n].setVertVel(newVertVel);



	}



	//Step 5 - Move the bird forward using local z-rotation of R by an angle that is product of omega and delta
	float rotationAmount;
	gmtl::Matrix44f change;
	gmtl::identity(change);
	
	for (int n = 0; n < nb; n++)
	{
		rotationAmount = birdArray[n].getOmega() * delta;

		birdArray[n].setRadius(birdArray[n].getRadius() + ( birdArray[n].getVertVel() * delta ));

		if (birdArray[n].getRadius() < 1.2)
		{
			birdVelocity[n] += plus_X;
		}
		else if (birdArray[n].getRadius() > 2.0)
		{
			birdVelocity[n] -= plus_X;
		}

		//printf("Current Radius for Bird %i is:\t%f\n\n\n", n, birdArray[n].getRadius());

		change[0][0] = cos(toRadians(rotationAmount));
		change[0][1] = sin(toRadians(rotationAmount)) * -1;
		change[1][0] = sin(toRadians(rotationAmount));
		change[1][1] = cos(toRadians(rotationAmount));
		//change[0][3] = birdArray[n].getRadius();
		change.setState(gmtl::Matrix44f::ORTHOGONAL);

		birdArray[n].setRot(birdArray[n].getRot() * change);
	}

}

/*
Call our function that performs opengl operations. This is where your changes for matrix and vertex
manipulation should be. You will also have to send in multiple VAOs (maybe use an array?) and change
the hard-coded count (4) to each index list size.
*/

void display(GLFWwindow* window, GLuint shader_program, GLuint vm_location, GLuint pvm_location, GLuint light_pos, gmtl::Matrix44f LightSourcePosition, GLuint Wte_loc, gmtl::Matrix44f World_to_Eye, gmtl::Matrix44f P[], GLuint VAO[], gmtl::Matrix44f mat[], gmtl::Vec4f O[])
{

	glUseProgram(shader_program);
	int indexLsize;
	indexLsize = (2 * (nm + 1) + 1) * (np + 6);
	//indexLsize = (2 * (nm + 1) + 1) * (np - 1);
	//indexLsize = (2 * (cnm + 1) + 1) * (height - 1);

	/* Make our background black */
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//Enable primitive restart index
	glEnable(GL_PRIMITIVE_RESTART);

	//Set primitive restart index
	glPrimitiveRestartIndex(0xFFFF);

	//Bind VAO for Sphere
	glBindVertexArray(VAO[0]);

	//Load in light source location
	glUniformMatrix4fv(light_pos, 1, GL_FALSE, LightSourcePosition.mData);

	//Load in World to Eye location
	glUniformMatrix4fv(Wte_loc, 1, GL_FALSE, World_to_Eye.mData);

	//Load M to transform Sphere
	glUniformMatrix4fv(
		vm_location,		// uniform location
		1,					// count
		GL_FALSE,			// transpose (no)
		mat[0].mData	// data
		);

	glUniformMatrix4fv(
		pvm_location,
		1,
		GL_FALSE,
		P[0].mData
		);

	//Draw Sphere
	glDrawElements(
		GL_TRIANGLE_STRIP,
		indexLsize,
		GL_UNSIGNED_INT,
		(void*)0
		);

	

	if (showW)
	{
		//Bind VAO for Y-aligned Cylinder
		glBindVertexArray(VAO[3]);

		glUniformMatrix4fv(
			vm_location,
			1,
			GL_FALSE,
			mat[1].mData
			);
		glUniformMatrix4fv(
			pvm_location,
			1,
			GL_FALSE,
			P[1].mData
			);

		//Draw Y-aligned Cylinder
		glDrawElements(
			GL_TRIANGLE_STRIP,		// mode
			indexLsize,				// count, should be updated with your index list size.
			GL_UNSIGNED_INT,		// type
			(void*)0				// element array buffer offset
			);
		//Load in matrix data from matrix array
		// Y_to_X_rot should be the first matrix passed in at mat[0]
		glUniformMatrix4fv(
			vm_location,
			1,
			GL_FALSE,
			mat[2].mData
			);

		glUniformMatrix4fv(
			pvm_location,
			1,
			GL_FALSE,
			P[2].mData
			);
		//Draw X-aligned Cylinder
		glDrawElements(
			GL_TRIANGLE_STRIP,
			indexLsize,
			GL_UNSIGNED_INT,
			(void*)0
			);
		//Create Matrix for rotating Y-aligned Cylinder
		glUniformMatrix4fv(
			vm_location,
			1,
			GL_FALSE,
			mat[3].mData
			);

		glUniformMatrix4fv(
			pvm_location,
			1,
			GL_FALSE,
			P[3].mData
			);
		//Draw Z-Aligned Cylinder
		glDrawElements(
			GL_TRIANGLE_STRIP,
			indexLsize,
			GL_UNSIGNED_INT,
			(void*)0
			);
	}

	if (showR)
	{
		//Bind VAO for Y-aligned Cylinder
		glBindVertexArray(VAO[3]);

		glUniformMatrix4fv(
			vm_location,
			1,
			GL_FALSE,
			mat[4].mData
			);

		glUniformMatrix4fv(
			pvm_location,
			1,
			GL_FALSE,
			P[4].mData
			);
		//Draw Y-aligned Cylinder
		glDrawElements(
			GL_TRIANGLE_STRIP,		// mode
			indexLsize,				// count, should be updated with your index list size.
			GL_UNSIGNED_INT,		// type
			(void*)0				// element array buffer offset
			);
		//Load in matrix data from matrix array
		// Y_to_X_rot should be the first matrix passed in at mat[0]
		glUniformMatrix4fv(
			vm_location,
			1,
			GL_FALSE,
			mat[5].mData
			);

		glUniformMatrix4fv(
			pvm_location,
			1,
			GL_FALSE,
			P[5].mData
			);
		//Draw X-aligned Cylinder
		glDrawElements(
			GL_TRIANGLE_STRIP,
			indexLsize,
			GL_UNSIGNED_INT,
			(void*)0
			);
		//Create Matrix for rotating Y-aligned Cylinder
		glUniformMatrix4fv(
			vm_location,
			1,
			GL_FALSE,
			mat[6].mData
			);

		glUniformMatrix4fv(
			pvm_location,
			1,
			GL_FALSE,
			P[6].mData
			);
		//Draw Z-Aligned Cylinder
		glDrawElements(
			GL_TRIANGLE_STRIP,
			indexLsize,
			GL_UNSIGNED_INT,
			(void*)0
			);

	}

	if (showO)
	{
		//Bind VAO for Y-aligned Cylinder
		glBindVertexArray(VAO[3]);

		glUniformMatrix4fv(
			vm_location,
			1,
			GL_FALSE,
			mat[7].mData
			);
		glUniformMatrix4fv(
			pvm_location,
			1,
			GL_FALSE,
			P[7].mData
			);
		//Draw Y-aligned Cylinder
		glDrawElements(
			GL_TRIANGLE_STRIP,		// mode
			indexLsize,				// count, should be updated with your index list size.
			GL_UNSIGNED_INT,		// type
			(void*)0				// element array buffer offset
			);
		//Load in matrix data from matrix array
		// Y_to_X_rot should be the first matrix passed in at mat[0]
		glUniformMatrix4fv(
			vm_location,
			1,
			GL_FALSE,
			mat[8].mData
			);
		glUniformMatrix4fv(
			pvm_location,
			1,
			GL_FALSE,
			P[8].mData
			);
		//Draw X-aligned Cylinder
		glDrawElements(
			GL_TRIANGLE_STRIP,
			indexLsize,
			GL_UNSIGNED_INT,
			(void*)0
			);
		//Create Matrix for rotating Y-aligned Cylinder
		glUniformMatrix4fv(
			vm_location,
			1,
			GL_FALSE,
			mat[9].mData
			);
		glUniformMatrix4fv(
			pvm_location,
			1,
			GL_FALSE,
			P[9].mData
			);
		//Draw Z-Aligned Cylinder
		glDrawElements(
			GL_TRIANGLE_STRIP,
			indexLsize,
			GL_UNSIGNED_INT,
			(void*)0
			);
	}

	indexLsize = (2 * (nm + 1) + 1) * (np - 1);
	
	if (!showCenter && !showDisp && !showMatch)
	{
		for (int i = 0; i < 3 * nb; i += 3)
		{

			//Bind VAO for bird
			glBindVertexArray(VAO[1]);

			//Load B as location to draw to
			glUniformMatrix4fv(
				vm_location,			// uniform location
				1,						// count
				GL_FALSE,				// transpose (no)
				mat[10 + i].mData		// data
				);

			glUniformMatrix4fv(
				pvm_location,
				1,
				GL_FALSE,
				P[10 + i].mData
				);

			//Draw elements
			glDrawElements(
				GL_TRIANGLE_STRIP,
				indexLsize,
				GL_UNSIGNED_INT,
				(void*)0
				);

			//Bind VAO for joint
			glBindVertexArray(VAO[2]);

			//Load Joint 1 model matrix
			glUniformMatrix4fv(
				vm_location,			// uniform location
				1,						// count
				GL_FALSE,				// transpose (no)
				mat[11 + i].mData		// data
				);

			glUniformMatrix4fv(
				pvm_location,
				1,
				GL_FALSE,
				P[11 + i].mData
				);

			//Draw Joint 1
			glDrawElements(
				GL_TRIANGLE_STRIP,
				indexLsize,
				GL_UNSIGNED_INT,
				(void*)0
				);

			//Load Joint 2 model matrix
			glUniformMatrix4fv(
				vm_location,			// uniform location
				1,						// count
				GL_FALSE,				// transpose (no)
				mat[12 + i].mData		// data
				);

			glUniformMatrix4fv(
				pvm_location,
				1,
				GL_FALSE,
				P[12 + i].mData
				);

			//Draw Joint 2
			glDrawElements(
				GL_TRIANGLE_STRIP,
				indexLsize,
				GL_UNSIGNED_INT,
				(void*)0
				);
		}
	}
	if(showDisp || showCenter || showMatch)
	{
		//printf("One of these things is true\n");
		for (int b = 0; b < nb; b++)
		{
			if (showDisp) 
			{
				glBegin(GL_LINES);
				glColor3f(0.0, 0.0, 0.0);
				glVertex4f(O[b][0], O[b][1], O[b][2], O[b][3]);
				glVertex4f(O[b][0] + BirdDisp[b][0], O[b][1] + BirdDisp[b][1], O[b][2] + BirdDisp[b][2], O[b][3] + BirdDisp[b][3]);
				glEnd();
			}

			if (showCenter) 
			{
				glBegin(GL_LINES);
				glColor3f(0.0, 0.0, 1.0);
				glVertex4f(O[b][0], O[b][1], O[b][2], O[b][3]);
				glVertex4f(O[b][0] + BirdCenter[b][0], O[b][1] + BirdCenter[b][1], O[b][2] + BirdCenter[b][2], O[b][3] + BirdCenter[b][3]);
				glEnd();
			}

			if (showMatch) 
			{
				glBegin(GL_LINES);
				glColor3f(0.0, 0.0, 1.0);
				glVertex4f(O[b][0], O[b][1], O[b][2], O[b][3]);
				glVertex4f(O[b][0] + BirdVelMatch[b][0], O[b][1] + BirdVelMatch[b][1], O[b][2] + BirdVelMatch[b][2], O[b][3] + BirdVelMatch[b][3]);
				glEnd();
			}
		}
	}

	indexLsize = (2 * (nm + 1) + 1) * (np + 6);

	for (int o = 0; o < no; o++)
	{
		//printf("Print Function: %i\n\n", 10 + (nb * 3) + o);

		//Bind VAO for bird
		glBindVertexArray(VAO[3]);

		//Load B as location to draw to
		glUniformMatrix4fv(
			vm_location,						// uniform location
			1,									// count
			GL_FALSE,							// transpose (no)
			mat[10 + (3 * nb ) + o].mData		// data
		);

		glUniformMatrix4fv(
			pvm_location,
			1,
			GL_FALSE,
			P[10 + (3 * nb) + o].mData
		);

		//Draw elements
		glDrawElements(
			GL_TRIANGLE_STRIP,
			indexLsize,
			GL_UNSIGNED_INT,
			(void*)0
		);
	}

	glBindVertexArray(0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

/* Our program's entry point */
int main(int argc, char *argv[])
{
	GLFWwindow* mainwindow = NULL;
	GLuint program = NULL, VAO = NULL, VAOb = NULL, VAOj = NULL, VAOc = NULL, VAOo = NULL;
	GLuint pos_location = NULL, color_location = NULL, vm_location = NULL, pvm_location = NULL, norm_location = NULL, light_pos = NULL, WtE_loc = NULL, Od_loc = NULL, UV_loc = NULL;

	//Array of Matrix elements for passing into display funciont
	gmtl::Matrix44f * modelMat = new gmtl::Matrix44f[10 + (nb * 3) + no];
	gmtl::Matrix44f * perspectiveMat = new gmtl::Matrix44f[10 + (nb * 3) + no];
	gmtl::Vec4f * lineOrigins = new gmtl::Vec4f[nb];

	gmtl::Matrix44f Y_rot;
	gmtl::identity(Y_rot);
	
	//Matrix describing the rotation needed to get the Y-axis into the x-axis space
	gmtl::Matrix44f Y_to_X_rot;
	gmtl::identity(Y_to_X_rot);
	Y_to_X_rot[0][0] = cos(toRadians(-90));
	Y_to_X_rot[0][1] = sin(toRadians(-90)) * -1;
	Y_to_X_rot[1][0] = sin(toRadians(-90));
	Y_to_X_rot[1][1] = cos(toRadians(-90));
	Y_to_X_rot.setState(gmtl::Matrix44f::ORTHOGONAL);
	
	//Matrix descritbing how to rotate the Y-axis into the Z-Axis space
	gmtl::Matrix44f Y_to_Z_rot;
	gmtl::identity(Y_to_Z_rot);
	Y_to_Z_rot[1][1] = cos(toRadians(90));
	Y_to_Z_rot[1][2] = sin(toRadians(90)) * -1;
	Y_to_Z_rot[2][1] = sin(toRadians(90));
	Y_to_Z_rot[2][2] = cos(toRadians(90));
	Y_to_Z_rot.setState(gmtl::Matrix44f::ORTHOGONAL);

	//Perspective Matrix things
	
	gmtl::Matrix44f P;
	gmtl::identity(P);

	float near, far, top, bottom, left, right;
	near = 0.1;
	far = 100;
	left = -.16;
	right = .16;
	top = .09;
	bottom = -.09;

	P[0][0] = (2 * near) / (right - left);
	P[1][1] = (2 * near) / (top - bottom);
	P[2][2] = ((far + near) * -1) / (far - near);
	P[3][2] = -1;
	P[2][3] = (-2 * far * near) / (far - near);
	P[0][2] = (right + left) / (right - left);
	P[1][2] = (top + bottom) / (top - bottom);

	gmtl::Matrix44f LightSourcePosition;
	gmtl::identity(LightSourcePosition);
	LightSourcePosition[0][3] = 1.5;
	LightSourcePosition[1][3] = 1.5;
	LightSourcePosition[2][3] = 1.5;

	gmtl::Matrix44f World_to_Eye;
	gmtl::identity(World_to_Eye);

	/* This function sets up our window, OpenGL context, etc. For assignments, you don't need to know how it works. */
	mainwindow = setupWindow();

	/* This function sets up shaders on the graphics card. For assignments, you don't need to know how it works. */
	program = setupShaderProgram();

	/* This function gets attribute and uniform locations from shaders. For _this_ assignment, you don't need to know how it works. */
	init(program, pos_location, color_location, vm_location, pvm_location, norm_location, light_pos, WtE_loc, Od_loc, UV_loc);
	initMatrices();
	/*
	This function sets up a Vertex Array Object and buffers vertex data on the graphics card.
	You should change the vertex and index lists to generate sphere geometry as described in
	the assignment handout and in class.
	*/
	VAO  = setupSphereVAO(pos_location, color_location, norm_location, Od_loc, UV_loc, 1.0, 1.0, 1.0, 1, true);
	VAOb = setupSphereVAO(pos_location, color_location, norm_location, Od_loc, UV_loc, 0.1, 1.0, 0.5, 2, false);
	VAOj = setupSphereVAO(pos_location, color_location, norm_location, Od_loc, UV_loc, 0.07, 0.3, 0.3, 3, false);
	VAOo = setupSphereVAO(pos_location, color_location, norm_location, Od_loc, UV_loc, 0.2, 1.0, 1.0, 1, true);

	//GLuint VAO_cylinder;
	VAOc = setupCylinder(pos_location, color_location);

	GLuint VAOarray[5];
	VAOarray[0] = VAO;
	VAOarray[1] = VAOb;
	VAOarray[2] = VAOj;
	VAOarray[3] = VAOo;
	//VAOarray[4] = VAOo;

	glfwSetKeyCallback(mainwindow, key_callback);
	glfwSetMouseButtonCallback(mainwindow, mouse_button_callback);

	initBird(false);

	gmtl::Matrix44f ob1;
	gmtl::identity(ob1);

	gmtl::Matrix44f ob2;
	gmtl::identity(ob2);

	ob2[0][0] = cos(toRadians(180));
	ob2[0][2] = sin(toRadians(180)) * -1;
	ob2[2][0] = sin(toRadians(180));
	ob2[2][2] = cos(toRadians(180));

	gmtl::Matrix44f ob3;
	gmtl::identity(ob3);

	ob3[0][0] = cos(toRadians(90));
	ob3[0][2] = sin(toRadians(90)) * -1;
	ob3[2][0] = sin(toRadians(90));
	ob3[2][2] = cos(toRadians(90));

	gmtl::Matrix44f change;
	gmtl::identity(change);
	
	change[0][0] = cos(toRadians(45));
	change[0][1] = sin(toRadians(45)) * -1;
	change[1][0] = sin(toRadians(45));
	change[1][1] = cos(toRadians(45));

	ob3 = ob3 * change;

	gmtl::Matrix44f ob4;
	gmtl::identity(ob4);

	ob4[0][0] = cos(toRadians(90));
	ob4[0][2] = sin(toRadians(90)) * -1;
	ob4[2][0] = sin(toRadians(90));
	ob4[2][2] = cos(toRadians(90));

	gmtl::identity(change);

	change[0][0] = cos(toRadians(-45));
	change[0][1] = sin(toRadians(-45)) * -1;
	change[1][0] = sin(toRadians(-45));
	change[1][1] = cos(toRadians(-45));

	ob4 = ob4 * change;

	obstacleList[0] = ob1;
	obstacleList[1] = ob2;
	obstacleList[2] = ob3;
	obstacleList[3] = ob4;

	while (!glfwWindowShouldClose(mainwindow))
	{	
		//Resets:
		gmtl::identity(OtC2);

		//Do Calculations First:
		simulationStep();
		simulationStep();
		simulationStep();

		//Mouse Calculations
		if (captureMouse)
		{
			glfwGetCursorPos(mainwindow, &x_End, &y_End);
			x_Diff = x_End - x_Beg;
			y_Diff = y_End - y_Beg;
			x_Diff *= 0.72;
			y_Diff *= 0.72;

			if (!activeCam) {
				//X-Rotation
				R_lat[1][1] = cos(toRadians(-(float)y_Diff));
				R_lat[1][2] = sin(toRadians(-(float)y_Diff)) * -1;
				R_lat[2][1] = sin(toRadians(-(float)y_Diff));
				R_lat[2][2] = cos(toRadians(-(float)y_Diff));
				R_lat.setState(gmtl::Matrix44f::ORTHOGONAL);

				//Y-Rotation
				R_long[0][0] = cos(toRadians((float)x_Diff));
				R_long[0][2] = sin(toRadians((float)x_Diff)) * -1;
				R_long[2][0] = sin(toRadians((float)x_Diff));
				R_long[2][2] = cos(toRadians((float)x_Diff));
				R_long.setState(gmtl::Matrix44f::ORTHOGONAL);
			}
			else if (activeCam)
			{
				//Y-rotation by ele
				R_ele[0][0] = cos(toRadians((float)x_Diff));
				R_ele[0][2] = sin(toRadians((float)x_Diff)) * -1;
				R_ele[2][0] = sin(toRadians((float)x_Diff));
				R_ele[2][2] = cos(toRadians((float)x_Diff));
				R_long.setState(gmtl::Matrix44f::ORTHOGONAL);

				//X-rotation by azi
				R_azi[1][1] = cos(toRadians(-(float)y_Diff));
				R_azi[1][2] = sin(toRadians(-(float)y_Diff)) * -1;
				R_azi[2][1] = sin(toRadians(-(float)y_Diff));
				R_azi[2][2] = cos(toRadians(-(float)y_Diff));
				R_long.setState(gmtl::Matrix44f::ORTHOGONAL);
			}

			glfwGetCursorPos(mainwindow, &x_Beg, &y_Beg);
		}

		//View Calculations
		if (!activeCam)
		{
			camChange =  R_long * R_lat * T1;
			gmtl::identity(T1);

			//WtC1 = WtC1 * camChange;

			V = camChange;
			V.setState(gmtl::Matrix44f::AFFINE);
			gmtl::invert(V);

			World_to_Eye[0][3] = V[0][3];
			World_to_Eye[1][3] = V[1][3];
			World_to_Eye[2][3] = V[2][3];
			World_to_Eye.setState(gmtl::Matrix44f::TRANS);
		}
		else
		{
			camChange = R_azi * R_ele * R_z * T2;
			gmtl::identity(T2);

			OtC2 = OtC2 * camChange;

			V = WtR * RtO * camChange;
			V.setState(gmtl::Matrix44f::AFFINE);
			gmtl::invert(V);

			World_to_Eye[0][3] = V[0][3];
			World_to_Eye[1][3] = V[1][3];
			World_to_Eye[2][3] = V[2][3];
			World_to_Eye.setState(gmtl::Matrix44f::TRANS);
		}

		//Create MV Matrices and PVM Matrices
		Wy = W * Y_rot;
		Wx = W * Y_to_X_rot;
		Wz = W * Y_to_Z_rot;

		Ry = WtR * Y_rot;
		Rx = WtR * Y_to_X_rot;
		Rz = WtR * Y_to_Z_rot;

		Ox = WtR * RtO * Y_rot;
		Oy = WtR * RtO * Y_to_X_rot;
		Oz = WtR * RtO * Y_to_Z_rot;
		
		W = W * V;
		//W = V * W;
		//LightSourcePosition = V * LightSourcePosition;

		//Load VM Matrices
		modelMat[0] = W;

		modelMat[1] = Wx;
		modelMat[2] = Wy;
		modelMat[3] = Wz;

		modelMat[4] = Rx;
		modelMat[5] = Ry;
		modelMat[6] = Rz;

		modelMat[7] = Ox;
		modelMat[8] = Oy;
		modelMat[9] = Oz;

		//Matrices with perspective added
		perspectiveMat[0] = P * W;

		perspectiveMat[1] = P * Wx;
		perspectiveMat[2] = P * Wy;
		perspectiveMat[3] = P * Wz;

		perspectiveMat[4] = P * Rx;
		perspectiveMat[5] = P * Ry;
		perspectiveMat[6] = P * Rz;

		perspectiveMat[7] = P * Ox;
		perspectiveMat[8] = P * Oy;
		perspectiveMat[9] = P * Oz;
		
		//Per Bird VM/PVM calculations and insertion into VM/PVM arrays
		for (int i = 0; i < nb; i++)
		{
			B =  W * birdArray[i].getRot() * RtO * OtB; //removed V multiplication
			prim1 =  W * birdArray[i].getRot() * RtO * OtB * BtJ1 * J1tP;
			prim2 =  W * birdArray[i].getRot() * RtO * OtB * BtJ2 * J2tP;

			//B = W * birdArray[i].getRot() * OtB; //removed V multiplication
			//prim1 = W * birdArray[i].getRot() * OtB * BtJ1 * J1tP;
			//prim2 = W * birdArray[i].getRot() * OtB * BtJ2 * J2tP;

			lineOrigins[i] = birdArray[i].getRot() * gmtl::Vec4f(RtO[0][3], RtO[1][3], RtO[2][3], RtO[3][3]);

			modelMat[10 + (3*i)] = B;
			modelMat[11 + (3*i)] = prim1;
			modelMat[12 + (3*i)] = prim2;

			//printf("Bird Loop: %i\n", 10 + (3 * i));
			//printf("Bird Loop: %i\n", 11 + (3 * i));
			//printf("Bird Loop: %i\n\n", 12 + (3 * i));

			perspectiveMat[10 + (3*i)] = P * B;
			perspectiveMat[11 + (3*i)] = P * prim1;
			perspectiveMat[12 + (3*i)] = P * prim2;
		}

		for (int i = 0; i < no; i++)
		{
			B = W * obstacleList[i] * RtO * OtB;

			modelMat[13 + (3 * (nb - 1)) + i] = B;

			//printf("Obsticle list: %i\n\n", 13 + (3*(nb - 1)) + i );

			perspectiveMat[13 + (3 * (nb - 1)) + i] = P * B;

		}


		// Call our function that performs opengl operations
		// This is where your changes for matrix and vertex manipulation should be. 
		display(mainwindow, program, vm_location, pvm_location, light_pos, LightSourcePosition, WtE_loc, World_to_Eye, perspectiveMat, VAOarray, modelMat, lineOrigins);

		/* Swap our buffers to make our changes visible */
		glfwSwapBuffers(mainwindow);
		glfwPollEvents();


	}

	/* Delete our opengl context, destroy our window, and shutdown SDL */
	glfwDestroyWindow(mainwindow);

	glUseProgram(NULL);

	glDisableVertexAttribArray(pos_location);
	glDisableVertexAttribArray(color_location);

	glDeleteProgram(program);
	glDeleteVertexArrays(1, &VAO);

	return 0;
}