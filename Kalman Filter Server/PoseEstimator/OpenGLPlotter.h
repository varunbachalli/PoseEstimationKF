#pragma once
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <array>
#include <glm.hpp>
#include <gtc/matrix_transform.hpp>
#include <gtc/quaternion.hpp>
#include <thread>
#include <mutex>
struct AttribData
{
	float* data;
	int size;
};

class OpenGLPlotter
{
public:
	OpenGLPlotter();
	void Init();
	void run();
	~OpenGLPlotter();
	
	

	static unsigned int CompileShader(const std::string& source, unsigned int type);
	void setMVPmatrix(double quarternion[4]);
	
	static std::array<std::string, 2> getShaders(const std::string path);
	static unsigned int CreateShaders();

private:
	GLFWwindow* window;
	glm::mat4 getMVPmatrix();
	void InitializePlotter();
	AttribData getColorData();
	AttribData getVertexData(float length, float width, float height);
	void WindowCreator();
	
	GLuint vertexbuffer, colorbuffer;

	unsigned int program;
	AttribData vertices;
	AttribData colors;
	GLuint MatrixID;

	glm::mat4 MVP;
	std::mutex MVPmutex;
};

