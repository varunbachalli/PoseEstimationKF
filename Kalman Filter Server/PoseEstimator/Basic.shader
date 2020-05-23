#shader vertex
#version 330 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 cube_color;
uniform mat4 MVP;
out vec3 fragmentColor;
void main()
{
	gl_Position = MVP * vec4(position,1);
	fragmentColor = cube_color;
};

#shader fragment
#version 330 core
in vec3 fragmentColor;
out vec3 color;
void main()
{
	color = fragmentColor;
};