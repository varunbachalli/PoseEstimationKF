#include "OpenGLPlotter.h"




/*
Goal : Recieve Quarternion 
plot values in the quarternion.
*/

using namespace glm;




AttribData OpenGLPlotter::getVertexData(float length, float width, float height)
{
    length = length / 2;
    width = width / 2;
    height = height / 2;

    AttribData VertexData;
    VertexData.data = (float*)malloc(sizeof(float) * 72);
    VertexData.size = 72;

    float vertices[] = {
        -length, -width, -height,
        -length, -width, height,
        -length, width, height,
        -length, width, -height,

        -length, -width, -height,
        -length, -width, height,
        length, -width, height,
        length, -width, -height,

        -length, -width, -height,
        -length, width, -height,
        length, width, -height,
        length, -width, -height,

        length, width, height,
        length, width, -height,
        length, -width, -height,
        length, -width, height,

        length, width, height,
        length, width, -height,
        -length, width, -height,
        -length, width,  height,

        length, width, height,
        length, -width, height,
        -length, -width, height,
        -length, width, height
    };
        
    memcpy(VertexData.data, &vertices, sizeof(float)* VertexData.size);
    return VertexData;
}


AttribData OpenGLPlotter::getColorData()
{
    AttribData colorData;
    colorData.data = (float*)malloc(sizeof(float) * 72);
    colorData.size = 72;
    float color[] = {
        0.0,1.0,1.0,
        0.0,1.0,1.0,
        0.0,1.0,1.0,
        0.0,1.0,1.0,

        1.0,0.0,0.0,
        1.0,0.0,0.0,
        1.0,0.0,0.0,
        1.0,0.0,0.0,

        0.0,0.0,1.0,
        0.0,0.0,1.0,
        0.0,0.0,1.0,
        0.0,0.0,1.0,

        0.0,1.0,0.0,
        0.0,1.0,0.0,
        0.0,1.0,0.0,
        0.0,1.0,0.0,

        1.0,0.0,1.0,
        1.0,0.0,1.0,
        1.0,0.0,1.0,
        1.0,0.0,1.0,

        1.0,1.0,0.0,
        1.0,1.0,0.0,
        1.0,1.0,0.0,
        1.0,1.0,0.0
    };

    memcpy(colorData.data, &color, sizeof(float) * colorData.size);
    return colorData;
}

unsigned int OpenGLPlotter::CompileShader(const std::string& source, unsigned int type)
{
    unsigned int id = glCreateShader(type);
    const char* src = source.c_str();
    glShaderSource(id, 1, &src, nullptr);
    glCompileShader(id);

    int result;
    glGetShaderiv(id, GL_COMPILE_STATUS, &result);
    
    if (result == GL_FALSE)
    {
        int length;
        glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
        char* message = (char*)malloc(sizeof(char)*length);
        glGetShaderInfoLog(id, length, &length, message);
        std::cout << message << std::endl;

        std::cout << "Failed to compile " <<
            (type == GL_VERTEX_SHADER ? "vertex " : "fragment ") << "shader \n";

        free(message);
        glDeleteShader(id);
        return 0;
    }

    std::cout << "id is " << id << std::endl;
    std::cout << "compilation done" << std::endl;
    
    return id;
}


void OpenGLPlotter::setMVPmatrix(double quarternion[4])
{
    glm::quat myQuat = glm::quat(quarternion[0],quarternion[1],quarternion[2],quarternion[3]);
    glm::mat4 Model = glm::mat4_cast(myQuat);
    glm::mat4 Projection = glm::perspective(glm::radians(45.0f), 4.0f / 3.0f, 0.01f, 100.0f);
    // Camera matrix
    glm::mat4 View = glm::lookAt(
        glm::vec3(4, 3, 3), // Camera is at (4,3,-3), in World Space
        glm::vec3(0, 0, 0), // and looks at the origin
        glm::vec3(0, 0, 1)  // Head is up (set to 0,-1,0 to look upside-down)
    );

    std::lock_guard<std::mutex> locker(MVPmutex);
    MVP = Projection * View * Model;
    glfwPostEmptyEvent();
}

glm::mat4 OpenGLPlotter::getMVPmatrix()
{
    std::lock_guard<std::mutex> locker(MVPmutex);
    return MVP;
}


std::array<std::string,2> OpenGLPlotter::getShaders(const std::string path)
{
    std::array<std::string, 2> shaders;
    std::stringstream ss[2];
    std::ifstream stream(path);
    std::string line;

    enum class shaderType
    {
        NONE = -1, VERTEX = 0, FRAGMENT = 1
    };
    shaderType type = shaderType::NONE;
    while (getline(stream, line))
    {
        
        if (line.find("#shader") != std::string::npos)
        {
            if (line.find("vertex") != std::string::npos)
            {
                type = shaderType::VERTEX;
                std::cout << "vertex shader found\n";
            }
            else if (line.find("fragment") != std::string::npos)
            {
                type = shaderType::FRAGMENT;
                std::cout << "FRAGMENT shader found\n";
            }
        }

        else
        {
            if (type == shaderType::VERTEX)
            {
                ss[(int)shaderType::VERTEX] << line << "\n";
            }
            else if (type == shaderType::FRAGMENT)
            {
                ss[(int)shaderType::FRAGMENT] << line << "\n";
            }
        }
    }

    shaders[(int)shaderType::VERTEX] = ss[(int)shaderType::VERTEX].str();
    shaders[(int)shaderType::FRAGMENT] = ss[(int)shaderType::FRAGMENT].str();



    std::cout << "done parsing shaders\n";
    return shaders;
}

unsigned int OpenGLPlotter::CreateShaders()
{
    unsigned int prog = glCreateProgram();
    std::array<std::string, 2> shaders = getShaders("D:\\GITProjects\\Kalman Filtering Server\\PoseEstimationKF\\PoseEstimator\\PoseEstimator\\Basic.shader");
   
    unsigned int vs = CompileShader(shaders[0], GL_VERTEX_SHADER);
    unsigned int fs = CompileShader(shaders[1], GL_FRAGMENT_SHADER);


    glAttachShader(prog, vs);

    glAttachShader(prog, fs);

    glLinkProgram(prog);

    glValidateProgram(prog);

    glDeleteShader(vs);

    glDeleteShader(fs);

    return prog;
}

OpenGLPlotter::OpenGLPlotter()
{
   /* InitializePlotter();
    WindowCreator();*/
}

void OpenGLPlotter::Init()
{
    InitializePlotter();
    WindowCreator();
}

OpenGLPlotter::~OpenGLPlotter()
{
    glfwTerminate();
}

void OpenGLPlotter::InitializePlotter()
{
    if (!glfwInit())
        std::cout << "initialization failed" << std::endl;
}

// void setupBuffers

// void set program and get MVP position

// void 
void OpenGLPlotter::WindowCreator()
{
    std::cout << "entered here" << std::endl;
    window = glfwCreateWindow(1024, 768, "3d plotting", nullptr, nullptr);
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
    std::cout << "window created " << std::endl;
    if (!window)
    {
        glfwTerminate();
    }

    else
    {
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);
    }
    int k = glewInit();
    if (k != GLEW_OK)
    {
        std::cout << "Glew init not possible." << std::endl;
        
    }
    std::cout << glGetString(GL_VERSION) << std::endl;
    vertices = getVertexData(2.0, 2.0, 1.0);
    colors = getColorData();

    glGenBuffers(1, &colorbuffer);
  
    program = CreateShaders();
    glUseProgram(program);
    double quart[] = { 1.0,0.0,0.0,0.0 };
    
    setMVPmatrix(quart);

    MatrixID = glGetUniformLocation(program, "MVP");

    glGenBuffers(1, &vertexbuffer);


    glEnableVertexAttribArray(0);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 72, vertices.data, GL_DYNAMIC_DRAW);
    
    glEnableVertexAttribArray(1);

    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 72, colors.data, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 72, colors.data, GL_DYNAMIC_DRAW);

    //for (int i = 0; i < 72; ++i)
    //{
    //    std::cout << *(colors.data + i) << "\t";
    //}
    //std::cout << std::endl;

    //for (int i = 0; i < 72; ++i)
    //{
    //    std::cout << *(vertices.data + i) << "\t";
    //}
    //std::cout << std::endl;
    glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
}

void OpenGLPlotter::run()
{
    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    glDepthFunc(GL_LESS);
    glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

    double pi = 3.14159265358979323846;
    float angle = 0.0;
    double factor = 1.0 / 120;
    while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS && !glfwWindowShouldClose(window))
    {
        
        /* Render here */
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glVertexAttribPointer(
            0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
            3,                  // size
            GL_FLOAT,           // type
            GL_FALSE,           // normalized?
            0,                  // stride
            (void*)0            // array buffer offset
        );
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
        glVertexAttribPointer(
            1,                  // attribute. No particular reason for 0, but must match the layout in the shader.
            3,                  // size
            GL_FLOAT,           // type
            GL_FALSE,           // normalized?
            0,                  // stride
            (void*)0            // array buffer offset
        );

        
        
       /* angle = angle + factor * pi;
        double quart[4] = { cos(angle / 2), 0.0 ,0.0, sin(angle / 2) };
        setMVPmatrix(quart);*/
        glm::mat4 mvp_ = getMVPmatrix();
        /*if (angle >= 719.00)
            angle = 0.0;*/
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &mvp_[0][0]);


        glDrawArrays(GL_QUADS, 0, 6*4);
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);

        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwWaitEvents();
    }
    
    glDeleteProgram(program);
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteBuffers(1, &colorbuffer);
    free(colors.data);
    free(vertices.data);
}


//void OpenGLPlotter::updateBuffers()
//{
//
//}


//int main()
//{
//    OpenGLPlotter open;
//    open.run();
//    return 0;
//}