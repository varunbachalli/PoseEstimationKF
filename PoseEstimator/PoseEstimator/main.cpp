#include "Parser.h"


double pi = 3.14159265358979323846;

void createAndRunOpenGL(OpenGLPlotter* plotter)
{
	plotter->Init();
	plotter->run();
}

void NormalizeValues(double& x, double& y, double& z)
{
	double denom = (y) * (y)+(z) * (z)+(x) * (x);
	denom = sqrt(denom);
	x = x / denom; y = y / denom; z = z / denom;
}

void addNoise_init(double& x, double& y, double& z, double Range)
{
	
	double v1 = (double)(rand() % 20000 - 10000)/10000.0;
	x += Range * v1;
	double v2 = (double)(rand() % 20000 - 10000)/10000.0;
	y += Range * v2;
	double v3 = (double)(rand() % 20000 - 10000)/10000.0;
	z += Range * v3;
}

int main()
{


	Server k;
	Parser p;
	

	std::mutex mut;
	p.setMutex(&mut);
	k.setSensorMutex(&mut); 
	p.setServer(&k);

	std::thread t1(&Server::run, &k);
	std::thread t2(&Parser::run, &p);
	

	t1.join();
	t2.join();
	// 
	// plotter.setMVPmatrix(quart);
	// std::thread t3(&createAndRunOpenGL, &plotter);

	/*std::this_thread::sleep_for(std::chrono::seconds(3));*/

	// for (int i = 0; i < 7200; ++i)
	// {
	// 	angle = i * pi / 180;
	// 	double quart[4] = { cos(angle / 2), 0.0 ,0.0, sin(angle / 2) };
	// 	std::this_thread::sleep_for(std::chrono::milliseconds(1));
	// 	plotter.setMVPmatrix(quart);
	// }
	// 
	// t3.join();


	_getch();
	return 0;
}
