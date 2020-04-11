#include <iostream>
#include <WS2tcpip.h>
#include <string>
#include <thread>
#include <regex>
#include <mutex>
#include <condition_variable>
#include "Structs_header.h"
#include <Eigen/Dense>
#include "Matrix_multiplication.cuh"
#pragma comment (lib, "ws2_32.lib")


class Server
{
public:
	Server();
	~Server();
	void run();
	void setLoop(bool b);
	bool getLoop();
	void run_2();
	
	std::condition_variable* getCV();
	void AddReading(std::string message);
	std::vector<std::string> getSensorReadings();
	void clearSensorReadings();

	void getSensorMutex(std::mutex* m);

private:
	bool loop_ = true;
	SOCKET clientSocket;
	SOCKET listening;
	sockaddr_in client;
	u_int messageSize = 0;
	std::condition_variable cv;
	std::vector<std::string> ServerText;
	std::mutex* Readings_mutex;
	std::mutex loop_mutex;
	
};


