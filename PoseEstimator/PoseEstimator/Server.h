#include <iostream>
#include <WS2tcpip.h>
#include <string>
#include <thread>
#pragma comment (lib, "ws2_32.lib")


class Server
{
public:
	Server();
	~Server();
	void run();

private:
	SOCKET clientSocket;
	SOCKET listening;
	sockaddr_in client;
	u_int messageSize = 0;
	
};

