#include "Server.h"


Server::Server()
{
	/*
	Creates the Server Socket
	*/

	// Initialze winsock
	WSADATA wsData;
	WORD ver = MAKEWORD(2, 2);

	int wsOk = WSAStartup(ver, &wsData);
	if (wsOk != 0)
	{
		std::cerr << "Can't Initialize winsock! Quitting" << std::endl;
		return;
	}

	// Create a socket
	listening = socket(AF_INET, SOCK_STREAM, 0);
	if (listening == INVALID_SOCKET)
	{
		std::cerr << "Can't create a socket! Quitting" << std::endl;
		return;
	}

	// Bind the ip address and port to a socket
	sockaddr_in hint;
	hint.sin_family = AF_INET;
	hint.sin_port = htons(53798);
	hint.sin_addr.S_un.S_addr = INADDR_ANY; // Could also use inet_pton .... 

	bind(listening, (sockaddr*)&hint, sizeof(hint));
	int string_max_int = sizeof(std::to_string(INT_MAX));
	int string_max_long = sizeof(std::to_string(LONG_MAX));
	int string_max_float = sizeof(std::to_string(FLT_MAX));
	messageSize = string_max_int + sizeof(':') + string_max_float + sizeof(',') + string_max_float + sizeof(',') + string_max_float + sizeof(',') + sizeof("t:") + string_max_long;

}

Server::~Server()
{
	/*
	Delete the server socket here
	*/

	closesocket(clientSocket);

	// Cleanup winsock
	WSACleanup();
}

void Server::run()
{
	listen(listening, SOMAXCONN);
	int clientSize = sizeof(client);
	clientSocket = accept(listening, (sockaddr*)&client, &clientSize);

	char host[NI_MAXHOST];		// Client's remote name
	char service[NI_MAXSERV];	// Service (i.e. port) the client is connect on

	ZeroMemory(host, NI_MAXHOST); // same as memset(host, 0, NI_MAXHOST);
	ZeroMemory(service, NI_MAXSERV);

	if (getnameinfo((sockaddr*)&client, sizeof(client), host, NI_MAXHOST, service, NI_MAXSERV, 0) == 0)
	{
		std::cout << host << " connected on port " << service << std::endl;
	}
	else
	{
		inet_ntop(AF_INET, &client.sin_addr, host, NI_MAXHOST);
		std::cout << host << " connected on port " <<
			ntohs(client.sin_port) << std::endl;
	}

	closesocket(listening);


	char* buf = (char*) calloc(messageSize,1);
	std::cout << buf << std::endl;
	while (true)
	{
		// Wait for client to send data
		int bytesReceived = recv(clientSocket, buf, messageSize, 0);
		if (bytesReceived == SOCKET_ERROR)
		{
			std::cerr << "Error in recv(). Quitting" << std::endl;
			break;
		}

		if (bytesReceived == 0)
		{
			std::cout << "Client disconnected " << std::endl;
			break;
		}

		std::cout << std::string(buf, 0, bytesReceived) << std::endl;
	}

	free(buf);
}


int main()
{
	int string_max_int = sizeof(std::to_string(INT_MAX));
	int string_max_long = sizeof(std::to_string(LONG_MAX));
	int string_max_float = sizeof(std::to_string(FLT_MAX));
	int messageSize = sizeof(int) + sizeof(':') + sizeof(float) + sizeof(',') + sizeof(float) + sizeof(',') + sizeof(float) + sizeof(',') + sizeof("t:") + sizeof(long);
	std::cout << "size of (type)" << messageSize << std::endl;
	messageSize = string_max_int + sizeof(':') + string_max_float + sizeof(',') + string_max_float + sizeof(',') + string_max_float + sizeof(',') + sizeof("t:") + string_max_long;
	std::cout << "size of max size" << messageSize << std::endl;
	std::cout << "buffer is " << std::endl;
	char* buf = (char*)calloc(messageSize, 1);
	std::cout << buf << std::endl;
}
