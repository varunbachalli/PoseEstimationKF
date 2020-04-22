#include "Parser.h"
Server::Server()
{

	/*
	Creates the Server Socket
	*/

	// Initialze winsock
	WSADATA wsData;
	WORD ver = MAKEWORD(2, 2);

	
	std::cout << "server started" << std::endl;
	int wsOk = WSAStartup(ver, &wsData);
	if (wsOk != 0)
	{
		std::cerr << "Can't Initialize winsock! Quitting" << std::endl;
	}

	// Create a socket
	listening = socket(AF_INET, SOCK_STREAM, 0);
	if (listening == INVALID_SOCKET)
	{
		std::cerr << "Can't create a socket! Quitting" << std::endl;
	}

	// Bind the ip address and port to a socket
	sockaddr_in hint;
	hint.sin_family = AF_INET;
	hint.sin_port = htons(53798);
	hint.sin_addr.S_un.S_addr = INADDR_ANY; // Could also use inet_pton .... 

	bind(listening, (sockaddr*)&hint, sizeof(hint));
	messageSize = 63;
	
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
	std::cout << "thread id of server run thread " << std::this_thread::get_id() << "\n";
	std::cout << "run called" << std::endl;
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
		// Wait for client to send 
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

		std::string Message = std::string(buf, 0, bytesReceived);
		std::cout << "message receieved is :\n";
		std::cout << Message << std::endl;
		std::cout << "number of chars in buffer is :\n";
		std::cout << sizeof(buf) / sizeof(char) << std::endl;
		std::cout << "number of chars in bytes is :\n";
		std::cout << sizeof(buf) << std::endl;
		std::cout << "bytes recieved is :" << bytesReceived << std::endl;
		std::cout << "message size is : " << messageSize << std::endl;
		std::this_thread::sleep_for(std::chrono::seconds(1));

		
		//parse.setValue(Message);
		
		AddReading(Message);
		cv.notify_one();

	}
	std::this_thread::sleep_for(std::chrono::seconds(1));
	setLoop(false);
	cv.notify_one();

	free(buf);

}

void Server::setLoop(bool b)
{
	std::lock_guard<std::mutex> locker(loop_mutex);
	loop_ = b;
}

bool Server::getLoop()
{
	std::lock_guard<std::mutex> locker(loop_mutex);
	return loop_;
}

std::condition_variable* Server::getCV()
{
	return &cv;
}

void Server::AddReading(std::string message)
{
	std::lock_guard<std::mutex> locker(*Readings_mutex);
	ServerText.push_back(message);
}

std::vector<std::string> Server::getSensorReadings()
{
	std::lock_guard<std::mutex> locker(*Readings_mutex);
	return ServerText;
}

void Server::clearSensorReadings()
{	
	std::lock_guard<std::mutex> locker(*Readings_mutex);
	ServerText.clear();
}

void Server::getSensorMutex(std::mutex* m)
{
	Readings_mutex = m;
}



//int main()
//{
//
//	/*Server k;
//	Parser p;
//	std::mutex mut;
//	p.setMutex(&mut);
//	k.getSensorMutex(&mut);
//	p.setServer(&k);
//	std::thread t1(&Server::run, &k);
//	std::thread t2(&Parser::run, &p);
//	t1.join();
//	t2.join();*/
//
//
//	Parser p;
//	p.KalmanFilter();
//
//
//
//	int a = 0;
//	std::cin >> a;
//	return 0;
//}


//int main()
//{
//	Eigen::MatrixXd m;
//
//
//
//	Parser p;
//	int k = 0;
//	std::cout << "exit" << std::endl;
//	std::cin >> k;
//	return 0;
//
//}
/*
		Calibration steps
		for magnetometer
		1. keep moving the meter and obtaining different data points for the estimation of the ellipsoid
		2. fit the ellipsoid (x/a)^2 + (y/b)^2 + (z/c)^2  = 1
		3. M = (U x sigma x V_*)
		4. find M^(-1)

		for acc mag and gyro
		1. fit 3d gaussian for N number of data points of all three sensors
		2. find the mean and covariance of these data points
		3. offset of gyro = mean of the data set
		4. offset of the acc
			a. let i be the position which has the highest mean value (~10m/s^2)
			b. offset at position i = mean[i]-9.8 m/s^2
			c. offset of rest  = mean[i]
		5. For Acc and Mag
			a. set initial measurement to be mean
		 */