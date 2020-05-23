#include "Parser.h"


double pi = 3.14159265358979323846;


int main()
{
	Server k;
	Parser p;
	

	std::mutex mut;
	k.setSensorMutex(&mut); 
	p.setServer(&k);

	std::thread t1(&Server::run, &k);
	std::thread t2(&Parser::run, &p);
	

	t1.join();
	t2.join();

	
	_getch();
	return 0;
}
