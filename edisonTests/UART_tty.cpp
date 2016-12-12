#include <mraa.hpp>
//#include <mraa.h>
#include <iostream>
#include <unistd.h>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <ctime>

int main(int argc, char** argv)
{
	// create a GPIO object from MRAA using it
	mraa::Gpio* pinA = NULL;
	mraa::Gpio* pinB = NULL;
	mraa::Gpio* pinC = NULL;
	mraa::Gpio* pinD = NULL;
	mraa::Gpio* pinE = NULL;
	mraa::Gpio* pinF = NULL;
	mraa::Uart* dev1;
//	mraa::Uart* dev2;

	pinA = new mraa::Gpio(13, true, false);
	pinB = new mraa::Gpio(12, true, false);
	pinC = new mraa::Gpio(11, true, false);
	pinD = new mraa::Gpio(10, true, false);
	pinE = new mraa::Gpio(9, true, false);
	pinF = new mraa::Gpio(8, true, false);
//	dev1 = new mraa::Uart(0);
//	dev2 = new mraa::Uart(1);

	// set the pin direction
	pinA->dir(mraa::DIR_OUT);
	pinB->dir(mraa::DIR_OUT);
	pinC->dir(mraa::DIR_OUT);
	pinD->dir(mraa::DIR_OUT);
	pinE->dir(mraa::DIR_OUT);
	pinF->dir(mraa::DIR_OUT);
	dev1 = new mraa::Uart("/dev/ttyMFD2");
	dev1->setBaudRate(115200);
	dev1->setMode(8, mraa::UART_PARITY_NONE, 2);
	dev1->setFlowcontrol(false,false);

	// declare variables
//	int size = 1;
	char buffer[] = {'H', 'E', '\0'};
	std::string str = "Hello\n"; // std:: needed if namespace nto declared

	// loop forever toggling the on board LED every second
	while(1){
		sleep(1);
		pinA->write(1);
		dev1->writeStr(str);
		sleep(1);
		pinA->write(0);
		dev1->writeStr(buffer);
	}

	return mraa::SUCCESS;
}
