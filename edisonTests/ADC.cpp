#include <mraa.hpp>
#include <iostream>
#include <unistd.h>

int main()
{
	// create a GPIO object from MRAA using it
	mraa::Gpio* pinA = NULL;
	mraa::Gpio* pinB = NULL;
	mraa::Gpio* pinC = NULL;
	mraa::Aio* pinD = NULL;
//	mraa::Gpio* pinE = NULL;
//	mraa::Gpio* pinF = NULL;

	pinA = new mraa::Gpio(13, true, false);
	pinB = new mraa::Gpio(12, true, false);
	pinC = new mraa::Gpio(11, true, false);
	pinD = new mraa::Aio(0);

	// set the pin direction
	pinA->dir(mraa::DIR_OUT);
	pinB->dir(mraa::DIR_OUT);
	pinC->dir(mraa::DIR_OUT);
//	pinD->dir(mraa::DIR_IN); //Analog pin is implicitly input.

	// declare variables
	float value = 0;
	float nValue = 0;

	// loop forever toggling the on board LED every second
	while(1){
		value = pinD->read();
		nValue = value/1024;


		fprintf(stdout, "Value: %.2f\n", value);
		fprintf(stdout, "Normalized value: %.2f\n", nValue);
		if(nValue>0.33){
			if(nValue>0.66){
				pinA->write(1);
				pinB->write(1);
				pinC->write(1);
			}
			else{
				pinA->write(1);
				pinB->write(1);
				pinC->write(0);
			}
		}
		else{
			pinA->write(1);
			pinB->write(0);
			pinC->write(0);
		}
	}

	return mraa::SUCCESS;
}
