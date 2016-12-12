#include <mraa.hpp>
#include <iostream>
#include <unistd.h>

int main()
{
	// create a GPIO object from MRAA using it
	mraa::Gpio* d_pin = NULL;
	mraa::Gpio* c_pin = NULL;

	c_pin = new mraa::Gpio(12, true, false);
	d_pin = new mraa::Gpio(13, true, false);

	// set the pin direction
	d_pin->dir(mraa::DIR_OUT);
	c_pin->dir(mraa::DIR_IN);

	// loop forever toggling the on board LED every second
	for (;;) {
		if (c_pin->read() == 1)
		{
			d_pin->write(1);
		}
		else
		{
			d_pin->write(0);
		}
	}

	return mraa::SUCCESS;
}
