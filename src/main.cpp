#include "math.h""
#include <iostream>
#include <stdlib.h>
#include "anovaPID.h"

float heaviside(float time, float delay) {
	float on_off = time >= delay ? 1 : 0;

	return on_off;
}



float cAction(float tAction)
{
	float val;
	float tDelay = 10;
	float toa = 100;
	float A = .1;

	val = A*(1 -  exp(-(tAction - tDelay) / toa)) * heaviside(tAction, tDelay);


	return val;
}





int main() {

	// constant variables
	int const predH = 100;
	int const dt = 1;
	float error;
	float setpt = 10;
	


	float cActArray[predH];
	float cValue[predH] = { 0 } ;
	for (int i = 0; i < predH; i++) {
		
		cActArray[i] = cAction(dt * (float)i);


	}
	
	

	AnovaPID PID = AnovaPID::AnovaPID(.1, 70.0, (float)dt, 1000, .1);
	float mAction;
	int preRand;
	float random;
	int j = 0;
	while (true) {

		preRand = rand() % 100;
		random = .0 * (float)preRand;
		error = setpt - cValue[0];
		mAction = PID.compute(error);
		// update cValue
		for (int i = 0; i < predH; i++) {

			cValue[i] = i != predH - 1 ? cValue[i + 1] + cActArray[i] * mAction : cValue[i] + cActArray[i] * mAction ;
			if (i == 0) {
				cValue[i] = cValue[i] + random;
			}

		}
		std::cout << error << std::endl;
	}

	std::cin.get();
	return 0;
}