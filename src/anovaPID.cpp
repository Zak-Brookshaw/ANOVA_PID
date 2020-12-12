#include "anovaPID.h"
#include "Eigen/Dense"
#include <stdlib.h>


AnovaPID::AnovaPID(float Kp, float toaI, float dt) :
	Kp(Kp), toaI(toaI), dt(dt)
{
	piControl = true;
}


AnovaPID::AnovaPID(float Kp, float toaI, float dt, float toaD) :
	Kp(Kp), toaI(toaI), dt(dt), toaD(toaD)
{
	pidControl = true;
}



AnovaPID::AnovaPID(float Kp, float toaI, float dt, int freq,  float grad_Step) :
	Kp(Kp), toaI(toaI), dt(dt), freq(freq), grad_Step(grad_Step), piControl(true), anova(true)
{
	// resize for pi control
	indexList.resize(5);
	piSettings.resize(5, 2);
	// set piSettings
	piSettings.row(0) << 0, 0;
	piSettings.row(1) << 1, 1;
	piSettings.row(2) << 1, -1;
	piSettings.row(3) << -1, 1;
	piSettings.row(4) << -1, -1;

}


AnovaPID::AnovaPID(float Kp, float toaI, float dt, float toaD, int freq, float grad_Step) :
	Kp(Kp), toaI(toaI), dt(dt), toaD(toaD), freq(freq), grad_Step(grad_Step), pidControl(true), anova(true)
{

	//resize for pid control
	indexList.resize(9);
	pidSettings.resize(9, 3);
	// set pidSettings
	pidSettings.row(0) << 0, 0, 0;
	pidSettings.row(1) << 1, 1, 1;
	pidSettings.row(2) << 1, -1, 1;
	pidSettings.row(3) << -1, 1, 1;
	pidSettings.row(4) << -1, -1, 1;
	pidSettings.row(5) << 1, 1, -1;
	pidSettings.row(6) << 1, -1, -1;
	pidSettings.row(7) << -1, 1, -1;
	pidSettings.row(8) << -1, -1, -1;
	
}


float AnovaPID::compute(float error) {

	// Naive / easy PID compute below

	if (piControl) {
		intAcc += Kp * error * dt / toaI;
		mAction = Kp * error + intAcc;
	}
	else {
		intAcc += Kp * error * dt / toaI;
		float derAction = Kp * toaD * (error - prevError) / dt;
		mAction = Kp * error + intAcc + derAction;
		prevError = error;
	}

	// determine if there's a windup count
	//windup_cnt = windup ? windup_cnt + 1 : windup_cnt;

	// detemrine if there's an anova count
	count = anova ? count + 1 : count; 
	if (count > freq && anova) {
		mvAnovaDOE();
	}

	return mAction;
}


void AnovaPID::mvAnovaDOE(void) {
	



	if (piControl) {
		

		cntDOE += 1;

	}




	
}


int AnovaPID::randNum(int length) {
	index = rand() % length;



	return index;
}



