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
	errorData.resize(5);
	errorData.setZero();
	piSettings.resize(5, 2);
	X_.resize(5, 4);
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
	errorData.resize(9);
	errorData.setZero();
	X_.resize(9, 8);
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
	if ((count > freq) && anova) {
		mvAnovaDOE();
	}

	return mAction;
}


void AnovaPID::mvAnovaDOE(void) {
	
	unsigned int index;
	cntDOE += 1;
	if (piControl) {

		if (cntDOE < 5) {
			// includes the center point of DOE
			index = randNum(5);
			// Set the new anova parameters
			setPiSettings(piSettings.row(index));
		}
		else {
			// Gradient Descent to find and set the new PI parameters
			piGradDescent();
		}
	}

	else {
		// PID Control
		if (cntDOE < 9) {
			index = randNum(9);
			setPidSettings(pidSettings.row(index));

		}
		else {
			// GRADIENT DESCENT
		}
	}

}

void AnovaPID::setPiSettings(const Eigen::Vector2i moveToVector) {

	Kp = decodeSetting(moveToVector(0, 0), Kp_center, Kp_Step);
	toaI = decodeSetting(moveToVector(0, 1), toaI_center, toaI_Step);
}

void AnovaPID::setPidSettings(const Eigen::Vector3i moveToVector) {

	Kp = decodeSetting(moveToVector(0, 0), Kp_center, Kp_Step);
	toaI = decodeSetting(moveToVector(0, 1), toaI_center, toaI_Step);
	toaD = decodeSetting(moveToVector(0, 2), toaD_center, toaD_Step);
}

void AnovaPID::piGradDescent() {
	
	// build the design matrix -- row by row
	unsigned int index;
	Eigen::Vector4f beta;
	Eigen::Matrix4f XTX_;
	Eigen::Matrix4f invXTX_;
	Eigen::Vector4f XTY_;
	Eigen::Vector2f gradVector;


	for (int i = 0; i < 5; ++i) {
		index = indexList(i);
		X_.row(i) << 1, (float)piSettings(index, 0), (float)piSettings(index, 0), (float)(piSettings(index, 0)*piSettings(index, 1));
	}

	XTX_ = X_.transpose() * X_;
	invXTX_ = XTX_.inverse();
	XTY_ = X_.transpose() * errorData;
	beta = invXTX_ * XTY_;

	gradVector << beta(1), beta(2);

	// make the PI parameter move
	setPiSettings(gradVector * grad_Step);


	resetParameters();
}


float AnovaPID::encodeSetting(float decoded, float center, float step) {
	float coded;
	coded = (decoded - center) / step;

	return coded;
}


float AnovaPID::decodeSetting(float coded, float center, float step) {
	float decoded;
	decoded = coded * step + center;

	return decoded;
}



unsigned int AnovaPID::randNum(int length) {
	unsigned int index = rand() % length;
	int test = 0;
	// determine if this index has been used
	for (int i=0; i < length; ++i) {
		test += 1 * (indexList[i] == index);
	}

	// cycle through until a unique one has been used
	if (test) {
		randNum(length);
	}
	else {
		return index;
	}
}

void AnovaPID::resetParameters() {

	// Reset all the vectors ... etc used to store data in each Anova iteration
	errorData.setZero();
	indexList.setZero();
	cntDOE = 0;
	prevError = 0;
	intAcc = 0;
	X_.setZero();
}

