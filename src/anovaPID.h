#pragma once
#include "Eigen/Dense"
#include <stdlib.h>

class AnovaPID {

private:

	// PID controller constants
	float Kp;
	float toaI;
	float toaD;
	float dt;

	//bool windup = false; // no windup reset considered unless asked for
	//int windup_cnt;

	float prevError = 0; // assumes initially no error -- only relevant for initial control action
	// discern which control scheme

	bool piControl;
	bool pidControl;
	// control action
	float mAction;
	float intAcc = 0; // integral accumulation
	// ANOVA parameters
	bool anova;
	int freq;
	float Kp_Step;
	float toaI_Step;
	float toaD_Step;
	float grad_Step;
	int count;
	int cntDOE = 0; // this indexes
	float Kp_center;
	float toaI_center;
	float toaD_center;
	Eigen::VectorXi indexList;
	Eigen::VectorXf errorData;  // length of vector will be known based on the options provided
	Eigen::MatrixX2i piSettings;  // upfront log of encoded pi settings	
	Eigen::MatrixX3i pidSettings;  // upfront log of encoded pid settings
	Eigen::MatrixXi X_; // this is the design matrix, changes size depending on PI or PID


	/// <summary>
	/// Provides a random index for pi/pid Settings
	/// </summary>
	/// <param name="length"></param>
	/// <returns></returns>
	unsigned int randNum(const int length);

	/// <summary>
	/// Takes in the new coded settings and converts the actual pi settings
	/// </summary>
	/// <param name=""></param>
	void setPiSettings(const Eigen::Vector2i moveToVector);

	/// <summary>
	/// Takes in the new coded settings and converts the actual pid settings
	/// </summary>
	/// <param name=""></param>
	void setPidSettings(const Eigen::Vector3i moveToVector);

	/// <summary>
	/// Moves the PID settings to the next DOE location
	/// </summary>
	/// <param name=""></param>
	void mvAnovaDOE(void);

	/// <summary>
	/// pi control gradient descent for find new center
	/// </summary>
	/// <param name=""></param>
	void piGradDescent(void);
	
	/// <summary>
	/// pid control gradient descent for find new center
	/// </summary>
	/// <param name=""></param>
	void pidGradDescent(void);


	/// <summary>
	/// This function encodes uncoded pi/pid Settings
	/// </summary>
	/// <param name="decoded"></param>
	/// <param name="center"></param>
	/// <param name="range"></param>
	/// <returns></returns>
	float encodeSetting(float decoded, float center, float step);


	/// <summary>
	/// This function decodeds coded pi/pid Settings
	/// </summary>
	/// <param name="coded"></param>
	/// <param name="center"></param>
	/// <param name="range"></param>
	/// <returns></returns>
	float decodeSetting(float coded, float center, float step);

	void resetParameters(void);


public:
	//
	//Constructors
	//

	/// <summary>
	/// PI controller Constructor
	/// </summary>
	/// <param name="Kp"></param>
	/// <param name="toaI"></param>
	/// <param name="dt"></param>
	AnovaPID(float Kp, float toaI, float dt);


	/// <summary>
	/// PID controller Constructor
	/// </summary>
	/// <param name="Kp"></param>
	/// <param name="toaI"></param>
	/// <param name="dt"></param>
	/// <param name="toaD"></param>
	AnovaPID(float Kp, float toaI, float dt, float toaD);
	
	// ANOVA parameter alterations


	/// <summary>
	/// PI Anova control Constructor
	/// </summary>
	/// <param name="Kp"></param>
	/// <param name="toaI"></param>
	/// <param name="dt"></param>
	/// <param name="freq"></param>
	/// <param name="piSettings"></param>
	/// <param name="grad_Step"></param>
	AnovaPID(float Kp, float toaI, float dt, int freq, float grad_Step);


	/// <summary>
	/// PID Anova control Constructor
	/// </summary>
	/// <param name="Kp"></param>
	/// <param name="toaI"></param>
	/// <param name="dt"></param>
	/// <param name="toaD"></param>
	/// <param name="freq"></param>
	/// <param name="pidSettings"></param>
	/// <param name="grad_Step"></param>
	AnovaPID(float Kp, float toaI, float dt, float toaD, int freq, float grad_Step);

	// windup considered

	//anovaPID(float Kp, int reset);

	//anovaPID(float Kp, float toaI, float dt, int reset);

	//anovaPID(float Kp, float toaI, float dt, float toaD, int reset);

	//// windup and ANOVA

	//anovaPID(float Kp, int freq, float step, int reset);

	//anovaPID(float Kp, float toaI, float dt, int freq, float step, int reset);

	//anovaPID(float Kp, float toaI, float dt, float toaD, int freq, float step, int reset);


	/// <summary>
	/// Computes the PID control action
	/// </summary>
	/// <param name="error"></param>
	/// <returns></returns>
	float compute(float error);


};



