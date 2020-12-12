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
	int reset;

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
	Eigen::MatrixX2f piSettings;  // upfront log of encoded pi settings	

	Eigen::MatrixX3f pidSettings;  // upfront log of encoded pid settings
	
	/// <summary>
	/// Provides a random index for pi/pid Settings
	/// </summary>
	/// <param name="length"></param>
	/// <returns></returns>
	int randNum(int length);
	/// <summary>
	/// moves the PI settings
	/// </summary>
	/// <param name=""></param>
	void setAnovaParameters(Eigen::Vector2f moveVector);

	/// <summary>
	/// moves the PID settings
	/// </summary>
	/// <param name=""></param>
	void setAnovaParameters(Eigen::Vector3f moveVector);

	/// <summary>
	/// Moves the PID settings to the next DOE location
	/// </summary>
	/// <param name=""></param>
	void mvAnovaDOE(void);

	/// <summary>
	/// Chooses the Anova settings
	/// </summary>
	/// <param name=""></param>
	void calculateAnovaParameters(void);

	/// <summary>
	/// Calculates new PI Anova Parameters, in wrt the encoded values.
	/// Ensures no duplicates
	/// </summary>
	/// <param name="pidSettings"></param>
	/// <returns></returns>
	Eigen::Vector2f chooseAnovaSettings(Eigen::MatrixX2f piSettings);

	/// <summary>
	/// Calculates new PID Anova Parameters, in wrt the encoded values.
	/// Ensures no duplicates
	/// </summary>
	/// <param name="pidSettings"></param>
	/// <returns></returns>
	Eigen::Vector3f chooseAnovaSettings(Eigen::MatrixX3f pidSettings);

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



