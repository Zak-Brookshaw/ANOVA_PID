#pragma once

#include "typedefs.h"
#include "Eigen/Dense"
#include <stdlib.h>
#include <iostream>
#include <vector>

class AnovaPID {

private:
	// parameters needed to find 
	float kp, taoi, taod;
	float er_sum = 0; 
	float er_prev = 0;
	float dt;
	// distances to move when creating DOE space and descending gradient
	float kp_mv, taoi_mv, taod_mv, grad_mv;
	
	// Rows of pid settings, raw values and encoded
	Eigen::MatrixXf pidDecoded = Eigen::MatrixXf::Zero(9, 3);
	Eigen::MatrixXi pidEncoded = Eigen::MatrixXi::Zero(9, 3);
	Eigen::Vector3f pidSettings = Eigen::Vector3f::Zero(3);
	// Error model values
	Eigen::VectorXf errorPid = Eigen::VectorXi::Zero(9);
	// stored errors
	errorLog erLog = errorLog::errorLog(9);
	// average error during DOE position
	float er_avg = 0;
	// iteration count
	int counter = 0;
	// number of computes before switching pid settings
	int resetCnt;  
	// count the number of times switched
	int switchCnt = 0;
	void reset();  // reset values when switching pid parameters
	// switch Pid parameters to pidRow
	void switchPid(int pidRow);  

	Eigen::VectorXi pidRows = Eigen::VectorXi::Zero(9);
	std::vector<int> rowsIndex = {0, 1, 2, 3, 4, 5, 6, 7, 8 };
	float _compute(float e);
	int randomInt(int high);
	float _encode(float raw, float avg, float range);
	float _decode(float coded, float avg, float range);

	// move pid to new region
	void movePid();
	Eigen::VectorXf gradient();

public:
	// Constructor
	AnovaPID(float kp, float taoi, float taod, float movePercent, float dt, int switchCnt);
	Eigen::Vector3f decodeSettings(Eigen::Vector3f encoded);
	Eigen::Vector3f encodeSettings(Eigen::Vector3f raw);
	float compute(float e);
	
};



