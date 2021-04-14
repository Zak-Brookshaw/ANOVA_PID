#include "anovaPID.h"
#include <cmath>

/// <summary>
/// Constructor
/// 
/// Defines several terms
/// </summary>
/// <param name="kp"></param>
/// <param name="taoi"></param>
/// <param name="taod"></param>
/// <param name="movePercent"></param>
/// <param name="dt"></param>
AnovaPID::AnovaPID(float kp, float taoi, float taod, float movePercent, float dt, int switchCnt) :
	kp(kp), taoi(taoi), taod(taod), kp_mv(movePercent* kp), taoi_mv(movePercent* taoi), taod_mv(movePercent* taod),
	grad_mv(1.41 * (pow(kp_mv, 2) + pow(taoi_mv, 2) + pow(taod_mv, 2))), dt(dt), switchCnt(switchCnt)
{
}

/// <summary>
/// primary function that user will interact with.
/// handles the decision tree and computes the control
/// action
/// </summary>
/// <param name="e"></param>
/// <returns></returns>
float AnovaPID::compute(float e)
{
	++counter;
	er_avg = er_avg * (counter - 1) / counter + e / counter;
	if (counter > resetCnt) {
		++switchCnt;
		if (switchCnt == 8) {
			erLog.error[switchCnt] = er_avg;
			movePid();
			switchCnt = 0;
			rowsIndex.resize(9);
			for (int i = 0; i < 9; i++)
			{
				rowsIndex[i] = i;
			}
			erLog.reset();
		}
		else
		{
			int row = randomInt(rowsIndex.size());
			switchPid(row);
		}
		reset();
	}
	float m_act = _compute(e);
	return m_act;
}

/// <summary>
/// Calculates the control action
/// </summary>
/// <param name="e"></param>
/// <returns></returns>
inline float AnovaPID::_compute(float e)
{
	// calculates the control action
	er_sum += e * dt;
	float er_der = (e - er_prev) / dt;
	float m_act = kp * e + (kp / taoi) * er_sum + kp * taod * er_der;
	return m_act;
}

/// <summary>
/// Method that moves the DOE to the new parameter region
/// </summary>
inline void AnovaPID::movePid() 
{
	Eigen::Vector3f grad = gradient();
	// Vector used to move from current pid parameter region
	Eigen::Vector3f moveVector = decodeSettings(grad * grad_mv*-1);
	pidDecoded.block(0, 0, 1, 3) += moveVector;
	kp = pidDecoded(0, 0);
	taoi = pidDecoded(0, 1);
	taod = pidDecoded(0, 2);
	for (int i = 0; i < 8; i++)
	{
		pidDecoded.block(i, 0, 1, 3) = decodeSettings(pidEncoded.block(i, 0, 1, 3));
	}
}

/// <summary>
/// Finds the parameter's gradient
/// </summary>
/// <returns></returns>
inline Eigen::VectorXf AnovaPID::gradient()
{
	Eigen::Vector3f eff = Eigen::Vector3f::Zero(3);
	Eigen::VectorXf _pidCol = Eigen::VectorXf::Zero(9);
	float er_avg, _enc;
	int ind;
	for (int i = 0; i < 3; i++)
	{
		_pidCol = pidEncoded.block(0, i, 9, 1);

		for (int j = 0; j < 9; j++)
		{
			er_avg = erLog.error[j];
			ind = erLog.index[j];
			_enc = (float)_pidCol[ind];
			eff[i] += _enc * er_avg;
		}
	}
	return eff/2;
}

/// <summary>
/// This method switches the kp, taoi and taod the ones on a randomly selected row
/// </summary>
/// <param name="row"></param>
inline void AnovaPID::switchPid(int row)
{
	Eigen::Vector3f _temp = pidEncoded.block(row, 0, 1, 3);
	kp = _temp(0);
	taoi = _temp(1);
	taod = _temp(2);
	erLog.error[switchCnt-1] = er_avg;
	erLog.index[switchCnt] = row;
	rowsIndex.erase(rowsIndex.begin() + row);
}

/// <summary>
/// This function resets all values to begin next
/// position in PID's DOE
/// </summary>
inline void AnovaPID::reset()
{
	er_sum = 0;
	er_prev = 0;
	errors.setZero();
	counter = 0;
}


/// <summary>
/// This method returns raw values of PID parameters, from encoded values
/// </summary>
/// <param name="encoded"></param>
/// <returns></returns>
inline Eigen::Vector3f AnovaPID::decodeSettings(Eigen::Vector3f encoded)
{
	float kpRange = kp_mv * 2;
	float taoiRange = taoi_mv * 2;
	float taodRange = taod_mv * 2;
	float kpAvg = pidDecoded(0, 0);
	float taoiAvg = pidDecoded(1, 0);
	float taodAvg = pidDecoded(2, 0);
	Eigen::Vector3f decoded; 

	decoded(0) = _decode(encoded(0), kpAvg, kpRange);
	decoded(1) = _decode(encoded(1), taoiAvg, taoiRange);
	decoded(2) = _decode(encoded(2), taodAvg, taodRange);
	return decoded;
}

/// <summary>
/// Method to decode the factorial input
/// </summary>
/// <param name="coded"></param>
/// <param name="avg"></param>
/// <param name="range"></param>
/// <returns></returns>
inline float AnovaPID::_decode(float coded, float avg, float range)
{
	float _temp = (float)coded;
	float raw = coded * (range / 2) + avg;
	return raw;
}

/// <summary>
/// Returns the encoded version of the raw vector PID parameter inputs
/// as a float, since finding new pid DOE region will be fractional (most likely)
/// </summary>
/// <param name="decoded"></param>
/// <returns></returns>
inline Eigen::Vector3f AnovaPID::encodeSettings(Eigen::Vector3f decoded)
{
	float kpRange = kp_mv * 2;
	float taoiRange = taoi_mv * 2;
	float taodRange = taod_mv * 2;
	float kpAvg = pidDecoded(0, 0);
	float taoiAvg = pidDecoded(1, 0);
	float taodAvg = pidDecoded(2, 0);


	Eigen::Vector3f encoded;
	encoded(0) = _encode(decoded(0), kpAvg, kpRange);
	encoded(1) = _encode(decoded(1), taoiAvg, taoiRange);
	encoded(2) = _encode(decoded(2), taodAvg, taodRange);
	return encoded;
}

/// <summary>
/// Encodes in factorial form
/// </summary>
/// <param name="raw"></param>
/// <param name="avg"></param>
/// <param name="range"></param>
/// <returns></returns>
inline float AnovaPID::_encode(float raw, float avg, float range)
{

	float encode = (int)((raw - avg) / (range / 2));
	return encode;
}

/// <summary>
/// Generates a random interger to choose from within pid row selection
/// </summary>
/// <param name="size"></param>
/// <returns></returns>
inline int AnovaPID::randomInt(int size)
{
	int value = rand() % size;

	return value;
}