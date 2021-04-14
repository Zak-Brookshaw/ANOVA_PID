#pragma once
#include <iostream>
#include <vector>

class errorLog
{
public:
	int length;
	void reset()
	{
		for (int i = 0; i < length; i++)
		{
			error[i] = 0;
			index[i] = 0;
		}
	}
	errorLog(int length):
		length(length)
	{
		error.resize(length);
		index.resize(length);
		reset();
	}
	std::vector<float> error = { 0 };
	std::vector<int> index = { 0 };
};