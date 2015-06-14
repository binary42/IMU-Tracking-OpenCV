#pragma once

#include <opencv2/opencv.hpp>
#include "CIMUInterface.h"
/**
 * author:		J. Neilan
 * email:		jimbolysses@gmail.com
 * version:		v0.8
 * notes:
 * 	CIMU filtering using OpenCV kalman filter and homegrown Kalman filter for comparison.
 * 
 * License info: Reference to the RTIMULib: https://github.com/richards-tech/RTIMULib
 * 
 * The MIT License (MIT)

	Copyright (c) 2015 All of Us :)

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
 * 	
 * 
 */
#include <pthread.h>
#include <signal.h>

struct TKalmanMatrices
{
	
};

class CFusionNode
{
public:
		CFusionNode();
		virtual ~CFusionNode();
		
		// Attributes
		CIMUInterface			*m_pImuInterface;
		bool					m_isDone;
		TKalmanMatrices			m_matrices;
		
		// Methods
		void Run();
		void HandleSignal( int signal );
	
private:
		// Attributes
		pthread_t				m_tNode;
		cv::KalmanFilter		*m_pFilter;
		
		// Methods
		
	
};
