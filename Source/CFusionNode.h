#pragma once
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
#include <opencv2/opencv.hpp>
#include "CIMUInterface.h"
#include "../MPU-Kalman-Arduino/CKalmanPosition.h"

#include <pthread.h>
#include <signal.h>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <stdarg.h>

struct TMatrices
{
	float m_V[3];
	
	//Opencv matrices
	float deltaT = 0.030;
	
	// Transition Matrix (x, y, z, vx, vy, vz)
	cv::Mat m_A = ( cv::Mat1f( 6, 6 ) << 1.0, 0.0, 0.0, deltaT,    0.0,    0.0,
		   0.0, 1.0, 0.0, 0.0,    deltaT,    0.0,
		   0.0, 0.0, 1.0, 0.0,       0.0, deltaT,
		   0.0, 0.0, 0.0, 1.0,       0.0,    0.0,
		   0.0, 0.0, 0.0, 0.0,       1.0,    0.0,
		   0.0, 0.0, 0.0, 0.0,       0.0,    1.0 );
	// Measurement Matrix (0, 0, z, vx, vy, vz)
	cv::Mat m_H = ( cv::Mat1f( 4, 6) << 0, 0, 1, 0, 0, 0,
		   0, 0, 0, 1, 0, 0,
		   0, 0, 0, 0, 1, 0,
		   0, 0, 0, 0, 0, 1 );
	// Process noise covariance (x, ty, z, vx, vy, vz)
	cv::Mat m_Q = ( cv::Mat1f( 6, 6 ) << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
		   0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
		   0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
		   0.0, 0.0, 0.0, 0.3, 0.0, 0.0,
		   0.0, 0.0, 0.0, 0.0, 0.3, 0.0,
		   0.0, 0.0, 0.0, 0.0, 0.0, 0.3 );
	// Measurement Noise covariance (z, vx, vy, vz)
	cv::Mat m_R = ( cv::Mat1f( 4, 4 ) << 0.1, 0.0, 0.0, 0.0,
		   0.0, 0.1, 0.0, 0.0,
		   0.0, 0.0, 0.05, 0.0,
		   0.0, 0.0, 0.0, 0.05 );
	
};

class CFusionNode
{
public:
		CFusionNode();
		virtual ~CFusionNode();
		
		// Attributes
		CIMUInterface			*m_pImuInterface;
		bool					m_isDone;
		TMatrices				m_matrices;
		cv::KalmanFilter		*m_pFilter1;
		CKalmanPosition			*m_pFilter2;
		RTIMU_DATA				m_imuData;


		// Methods
		void Run();
		void HandleSignal( int signal );
		void SetVelocities( const RTVector3 &vect, int deltaT );
		void Print();
		

		
		friend std::ostream &operator<<( std::ostream &strm, const CFusionNode &node );

		

private:
		pthread_t				m_tNode;
	
		static void* RunFusionThread( void* fusionNodeIn );
};
