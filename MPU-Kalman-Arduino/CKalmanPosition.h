
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
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "MatrixMath.h"

class CKalmanPosition
{
public:
  CKalmanPosition();
  ~CKalmanPosition();

  

  // Methods
  void Init( float dtIn );
  void Prediction();
  void CalcKalmanGain();
  void Measure( float altitudeIn, float vxIn, float vyIn, float vzIn, float rollIn, float pitchIn, float yawIn );
  void Correct();
  void Update();
  
private:
	// Attributes
	float 	m_A[6][6]; 		// Transition
	int 	m_H[4][6]; 		// Measurement
	float 	m_Q[6][6];		// Process noise
	float 	m_R[4][4];		// Measurement noise
	float 	m_P[6][6]; 		// Prediction - x,y,z,r,p,yaw
	float 	m_PTemp[6][6];
	float	m_PLast[6][6];
	float 	m_K[6][4];		// Kalman gains

	float 	m_dt;

	float 	m_xEstLast[6];
	float 	m_xTempEst[6];
	float 	m_xEst[6];
	float 	m_zMeasured[4];
	float		m_zReal[4];

	float 	m_sumErrorKalman;
	float 	m_sumErrorMeasure;

};
