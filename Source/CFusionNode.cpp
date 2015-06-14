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
#include "CFusionNode.h"

void* RunFusionThread( void *fusionNode_in )
{
	int deltaT = 0.03; //30ms
	
	CFusionNode *fusion = (CFusionNode*)fusionNode_in;
	
	while( !fusion->m_isDone )
	{
		// Get IMU, run filter, display track
		fusion->m_pFilter2->Prediction();
		fusion->m_pFilter2->CalcKalmanGain();
		fusion->GetVelocities( MPU.m_calAccel, deltaT );
		fusion->m_pFitler2->Measure( 1.0, fusion->m_matrices.m_V[0], fusion->m_matrices.m_V[1],
								fusion->m_matrices.m_V[2], fusion->m_MPU.fusedEulerPose[0],
								fusion->m_MPU.fusedEulerPose[1], fusion->m_MPU.fusedEulerPose[2] );
		
		
		usleep( ( 0.03 * 1000000 ) ); //30ms read delay for Kalman filter
	}
	
	// close all windows and additional threads
	
	return nullptr;
}

CFusionNode::CFusionNode() : m_isDone( false )
{
	m_pImuInterface = new CIMUInterface();

	m_pFilter1 = new cv::KalmanFilter();
	
	m_pFilter2 = new CKalmanPosition();
	
	m_matrices.m_V = {};
}

CFusionNode::~CFusionNode()
{
	delete( m_pImuInterface );
	delete( m_pFilter1 );
	delete( m_pFilter2 );
}

void CFusionNode::Run()
{
	//signal( SIGINT, CFusionNode::HandleSignal );
	
	int ret = pthread_create( &m_tNode, nullptr, RunFusionThread, (void*)this );
	
	if( ret )
	{
		std::cout << "Error creating fusion thread. Exiting." << std::endl;
	}
	
	pthread_exit( nullptr );
}

void CFusionNode::HandleSignal( int sig )
{
	m_isDone = true;
}

void CFusionNode::SetVelocities( short *vect, int deltaT )
{
	for( int i = 0; i < vect->size(); ++i )
	{
		m_matrices.m_V[i] = (float)vec[i] * deltaT;
	}
}

std::ostream &operator<<( std::ostream &strm, const CFusionNode &node )
{
	//strm << "Filter(s) return values:" << strm::endl;
	
	//if( node.filter1 != nullptr )
	//{
	//	strm << node.m_pFitler1->m_xEst[0] << " " << node.m_pFilter1->m_xEst[1] << strm::endl;
	//}
	//if( node.filter1 != nullptr )
	//{
	//	strm << node.m_pFitler2->m_xEst[0] << " " << node.m_pFilter2->m_xEst[1] << strm::endl;
	//}
	
	return strm;
}
