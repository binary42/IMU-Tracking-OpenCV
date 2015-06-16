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
		fusion->m_imuData = fusion->m_pImuInterface->GetPoseInfo();
		
		// Get IMU, run filter, display track
		fusion->m_pFilter2->Prediction();
		fusion->m_pFilter2->CalcKalmanGain();
		fusion->SetVelocities( fusion->m_imuData.accel, deltaT );
		fusion->m_pFilter2->Measure( 1.0, fusion->m_matrices.m_V[0], fusion->m_matrices.m_V[1],
						fusion->m_matrices.m_V[2], fusion->m_imuData.fusionPose.x(),
						fusion->m_imuData.fusionPose.y(), fusion->m_imuData.fusionPose.z() );
		fusion->m_pFilter2->Correct();
		
		// debug output
		//std::cout << fusion->m_imuData.fusionPose.x() << " " << 
		//				fusion->m_imuData.fusionPose.y() << " " <<  fusion->m_imuData.fusionPose.z();
		
		fusion->Print();
		
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
	
	m_pImuInterface->Setup( 0.02, true, true, true );
	
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

void CFusionNode::SetVelocities( const RTVector3 &vect, int deltaT )
{
	m_matrices.m_V[0] = (float)vect.x() * deltaT;
	m_matrices.m_V[1] = (float)vect.y() * deltaT;
	m_matrices.m_V[2] = (float)vect.z() * deltaT;
}

void CFusionNode::Print() 
{
	if( m_pFilter1 != nullptr )
	{
	//	out << "OpenCV Filter: " << out::endl;
	//	out << "x: " << m_pFilter1->m_xEst[0] << " y: " << m_pFilter1->m_xEst[1] << " z: " 
	//		<< m_pFilter1->m_xEst[2] << out::endl;
	}
	if( m_pFilter2 != nullptr )
	{
		std::cout << "HomeGrown Filter: " << std::endl;
		std::cout << "x: " << m_pFilter2->m_xEst[0] << " y: " << m_pFilter2->m_xEst[1] << " z: " 
			<< m_pFilter2->m_xEst[2] << std::endl;
	}
}

std::ostream &operator<<( std::ostream &strm, const CFusionNode &node )
{
	//node.Print( strm );
	
	return strm;
}
