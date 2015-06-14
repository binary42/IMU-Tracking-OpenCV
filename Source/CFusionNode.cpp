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
	CFusionNode *fusion = (CFusionNode*)fusionNode_in;
	
	while( !fusion->m_isDone )
	{
		// Get IMU, run filter, display track
	}
	
	// close all windows and additional threads
	
	return nullptr;
}

CFusionNode::CFusionNode() : m_isDone( false )
{
	m_pImuInterface = new CIMUInterface();
	
	m_matrices = {};
}

CFusionNode::~CFusionNode()
{
	delete( m_pImuInterface );
}

void CFusionNode::Run()
{
	//signal( SIGINT, HandleSignal );
	
	int ret = pthread_create( &m_tNode, nullptr, RunFusionThread, (void*)this );
	
	if( ret )
	{
		std::cout << "Error creating fusion thread. Exiting." << std::endl;
	}
	
	pthread_exit( nullptr );
}

void CFusionNode::HandleSignal( int signal )
{
	m_isDone = true;
}
