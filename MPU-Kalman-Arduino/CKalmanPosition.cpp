#include "CKalmanPosition.h"

CKalmanPosition::CKalmanPosition() //: m_dt( 0.02f ), m_xEstLast( 0.0f )
		//, m_zMeasured( 0.0f ), m_xTempEst( 0.0f )
		//, m_xEst( 0.0f ), m_zReal( 0.0), m_sumErrorKalman( 0.0f )
		//, m_sumErrorMeasure( 0.0f )
{

}

CKalmanPosition::~CKalmanPosition()
{
}

void CKalmanPosition::Init( float dtIn )
{
	m_dt = dtIn; // 0.030 sec

	// Measurement matrix (0,0,z,vx,vy,vz)
	for( int i = 0; i < 4; i++ )
	{
		for( int j = 0; j < 6; j++ )
		{
			m_H[i][j] = 0;
		}
	}

	m_H[0][2] = 1;
	m_H[1][3] = 1;
	m_H[2][4] = 1;
	m_H[3][5] = 1;

	// Process noise matrix (x,y,z,vx,vy,vz)
	for( int i = 0; i < 6; i++ )
	{
		for( int j = 0; j < 6; j++ )
		{
			m_Q[i][j] = 0.0;

			// zero temp arraus as well
			m_PTemp[i][j] = 0.0;
			m_P[i][j] = 0.0;
			m_PLast[i][j] = 0.0;
		}
	}

	m_Q[0][0] = 0.1;
	m_Q[1][1] = 0.1;
	m_Q[2][2] = 0.1;
	m_Q[3][3] = 0.4;
	m_Q[4][4] = 0.4;
	m_Q[5][5] = 0.4;

	// Measurement noise matrix (z,vx,vy,vz)
	for( int i = 0; i < 4; i++ )
	{
		for( int j = 0; j < 4; j++ )
		{
			m_R[i][j] = 0.0;
		}
	}

	m_R[0][0] = 0.1;
	m_R[1][1] = 0.1;
	m_R[2][2] = 0.5;
	m_R[3][3] = 0.5;

	// Transition matrix
	for( int i = 0; i < 6; i++ )
	{
		for( int j = 0; j < 6; j++ )
		{
			m_A[i][j] = 0;
		}
	}

	m_A[0][0] = 0.1;
	m_A[1][1] = 0.1;
	m_A[2][2] = 0.1;
	m_A[3][3] = 0.1;
	m_A[4][4] = 0.1;
	m_A[5][5] = 0.1;
	m_A[0][3] = m_dt;
	m_A[1][4] = m_dt;
	m_A[3][5] = m_dt;

	// Init the variable matrices to 0.0
	for( int i = 0; i < 6; i++ )
	{
		if( i < 4 )
		{
			m_zMeasured[i] = 0.0;
			m_zReal[i] = 0.0;
		}
		m_xEstLast[i] = 0.1;
		m_xTempEst[i] = 0.1;
		m_xEst[i] = 0.1;
                for( int j = 0; j < 6; j++ )
                {
                  m_PLast[i][j] = 0.0;
                }
	}

}

void CKalmanPosition::Prediction()
{
	float transpA[6][6];
	float tempM1[6][6];
	float tempM2[6][6];

	// m_xTempEst = m_A * m_xEstLast;
	Matrix.Multiply( (float*)m_A, (float*)m_xEstLast, 6, 6, 1, (float*)m_xTempEst );

	// Transpose m_A
	Matrix.Transpose( (float*)m_A, 6, 6, (float*)transpA );

	// m_A * m_PLast
	Matrix.Multiply( (float*)m_A, (float*)m_PLast, 6, 6, 6, (float*)tempM1 );

	// tempM1 * transpA
	Matrix.Multiply( (float*)tempM1, (float*)transpA, 6, 6, 6, (float*)tempM2 );

	// tempM2 + m_Q => m_PTemp = m_A * m_PLast * m_A(Transpose) + m_Q;
	Matrix.Add( (float*)tempM2, (float*)m_Q, 6, 6, (float*)m_PTemp );

      // debugging
//        for(int i=0;i<6;i++)
//        {
          // Serial.print(m_PLast[i][0]);
//          for(int j=0;j<3;j++)
//          {
//            Serial.print(m_PLast[i][j]);
//            Serial.print(" ");
//          }
//          Serial.println();
//        }

}

void CKalmanPosition::CalcKalmanGain()
{
	float transpH[6][4];
	float tempM1[6][6];
	float tempM2[4][6]; //m_H * m_PTemp
	float tempM3[4][4]; //tempM2 * m_H transpose
	float tempM4[4][4]; //tempM3 + m_R and also used as the inversion of (m_H * m_PTemp * m_H(transpose) + m_R)

	// transpose m_H
	Matrix.Transpose( (float*)m_H, 4, 6, (float*)transpH );

	// m_PTemp * m_H transpose
	Matrix.Multiply( (float*) m_PTemp, (float*)transpH, 6, 6, 4, (float*)tempM1 );

	// m_H * m_PTemp
	Matrix.Multiply( (float*)m_H, (float*)m_PTemp, 4, 6, 6, (float*)tempM2 );

	// tempM2 * m_H transpose
	Matrix.Multiply( (float*)tempM2, (float*)transpH, 4, 6, 4, (float*)tempM3 );

	// tempM3 + m_R
	Matrix.Add( (float*)tempM3, (float*)m_R, 4, 4, (float*)tempM4 );

	// (m_H * m_PTemp * m_H(transpose) + m_R)^-1
	Matrix.Invert( (float*)tempM4, 4 );

	// m_K = m_PTemp * m_H(Transpose) * (m_H * m_PTemp * m_H(transpose) + m_R) ^-1;
	Matrix.Multiply( (float*)tempM1, (float*)tempM4, 6, 6, 4, (float*)m_K );

        // debugging
//        for(int i=0;i<6;i++)
//        {
//          for(int j=0;j<4;j++)
//          {
//            Serial.print(m_K[i][j]);
//            Serial.print(" ");
//          }
//          Serial.println();
//        }
}

void CKalmanPosition::Measure( float altitudeIn, float vxIn, float vyIn, float vzIn, float rollIn, float pitchIn, float yawIn )
{
	// calc the rotations
	float RZ[3][3];
	float RY[3][3];
	float RX[3][3];
	float V[3];

	float tempM2[3][3];// RZ*RY
	float tempM3[3][3];// tempM1 * RX
	float tempM4[3];// TempM2 * V
        float tempM5[3];// tempM4*dt

	float tempM1[4][4]; // vx vy vz - zmeasured looks like z,vx,vy,vz

	// Rotation matrices
	RZ[0][0] = cos( yawIn );
	RZ[0][1] = -sin( yawIn );
	RZ[0][2] = 0.0;
	RZ[1][0] = sin( yawIn );
	RZ[1][1] = cos( yawIn );
	RZ[1][2] = 0.0;
	RZ[2][0] = 0.0;
	RZ[2][1] = 0.0;
	RZ[2][2] = 1.0;

	RY[0][0] = cos( pitchIn );
	RY[0][1] = 0.0;
	RY[0][2] = sin( pitchIn );
	RY[1][0] = 0.0;
	RY[1][1] = 1.0;
	RY[1][2] = 0.0;
	RY[2][0] = -sin( pitchIn );
	RY[2][1] = 0.0;
	RY[2][2] = cos( pitchIn );

	RX[0][0] = 1.0;
	RX[0][1] = 0.0;
	RX[0][2] = 0.0;
	RX[1][0] = 0.0;
	RX[1][1] = cos( rollIn );
	RX[1][2] = -sin( rollIn );
	RX[2][0] = 0.0;
	RX[2][1] = sin( rollIn );
	RX[2][2] = cos( rollIn );

	// Velocity
	V[0] = vxIn;
	V[1] = vyIn;
	V[2] = vzIn;

        // dt matrix for V*dt
        float dt[1];
        dt[0] = 0.02;
    
	// calc m_zReal?
	
        Matrix.Multiply( (float*)RZ, (float*)RY, 3, 3, 3, (float*)tempM2 );
	Matrix.Multiply( (float*)tempM2, (float*)RX, 3, 3, 3, (float*)tempM3 );
	Matrix.Multiply( (float*)tempM3, (float*)V, 3, 3, 1, (float*)tempM4 );
        Matrix.Multiply( (float*)tempM4, (float*)dt, 3, 1, 1, (float*)tempM5 );

        // from cvdrone sample dead reck kalman
        m_zMeasured[0] = altitudeIn;
        m_zMeasured[1] = tempM5[0];
        m_zMeasured[2] = tempM5[1];
        m_zMeasured[3] = tempM5[2];
        
        // debugging
//        for(int i=0;i<4;i++)
//        {
//          Serial.println(m_zMeasured[i]);
//        }


	// m_zMeasured = m_zReal + m_R;
//	m_zMeasured[0] = altitudeIn;
//	m_zReal[0] = 0.0;//no noise for altitude at this moment we just the altitude message
//	m_zReal[1] = vxIn;
//	m_zReal[2] = vyIn;
//	m_zReal[3] = vzIn;
//
//	Matrix.Add( (float*)m_zReal, (float*)m_R, 4, 4, (float*)tempM1 );
//
//	m_zMeasured[1] = tempM1[0][0];
//	m_zMeasured[2] = tempM1[1][0];
//	m_zMeasured[3] = tempM1[2][0];
}

void CKalmanPosition::Correct()
{
	float tempM1[4][6]; //m_H * m_xTempEst
	float tempM2[4]; //(m_zMeasured - m_H * m_xTempEst )
	float tempM3[6]; //m_K * (m_zMeasured - m_H * m_xTempEst )
	float tempM4[6][6]; //m_K * m_H;

	float tempM5[6][6]; //( 1 - m_K*m_H )

	float ident[6][6]; // 1

	// identity matrix setup


	for( int i = 0; i < 6; i++ )
	{
		for( int j = 0; j < 6; j++ )
		{
			ident[i][j] = 0.0;
		}
	}

	ident[0][0] = 1.0;
	ident[1][1] = 1.0;
	ident[2][2] = 1.0;
	ident[3][3] = 1.0;
	ident[4][4] = 1.0;
	ident[5][5] = 1.0;

	// m_H * m_xTempEst
	Matrix.Multiply( (float*)m_H, (float*)m_xTempEst, 4, 6, 1, (float*)tempM1 );

	// (m_zMeasured - m_H * m_xTempEst )
	Matrix.Subtract( (float*)m_zMeasured, (float*)tempM1, 4, 1, (float*)tempM2 );

	// m_K * (m_zMeasured - m_H * m_xTempEst )
	Matrix.Multiply( (float*)m_K, (float*)tempM2, 6, 4, 1, (float*)tempM3 );	

	// m_xEst = m_xTempEst + m_K * (m_zMeasured - m_H * m_xTempEst );
	Matrix.Add( (float*)m_xTempEst, (float*)tempM3, 6, 1, (float*)m_xEst );
  // debugging
//        for(int i=0;i<6;i++)
//        {
//           Serial.print(m_xEst[i]);
////          for(int j=0;j<3;j++)
////          {
////            Serial.print(m_xEst[i][j]);
////            Serial.print(" ");
////          }
//          Serial.println();
//        }
        //--------------------------------------
    
        // m_P
        // m_K * m_H;
	Matrix.Multiply( (float*)m_K, (float*)m_H, 6, 4, 6, (float*)tempM4 );

	// ( 1 - m_K * m_H )
	Matrix.Subtract( (float*)ident, (float*)tempM4, 6, 6, (float*) tempM5 );

	// m_P = ( 1 - m_K * m_H) * m_PTemp;
	Matrix.Multiply( (float*)tempM5, (float*)m_PTemp, 6, 6, 6, (float*)m_P );

    
}

void CKalmanPosition::Update()
{
	// Do a deep copy - arduino does not have stl library so long hand it
	for( int i = 0; i < 6; i++ )
	{
		for( int j = 0; j < 6; j++ )
		{
			m_PLast[i][j] = m_P[i][j];
		}
	}
	for( int i = 0; i < 6; i++ )
	{
		m_xEstLast[i] = m_xEst[i];
	}
}
