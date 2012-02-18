#include "RobotPositionKalman.hpp"
RobotPositionKalman::RobotPositionKalman(OSMC_4wd_driver* driver, gps& gpsObject, IMU_Control& imuObject)
{
	magnetometer=new MagnetometerTracking(driver);
	(*magnetometer).update();
	encoder=new EncoderTracking(driver);
	IMU=&imuObject;
	gpsA=&gpsObject;
	(*gpsA).get_last_state(gpsFirstState);


	double X[]={0,0,(*magnetometer).getBearing(),0,0,0,0};
	x=cvCreateMat(7,1,CV_64FC1,X);//7x1 - state at step k 
	double Z[]={0,0,0,0,X[2],X[2]};
	z=cvCreateMat(6,1,CV_64FC1,Z);//6x1 - state measurements at step k
	double a[]={{1,0,0,CALL_PERIOD,0,CALL_PERIOD^2/2,0},
		    {0,1,0,0,CALL_PERIOD,0,CALL_PERIOD^2/2},
		    {0,0,1,0,0,0,0},
		    {0,0,1,0,0,0,0},
		    {0,0,1,0,0,0,0},
		    {0,0,1,0,0,0,0},
		    {0,0,1,0,0,0,0}};
	A=cvCreateMat(3,3,CV_64FC1,a);//7x7 - Relation of x_k-1 to x_k
	double U[]={3,3,0,0,0,0,0};
	B=cvCreateMat(3,2,CV_64FC1,U);//7x2 - Relation of u_k(control inputs at step K) to x_k
	double zToX[]={1,1,1,1/CALL_PERIOD,1/CALL_PERIOD,0,0,0,0};
	H=cvCreateMat(6,3,CV_64FC1);//6x3 - Relation of z_k to x_k
	K=cvCreateMat(3,6,CV_64FC1);//3x6 - gain or blending factor in x_k=x_k-1+K(z_k-x_k_apr)	
	P=cvCreateMat(3,3,CV_64FC1);//3x3 - Estimate error covariance at step k
	R=cvCreateMat(3,3,CV_64FC1);//3x3 - Process noise covariance
	Q=cvCreateMat(3,3,CV_64FC1);//3x3 - Measurement noise covariance
/**

	z[0]=0;//Encoder x measurement
	z[1]=0;//Encoder y measurement
	z[2]=0;//GPS x measurement
	z[3]=0;//GPS y measurement
	z[4]=x[2];//Magentometer bearing measurement
	z[5]=x[2];//Encoder bearing measurement

	//Placeholder values
	A[0]={1,0,0};A[1]={0,1,0};A[2]={0,0,1};

	B[0]={1,1};B[1]={1,1};B[2]={0,0};

	H[0]={1,0,0};H[1]={0,1,0};H[2]={1,0,0};H[3]={0,1,0};H[4]={0,0,1};H[5]={0,0,1};

	K[0]={1,1,0,0,0,0};K[1]={0,0,1,1,0,0};K[2]={0,0,0,0,1,1};

	P[0]={0,0,0};P[1]={0,0,0};P[2]={0,0,0};*/
	

	
}

void RobotPositionKalman::update(double u_k[2])
{/*
double sizesOne[][]={{3,3},{3,1}};
double sizesTwo[][]={{3,2},{2,1}};
double sizesThree[][]={{3,3},{3,3}};
//Time Update or Predict stage
//x_aPriori=Ax+Bu_k
double x_aPriori[]=x_aPriori=matrixSum(matrixProduct(A,x,sizesOne),matrixProduct(B,u_k,sizesTwo),3,1);

//P_aPriori=A*P_k-1*A^T+Q
double P_aPriori[][]=matrixSum(matrixProduct(matrixProduct(A,P,sizesThree),matrixTranspose(A,3,3),sizesThree),);
double AP[3][3];
for(int m=0;m<3;m++)
	for(int n=0;n<3;n++)
		AP[m][n]=A[m][0]*P[0][n]+A[m][1]*P[1][n]+A[m][2]*P[2][n];
for(int m=0;m<3;m++)
	for(int n=0;n<3;n++)	
		PaPriori[m][n]=AP[m][0]*A[n][0]+AP[m][1]*A[n][1]+AP[m][2]*A[n][2]+Q;

//Kalman_Gain=P_aPriori*H^T*(H*P_aPriori*H^T+R)^-1;

//Measurement Update or Correct stage
for(int i=0;i<3;i++)
	x[i]=x_aPriori[i]+K[i][0]*(z[0]-H[0][0]*xaPriori[0]-H[0][1]*xaPriori[1]-H[0][2]*xaPriori[2])+
			  K[i][1]*(z[1]-H[1][0]*xaPriori[0]-H[1][1]*xaPriori[1]-H[1][2]*xaPriori[2])+
			  K[i][2]*(z[2]-H[2][0]*xaPriori[0]-H[2][1]*xaPriori[1]-H[2][2]*xaPriori[2])+
			  K[i][3]*(z[3]-H[3][0]*xaPriori[0]-H[3][1]*xaPriori[1]-H[3][2]*xaPriori[2])+
			  K[i][4]*(z[4]-H[4][0]*xaPriori[0]-H[4][1]*xaPriori[1]-H[4][2]*xaPriori[2])+
			  K[i][5]*(z[5]-H[5][0]*xaPriori[0]-H[5][1]*xaPriori[1]-H[5][2]*xaPriori[2]);*/
}

/*
Gets the robot's x position in meters with respect to the starting position.
*/
double RobotPositionKalman::getX()
{
//return x[0];
}
/*
Gets the robot's x position in meters with respect to the starting position.
*/
double RobotPositionKalman::getY()
{
//return x[1];
}
/*
Gets the robot's bearing.
*/
double RobotPositionKalman::getBearing()
{
//return x[2];
}
/*
Gets the robot's angle with respect to the starting angle(starts as PI/2).
*/
double RobotPositionKalman::getAngle()
{
//return 450-x[2];
}



