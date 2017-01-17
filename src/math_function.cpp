
#include "minimum_snap_traj.h"



//
//function Rout=quaternion_to_R(u)
//%%compute quaternion from rotation matrix
//%u: 4-by-1 vector, the 4 elements of the quaternion
//%Rout: 9-by-1 vector, the 9 elements of the rotation matrix
//
//
//%input quaternion:
//a=u(1);
//b=u(2);
//c=u(3);
//d=u(4);
//
//
//%rotation matrix:
//R=[a^2+b^2-c^2-d^2, 2*(b*c-a*d), 2*(b*d+a*c);
//    2*(b*c+a*d), a^2-b^2+c^2-d^2, 2*(c*d-a*b);
//    2*(b*d-a*c), 2*(c*d+a*b), a^2-b^2-c^2+d^2];
//
//Rout=[R(1,:),R(2,:),R(3,:)]';

void Minimumsnap::quaternion_to_R(double *q, double *r){

	//input quaternion:
	double a,b,c,d;
	a= *q;
	b= *(q+1);
	c= *(q+2);
	d= *(q+3);


	//rotation matrix:
//	R=[a^2+b^2-c^2-d^2, 2*(b*c-a*d), 2*(b*d+a*c);
//	    2*(b*c+a*d), a^2-b^2+c^2-d^2, 2*(c*d-a*b);
//	    2*(b*d-a*c), 2*(c*d+a*b), a^2-b^2-c^2+d^2];

	//notice the order of r:

	*r=a*a+b*b-c*c-d*d;
	*(r+1)=2*(b*c-a*d);
	*(r+2)=2*(b*d+a*c);
	*(r+3)=2*(b*c+a*d);
	*(r+4)=a*a-b*b+c*c-d*d;
	*(r+5)=2*(c*d-a*b);
	*(r+6)=2*(b*d-a*c);
	*(r+7)=2*(c*d+a*b);
	*(r+8)=a*a-b*b-c*c+d*d;
}

void Minimumsnap::RtoEulerangle(double *r, double *angle){
//	function ea=Euler_angles(u)
//	%calculate the Euler angles from rotation matrix
//
//
//	%rotation matrix
//	R=[u(1), u(2), u(3);
//	   u(4), u(5), u(6);
//	   u(7), u(8), u(9)];

	double R[3][3];
	double ea_m[3];

	double theta;
	double phi;
	double psi;
	double pi=3.14159265359;

	R[0][0]=*r;
	R[0][1]=*(r+1);
	R[0][2]=*(r+2);
	R[1][0]=*(r+3);
	R[1][1]=*(r+4);
	R[1][2]=*(r+5);
	R[2][0]=*(r+6);
	R[2][1]=*(r+7);
	R[2][2]=*(r+8);


	ea_m[0]= atan(R[2][1]/R[2][2]);
	ea_m[1]= -asin(R[2][0]);
	ea_m[2]= atan(R[1][0]/R[0][0]);  //main Euler angles

	theta = ea_m[1];  //pitch angle equals to the main pitch angle

	if((R[2][2]==0) && (R[2][1]>=0))  //roll angle
	{
	    phi = pi/2;
	}
	else if((R[2][2]==0) && (R[2][1]<0))
	{
	    phi = -pi/2;
	}
	else if(R[2][2]>0)
	{
	    phi = ea_m[0];
	}
	else if ((R[2][2]<0) && (R[2][1]>=0))
	{
	    phi = ea_m[0]+pi;
	}
	else if ((R[2][2]<0) && (R[2][1]<0))
	{
	    phi = ea_m[0]-pi;
	}

	if((R[0][0]==0) && (R[1][0]>=0))  //yaw angle
	    psi = pi/2;
	else if((R[0][0]==0) && (R[1][0]<0))
	    psi = -pi/2;
	else if(R[0][0]>0)
	    psi = ea_m[2];
	else if ((R[0][0]<0) && (R[1][0]>=0))
	    psi = ea_m[2]+pi;
	else if ((R[0][0]<0) && (R[1][0]<0))
	    psi = ea_m[2]-pi;

	*angle = phi;
	*(angle+1) = theta;
	*(angle+2) = psi;
}





