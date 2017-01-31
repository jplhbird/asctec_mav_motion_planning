/*

Copyright (c) 2017, Yu Yushu @ NTU

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef MATH_FUNCTION_H_
#define MATH_FUNCTION_H_

#include "minimum_snap_traj.h"
#include <math.h>


namespace math_function{
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

inline void quaternion_to_R(double *q, double *r){

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

inline void RtoEulerangle(double *r, double *angle){
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



inline void computeR(double *gamma, double *R)  {
	//compute rotation matrix from Euler angles
	double phi, theta, psi;

//	R=[cos(theta)*cos(psi),sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);...
//  cos(theta)*sin(psi),sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);...
//  -sin(theta),sin(phi)*cos(theta),cos(phi)*cos(theta)];

	phi=*gamma;
	theta=*(gamma+1);
	psi=*(gamma+2);

	*R=cos(theta)*cos(psi);
	*(R+1) = sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);
	*(R+2)= cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);

	*(R+3)= cos(theta)*sin(psi);
	*(R+4)= sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi);
	*(R+5) = cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);

	*(R+6) = -sin(theta);
	*(R+7) = sin(phi)*cos(theta);
	*(R+8) = cos(phi)*cos(theta);
}



inline void computequaternion(double *r, double *q){
	//compute the quaternion q from rotation matrix R,
	int i;
	double R[3][3];
	double theta_xi;
	double omega_xi[3];
	double ec[3];
	double pi=3.14159265359;
	double pi_least=3.14159265359;

	R[0][0]=*r;
	R[0][1]=*(r+1);
	R[0][2]=*(r+2);
	R[1][0]=*(r+3);
	R[1][1]=*(r+4);
	R[1][2]=*(r+5);
	R[2][0]=*(r+6);
	R[2][1]=*(r+7);
	R[2][2]=*(r+8);

	//eigen-angle:
 	theta_xi = acos((R[0][0]+ R[1][1]+R[2][2]-1.0)/2.0);

	if ((theta_xi!=0))
	{//small conditions, simple calculate:
		//the eigen-axis of the rotation matrix:
		omega_xi[0]=1.0/(2.0*sin(theta_xi))*(R[2][1] - R[1][2]);
		omega_xi[1]=1.0/(2.0*sin(theta_xi))*(R[0][2] - R[2][0]);
		omega_xi[2]=1.0/(2.0*sin(theta_xi))*(R[1][0] - R[0][1]);

		//exponential coordinates:
		for(i=0;i<3;i++)
		{
			ec[i]=omega_xi[i]*theta_xi;
		}
	}
	else
	{
		omega_xi[0]=0;
		omega_xi[1]=0;
		omega_xi[2]=1;
	}

	//quaternion
	*q=cos(theta_xi/2.0);
	*(q+1)=sin(theta_xi/2.0)*omega_xi[0];
	*(q+2)=sin(theta_xi/2.0)*omega_xi[1];
	*(q+3)=sin(theta_xi/2.0)*omega_xi[2];
}

}



#endif /* MATH_FUNCTION_H_ */


