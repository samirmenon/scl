#include <stdio.h>
#include <math.h>

namespace bfr
{
const double g = 9.8;//m/s2
const double L1 = 1.2 - 0.45;
const double L2 = .45;
const double L3 = .45;
const double L4 = L3;
const double L5 = L2;
const double L6 = L2/2;
const double L7 = L4/2;
const double L8 = (L1 + L5)/2;
const double LC = .037;
const double m0 = 3.12;
const double m1 = 0.697984;
const double m2 = 0.17024;
const double m3 = 0.17024;
const double m4 = 0.17024;
const double mC1 = .45;
const double mC2 = mC1;
const double D1 = .081;
const double D2 = .096;
const double D3 = .017;
const double D4 = .03;
const double D5 = .045;



//I0 = [(1/1000000),0,0;0,(14433/500000),0;0,0,(1/1000000)];
const double Ixx0 = 1.0/1000000.0;
const double Iyy0 = 14433.0/500000.0;
const double Izz0 = 1.0/1000000.0;

//I1 = [0.11758E-3,0,0;0,0.244499E0,0;0,0,0.244499E0];
const double Ixx1 = 0.00011758;
const double Iyy1 = 0.244499;
const double Izz1 = 0.244499;

//I2 = [0.286781E-4,0,0;0,0.356101E-2,0;0,0,0.356101E-2];
const double Ixx2 = 0.0000286781;
const double Iyy2 = 0.00356101;
const double Izz2 = 0.00356101;

//I3 = [0.356101E-2,0,0;0,0.286781E-4,0;0,0,0.356101E-2];
const double Ixx3 = 0.00356101;
const double Iyy3 = 0.0000286781;
const double Izz3 = 0.00356101;

//I4 = [0.356101E-2,0,0;0,0.286781E-4,0;0,0,0.356101E-2];
const double Ixx4 = 0.00356101;
const double Iyy4 = 0.0000286781;
const double Izz4 = 0.00356101;

//IC1 = [(89/100000),0,0;0,(223/100000),0;0,0,(303/100000)];
const double IxxC1 = 89.0/100000.0;
const double IyyC1 = 223.0/100000.0;
const double IzzC1 = 303.0/100000.0;

//IC2 = [(223/100000),0,0;0,(89/100000),0;0,0,(303/100000)];
const double IxxC2 = 223.0/100000.0;
const double IyyC2 = 89.0/100000.0;
const double IzzC2 = 303.0/100000.0;

const double Imotor0 = 0.0109025;
const double Imotor1 = 0.00540935;
const double Imotor2 = 0.00540935;


void bfrGravity(double q0, double q1, double q2, double& g0, double& g1, double& g2)
{
	// Gravity Compensation
	g0 = -g*m1*(cos(q0)*(L4*sin(q2)-cos(q1)*(L1*(1.0/2.0)+L5*(1.0/2.0)))+sin(q0)*(D1+D2)-D3*cos(q0))+g*m2*(-sin(q0)*(D1+D2)+D3*cos(q0)+L6*cos(q0)*cos(q1))-g*m0*(D4*cos(q0)+D5*sin(q0))-g*m4*(sin(q0)*(D1+D2)-D3*cos(q0)+L7*cos(q0)*sin(q2))+g*m3*(-sin(q0)*(D1+D2)+D3*cos(q0)+cos(q0)*(L2*cos(q1)-L3*sin(q2)*(1.0/2.0)));
	g0 = -g0/30;
	g1 = -L2*g*m3*sin(q0)*sin(q1)-L6*g*m2*sin(q0)*sin(q1)-g*m1*sin(q0)*sin(q1)*(L1*(1.0/2.0)+L5*(1.0/2.0));
	g1 = g1/20;
	g2 = -L4*g*m1*cos(q2)*sin(q0)-L3*g*m3*cos(q2)*sin(q0)*(1.0/2.0)-L7*g*m4*cos(q2)*sin(q0);
	g2 = g2/20;
}
}







