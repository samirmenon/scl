/*
 * SBfrKinematicAndInertialParams.hpp
 *
 *  Created on: Apr 28, 2013
 *      Author: samir
 */

#ifndef SBFRKINEMATICANDINERTIALPARAMS_HPP_
#define SBFRKINEMATICANDINERTIALPARAMS_HPP_

#ifdef GCC_4.4.3
 #define constexpr const
#endif

namespace bfr
{

  class SBfrKinematicAndInertialParamsV2
  {
  public:
    SBfrKinematicAndInertialParamsV2(){}
    virtual ~SBfrKinematicAndInertialParamsV2(){}

    /** The link properties */
    static constexpr double g = 9.81;//m/s2
    static constexpr double L1 = .978;
    static constexpr double L2 = .457;
    static constexpr double L3 = .457;
    static constexpr double L4 = .457; // = L3;
    static constexpr double L5 = .457; // = L2;
    static constexpr double L6 = .457/2.0; // = L2/2;
    static constexpr double L7 = .457/2.0; // = L4/2;
    static constexpr double L8 = (.978 + .457)/2.0; // = (L1 + L5)/2;
    static constexpr double LC = .037;
    static constexpr double m0 = 3.12;
    static constexpr double m1 = 0.697984;
    static constexpr double m2 = 0.17024;
    static constexpr double m3 = 0.17024;
    static constexpr double m4 = 0.17024;
    static constexpr double mC1 = .45;
    static constexpr double mC2 = .45; // = mC1;
    static constexpr double D1 = .08;
    static constexpr double D2 = .09;
    static constexpr double D3 = .017;
    static constexpr double D4 = .03;
    static constexpr double D5 = .045;



    //I0 = [(1/1000000),0,0;0,(14433/500000),0;0,0,(1/1000000)];
    static constexpr double Ixx0 = 1.0/1000000.0;
    static constexpr double Iyy0 = 14433.0/500000.0;
    static constexpr double Izz0 = 1.0/1000000.0;

    //I1 = [0.11758E-3,0,0;0,0.244499E0,0;0,0,0.244499E0];
    static constexpr double Ixx1 = 0.00011758;
    static constexpr double Iyy1 = 0.244499;
    static constexpr double Izz1 = 0.244499;

    //I2 = [0.286781E-4,0,0;0,0.356101E-2,0;0,0,0.356101E-2];
    static constexpr double Ixx2 = 0.0000286781;
    static constexpr double Iyy2 = 0.00356101;
    static constexpr double Izz2 = 0.00356101;

    //I3 = [0.356101E-2,0,0;0,0.286781E-4,0;0,0,0.356101E-2];
    static constexpr double Ixx3 = 0.00356101;
    static constexpr double Iyy3 = 0.0000286781;
    static constexpr double Izz3 = 0.00356101;

    //I4 = [0.356101E-2,0,0;0,0.286781E-4,0;0,0,0.356101E-2];
    static constexpr double Ixx4 = 0.00356101;
    static constexpr double Iyy4 = 0.0000286781;
    static constexpr double Izz4 = 0.00356101;

    //IC1 = [(89/100000),0,0;0,(223/100000),0;0,0,(303/100000)];
    static constexpr double IxxC1 = 89.0/100000.0;
    static constexpr double IyyC1 = 223.0/100000.0;
    static constexpr double IzzC1 = 303.0/100000.0;

    //IC2 = [(223/100000),0,0;0,(89/100000),0;0,0,(303/100000)];
    static constexpr double IxxC2 = 223.0/100000.0;
    static constexpr double IyyC2 = 89.0/100000.0;
    static constexpr double IzzC2 = 303.0/100000.0;

    static constexpr double Imotor0 = 0.0109025;
    static constexpr double Imotor1 = 0.00540935;
    static constexpr double Imotor2 = 0.00540935;
  };

} /* namespace bfr */
#endif /* SBFRKINEMATICANDINERTIALPARAMS_HPP_ */
