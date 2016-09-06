/************************************
* File Name          : -
* Author             : Lorenzo Saccares
* Version            : V1.0.0
* Date               : 09/26/2009
* Description        : This file provides all of the firmware functions specific
*                    : to the VN100.
********************************************************************************


/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "JointTorquefunction.h"


/* Exported constants --------------------------------------------------------*/
#define PI              3.14159265

JointTorque JointTorquefunction(HumanBodyParametr HB, float RollIMU, int Footsensor_AnkleTorque, double ATI_Torque ){

	double A, B, C, D, E, F, G, Theta1, Theta2, Theta3, Theta1s1, Theta1s2;
	A = 0;	B = 0;	C = 0; D = 0; E = 0; F = 0; G = 0;

	JointTorque JT;

	Theta3 = acos(RollIMU);
			
	A = Footsensor_AnkleTorque - HB.F_torso*HB.l_torso*cos(3);
	B = HB.l_shank_a*HB.F_shank + HB.l_ATIsensor*HB.F_ATIsensor + HB.l_shank*(HB.F_thigh + HB.F_iTmotor + HB.F_pelvis + HB.F_torso);
	C = HB.l_thigh_a*HB.F_thigh + HB.l_iTmotor*HB.F_iTmotor + HB.l_thigh*(HB.F_pelvis + HB.F_torso);
	D = C*cos(ATI_Torque);
	E = C*sin(ATI_Torque);

	if (ATI_Torque == PI / 2){			//DEFINIRE MEGLIOUN RANGE
		Theta1 = asin(A / (B + E));
	}
	else if (((B / C) == cos(ATI_Torque) - sin(ATI_Torque)) || ((B / C) == cos(ATI_Torque) - sin(ATI_Torque)))
	{
		printf("Case: 1-F^2 = 0; necessary an implementation");
		return;
	}
	else {
		F = (B + E) / D;
		G = A / D;
		Theta1s1 = asin((G + F*sqrt(pow(F, 2) + pow(G, 2) + 1)) / (1 - pow(F, 2)));
		Theta1s2 = asin((G - F*sqrt(pow(F, 2) + pow(G, 2) + 1)) / (1 - pow(F, 2)));

		int  count = 0, flagTh1_1 = 0, flagTh1_2 = 0;
		if ((Theta1s1 >= 0) && (Theta1s1 < 150))		// METTERE IN RAISNATI
		{
			count++;
			flagTh1_1 = 1;
		}
		if ((Theta1s2 >= 0) && (Theta1s2< 150))		// METTERE IN RAISNATI
		{
			count++;
			flagTh1_2 = 1;
		}
		if (count == 2)
		{
			printf("Th1.1 e th1.2 both in the range");
			return;
		}
		if (flagTh1_1 == 1) { Theta1 = Theta1s1; }
		else if (flagTh1_2 == 1) { Theta1 = Theta1s2; }

		Theta2 = Theta1 + ATI_Torque;

		JT.Torque_Hip = HB.l_torso*cos(3)*HB.F_torso;
		JT.Torque_Knee = HB.F_torso*(HB.l_torso*cos(3) + HB.l_thigh*cos(2)) + (HB.F_pelvis*HB.l_thigh + HB.F_iTmotor*HB.l_iTmotor + HB.F_thigh*HB.l_thigh_a)*cos(2);
		JT.Torque_Ankle = HB.F_torso*(HB.l_torso*cos(3) + HB.l_thigh*cos(2) + HB.l_shank*cos(1)) + HB.F_pelvis*(HB.l_thigh*cos(2) + HB.l_shank*cos(1)) + HB.F_iTmotor*(HB.l_iTmotor*cos(2) + HB.l_shank*cos(1)) + HB.F_thigh*(HB.l_thigh_a*cos(2) + HB.l_shank*cos(1)) + cos(1)*(HB.F_ATIsensor*HB.l_ATIsensor + HB.F_shank*HB.l_shank_a);

		if ((abs(JT.Torque_Ankle) - abs(Footsensor_AnkleTorque)) > 5)
		{
			printf("Ankle torque evaluation is too different from the measured");
			return;
		}
	}
	return JT;
}


//	JointTorque.Toruqe_Ankle = HB.F_torso*(HB.l_torso*cos(3) + HB.l_thigh*cos(2) + HB.l_shank*cos(1));
//}