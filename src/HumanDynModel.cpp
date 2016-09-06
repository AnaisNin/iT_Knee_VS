#include "HumanDynModel.h"
#include "mUtils.h"
#include <cmath>
#include <stdio.h>
#include <windows.h>

void HumanDynModel::set_theta1(const double & a_theta1)
{
	this->m_theta1 = a_theta1;
}

void HumanDynModel::set_theta3(const double & a_theta3)
{
	this->m_theta3 = a_theta3;
}

//void HumanDynModel::compute_torqueKnee(HumanBodyParam * ap_HBparam, const double & a_ankleTorque)
void HumanDynModel::compute_torqueKnee(HumanBodyParam * ap_HBparam)
{
	this->compute_torqueKnee(ap_HBparam, this->m_theta1, this->m_tauAnkle_filtered);
}

void HumanDynModel::set_theta2(const double & a_theta2)
{
	this->m_theta2 = a_theta2;
}

void HumanDynModel::compute_torqueKnee(HumanBodyParam * ap_HBparam, const double & a_theta2, const double & a_theta3, const double & a_theta4, const double & a_theta5, const double & a_theta6, const double & a_theta7)
{
	//TO DO: divide by 2 mass head and torso!
	double l_torqueKnee =
		g_GRAVITYacc*ap_HBparam->m_mThigh*ap_HBparam->m_lThigh_com*cos(a_theta2) //Thigh contribution
		+ g_GRAVITYacc*ap_HBparam->m_mITmotor*ap_HBparam->m_liTmotor*cos(a_theta2) //exo motor contribution
		+ g_GRAVITYacc*ap_HBparam->m_mTorso*(ap_HBparam->m_lTorso_com*cos(a_theta3) + ap_HBparam->m_lThigh*cos(a_theta2)) //Torso contribution
		+ g_GRAVITYacc*ap_HBparam->m_mHead*(ap_HBparam->m_lThigh*cos(a_theta2) + ap_HBparam->m_lTorso*cos(a_theta3) + ap_HBparam->m_lHand_com*cos(a_theta4)) //head contribution
		+ g_GRAVITYacc* ap_HBparam->m_mUpperArm*(ap_HBparam->m_lThigh*cos(a_theta2) + ap_HBparam->m_lTorso*cos(a_theta3) + ap_HBparam->m_lUpperArm_com*cos(a_theta5)) //upper arm contribution
		+ g_GRAVITYacc*ap_HBparam->m_mForearm*(ap_HBparam->m_lThigh*cos(a_theta2) + ap_HBparam->m_lTorso*cos(a_theta3)+ ap_HBparam->m_lUpperArm*cos(a_theta5)+ ap_HBparam->m_lForearm_com*cos(a_theta6)) //forearm contribution
	    + g_GRAVITYacc*ap_HBparam->m_mHand*(ap_HBparam->m_lThigh*cos(a_theta2) + ap_HBparam->m_lTorso*cos(a_theta3) + ap_HBparam->m_lUpperArm*cos(a_theta5) + ap_HBparam->m_lForearm*cos(a_theta6)+ ap_HBparam->m_lHand_com*cos(a_theta7)); //hand contribution

	this->m_tauKnee = l_torqueKnee;
}


void HumanDynModel::set_ankleTorque(const double & a_ankleTorque, const double & a_ankleTorqueFiltered)
{
	this->m_tauAnkle = a_ankleTorque;
	this->m_tauAnkle_filtered = a_ankleTorqueFiltered;
}


void HumanDynModel::compute_torqueKnee(HumanBodyParam * ap_HBparam, const double & a_theta1, const double & a_ankleTorque)
{
	double l_torqueKnee =
		a_ankleTorque
		- g_GRAVITYacc*ap_HBparam->m_mShank*ap_HBparam->m_lShank_com*cos(a_theta1)
		- g_GRAVITYacc*ap_HBparam->m_mATIsensor*ap_HBparam->m_lShank_ATIsensor*cos(a_theta1)
		- ap_HBparam->m_lShank*cos(a_theta1)*g_GRAVITYacc*(ap_HBparam->m_mThigh+ ap_HBparam->m_mITmotor+ 0.5*ap_HBparam->m_mTorso+ 0.5*ap_HBparam->m_mHead+ ap_HBparam->m_mUpperArm+ ap_HBparam->m_mForearm+ ap_HBparam->m_mHand);

	//double l_A= g_GRAVITYacc*ap_HBparam->m_mShank*ap_HBparam->m_lShank_com*cos(a_theta1)
	//	+ g_GRAVITYacc*ap_HBparam->m_mATIsensor*ap_HBparam->m_lShank_ATIsensor*cos(a_theta1)
	//	+ ap_HBparam->m_lShank*cos(a_theta1)*g_GRAVITYacc*(ap_HBparam->m_mThigh + 0.5*ap_HBparam->m_mTorso + 0.5*ap_HBparam->m_mHead + ap_HBparam->m_mUpperArm + ap_HBparam->m_mForearm + ap_HBparam->m_mHand);
	////ADD EXO - ap_HBparam->m_mITmotor 
	//double l_torqueKnee = a_ankleTorque - l_A;

	this->m_tauKnee = l_torqueKnee;

	//printf("\n torque knee: %g \n",this->m_tauKnee);//Should be negative and 
	//printf("ankle torque: %g \n", a_ankleTorque);//Should be positive and increase when bending, keeping upperbody same posture
	////printf("TorqueKnee - TorqueAnkle: %g \n", l_A);
	//printf("theta1 [deg]: %g \n",g_RAD2DEG(a_theta1));


}



//Set m_theta1,m_theta2,m_theta3
void HumanDynModel::compute_jointTorques( HumanBodyParam * ap_HBP,const float a_euler312[3],const double & a_ankleTorque,const double & a_knee_RelAngle) //rad, Nm, rad
{
	this->set_theta3(a_euler312);

	//TO CHECK: C:  Why knee and ankle are not there? 
	printf("ankle torque measured: %g \n",a_ankleTorque);

	double A, B, C, D, E, F, G;
	//double l_theta1;//rad
	double l_theta1s1, l_theta1s2; //rad
	double l_alpha2 = M_PI - a_knee_RelAngle;//rad

	A = a_ankleTorque - ap_HBP->m_mTorso*g_GRAVITYacc*ap_HBP->m_lTorso*cos(this->get_theta3());
    B = g_GRAVITYacc*(ap_HBP->m_lShank_com*ap_HBP->m_mShank + ap_HBP->m_lShank_ATIsensor*ap_HBP->m_mATIsensor + ap_HBP->m_lShank*( ap_HBP->m_mThigh + ap_HBP->m_mITmotor + ap_HBP->m_mTorso));
	C=g_GRAVITYacc*(ap_HBP->m_lThigh_com*ap_HBP->m_mThigh + ap_HBP->m_liTmotor*ap_HBP->m_mITmotor + ap_HBP->m_lThigh*ap_HBP->m_mTorso); //PELVIS??
	D=C*cos(l_alpha2);
	E=C*sin(l_alpha2);
	G = A / E;
	F = (B + D) / E;
	//A = a_ankleTorque - ap_HBP->get_wTorso()*ap_HBP->get_lTorso()*cos(a_theta3);
	//A = Baias_AnkleTorque - HB.F_torso*HB.l_torso*cos(JT.Theta3);
	//B = ap_HBP->get_lShank_com()*ap_HBP->get_wShank() + ap_HBP->get_lShank_ATIsensor()*ap_HBP->get_wATIsensor() + ap_HBP->get_lShank()*( ap_HBP->get_wThigh() + ap_HBP->get_wITmotor() + ap_HBP->get_wTorso() );
	//B=HB.l_shank_a*HB.F_shank + HB.l_ATIsensor*HB.F_ATIsensor + HB.l_shank*(HB.F_thigh + HB.F_iTmotor + HB.F_pelvis + HB.F_torso);
	//C=ap_HBP->get_lThigh_com()*ap_HBP->get_wThigh() + ap_HBP->get_liTmotor() + ap_HBP->get_lThigh()*ap_HBP->get_wTorso(); //PELVIS??
	//C = HB.l_thigh_a*HB.F_thigh + HB.l_iTmotor*HB.F_iTmotor + HB.l_thigh*(HB.F_pelvis + HB.F_torso);

	//Case l_aspha2 equal to 0 (plus minus range)
	if ((l_alpha2 <= g_DEG2RAD(0.5)) && (l_alpha2 >= - g_DEG2RAD(0.5)))//Adjust range, to check
	{			
		this->m_theta1 = acos(A / (B + C));
	}
	else if ((G>sqrt(1 + pow(F, 2))) || (G<-sqrt(1 + pow(F, 2))))
	{
		printf("ERROR - Case: 1+F^2-G^2 < 0; TO IMPLEMENT [HumanDynModel::comute_jointTorques] \n");
		//Sleep(5000);
	}
	else 
	{
		l_theta1s1 = asin((-G + F*sqrt(pow(F, 2) - pow(G, 2) + 1)) / (1 + pow(F, 2)));//solution 1
		l_theta1s2 = asin((-G - F*sqrt(pow(F, 2) - pow(G, 2) + 1)) / (1 + pow(F, 2)));//solution 2

		int  count = 0, flagTh1_1 = 0, flagTh1_2 = 0;

		if ((l_theta1s1 >= g_DEG2RAD(0)) && (l_theta1s1 < g_DEG2RAD(150)))	//Check if physically feasible 	
		{
			//count++;
			flagTh1_1 = 1;
		}
		if ((l_theta1s2 >= g_DEG2RAD(0)) && (l_theta1s2< g_DEG2RAD(150)))	//Check if physically feasible 	
		{
			//count++;
			flagTh1_2 = 1;
		}
		//if (count == 2)//both feasible
		if (flagTh1_1==1 && flagTh1_2==1)//both feasible
		{
			printf("Th1.1 e Th1.2 are both physically feasible \n");
			printf("DECIDE which one to take [HumanDynModel] \n");
			Sleep(5000);
			this->m_theta1 = l_theta1s1;
		}
		else if (flagTh1_1 == 1)
		{
			this->m_theta1 = l_theta1s1;
		}
		else if (flagTh1_2 == 1)
		{
			this->m_theta1 = l_theta1s2;
		}
		else //none is valid
		{
			printf("ERROR in the evaluation of theta1 [HumanDynModel::compute_jointTorques] \n");
			Sleep(5000);
		}
	}

	this->m_theta2 = this->m_theta1 + M_PI - a_knee_RelAngle;

	this->m_tauHip = ap_HBP->m_lTorso*cos(this->get_theta3())*ap_HBP->m_mTorso*g_GRAVITYacc;
	this->m_tauKnee = ap_HBP->m_mTorso*g_GRAVITYacc*( ap_HBP->m_lTorso*cos(this->get_theta3())+ap_HBP->m_lThigh*cos(this->m_theta2) ) + (ap_HBP->m_liTmotor*ap_HBP->m_mITmotor+ap_HBP->m_mThigh*ap_HBP->m_lThigh_com)*g_GRAVITYacc*cos(this->m_theta2);
	this->m_tauAnkle = ap_HBP->m_mTorso*g_GRAVITYacc*( ap_HBP->m_lTorso*cos(this->get_theta3())+ap_HBP->m_lThigh*cos(this->m_theta2)+ap_HBP->m_lShank*cos(this->m_theta1) ) + ap_HBP->m_mITmotor*g_GRAVITYacc*(ap_HBP->m_liTmotor*cos(this->m_theta2)+ap_HBP->m_lShank*cos(this->m_theta1))+ap_HBP->m_mThigh*g_GRAVITYacc*(ap_HBP->m_lThigh_com*cos(this->m_theta2)+ap_HBP->m_lShank*cos(this->m_theta1))+cos(this->m_theta1)*g_GRAVITYacc*(ap_HBP->m_mATIsensor*ap_HBP->m_lShank_ATIsensor+ap_HBP->m_mShank*ap_HBP->m_lShank_com);

	if ((abs(this->m_tauAnkle) - abs(a_ankleTorque)) > 5)
	{
		printf("WARNING - Ankle torque evaluation is too different from the measured \n");
	}

	printf("theta1: %g rad, %g deg \n theta2: %g rad, %g deg \n theta3: %g rad, %g deg \n", this->m_theta1,g_RAD2DEG(this->m_theta1), this->m_theta2, g_RAD2DEG(this->m_theta2), this->m_theta3, g_RAD2DEG(this->m_theta3));
	printf("joint torques (Nm):\n Ankle estimated: %g \t measured %g \n Knee: %g \n Hip: %g \n", this->m_tauAnkle, a_ankleTorque, this->m_tauKnee, this->m_tauHip);
}

void HumanDynModel::set_theta3(const float euler312[3])
{
	//euler312[0]=Z
	//euler312[1]=X
	//euler312[2]=Y
	// euler312 gives +90 when upright, +180deg when bending forward 
	//Conversion to theta3 st +90 when upright, 0 when bending forward
	this->m_theta3 = -euler312[2] + M_PI;
	//printf("m_theta3: %g \n",m_theta3);
}

//void HumanDynModel::set_weight(const double & a_weight)
//{
//	if (a_weight >= 0)
//	{
//		this->m_weight = a_weight;
//	}
//	else
//	{
//		this->m_weight = 0;
//		//printf("ERROR - negative weight, set to zero [HumanDynModel::set_weight]\n");
//	}
//	
//}

void HumanDynModel::compute_torqueKnee_withLoad(HumanBodyParam * ap_HBparam,const double & a_massLoad)
{

	double l_torqueKnee =
		this->m_tauAnkle_filtered
		- g_GRAVITYacc*ap_HBparam->m_mShank*ap_HBparam->m_lShank_com*cos(this->m_theta1)
		- g_GRAVITYacc*ap_HBparam->m_mATIsensor*ap_HBparam->m_lShank_ATIsensor*cos(this->m_theta1)
		- ap_HBparam->m_lShank*cos(this->m_theta1)*g_GRAVITYacc*(ap_HBparam->m_mThigh + ap_HBparam->m_mITmotor + 0.5*ap_HBparam->m_mTorso + 0.5*ap_HBparam->m_mHead + ap_HBparam->m_mUpperArm + ap_HBparam->m_mForearm + ap_HBparam->m_mHand+ 0.5*a_massLoad);

	this->m_tauKnee = l_torqueKnee;
}