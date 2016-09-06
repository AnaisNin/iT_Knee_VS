#include "HumanBodyParam.h"
#include <stdio.h>
#include <cmath>
#include "mUtils.h"

//decl
const double HumanBodyParam::m_llr_m[16]={0.25290,0.24250,0.34655,0.5605,0.8535,0.5905,0.897625,0.4862,0.1395,0.4998,0.1618,0.5772,0.15445,0.4574,0.00495,0.49};//links length ratio man - a,b,c,d,e,f,h,i,j,k,n,o,p,q,r,s
const double HumanBodyParam::m_llr_w[16]={0.25280,0.21239,0.35435,0.5648,0.854933,0.6388,0.9097,0.5036,0.14046,0.5159,0.15856,0.5754,0.1523,0.4559,0.044957,0.7474};//links length ratio woman

//public
void HumanBodyParam::calibrate(const double & a_theta1, const double & a_theta2, const double & a_theta3, double & a_ankleTorque, const double & a_Fz) //rad, Nm, N (vertical force)
{
	this->compute_bodyMass(a_Fz);
	this->compute_linksMasses();
	this->compute_height(a_theta1,a_theta2,a_theta3,a_ankleTorque); 
	this->compute_linksLengths();
}

void HumanBodyParam::compute_linksMasses() //Cf statistical model
{
	if (this->m_gender == 0) //man
	{
		//Masses
		this->m_mTorso = 0.4346*this->m_bodyMass; //Just torso
		//this->m_mTorso = (0.4346+2*0.0494)*this->m_bodyMass; //mass torso+arms
		//this->m_mTorso = (0.65)*this->m_bodyMass; //mass torso+arms - 0.5334

		this->m_mThigh = 0.1416*this->m_bodyMass;
		this->m_mShank = 0.0433*this->m_bodyMass;
		this->m_mHead = 0.0694*this->m_bodyMass;
		this->m_mForearm= 0.0162*this->m_bodyMass;
		this->m_mUpperArm = 0.0271*this->m_bodyMass;
		this->m_mHand=0.0061*this->m_bodyMass;

		this->m_mITmotor = 1;
		this->m_mATIsensor = 0.5;

	}
	else //woman
	{
		//Masses
		this->m_mTorso = 0.4257*this->m_bodyMass;
		this->m_mThigh = 0.1478*this->m_bodyMass;
		this->m_mShank = 0.0481*this->m_bodyMass;
		this->m_mHead = 0.0449*this->m_bodyMass;
		this->m_mForearm = 0.0138*this->m_bodyMass;
		this->m_mUpperArm = 0.0255*this->m_bodyMass;
		this->m_mHand = 0.0056*this->m_bodyMass;

		this->m_mITmotor = 1;
		this->m_mATIsensor = 0.5;
	}

	//printf("Links masses (kg): Shank: %g \t Thigh: %g \t Torso: %g \n", this->m_mShank, this->m_mThigh, this->m_mTorso);
}

//FOR TESTING PURPOSES ONLY - to delete
//void HumanBodyParam::set_HBparam(const double & a_height, const double & a_bodyMass, const bool & a_gender)
//{
//	HumanBodyParam::set_userParam(a_height, a_bodyMass, a_gender);
//	HumanBodyParam::compute_linksLengths();
//}


HumanBodyParam::~HumanBodyParam()
{
	//
}

void HumanBodyParam::set_height(const double & a_height)
{
	this->m_height = a_height;
}


//private

void HumanBodyParam::compute_bodyMass(const double & a_Fz)
{
	//HumanBodyParam::m_bodyMass=std::abs(a_Fz)/g_GRAVITYacc;
	this->m_bodyMass = 2*std::abs(a_Fz) / g_GRAVITYacc;
}

//FOR TESTING PURPOSES ONLY - to delete
void HumanBodyParam::set_userParam(const double & a_height, const double & a_bodyMass, const bool & a_gender) //0: male, 1: female
{
	//Height
	if (a_height < 0)
	{
		printf("ERROR - length negative, set default value [HumanBodyParam.cpp]\n");
		//this->m_height = 1.80;
		this->m_height = a_height;
	}
	else
	{
		this->m_height = a_height;
	}

	//Mass
	if (a_bodyMass < 0)
	{
		printf("ERROR - body mass negative, set default value [HumanBodyParam.cpp]");
		this->m_bodyMass = 65;
	}
	else
	{
		this->m_bodyMass = a_bodyMass;
	}
	
	//Gender
	this->m_gender = a_gender;
}

void HumanBodyParam::compute_linksLengths()
{
	if (this->m_gender == 0) //man
	{
		//Lengths
		this->m_lHand = this->m_llr_m[14] * this->m_height;//r
		this->m_lForearm = this->m_llr_m[12] * this->m_height;//p
		this->m_lUpperArm = this->m_llr_m[10] * this->m_height;//n
		this->m_lHead = this->m_llr_m[8] * this->m_height;//j
		this->m_lTorso = this->m_llr_m[2] * this->m_height;//c
		this->m_lThigh = this->m_llr_m[1]  * this->m_height;//b
		this->m_lShank = this->m_llr_m[0] * this->m_height;//a

		this->m_lHand_com = this->m_llr_m[15] * this->m_lHand;//s
		this->m_lForearm_com = this->m_llr_m[13] * this->m_lForearm;//q
		this->m_lUpperArm_com = this->m_llr_m[11] * this->m_lUpperArm;//o
		this->m_lHead_com = this->m_llr_m[9] * this->m_lHead;//k
		this->m_lTorso_com = this->m_llr_m[7] * this->m_lTorso;//i
		this->m_lThigh_com = this->m_llr_m[5] * this->m_lThigh;//f
		this->m_lShank_com = this->m_llr_m[3] * this->m_lShank;//d

		this->m_lShank_ATIsensor = this->m_llr_m[4]* this->m_lShank;//e 
		this->m_liTmotor = this->m_llr_m[6]* this->m_lThigh;//h 	
	}

	else //woman
	{
		this->m_lHand = this->m_llr_w[14] * this->m_height;//r
		this->m_lForearm = this->m_llr_w[12] * this->m_height;//p
		this->m_lUpperArm = this->m_llr_w[10] * this->m_height;//n
		this->m_lHead = this->m_llr_w[8] * this->m_height;//j
		this->m_lTorso = this->m_llr_w[2] * this->m_height;//c
		this->m_lThigh = this->m_llr_w[1]  * this->m_height;//b
		this->m_lShank = this->m_llr_w[0] * this->m_height;//a

		this->m_lHand_com = this->m_llr_w[15] * this->m_lHand;//s
		this->m_lForearm_com = this->m_llr_w[13] * this->m_lForearm;//q
		this->m_lUpperArm_com = this->m_llr_w[11] * this->m_lUpperArm;//o
		this->m_lHead_com = this->m_llr_w[9] * this->m_lHead;//k
		this->m_lTorso_com = this->m_llr_w[7] * this->m_lTorso;//i
		this->m_lThigh_com = this->m_llr_w[5] * this->m_lThigh;//f
		this->m_lShank_com = this->m_llr_w[3] * this->m_lShank;//d

		this->m_lShank_ATIsensor = this->m_llr_w[4]* this->m_lShank;//e 
		this->m_liTmotor = this->m_llr_w[6]* this->m_lThigh;//h 
	}

	//printf("Height: %g \n", this->m_height);
	//printf("Links lengths: \n");
	//printf("Shank: %g \t Shank_com: %g \t ATI: %g \n Thigh: %g \t thigh_com: %g \t thigh_motor: %g \n Torso: %g \n", this->m_lShank, this->m_lShank_com, this->m_lShank_ATIsensor, this->m_lThigh, this->m_lThigh_com, this->m_liTmotor, this->m_lTorso);

}



void HumanBodyParam::compute_height(const double & a_theta1, const double & a_theta2, const double & a_theta3, double & a_ankleTorque)//TRUE SSI theta3 = 90deg(torso upright)
{

	double l_denom = 0;
	if (this->m_gender == 0)//man
	{
		l_denom= g_GRAVITYacc*this->m_llr_m[0]*cos(a_theta1)*(this->m_llr_m[3] *this->m_mShank + this->m_llr_m[4] *this->m_mATIsensor + this->m_mThigh + this->m_mITmotor + 0.5*this->m_mTorso + 0.5*this->m_mHead)
			+ g_GRAVITYacc*this->m_llr_m[1] *cos(a_theta2)*(this->m_llr_m[5] *this->m_mThigh + this->m_llr_m[6] *this->m_mITmotor + 0.5*this->m_mTorso + 0.5*this->m_mHead)
			+ g_GRAVITYacc*cos(a_theta3)*(this->m_llr_m[7]* this->m_llr_m[2]*0.5*this->m_mTorso + (this->m_llr_m[2] +this->m_llr_m[9] * this->m_llr_m[8]) *0.5* m_mHead);
		/*l_denom = g_GRAVITYacc*this->m_llr_m[0] * cos(a_theta1)*(this->m_llr_m[3] * this->m_mShank + this->m_llr_m[4] * this->m_mATIsensor + this->m_mThigh + this->m_mITmotor + this->m_mTorso + this->m_mHead)
			+ g_GRAVITYacc*this->m_llr_m[1] * cos(a_theta2)*(this->m_llr_m[5] * this->m_mThigh + this->m_llr_m[6] * this->m_mITmotor + 0.5*this->m_mTorso +this->m_mHead)
			+ g_GRAVITYacc*cos(a_theta3)*(this->m_llr_m[7] * this->m_llr_m[2] * this->m_mTorso + (this->m_llr_m[2] + this->m_llr_m[9] * this->m_llr_m[8]) * m_mHead);*/
	}
	else //woman
	{
		l_denom = g_GRAVITYacc*this->m_llr_w[0] * cos(a_theta1)* (this->m_llr_w[3] * this->m_mShank + this->m_llr_w[4] * this->m_mATIsensor + this->m_mThigh + this->m_mITmotor + 0.5*this->m_mTorso + 0.5*this->m_mHead)
			+ g_GRAVITYacc*this->m_llr_w[1] * cos(a_theta2)* (this->m_llr_w[5] * this->m_mThigh + this->m_llr_w[6] * this->m_mITmotor + 0.5*this->m_mTorso + 0.5*this->m_mHead)
			+ g_GRAVITYacc*cos(a_theta3)*(this->m_llr_w[7] * this->m_llr_w[2] * 0.5*this->m_mTorso + (this->m_llr_w[2] + this->m_llr_w[9] * this->m_llr_w[8]) * 0.5*m_mHead);
		/*l_denom = g_GRAVITYacc*this->m_llr_w[0] * cos(a_theta1)* (this->m_llr_w[3] * this->m_mShank + this->m_llr_w[4] * this->m_mATIsensor + this->m_mThigh + this->m_mITmotor + this->m_mTorso + this->m_mHead)
			+ g_GRAVITYacc*this->m_llr_w[1] * cos(a_theta2)* (this->m_llr_w[5] * this->m_mThigh + this->m_llr_w[6] * this->m_mITmotor + this->m_mTorso +this->m_mHead)
			+ g_GRAVITYacc*cos(a_theta3)*(this->m_llr_w[7] * this->m_llr_w[2] * this->m_mTorso + (this->m_llr_w[2] + this->m_llr_w[9] * this->m_llr_w[8]) * m_mHead);*/
	}

	double l_height = std::abs(a_ankleTorque) / (l_denom);
	if (l_height <= 0)
	{
		printf("ERROR - Height invalid, set to default [HumanBodyParam::compute_height()]\n");
		this->m_height = 1.80;//m
	}
	else
	{
		this->m_height = l_height;
	}

	//printf("[HumanBodyParam::compute_height()]\n");
	//printf("Theta1: %g rad, %g deg \n", a_theta1, g_RAD2DEG(a_theta1));
	//printf("Theta2: %g rad, %g deg \n", a_theta2, g_RAD2DEG(a_theta2));
	//printf("Theta3: %g rad, %g deg \n", a_theta3, g_RAD2DEG(a_theta3));
	////printf("Ankle Torque: %g Nm \n", a_ankleTorque);
	//printf("l_denom: %g \n", l_denom);
	//printf("l_height: %g \n", l_height);
	//printf("\n");
	//printf("Links masses: %g \t %g \t %g \n", this->m_mShank, this->m_mThigh, this->m_mTorso);
	//printf("%g \n", g_GRAVITYacc*this->m_llr_w[0] * cos(a_theta1)* (this->m_llr_w[3] * this->m_mShank + this->m_llr_w[4] * this->m_mATIsensor + this->m_mThigh + this->m_mITmotor + this->m_mTorso));
	//printf("%g \n", g_GRAVITYacc*this->m_llr_w[1] * cos(a_theta2)* (this->m_llr_w[5] * this->m_mThigh + this->m_llr_w[6] * this->m_mITmotor + this->m_mTorso));
	//printf("%g \n", g_GRAVITYacc*this->m_llr_w[2] * cos(a_theta3)*this->m_mTorso);
}
