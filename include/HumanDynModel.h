#ifndef HUMAN_DYN_MODEL_H
#define HUMAN_DYN_MODEL_H
#include "HumanBodyParam.h"

class HumanDynModel
{
	public:
		void compute_jointTorques( HumanBodyParam * ap_HBparam, const float a_euler312[3],const double & a_ankleTorque,const double & Knee_RelAngle);//REMOVE?
		
		//If theta3 is measured
		void compute_torqueKnee(HumanBodyParam * ap_HBparam,const double & a_theta2,const double & a_theta3,const double & a_theta4,const double & a_theta5,const double & a_theta6,const double & a_theta7);
		//If theta3 needs to be estimated from torqueAnkle
		
		//No Load
		void compute_torqueKnee(HumanBodyParam * ap_HBparam, const double & a_theta1, const double & a_ankleTorque);//Note that theta1 is class member so not needed as arg, for debugging purposes
		//void compute_torqueKnee(HumanBodyParam * ap_HBparam, const double & a_ankleTorque);//User fnt - uses m_theta1
		void compute_torqueKnee(HumanBodyParam * ap_HBparam);//User fnt - uses m_theta1

		//With load
		void compute_torqueKnee_withLoad(HumanBodyParam * ap_HBparam, const double & a_massLoad);//Note that theta1 is class member so not needed as arg, for debugging purposes
		
		void set_theta1(const double & a_theta1);
		void set_theta3(const double & a_theta3);
		void set_theta2(const double & a_theta2);//note: theta2 is a fnt of theta1! void set_theta2(const double & a_alpha2) with internal access to m_theta1 could me more appropriate
		
		//void set_weight(const double & a_weight);//Remove?
		
		//Getters
		double get_torqueAnkle_filtered() { return m_tauAnkle_filtered; }
		double get_torqueAnkle(){return m_tauAnkle;}
		double get_torqueKnee(){return m_tauKnee;}
		double get_torqueHip(){return m_tauHip;}
		double get_theta1(){return m_theta1;}
		double get_theta2(){return m_theta2;}
		double get_theta3(){return m_theta3;}
		void set_ankleTorque(const double & a_ankleTorque,const double & a_ankleTorque_filtered);
		//double get_weight() { return m_weight; }
	
	private:
		void set_theta3(const float euler312[3]);

		//Angles, rad
		double m_theta1;
		double m_theta2;
		double m_theta3;
		//double m_weight;//vertical force
		
		//Joint torques, Nm
		double m_tauHip;
		double m_tauKnee;
		double m_tauAnkle;
		double m_tauAnkle_filtered;

};

#endif