#ifndef HUMANBODYPARAM
#define HUMANBODYPARAM

class HumanBodyParam {
	friend class HumanDynModel;

public:
	
	void calibrate(const double & a_theta1, const double & a_theta2, const double & a_theta3, double & a_ankleTorque, const double & a_Fz);//rad, Nm, N (vertical force)
	HumanBodyParam():m_gender(0){} //Default constructor, man
	HumanBodyParam(bool a_gender):m_gender(a_gender){}
	~HumanBodyParam();
//CHECK AND SET AS PRIVATE
	void compute_height(const double & a_theta1, const double & a_theta2, const double & a_theta3, double & a_ankleTorque);//rad, Nm 
	void compute_bodyMass(const double & a_Fz); //N - vertical force ankle as read by foot FT plate
	void compute_linksMasses();//bodyMass should have been set previously
	void compute_linksLengths();
//FOR TESTING PURPOSES ONLY, TO DELETE
	//void set_HBparam(const double & a_height,const double & a_bodyMass,const bool & a_gender);//m,kg, 0=man, 1=woman
	void set_userParam(const double & a_height, const double & a_bodyMass, const bool & a_gender);
	void set_height(const double & a_height); //m
//Getters
	bool getGender(){return m_gender;}
	double get_height() { return this->m_height; }
	double get_mass(){return this->m_bodyMass; }
	////Lengths - m
	//double get_lShank(){return m_lShank;} 
	//double get_lShank_com(){return m_lShank_com;}
	//double get_lShank_ATIsensor(){return m_lShank_ATIsensor;}
	//double get_lThigh(){return m_lThigh;}
	//double get_lThigh_com(){return m_lThigh_com;}
	//double get_liTmotor(){return m_liTmotor;}
	//double get_lTorso(){return m_lTorso;}
	////Weights - N
	//double get_wTorso(){return m_mTorso*g_GRAVITYacc;}
	//double get_wThigh(){return m_mThigh*g_GRAVITYacc;}
	//double get_wShank(){return m_mShank*g_GRAVITYacc;}
	//double get_wITmotor(){return m_mITmotor*g_GRAVITYacc;}
	//double get_wATIsensor(){return m_mATIsensor*g_GRAVITYacc;}	

private:

	//User params
	bool m_gender;//0: male, 1: female
	double m_bodyMass;//kg
	double m_height; //m

	//Links lengths, m
	double m_lShank;//shank length, AJC-KJC
	double m_lShank_com; //AJC - com shank
	double m_lShank_ATIsensor;//AJC -exo ATI sensor
	double m_lThigh;//tight length, KJC-HJC
	double m_lThigh_com;//lKJC - thigh com
	double m_liTmotor;//KJC-iTmotor
	double m_lTorso;//MIDH-CERV
	double m_lTorso_com;//MIDH-torso com
	double m_lHead;//CERV-VERT
	double m_lHead_com;//CERV-head com
	double m_lUpperArm;
	double m_lUpperArm_com;
	double m_lForearm;
	double m_lForearm_com;
	double m_lHand;
	double m_lHand_com;
	//See Paolo de Leva, "Adjustments to Zatsiorsky-Seluyanov's segment inertia parameters", 1996, J. biomechanics, Vol 29. no9, 
	//AJC=ankle joint center, KJC=knee joint center, HJC=hips joint center, MIDH=mid hips center, CERV=cervicale

	static const double m_llr_m[16];//links length ratio man - a,b,c,d,e,f,h,i,j,k
	static const double m_llr_w[16];//links length ratio woman
	//a=m_llr[0]=l_shank/height
	//b=m_llr[1]=l_thigh/height
	//c=m_llr[2]= l_torso/height
	//d=m_llr[3]= l_shank_com/l_shank
	//e=m_llr[4]= l_ATI/l_shank // l_ATI=l_shank_com + 2/3(l_shank-l_shank_com) = 1/3.d.l_shank+2/3l_shank=l_shank( 1/3d + 2/3) => e=1/3d+2/3
	//f=m_llr[5]= l_thigh_com/l_thigh
	//h=m_llr[6]=l_iTmotor/l_thigh // l_motor=l_thigh_com+3/4(l_thigh-l_thigh_com)=1/4l_thigh_com+3/4l_thigh=(1/4f+3/4)l_thigh => h=1/4f+3/4

	//Links masses, kg
	double m_mTorso;
	double m_mThigh;
	double m_mShank;
	double m_mITmotor;
	double m_mATIsensor;
	double m_mHead;
	double m_mUpperArm;
	double m_mForearm;
	double m_mHand;
	//double m_mPelvis;
	
};


#endif
