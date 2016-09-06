//#include "kinematics.h"
#include "math.h"
#define gk1 0.0603
#define gk2 0.00115
#define gk3 0.2013
#define gk4 0.22663
#define gk5 8.4e-004
#define MIN 1.0e-10
#define MAX 1.0e+10
float *IKRF_Body(float RFConfig[], float PrevRFConfig[], float JointAngles[]);
float (*MatrixInverse66(float InMat[][6]))[6];
int main(){
// MAIN FUNCTION
// FICTIOUS EXAMPLE:
int i;
float *Result;
float Calculate_Angles[6];
float PrevRFConfig[6];
float RFConfig[6];
RFConfig[0] = 0.0;
RFConfig[1] = 0.0; 
RFConfig[2] = 0.0;
RFConfig[3] = 0.0;
RFConfig[4] = 0.0;
RFConfig[5] = 0.0;
PrevRFConfig[0] = 0.0;
PrevRFConfig[1] = 0.0; 
PrevRFConfig[2] = 0.0;
PrevRFConfig[3] = 0.0;
PrevRFConfig[4] = 0.0;
PrevRFConfig[5] = 0.0;
Calculate_Angles[0] = 0.0;
Calculate_Angles[1] = 0.0; 
Calculate_Angles[2] = 0.0;
Calculate_Angles[3] = 0.0;
Calculate_Angles[4] = 0.0;
Calculate_Angles[5] = 0.0;
Result = IKRF_Body(RFConfig, PrevRFConfig, Calculate_Angles);
for(i=0; i<6; i++) Calculate_Angles[i] = Result[i];
}

float *IKRF_Body(float RFConfig[], float PrevRFConfig[], float JointAngles[]){
int i,j;
float s1, s2, s3, s4, s5, s6, s45;
float c1, c2, c3, c4, c5, c6, c45;
float fk1, fk2 , fk3 , fk4 , fk5 , fk6 ;
float fk7, fk8 , fk9 , fk10 , fk11 , fk12;
float fk13, fk14, fk15, fk16, fk17, fk18;
float fk19, fk20, fk21, fk22, fk23, fk24;
float fk25, fk26, fk27, fk28, fk29, fk30;
float fk31, fk32, fk33, fk34, fk35, fk36;
float fk37, fk38, fk39, fk40;
float hk1, hk2, hk3, hk4, hk5, hk6, hk7;
static float JointAnglesReturn[6], DRFConfig[6];
static  float (*Inv_Jacobc)[6];
static float Jacobc[6][6];
// Trigonometic Parameters
s1 = sin(JointAngles[0]);
s2 = sin(JointAngles[1]);
s3 = sin(JointAngles[2]);
s4 = sin(JointAngles[3]);
s5 = sin(JointAngles[4]);
s6 = sin(JointAngles[5]);
c1 = cos(JointAngles[0]);
c2 = cos(JointAngles[1]);
c3 = cos(JointAngles[2]);
c4 = cos(JointAngles[3]);
c5 = cos(JointAngles[4]);
c6 = cos(JointAngles[5]);
s45 = s4*c5 + c4*s5;
c45 = c4*c5 - s4*s5;

//Subexpressions Found by MATLAB
fk1 = c2*c3*c6;
fk2 = c2*s3*c4;
fk3 = fk2 + s2*s4;
fk4 = c2*s3*s4;
fk5 = fk4 - s2*c4;
fk6 = fk5*c5 + fk3*s5;
fk7 = -(s1*s2*c3 + c1*s3);
fk8 = s1*c2*s4;
fk9 = s1*s2*s3;
fk10 = c1*c3 - fk9;
fk11 = s5*(fk8 + fk10*c4);
fk12 = s1*c2*c4;
fk13 = c5*(fk10*s4 - fk12);
fk14 = s6*(fk13 + fk11) + c6*fk7;
fk15 = c1*c2*s4;
fk16 = c1*s2*s3;
fk17 = fk16 + s1*c3;
fk18 = c5*(fk17*c4 - fk15);
fk19 = c1*c2*c4;
fk20 = s5*(fk17*s4 + fk19);
fk21 = c1*s2*c3;
fk22 = fk21 - s1*s3;
fk23 = (fk17*c4 - fk15)*s5;
fk24 = (fk17*s4 + fk19)*c5;
fk25 = fk22*s6 - c6*(fk24 + fk23);
fk26 = fk22*c6 + s6*(fk24 + fk23);
fk27 = c5*(fk8 + fk10*c4);
fk28 = s5*(fk10*s4 - fk12);
fk29 = 1 + ((fk14*fk14)/((fk1 + s6*fk6)*(fk1 + s6*fk6)));
fk30 = c2*s4 - s2*s3*c4;
fk31 = s2*s3*s4;
fk32 = -s1*(c2*s3*c4 + s2*s4);
fk33 = s1*s2*c4;
fk34 = s1*c2*s3*s4;
fk35 = -(fk16 + s1*c3);
fk36 = fk22*s45;
fk37 = c1*(c2*s3*c4 + s2*s4);
fk38 = c1*s2*c4;
fk39 = c1*c2*s3*s4;

// Manually Found Subexpressions
fk40 = c1*c2*c3;
hk1 = 1/sqrt(1 - fk26*fk26);
hk2 = (fk18 - fk20);
hk3 = hk2*hk2;
hk4 = 1/(fk25*fk25 + hk3);
hk5 = -1/(fk6*s6 + fk1);
hk6 = hk5*hk5;
hk7 = 1/fk29;

// First Row is Confirmed
Jacobc[0][0] = gk1*(-c6*(c5*(fk35*s4 - fk19) + s5*(fk35*c4 + fk15)) + s6*(s1*s3 - fk21))  + gk2*(s5*(fk35*s4 - fk19) - c5*(fk35*c4 + fk15)) + gk3*(fk19 - fk35*s4) + gk4*c1*c2 + gk5*s1;
Jacobc[0][1] = -gk1*(c6*(c5*(fk33 - fk34) + s5*fk32) + s6*s1*c2*c3) + gk2*(s5*(fk33 - fk34) - c5*fk32) + gk3*(fk34 - fk33) - gk4*s1*s2;
Jacobc[0][2] = gk1*(s6*(fk9 - c1*c3) - c6*fk7*s45) - gk2*fk7*c45 - gk3*fk7*s4;
Jacobc[0][3] = -gk1*c6*(s5*(fk12 - fk10*s4) + fk27) + gk2*(fk11 - c5*(fk12 - fk10*s4)) - gk3*(fk10*c4 + fk8);
Jacobc[0][4] = gk1*c6*(fk28 - fk27) + gk2*(fk11 + fk13);
Jacobc[0][5] = gk1*(c6*fk7 + s6*(fk11 + fk13));

// Second Row is Confirmed
Jacobc[1][0] = 0.0;
Jacobc[1][1] = -gk1*(c6*(-c5*(c4*c2 + fk31) + s5*fk30) + s6*(s2*c3)) - gk2*(s5*(c2*c4 + fk31) + c5*fk30) + gk3*(fk31 + c2*c4) + gk4*c2;
Jacobc[1][2] = -gk1*(c6*c2*c3*s45 + s6*c2*s3) - gk2*c2*c3*c45 - gk3*c2*c3*s4;
Jacobc[1][3] = -gk1*c6*(fk3*c5 + s5*(s2*c4 - fk4)) + gk2*(fk3*s5 - c5*(s2*c4 - fk4))  -gk3*(fk2 + s2*s4);
Jacobc[1][4] = -gk1*c6*(fk3*c5 - fk5*s5) + gk2*(fk5*c5 + fk3*s5);
Jacobc[1][5] = gk1*(fk1 + fk6*s6);

// Third Row is Confirmed
Jacobc[2][0] = gk1*(s6*fk7 - c6*(fk11 + fk13)) + gk2*(fk28 - fk27) + gk3*(fk12 - fk10*s4) + gk4*s1*c2 - gk5*c1;
Jacobc[2][1] = gk1*(-c6*(c5*(fk39 - fk38) + s5*fk37) + s6*fk40)  + gk2*(s5*(fk39 - fk38) - c5*fk37) + gk3*(fk38 - fk39) + gk4*c1*s2;
Jacobc[2][2] = gk1*(fk35*s6 - fk36*c6) - gk2*fk22*c45 - gk3*fk22*s4;
Jacobc[2][3] = gk1*c6*(s5*(fk19 + fk17*s4) - fk18) + gk2*(fk23 + c5*(fk19 + fk17*s4))   + gk3*(fk15 - fk17*c4);
Jacobc[2][4] = gk1*c6*(fk20 - fk18) + gk2*(fk24 + fk23);
Jacobc[2][5] = gk1*(s6*(fk24 + fk23) + c6*fk22);

// Fourth Row is Confirmed
Jacobc[3][0] = -fk14*hk1;
Jacobc[3][1] = -(s6*(fk37*s5 + c5*(fk39 - fk38)) + c6*fk40)*hk1;
Jacobc[3][2] = -(fk36*s6 + fk35*c6)*hk1;
Jacobc[3][3] = s6*(s5*(fk19 + fk17*s4) - fk18)*hk1;
Jacobc[3][4] = -(hk2*s6)*hk1;
Jacobc[3][5] = (s6*fk22 - c6*(fk23 + fk24))*hk1;

// Fifth Row is Confirmed
Jacobc[4][0] = (hk2*(s6*fk7 - c6*(fk11 + fk13)) + fk25*(fk28 - fk27))*hk4;
Jacobc[4][1] = (hk2*(-c6*(c5*(fk39 - fk38) + fk37*s5) + s6*fk40) - (fk25*(fk37*c5 + s5*(fk38 - fk39))))*hk4;
Jacobc[4][2] = (hk2*(s6*fk35 - c6*fk36) - fk25*(fk22*c45))*hk4;
Jacobc[4][3] = (hk2*c6*(s5*(fk19 + fk17*s4) - fk18) + fk25*(fk23 + c5*(fk19 + fk17*s4)))*hk4;
Jacobc[4][4] = (fk25*(fk24 + fk23) - c6*hk3)*hk4;
Jacobc[4][5] = hk2*fk26*hk4;

// Sixth Row is Confirmed
Jacobc[5][0] = (s6*(c5*(fk35*s4 - fk19) + s5*(fk15 + fk35*c4)) + c6*(s1*s3 - fk21))*hk5*hk7;
Jacobc[5][1] = ((((fk33 - fk34)*c5 + s5*fk32)*s6 - s1*c2*c3*c6)*hk5 - fk14*(((c2*c4 + fk31)*c5 - fk30*s5)*s6 + s2*c3*c6)*hk6)*hk7;
Jacobc[5][2] = hk7*(hk5*(s6*fk7*s45 + c6*(fk9 - c3*c1))  + fk14*(s6*c2*c3*s45 - c6*c2*s3)*hk6);
Jacobc[5][3] = s6*hk7*((fk27 + s5*(fk12 - fk10*s4))*hk5 + fk14*hk6*(fk3*c5 + s5*(s2*c4 - fk4)));
Jacobc[5][4] = s6*hk7*(hk5*(fk27 - fk28) + hk6*fk14*(fk3*c5 - fk5*s5));
Jacobc[5][5] = hk7*(hk5*(c6*(fk13 + fk11) - fk7*s6) - fk14*hk6*(s6*c2*c3 - fk6*c6));

Inv_Jacobc = MatrixInverse66(Jacobc);
for(i=0; i<6; i++) {
	DRFConfig[i] = RFConfig[i] - PrevRFConfig[i];
}
for(i=0; i<6; i++){
	for(j=0; j<6; j++){
		JointAnglesReturn[i] += Inv_Jacobc[i][j] * DRFConfig[j];
	}	
}
	
return JointAnglesReturn;
}

/*float (*MatrixAdd66(float InMat1[][6], float InMat2[][6], int Sign))[6]{
	int i, j;
	static float OutMat[6][6];
	for(i=0; i<6; i++){	
		for(j=0; j<6; j++){
			if(Sign > 0){
				OutMat[i][j] = InMat1[i][j] + InMat2[i][j];
			}
			else{
				OutMat[i][j] = InMat1[i][j] - InMat2[i][j];
			}
		}
	}
	return OutMat;
}

float (*MatrixMult66(float InMat1[][6], float InMat2[][6]))[6]{
	int i, j, k;
	static float OutMat[6][6];
	for(i=0; i<6; i++) for(j=0; j<6; j++) OutMat[i][j] = 0.0;
	for(i=0; i<6; i++){
		for(j=0; j<6; j++){
			for(k=0; k<6; k++){
				OutMat[i][j] += InMat1[i][k] * InMat2[k][j];
			}
		}
	}
	return OutMat;
}*/

float (*MatrixInverse66(float InMat[][6]))[6]{
  int i, j;// k, m;
  static float OutMat[6][6];
 static float L[6][6]; // Ask why they should also be static??
static  float U[6][6];
 static float I[6][6];
 static float Z[6][6];

  // First Column
  U[0][0] = InMat[0][0];
  if((U[0][0] <= MIN) && (U[0][0] > 0.0)) U[0][0] = MIN;
  if((U[0][0] >= -MIN) && (U[0][0] < 0.0)) U[0][0] = -MIN;

  for(i=0; i<6; i++){
    L[i][0] = InMat[i][0]/U[0][0];
    L[i][i] = 1.0;
  }

  // Second Column
  U[0][1] = InMat[0][1];
  U[1][1] = InMat[1][1]-L[1][0]*U[0][1];
  if((U[1][1] <= MIN) && (U[1][1] > 0.0)) U[1][1] = MIN;
  if((U[1][1] >= -MIN) && (U[1][1] < 0.0)) U[1][1] = -MIN;
  for(i=2; i<6; i++) L[i][1] = (InMat[i][1]-L[i][0]*U[0][1])/U[1][1];

  // Third Column
  U[0][2] = InMat[0][2];
  U[1][2] = InMat[1][2]-L[1][0]*U[0][2];
  U[2][2] = InMat[2][2]-L[2][0]*U[0][2]-L[2][1]*U[1][2];
  if((U[2][2] <= MIN) && (U[2][2] > 0.0)) U[2][2] = MIN;
  if((U[2][2] >= -MIN) && (U[2][2] < 0.0)) U[2][2] = -MIN;
  for(i=3; i<6; i++) L[i][2] = (InMat[i][2]-L[i][0]*U[0][2]-L[i][1]*U[1][2])/U[2][2];

  // Fourth Column
  U[0][3] = InMat[0][3];
  U[1][3] = InMat[1][3]-L[1][0]*U[0][3];
  U[2][3] = InMat[2][3]-L[2][0]*U[0][3]-L[2][1]*U[1][3];
  U[3][3] = InMat[3][3]-L[3][0]*U[0][3]-L[3][1]*U[1][3]-L[3][2]*U[2][3];
  if((U[3][3] <= MIN) && (U[3][3] > 0.0)) U[3][3] = MIN;
  if((U[3][3] >= -MIN) && (U[3][3] < 0.0)) U[3][3] = -MIN;
  L[4][3] = (InMat[4][3]-L[4][0]*U[0][3]-L[4][1]*U[1][3]-L[4][2]*U[2][3])/U[3][3];
  L[5][3] = (InMat[5][3]-L[5][0]*U[0][3]-L[5][1]*U[1][3]-L[5][2]*U[2][3])/U[3][3];

  // Fifth Column
  U[0][4] = InMat[0][4];
  U[1][4] = InMat[1][4]-L[1][0]*U[0][4];
  U[2][4] = InMat[2][4]-L[2][0]*U[0][4]-L[2][1]*U[1][4];
  U[3][4] = InMat[3][4]-L[3][0]*U[0][4]-L[3][1]*U[1][4]-L[3][2]*U[2][4];
  U[4][4] = InMat[4][4]-L[4][0]*U[0][4]-L[4][1]*U[1][4]-L[4][2]*U[2][4]-L[4][3]*U[3][4];
  if((U[4][4] <= MIN) && (U[4][4] > 0.0)) U[4][4] = MIN;
  if((U[4][4] >= -MIN) && (U[4][4] < 0.0)) U[4][4] = -MIN;
  L[5][4] = (InMat[5][4]-L[5][0]*U[0][4]-L[5][1]*U[1][4]-L[5][2]*U[2][4]-L[5][3]*U[3][4])/U[4][4];

  // Sixth Column
  U[0][5] = InMat[0][5];
  U[1][5] = InMat[1][5]-L[1][0]*U[0][5];
  U[2][5] = InMat[2][5]-L[2][0]*U[0][5]-L[2][1]*U[1][5];
  U[3][5] = InMat[3][5]-L[3][0]*U[0][5]-L[3][1]*U[1][5]-L[3][2]*U[2][5];
  U[4][5] = InMat[4][5]-L[4][0]*U[0][5]-L[4][1]*U[1][5]-L[4][2]*U[2][5]-L[4][3]*U[3][5];
  U[5][5] = InMat[5][5]-L[5][0]*U[0][5]-L[5][1]*U[1][5]-L[5][2]*U[2][5]-L[5][3]*U[3][5]-L[5][4]*U[4][5];
  if((U[5][5] <= MIN) && (U[5][5] > 0.0)) U[5][5] = MIN;
  if((U[5][5] >= -MIN) && (U[5][5] < 0.0)) U[5][5] = -MIN;


  for(i=0; i<6; i++){
	  for(j=0; j<6; j++){
		I[i][j] = 0.0;
        	I[i][i] = 1.0;
        	Z[i][j] = (I[i][j]-L[i][0]*Z[0][j]-L[i][1]*Z[1][j]-L[i][2]*Z[2][j]-L[i][3]*Z[3][j]-L[i][4]*Z[4][j]-L[i][5]*Z[5][j])/L[i][i];

	  }
 }

 for(i=5; i>=0; i--){
	  for(j=5; j>=0; j--){
          OutMat[i][j] = (Z[i][j]-U[i][0]*OutMat[0][j]-U[i][1]*OutMat[1][j]-U[i][2]*OutMat[2][j]-U[i][3]*OutMat[3][j]-U[i][4]*OutMat[4][j]-U[i][5]*OutMat[5][j])/U[i][i];
	  }
  }
	//for(i=0; i<6; i++)for(j=0; j<6; j++) OutMat[i][j] = InMat[i][j];
	
  return OutMat;
}
