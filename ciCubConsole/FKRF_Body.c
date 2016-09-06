//#include "kinematics.h"
#include "math.h"
#define gk1 0.0603
#define gk2 0.00115
#define gk3 0.2013
#define gk4 0.22663
#define gk5 8.4e-004
#define MIN 1.0e-10
#define MAX 1.0e+10
float *FKRF_Body(float Angles[]);
int main(){
// MAIN FUNCTION
// FICTIOUS EXAMPLE:
float *RFConfig;
float JointAngles[6];
JointAngles[0] = 0.0;
JointAngles[1] = 0.0;
JointAngles[2] = 0.0;;
JointAngles[3] = 0.0;
JointAngles[4] = 0.0;
JointAngles[5] = 0.0;

RFConfig = FKRF_Body(JointAngles);
}


float *FKRF_Body(float Angles[]){
  static float RF_Body[6];
float s1, s2, s3, s4, s5, s6, s45;
float c1, c2, c3, c4, c5, c6, c45;
float fk1, fk2 , fk3 , fk4 , fk5 , fk6 ;
float fk7, fk8 , fk9 , fk10 , fk11 , fk12;
float fk13, fk14, fk15, fk16, fk17, fk18;
float fk19, fk20, fk21, fk22, fk23, fk24;
float fk25, fk26, fk27, fk28;
float hk2, hk5;  
  // Trigonometic Parameters
  s1 = sin(Angles[0]); 
  s2 = sin(Angles[1]);
  s3 = sin(Angles[2]);
  s4 = sin(Angles[3]);
  s5 = sin(Angles[4]);
  s6 = sin(Angles[5]);
  c1 = cos(Angles[0]);
  c2 = cos(Angles[1]);
  c3 = cos(Angles[2]);
  c4 = cos(Angles[3]);
  c5 = cos(Angles[4]);
  c6 = cos(Angles[5]);
  s45 = s4*c5 + c4*s5;
  c45 = c4*c5 - s4*s5;

  //Subexpressions 
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
  hk2 = (fk18 -   fk20);
  hk5 = -1/(fk6*s6 + fk1);
  
  // Avoid Singularity
  if((hk2<=MIN) && (hk2>0.0)) hk2 = MIN;
  if((hk2>=-MIN) && (hk2<0.0)) hk2 = -MIN;
  if((hk5>=MAX) && (hk5>0.0)) hk5 = MAX;
  if((hk5<=-MAX) && (hk5<0.0)) hk5 = -MAX; 

  // Right Foot Configuration, Position and Orientation
  RF_Body[0] = gk1*(fk7*s6 - c6*(fk11 + fk13)) + gk2*(fk28 - fk27) + gk3*(fk12 - fk10*s4) + gk4*s1*c2 - gk5*c1;
  RF_Body[1] =  gk1*(c2*c3*s6 - fk6*c6) + gk2*(fk5*s5 - fk3*c5) + gk3*(s2*c4 - fk4) + gk4*s2 - 0.0821;
  RF_Body[2] = gk1*(fk22*s6 - c6*(fk24 + fk23)) - gk2*hk2 - gk3*(fk17*s4 + fk19) - gk4*c1*c2 - gk5*s1;
  RF_Body[3] = -asin(fk26);
  RF_Body[4] = atan(fk25/hk2);
  RF_Body[5] = atan(fk14*hk5);
  return RF_Body;
}
