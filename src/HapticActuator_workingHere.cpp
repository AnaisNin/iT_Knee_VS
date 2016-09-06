// ciCubConsole.cpp : Defines the entry point for the console application.

//
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

//#define ATISENSOR //DEFINE usage of ATI sensor.  If not defined relevant code is dissabled
#define IMU_AHRS_SENSOR 
#define IMU_LPMS_USBAL
#define FOOT_SENSOR
#define KNEEMOTOR_NEW //Left leg
//#define KNEEMOTOR_NEW_2 //right leg
//#define KNEEMOTOR_OLD

#include "stdafx.h"
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdio.h>
#include <time.h>


#pragma comment(lib, "Ws2_32.lib")
#include "UDP_Communication.h"
#include "devMeasurementFns.h"


#include "math.h"
#include "conio.h"
#include <iostream>	//yogesh
using namespace std;
#include "shTime.h"
#include <fstream>  //yogesh
#include <windows.h>

//Timer and thread
#include "CPrecisionClock.h"
#include "CThread.h"
#include "UDP_server.h"
#include <stdio.h>
#include <string.h>


//////################################################### FT_Sensor Foot Plate
#include "BoardLibrary.h"
#pragma comment(lib, "BoardLibrary.lib")
//#include "HumanBodyParameter.h"
//#include "JointTorquefunction.h"

//////################################################### Sensor IMU_AHSR
#include <tchar.h>
#include "vectornav.h"
extern "C"
{
#include "VN_math.h"
}

//IMU LPMS
//extern "C"
//{
#include "LpmsSensorI.h"
#include "LpmsSensorManagerI.h"
//}

//###################################################
#include "HumanBodyParam.h"
#include "mUtils.h"
#include "HumanDynModel.h"


double Timer2 = 0.0;

double FTData[6] = {0};
#define MaxNum_FTfoot 2 //Max id of the FTsensors boards
FT_Sensor FTfoot_data[MaxNum_FTfoot];//Foot FTsensor data
double Ffoot_offset[MaxNum_FTfoot][3] = {0};//Force offset data when biased


#define TRAJ_SAMPLES 50000
#define SAMPLES 1000000
#define Rad2Deg 180/M_PI			//REMOVE
#define Deg2Rad M_PI/180			//REMOVE
#define SIMULATION_TIME 3600
#define TSAMPLE 0.001//0.0005

#define BCAST_POLICY 191//5887//motor
#define BCAST_POLICY_FT 127//5887//
#define EXTRA_BCAST_POLICY 7//3584//4032
#define BCAST_RATE 2
//#define BASE_ADDRESS 70

#define Fgain 0.0073 
#define Tgain 1
#define offsetF 46895 //30358 // changed values on 26/2/13
#define offsetT 49150//30675 //
#define gainA 35.53
#define gainB 1.0876e-004
#define gainC 0.00175// 0.007 //original 0.007


#define Cdes 0.008 //// value of damping set manually for testing/////
#define FIRST_LBODY_BOARD 0	//CLEAN UP
#define LAST_LBODY_BOARD 10	//CLEAN UP

#define MOVETOHOME_VEL	400

#define RPMTODEGSEC(x)  x*360.0/(100.0*60.0)	
#define DEGSECTORPM(x)  100.0*60*x/360.0
#define IN_DEG 1
#define IN_RAD 2

#define USE_MOTOR_ENCODER 1
#define USE_JOINT_ENCODER 2

#define PID_CONTROL 1
#define LQR_CONTROL 2
#define gainSet 1 // for voltage control **Yogesh**
#define ON 1
#define OFF 0
#define FORWARD 1
#define REVERSE 0
#define posMAX 700000//1200000
#define	posMIN 200000
#define	AnuglarVelocity 1000/1// 10urad/ms	for costVelocity		10^-4rad/s teorical....but dynamics brings it down

//enum Motor
//{
//	MAINMOTOR,
//	DAMPERMOTOR
//};
//char MotorSelect=MAINMOTOR;

int desVel[MAX_BOARDS]={0};

int desPos[MAX_BOARDS]={0};

int desPosThred[MAX_BOARDS]={0};

int PosOff[MAX_BOARDS]={0};

int Destor[MAX_BOARDS]={0};
//int Destor[MAX_BOARDS] = { 4000, 3, 3, 4, 5, 6, 7, 8, 9, 10 };
int homePos[MAX_BOARDS]={0};

int homeVel[MAX_BOARDS]={20};

//float Cmatrix[2][2] = { { -0.00449707834668421, -1.16232662072916e-06}, { -1.29645017699755e-05, -4.73804273062362e-05 } }; 
//float Cmatrix[2][2] = { { -0.00429463384632091, -5.73945959687287e-07}, { 6.00819012212136e-05, -4.81132325882713e-05 } }; //originaly Working for low gains 10k resistors for torque and force amplifiers gain=4

//New matrix for lower gains (larger range) for force and torque (gain resistors 20k force and torque gain=2.1)
//float Cmatrix[2][2] = { { -0.008184640476527, -0.000002449108650}, { 0.000359306547899, -0.000078237689110 } };
//float Cmatrix[2][2] = { { -0.007213519306941, -0.000002173443882}, { 0.000115202066333, -0.000077652079235 } };
float Cmatrix[2][2] = { { -0.007305509383505, -0.000001562425665 },{ 0.000078091638029, -0.000076672664769 } };//Generated with VDA_FTsensor_Calibration_High_5_BEST.m


int activeBoards[MAX_BOARDS]={0};

float desPosDeg[MAX_BOARDS]={0};

int TorqueOffMtr[720] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1092.14, 1238.85, 1522.69, 1823.81, 1827.38, 1462.73, 1243.62, 1007.55, 844.041, 564.906, 279.311, 117.42, 8.31126, -24.0626, 117.413, 87.7707, -56.4106, 53.5, 98.0954, 362.018, 605.426, 900.132, 1131.65, 1383.56, 1792.41, 2084.83, 2156.19, 2022.15, 1728.14, 1432.56, 1167.29, 899.692, 691.232, 375.397, 29.8578, -0.414847, 250.15, 255.561, 206.869, 285.458, 303.508, 362.686, 527.225, 781.438, 816.317, 751.965, 970.114, 1268.08, 1334.15, 1291.6, 1088.31, 888.032, 792.946, 789.545, 768.829, 782.231, 486.109, 670.086, 859.393, 945.656, 1114.27, 1111.82, 898.169, 895.186, 1114.25, 1022.72, 825.1, 602.013, 539.283, 554.943, 370.943, 297.919, 334.325, 184.722, 83.7742, 409.864, 607.804, 705.174, 830.767, 915.668, 1146.44, 1361.13, 1442.32, 1489.53, 1276.33, 1081.29, 1169.34, 1132.19, 953.949, 834.465, 868.781, 878.703, 832.494, 934.703, 876.516, 721.69, 627.68, 612.703, 584.33, 573.428, 586.702, 606.548, 609.737, 690.446, 857.043, 858.972, 798.545, 794.45, 834.919, 873.981, 843.292, 918.998, 946.052, 952.579, 1204.81, 1355.84, 1363.39, 1283.1, 1106.47, 1058.91, 944.798, 798.567, 777.604, 609.333, 489.693, 601.732, 739.727, 700.876, 772.376, 775.136, 471.717, 452.996, 533.476, 814.429, 576.171, 547.161, 706.9, 752.355, 770.647, 728.27, 742.067, 564.784, 530.095, 554.242, 423.998, 253.675, -1.45254, 129.221, 208.657, 341.226, 536.703, 543.024, 210.984, 284.44, 522.174, 457.626, 503.35, 585.273, 774.461, 839.632, 1059.77, 958.687, 1014.04, 760.723, 599.593, 570.82, 499.356, 316.291, 189.217, 220.093, 282.638, 515.39, 497.09, 456.51, 390.642, 175.797, 426.732, 517.318, 341.238, 459.507, 272.25, 502.822, 628.086, 726.35, 727.134, 611.157, 416.148, 190.12, 242.378, -14.2, 43.8692, -248.447, -244.174, -49.2069, 6.42437, 135.338, -52.5076, 75.5198, 19.4551, -4.52242, 36.2902, 16.2923, 57.7096, 12.0788, 137.068, 213.622, 322.494, 273.215, 170.019, -23.7052, 57.0496, 34.4456, -106.244, -239.262, -235.471, -108.307, -33.8645, 46.185, 69.5455, -23.348, 10.3356, 62.5033, -29.5345, -65.7084, -77.2385, -49.793, 162.284, 456.614, 593.149, 551.35, 369.39, 105.753, -18.1516, 74.1404, -54.9227, -185.719, -258.582, -259.043, -34.4978, -35.5406, 36.8976, 41.757, -82.4745, -167.733, 34.2708, 54.5579, 38.9037, 48.3478, 9.89552, 257.138, 346.92, 537.153, 371.384, 118.679, 58.1979, 65.3467, -32.6957, -208.866, -257.107, -379.133, -275.253, -101.799, -35.5968, -35.65, -40.9632, -208.07, -106.495, -132.639, -252.859, -103.633, -126.619, 8.21569, 221.426, 187.93, 230.179, 115.134, 29.9441, -169.072, -255.777, -271.574, -490.939, -492.27, -465.357, -406.813, -277.287, -283.936, -253.452, -404.641, -505.662, -424.824, -274.048, -228.238, -269.162, -239.867, -47.1917, 86, 154.886, -21.3599, -98.4857, -279.303, -343.338, -459.237, -487.077, -716.123, -791.245, -601.518, -408.628, -282.403, -253.379, -329.025, -520.029, -563.368, -573.153, -565.138, -495.399, -516.678, -439.386, -275.327, -139.083, 23.6396, -227.914, -372.512, -482.766, -509.597, -544.15, -790.466, -708.073, -718.984, -665.957, -523.123, -548.94, -486.375, -746.082, -772.313, -517.228, -546.552, -631.96, -643.87, -680.173, -525.964, -289.078, -301.516, -236.484, -484.47, -713.485, -788.254, -904.227, -993.367, -1108.79, -1323.45, -1284.15, -1006.25, -1089.85, -1093.64, -994.006, -1089.35, -948.246, -978.865, -1046.96, -1056.64, -969.167, -978.89, -761.184, -495.915, -443.519, -730.99, -820.814, -1065.55, -1033.63, -1047.1, -1164.59, -1323.4, -1321.05, -1036.41, -989.118, -904.346, -902.184, -992.505, -1053.32, -1094.59, -1078.56, -1013.11, -1025.9, -1017.92, -815.656, -591.123, -447.13, -479.31, -767.447, -993.144, -1064.1, -1102, -1227.32, -1260.18, -1212.31, -1269.93, -1115.96, -1021.62, -956.383, -1047.85, -1192.03, -1095.46, -1069.94, -1144.78, -1145.97, -1107.9, -950.595, -720.492, -451.232, -504.373, -692.875, -930.902, -1061.16, -1138.65, -1278.55, -1305.02, -1565.57, -1526.46, -1341.75, -1248.45, -1179.12, -1218.32, -1255.78, -1263.67, -1138.64, -1016.59, -794.516, -696.769, -485.112, -181.414, -16.4474, -39.4202, 4.11429, -225.052, -493.803, -503.987, -558.519, -485.607, -644.616, -852.997, -622.133, -487.257, -239.686, -294.738, -300.573, -516.068, -572.468, -460.336, -798.033, -710.726, -956.709, -923.766, -794.846, -589.842, -546.139, -802.57, -1004.1, -1085.62, -1024.05, -1007.33, -1008.44, -1135.67, -1083.94, -925.948, -761.49, -580.672, -609.613, -763.792, -759.773, -820.371, -670.801, -469.274, -519, -445.438, -208.81, -17.5113, -61.5784, -29.8262, -145.83, -255.612, -256.881, -472.855, -513.997, -526.01, -783.026, -526.887, -256.696, -245.697, -259.833, -273.968, -391.614, -255.967, -201.459, -293.006, -247.175, -268.037, -130.031, -1.39934, 321.333, 272.235, 216.211, 3.33007, -51.8529, -201.316, -264.824, -427.795, -541.92, -565.922, -246.488, -290.498, 44.7329, 37.3252, -344.492, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

int TorqueOffMtrFK[720] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 598.87, 562.137, 575.349, 734.655, 696.196, 616.951, 258.032, 298.467, 428.264, 480.284, 551.146, 425.245, 688.746, 877.464, 895.177, 909.145, 806.711, 646.424, 370.591, 445.082, 111.441, 210.721, -10.698, -67.7004, 184.165, 417.677, 486.629, 351.845, 91.885, 124.996, 144.839, 195.118, 128.129, 197.066, 218.611, 365.379, 495.035, 564.804, 486.377, 390.4, 114.148, 108.136, 187.451, 154.698, -30.8292, -3.43612, 100.265, 227.765, 468.303, 466.696, 227.691, 140.262, 202.228, 186.302, 143.418, 155.596, 173.048, 346.032, 532.71, 719.509, 723.165, 512.683, 226.942, 170.911, 187.249, 15.0156, -34.708, -88.4599, -64.0938, 150.236, 146.803, 296.934, 208.721, 48.5448, 3.375, 114.388, 103.188, 84.7451, 142.315, 184.426, 361.097, 527.272, 623.038, 461.183, 242.727, 191.303, 131.867, 45.2902, -94.69, -169.812, -274.479, -154.286, 76.2476, 104.901, 136.78, 34.0099, -55.4575, -177.162, -203.87, -187.274, -158.708, -132.506, 5.60328, 254.839, 417.221, 386.104, 122.58, -39.8585, -131.947, -161.389, -275.399, -363.199, -584.441, -643.645, -361.636, -138.971, -135.301, -235.791, -363.494, -398.977, -402.049, -433.039, -401.156, -377.82, -271.851, -12.1748, 138.544, 78.9477, 159.787, -78.2059, -272.069, -333.792, -414.273, -631.906, -589.043, -727.845, -621.929, -368.06, -318.039, -308.594, -383.693, -591.714, -564.032, -513.939, -608.937, -646.193, -639.63, -581.968, -403.516, -327.194, -165.666, -400.444, -584.222, -615.502, -591.294, -901.469, -845.139, -1052.89, -1125.71, -896.437, -765.717, -574.333, -676.781, -878.803, -857.158, -940.204, -897.28, -863.748, -860.74, -846.416, -868.895, -631.7, -693.693, -655.505, -807.013, -862.872, -1079.04, -1156.3, -1422.33, -1375.82, -1701.36, -1632.81, -1426.68, -1446.95, -1367.14, -1428.13, -1348.31, -1433.3, -1469.78, -1431.98, -1447.3, -1378.31, -1373.69, -1151.56, -875.623, -846.851, -1055.48, -1174.83, -1439.23, -1406.95, -1396.97, -1683.18, -1636.45, -1620.38, -1568.7, -1380.51, -1429.77, -1424.14, -1423.06, -1561.71, -1420.1, -1409.96, -1546.02, -1656.46, -1708.6, -1399.46, -1119.15, -1155.22, -1115.57, -1328.05, -1550.58, -1697.05, -1608.93, -1810.76, -1915.5, -1952.56, -1901.08, -1785.72, -1590.73, -1650.28, -1635.7, -1984.74, -1934.07, -1885.86, -1975.66, -1988.35, -1974.81, -1687.74, -1501.21, -1378.59, -1368.46, -1517.21, -1789.04, -2028.96, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
}; // prende in considerazione dinamica morote + sistema assemblato+ sforzo per gravità!

int GravtyC_Mtr[720] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 242.055, 233.337, 220.84, 208.107, 199.068, 190.188, 180.674, 171.821, 162.1, 152.706, 142.49, 134.134, 124.471, 117.517, 108.585, 98.6975, 89.9827, 81.7366, 72.4235, 63.8317, 54.9807, 44.9166, 35.8272, 24.9172, 15.9568, 7.66094, -3.89336, -12.8575, -22.4527, -32.0717, -42.6768, -51.3586, -60.9458, -70.8296, -79.8574, -89.2167, -98.7596, -108.026, -118.441, -128.558, -137.252, -147.56, -160.318, -168.521, -181.566, -189.231, -200.885, -212.167, -222.495, -230.633, -243.984, -252.46, -261.208, -273.651, -282.263, -295.205, -306.101, -313.89, -325.91, -337.598, -345.969, -355.093, -368.564, -378.689, -384.624, -399.193, -407.63, -420.37, -431.823, -438.712, -450.146, -461.184, -470.684, -479.145, -490.983, -501.291, -508.355, -520.093, -527.479, -540.296, -549.294, -559.921, -570.407, -582.607, -596.025, -603.372, -614.08, -621.821, -631.919, -642.804, -648.549, -662.075, -669.822, -678.647, -689.49, -697.781, -708.25, -717.241, -727.357, -735.154, -748.885, -754.526, -767.315, -775.362, -782.025, -793.9, -803.475, -810.599, -820.349, -831.604, -838.497, -848.322, -857.916, -866.492, -876.65, -885.629, -892, -902.184, -912.391, -918.479, -926.685, -941.662, -944.733, -955.798, -961.699, -970.187, -981.825, -989.027, -993.581, -1006.19, -1014.18, -1016.72, -1027.58, -1036.08, -1043.18, -1052.93, -1055.98, -1066.92, -1075.03, -1082.05, -1087.98, -1097.04, -1106.21, -1110.24, -1121.04, -1123.03, -1136.33, -1144.63, -1147.63, -1155.05, -1164.36, -1169.52, -1175.42, -1184.7, -1193.78, -1194.82, -1204.04, -1207.03, -1218.16, -1225.46, -1226.4, -1234.69, -1244.01, -1247.98, -1250.79, -1262.83, -1267.06, -1276.12, -1281.85, -1282.07, -1292.27, -1301.77, -1300.83, -1306.52, -1313.75, -1318.88, -1319.11, -1328.29, -1334.7, -1339.31, -1345.49, -1344.98, -1354.53, -1363.08, -1359.43, -1367.2, -1378.17, -1377.16, -1382.02, -1386.04, -1387.84, -1398.21, -1397.43, -1402.9, -1407.7, -1414.18, -1415.65, -1419.52, -1425.47, -1425.71, -1428.04, -1435.84, -1435.47, -1442.16, -1444.53, -1443.57, -1449.92, -1454.93, -1450.85, -1458.03, -1462.07, -1461.79, -1468.59, -1464.33, -1472.69, -1471.76, -1478.74, -1474.04, -1480.36, -1484.01, -1478.39, -1484.73, -1488.04, -1487.2, -1494.69, -1490.17, -1492.42, -1498.68, -1493, -1497.39, -1500.64, -1498.9, -1497.2, -1498.23, -1501.41, -1497.95, -1505.48, -1506.35, -1500.98, -1481.33, -1497.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

int Motor_GC_Mtr[720] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 576.638, 440.842, 311.881, 43.9725, 67.148, 202.829, 203.206, 220.444, 175.549, 247.498, 475.739, 654.245, 634.451, 526.661, 309.764, 178.245, 40.3048, -98.7381, -211.681, -323.491, -411.364, -181.157, -53.183, -27.9174, -62.9709, -183.44, -186.779, -246.872, -267.493, -195.335, -212.389, -201.789, -90.8994, 69.691, 88.3068, 17.9399, -48.4946, -210.089, -230.855, -374.609, -410.458, -406.31, -474.873, -425.97, -289.23, -187.297, -146.945, -271.145, -420.759, -501.487, -503.31, -465.093, -452.743, -421.677, -342.625, -89.0322, 137.538, 109.714, -154.402, -388.85, -411.096, -526.794, -708.34, -662.378, -790.765, -804.713, -662.834, -419.152, -413.499, -419.565, -674.976, -658.609, -722.87, -722.992, -667.002, -715.367, -644.142, -434.755, -163.219, -225.146, -175.938, -439.698, -743.737, -728.345, -695.983, -876.077, -955.48, -922.726, -958.365, -804.519, -705.727, -722.655, -736.42, -952.118, -975.508, -926.441, -878.804, -916.844, -892.628, -914.652, -740.563, -692.851, -728.218, -725.093, -944.948, -912.957, -1158.18, -1206.84, -1350.29, -1479.43, -1405.94, -1399.97, -1189.33, -1264.51, -1203.49, -1338.93, -1495.05, -1497.39, -1412.11, -1190.94, -1154.87, -1198.79, -993.417, -957.628, -860.48, -1003.77, -1029.55, -1177.5, -1254.1, -1503.89, -1449.92, -1697.76, -1747.9, -1585.53, -1479.28, -1245.16, -1177.6, -1409.18, -1456.04, -1470.87, -1460.02, -1405.54, -1445.89, -1519.96, -1472.24, -1413.68, -1233.25, -1222.28, -1323.28, -1478.01, -1699.11, -1726.3, -1776.37, -1959.58, -1973.44, -1984.85, -1940.65, -1725.09, -1714.23, -1772.67, -1908.55, -1962.78, -1922.58, -1951.45, -1997, -1969.9, -1960.44, -1864.12, -1658, -1613.65, -1660.54, -1753.52, -1939.31, -2028.7, -2162.33, -2359.79, -2401.92, -2423.82, -2472.29, -2350.8, -2242.62, -2217.95, -2395.58, -2395.84, -2409.99, -2342.97, -2277.12, -2234.7, -2252.15, -2198.59, -2051.26, -1867.6, -1901.67, -1948.38, -2102.95, -2182.8, -2363.02, -2408.71, -2377.36, -2429.98, -2487.24, -2389.27, -2276.35, -2180.55, -2238.97, -2341.43, -2402.32, -2414.15, -2392.05, -2430.8, -2392.6, -2440.88, -2196.01, -2046.65, -1923.44, -1936.04, -2146.8, -2390.29, -2382.46, -2354.46, -2548.98, -2673.72, -2670.45, -2609.45, -2392.2, -2431.04, -2435.56, -2419.61, -2435.6, -2447.99, -2377.93, -2378.8, -2367.72, -2392.87, -2277.61, -2030.33, -1843.14, -1808.65, -1992.68, -2253.77, -2527.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static int noActiveBoards=0;

float traj[MAX_BOARDS][TRAJ_SAMPLES];
int trajPoints;
int trajdt=1;
int curTrajpoint=1;

int b=0;
int counter =0;
shTime timeObject;
double startLoopTime;
double startSimTime;
double curSimTime, starTime, sineZero;
double ts;
double feq;
double timeudp, t1, t2;
char TogglePrint=1;
float Force;
float Torque;

int desTor;
int pidOutput;
long pidError;
int pid_off[15];

int desStiff[MAX_BOARDS]={0};
int desDamp[MAX_BOARDS]={0};

float chirpSignal = 0.0;

long gains[3];
char* Cgains;

int DesOffset[MAX_BOARDS]={0}; // for voltage control **Yogesh**

CharBuff packetToReceive;

// Stucture
typedef struct joint_data {
    
	//long radPos;
	//long degPos;
	
	int id;
	long pos; //long
	short vel;
	short tor;
	int pidOutput;
	long pidError;
	long temp_Vdc;
	long tStamp;

	int fault;
	int anInput1;
	int anInput2;
	int anInput3;
	int anInput4;
	int absPos1;
	int absPos2;
	int quickSpeed;
	//int realCurrent;
	long realCurrent;

	long tpos;
	long temp_tpos;
	long req_tpos;
	int TwinPos;
	short angle;
	short targAngle;
	long TwinTargPos;
	short TwinVel;
	int Xaccleration;
	int Yaccleration;
	int Zaccleration;
	int LinkVel;

	float tposRAD;
	float finalTime;

	int tvel;

	long TwinActPos;


	int ttor;
	int motorEncLimit1;
	int motorEncLimit2;
	int jointEncLimit1;
	int jointEncLimit2;
	float jointLimit1DEG;
	float jointLimit2DEG;
	float jointLimit1RAD;
	float jointLimit2RAD;
	float motorEncLine;
	float jointEncLine;
	float jointPosRAD;
	float jointPosDEG;
	int minvel;
	int maxvel;
	int maxCur;
	int pgain;
	int igain;
	int dgain;
	int ilimit;
	int gainscale;
	int offset;

	int trajDSP;

	
	int anInput1_Adjusted;
	int	anInput1_Offset;
	int anInput2_Adjusted;
	int	anInput2_Offset;



	int isActive;
	int isConnected;
	int controlType;
	int BoardType;

	float Force;
	float Torque;
	float Force2;
	float Torque2;
	short DamperRefNormForce;
	short DamperRefTorque;
	long DamperRefDeflection;
	long DamperActDeflection;

	float getJointPos(int fromEncoder, int posUnit)
	{
	
		if(fromEncoder==USE_MOTOR_ENCODER)
			jointPosRAD=jointLimit1RAD + motorEncLine*((float)pos - (float)motorEncLimit1);
		else if(fromEncoder==USE_JOINT_ENCODER)
			jointPosRAD=jointLimit1RAD + jointEncLine*((float)absPos1 - (float)jointEncLimit1);	
			jointPosDEG=g_RAD2DEG(jointPosRAD);

		if(posUnit==IN_RAD)
			return jointPosRAD;
		else if(posUnit==IN_DEG)
			return jointPosDEG;
		else return 0;
	};

	void setJointTargetPos(float targetPos, int posUnit)
	{
		if(posUnit==IN_DEG)
			//targetPos=DEGTORAD(targetPos);
			targetPos = g_DEG2RAD(targetPos);
		if(motorEncLine!=0)
		{
			tpos=motorEncLimit1 + (int)((targetPos - jointLimit1RAD)/motorEncLine);
		
		}
		else tpos=pos;
			
	};

} jointData;

//////################################################### Sensor Foot_Plate

//REMOVE
//typedef struct HumanBodyParametr{
//
//	// lenghts
//	double	l_shank=0.515;
//	double	l_shank_a = 0.360;			// define in the copybook IIT 5
//	double	l_ATIsensor = 0.405;
//
//
//	double  l_thigh = 0.405;
//	double  l_thigh_a=0.250;
//	double	l_iTmotor = 0.300;
//
//	double	l_torso = 0.410;
//
//	// masses in KG from Nasa Standrd
//	double	F_torso = 41.32*g_GRAVITYacc;		// N 
//	double	F_pelvis = 6.15*g_GRAVITYacc;
//	double	F_iTmotor = 1 * g_GRAVITYacc;
//	double	F_thigh = 10.4*g_GRAVITYacc;
//	double	F_ATIsensor = 0.5*g_GRAVITYacc;
//	double	F_shank = 4.04*g_GRAVITYacc;
//
//} HumanBodyParametr;

//REMOVE
//typedef struct JointTorque{
//
//	float	Torque_Hip, Torque_Knee, Torque_Ankle, Theta1, Theta2, Theta3;
//	
//} JointTorque;

//////###################################################


// Precision clock
cPrecisionClock* Timer2Clock;
cPrecisionClock* mainClock;
cPrecisionClock* runClock;

// main haptics loop
void updateLoop(void);

//second prototype 
jointData joint[1]={

1,0,0,0,0,0,200,0,0,10000,10000,0,0,60,-60,0,0,0,0,0,0,0,3333,0,1600,16,1000,32000,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,PID_CONTROL,
};

//jointData joint[MAX_BOARDS];

int oldTarget=0;
int r;
int loop=0;
int sampleloop=0;
int ch=0;
int stopSim = 0;


int accn[SAMPLES][3]; //created for aceleration on 13-2-13
//11/3/2013 original double data[SAMPLES][12];
int data_motor[SAMPLES][12];
float fdata[SAMPLES][12];
float FT_data[SAMPLES][6];
float fCalibFTdata[SAMPLES][8];
long dCalibFTdata[SAMPLES][8];
float sT[SAMPLES];

///////////////////////////////////////////////////////////////////////////////////////////////////////

double radPos;
double degPos;
float Anglestep = 872.665;//in 10urad = 0.5°
int posindex, TorqueOff, TorqueOffFK, GravityComp, Motor_GravtyComp;

long temp;
long posPID[3], torPID[3], Vwallk[3];
long velPID[3];
double A0 = 104720, Amplincrem = 3490.66;// 75°=130900,	60°= 	30°=52359.9		130°=226893     urad!!!!!!!!!!!!!!!!!!!!
double Ashift = -26179.9; // 11°=19198.6		15° =		18°=31415.9	 20°=34906.6	22°=38397.2	25°= 43633.2	urad!!!!!!!!!!!!!!!!!!!!		in order to fit with ROM of FakeKnee
long VelAng, Ai;//Initial sine amplitude
float alpha2;//value coming from motor, for knee joint angle pos
char pidGains[20];
char sinewaveON=OFF;
double sinecounter , Ti=0, Tchange=0;
double AngVel = 0; 
char damperSinewaveON=OFF,damperPosSinewaveON=OFF;
long FunctionOffset = 0;
double InitialPos, phi1;
char rampON=OFF, rampDirection=FORWARD;
char costVelON = OFF;
char	storeData=OFF,	deleteData=OFF;
char	positionControl = OFF;
char	torqueControl = OFF;
char	GravtyCompFlag = OFF ;
char	Motor_GravtyCompFlag = OFF;
char	Posture_Torque_Est = OFF;
long	dataCounter=0;
double	sinefreq = 0.1; 
int	rampSTEP=100;
int	rampPos=0;

//--------------Thread variables
//double	Tsine=0.0;
int printTimer=0;
int rampSTEPthread=100;
double t=0.0;
char mConfig[10] = { 0 }, mConfig2[10] = { 0 };

//////################################################### Sensor IMU_AHSR

#ifdef IMU_AHRS_SENSOR
/* Change the connection settings to your configuration. */
const char* const COM_PORT = "COM3";
const int BAUD_RATE = 115200; // 115200;	//// BCAST_RATE*0.5;

VN_ERROR_CODE errorCode;
Vn100 vn100;
VnYpr ypr;
VnQuaternion quaternion;
//VnDeviceCompositeData dataa;

//float Euler231[3] = { 0 };
#endif 
float quat_ahrs[4] = { 0 };
VnQuaternion quat;
float euler312_ahrs[3];//Torso

//////################################################### Sensor Foot_Plate
int id_FTfoot = 2; //BoardNumber 
float divs = 1000; //divide FTfoot sensor reading by divs to get SI units
//Incoherence somewhere,  FTfoot_data[id_FTfoot].FT[0] is supposed to return SI units * 1000000 (cf def FT_Sensor in BoardLibrary.h)
double AnkleTorque_sensor;// , AnkleT_sensor;
double AnkleTorque_sensor_filt; //Computed from filtered FTreading
const int size_bufferAT = 20;//50 - size buffer ankle torque
double buffer_ankleTorque[size_bufferAT] = {0};

float KneeMotorPos;

//IMU_LPMS_USBAL
ImuData * pData_imu_lpms;//d
LpmsSensorI* p_LpmsSensor;//lpms
LpmsSensorManagerI* p_LpmsSensorManager;
float quat_lpms[4] = {0};
float euler213_lpms[3];//shank


//HumanBodyParametr HB;	//REMOVE
//JointTorque JT;			//REMOVE
/////////////////////////////////////////////////////////////////////////////////////////////////
HumanBodyParam * p_HBparam;
HumanDynModel * p_HDynM;
double theta1_calib_deg=45.0;//TO SET
double Knee_RelAngle;//rad, from shank to thigh
FILE * pFile;
bool flg_isRunning = false;
double runTimer = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////////
//Fnt prototypes
int sgn(float num);
void gotoxy(int column, int line); 
float chirpFnc(double currentTime, int f0, int f1, int tf);
void setControllerGains(); 
void loadJointDataFromDrivers();
int scanSystem();
int connectUDP();
void connectTCP();
void startControlAll();
void startControl(int BoardNumber);
void stopControl();
void setTorqueCntrlFlag(int BoardNumber, int torqueFlag);
void setBroadCastRate(int boardNumber, int BCastRate);
void setBroadCastPolicy(int boardNumber, int BCastPolicy);
void setExtraBroadCastPolicy(int boardNumber, int ExtraPolicy);
void stopBroadCast(int BoardNumber);
void saveData();
void ClearScreen(void);
void closeSim(void);
int getBroadCastData(int numActiveBoards);
int getBroadCastFTData(int numActiveBoards);
void setupPositionControlPID();
void setupTorqueControlPID();
void moveToHome();

/******************************************/

void Myprint(void)
{
	gotoxy(1,1);

	if (storeData==ON)
		printf("\n ----------------------Storing data.        Data counter:%*d --------------\n\n",7,dataCounter);
	else if(deleteData==OFF)
		printf("\n ----------------------NOT Storing data.    Data counter:%*d --------------\n\n",7,dataCounter);
	else
		printf("\n ----------------------Deleted Stored data. Data counter:%*d --------------\n\n",7,dataCounter);

	//printf(" Frequency = %0.1f Hz\n", 1 / Timer2);
	printf("\n runTimer: %g \n",runTimer);

	#ifdef KNEEMOTOR_OLD
	printf("\n Position=%*d 10urad \t Velocity=%*d mrad/s     \n", 7, joint[0].pos, 7, joint[0].vel );
	printf("\n Position: %+#7.2f deg \t \n", Rad2Deg*(KneeMotorPos));
	printf("\n\n Torque=%*d   \t TorqueOffs=%*d \t GravityComp=%*d\n", 7, joint[0].tor, 7, TorqueOff, 7, GravityComp);
	printf("\n Current=%*d \n", 7, joint[0].realCurrent);
	printf("\n PIDOutput=%*d  \t PIDError=%*ld   \n", 7, joint[0].pidOutput, 7, joint[0].pidError);
	printf("\n TimeStamp=%*ld \n",  7, joint[0].tStamp);
	printf("\n TargetPos=%*d  \t tempTargetPos=%*d  \t ReqTargetPos=%*d   \n", 7, joint[0].tpos, 7, joint[0].temp_tpos, 7, joint[0].req_tpos);
	#endif

	#ifdef KNEEMOTOR_NEW
	
	printf("\n Exo motor \n");
	printf("\t Position=%*d 10urad \t Velocity=%*d mrad/s \n", 7, joint[0].pos, 7, joint[0].vel );
	printf("\t Position_Motor: %+#7.2f deg \t \n", (float)(Rad2Deg*(joint[0].pos*0.00001)));// %+#7.2f(KneeMotorPos));
	printf("\t alpha2: %+#7.2f deg \t \n", alpha2);// %+#7.2f(KneeMotorPos));
	printf("\t Knee_RelAngle (deg): %g \n", g_RAD2DEG(Knee_RelAngle));// %+#7.2f(KneeMotorPos));
	//Do properly
	//printf("\t theta2 [deg]: %g \n", g_RAD2DEG(l_theta1 + M_PI - Knee_RelAngle));
	printf("\t Torque=%*d   \t TorqueOffs=%*d \t GravityComp=%*d\n", 7, joint[0].tor, 7, TorqueOff, 7, GravityComp);
	//printf("\t Current=%*d \n", 7, joint[0].realCurrent);
	//printf("\t PIDOutput=%*d  \t PIDError=%*ld   \n", 7, joint[0].pidOutput, 7, joint[0].pidError);
	//printf("\t TimeStamp=%*ld \n", 7, joint[0].tStamp);
	printf("\t TargetPos=%*d  \t tempTargetPos=%*d  \t ReqTargetPos=%*d   \n", 7, joint[0].tpos, 7, joint[0].temp_tpos, 7, joint[0].req_tpos);
	#endif

	#if defined(KNEEMOTOR_NEW)&&defined(IMU_LPMS_USBAL)
		printf("theta2 [deg]: %g \n",g_RAD2DEG(p_HDynM->get_theta2()));
	#endif

	#ifdef ATISENSOR
	printf("\n\n ATI_sensor data:  \n");
	printf("\n Fx=%10f Fy=%10f Fz=%10f \n", FTData[0], FTData[1], FTData[2]);
	printf(" Tx=%10f Ty=%10f Tz=%10f \n", FTData[3], FTData[4], FTData[5]);
	#endif

	#ifdef FOOT_SENSOR
	printf("\n\n Foot_sensor data:  \n");
	printf("\t Fx=%10f Fy=%10f Fz=%10f \n", (float)FTfoot_data[id_FTfoot].FT[0] / divs, (float)FTfoot_data[id_FTfoot].FT[1] / divs, (float)FTfoot_data[id_FTfoot].FT[2] / divs);
	printf("\t Fx_offset=%10f Fy_offset=%10f Fz_offset=%10f \n", (float)(FTfoot_data[id_FTfoot].FT[0]+Ffoot_offset[id_FTfoot][0]) / divs, (float)(FTfoot_data[id_FTfoot].FT[1]+Ffoot_offset[id_FTfoot][1]) / divs, (float)(FTfoot_data[id_FTfoot].FT[2]+Ffoot_offset[id_FTfoot][2]) / divs);
	printf("\t Tx=%10f Ty=%10f Tz=%10f \n", (float)FTfoot_data[id_FTfoot].FT[3] / divs, (float)FTfoot_data[id_FTfoot].FT[4] / divs, (float)FTfoot_data[id_FTfoot].FT[5] / divs);
	//printf(" Sensor data:  ch1=%10d ch2=%10d ch3=%10d \n", FTfoot_data[id_FTfoot].ChRaw_Offs[0], FTfoot_data[id_FTfoot].ChRaw_Offs[1],FTfoot_data[id_FTfoot].ChRaw_Offs[2]);
	//printf("\t Weight: %g \n",p_HDynM->get_weight());
	printf("\t AnkleTorque: raw = %10g \t filtered=%10g \n", AnkleTorque_sensor, AnkleTorque_sensor_filt);		// Because rotated frame
	#endif

	if (sinewaveON == ON)
	{	
		Ai = Rad2Deg*((A0)*0.00001); //+AmpliduteUD +
		VelAng = 2*M_PI*(sinefreq);	// + fi
		printf("\n\n SINE characterstic:  \n");
		printf("\n A: %*d  \t f:%0.6f \t", 7, Ai, sinefreq);	//+fi
		printf("\n VelAng: %*d  \t", 7,  VelAng);
	}
	
	#ifdef IMU_AHRS_SENSOR //IMU Torso
	if (errorCode == VNERR_NO_ERROR)
	{
	//	/*printf("yaw, pitch, roll\n");
	//	printf("  %+#7.2f %+#7.2f %+#7.2f\n",ypr.yaw,ypr.pitch,	ypr.roll);*/

	//	// quaternion
	//	/*printf("x, y, z, w\n");
	//	printf("  %+#7.2f \t  %+#7.2f \t %+#7.2f \t %+#7.2f\n", quat.x, quat.y, quat.z, quat.w);*/


		printf("\n\n IMU torso:\n");
		//printf("\n Roll: %+#7.2f \t Yaw: %+#7.2f \t Pitch*: %+#7.2f (abs)   (deg) \n", Rad2Deg*Euler231[0], Rad2Deg*Euler231[1], Rad2Deg*Euler231[2]);//Lorenzo
		printf("\t Raw data - [0]:Z %+#7.2f \t [1]:X %+#7.2f \t [2]:Y %+#7.2f   (deg) \n", g_RAD2DEG(euler312_ahrs[0]), g_RAD2DEG(euler312_ahrs[1]), g_RAD2DEG(euler312_ahrs[2]));
		//printf("\t theta3 [deg]: %g \t local: %g \n",g_RAD2DEG(p_HDynM->get_theta3()), g_RAD2DEG(-euler312_ahrs[2] + M_PI));
		//printf("\t theta3 [deg]: imu front %g \t back: %g \n", g_RAD2DEG(-euler312_ahrs[2] + M_PI), g_RAD2DEG(euler312_ahrs[2]));
		printf("\t theta3 [deg]: %g \n", g_RAD2DEG(p_HDynM->get_theta3()));
		//printf("\t theta3 with offset [deg]: imu front %g \t back: %g \n", g_RAD2DEG(-euler312_ahrs[2] + M_PI+g_DEG2RAD(6)), g_RAD2DEG(euler312_ahrs[2])-15);

		



		if (Posture_Torque_Est == ON){
			printf("\n\n Body Posture(deg):\n");
			//printf(" Hip: %+#7.2f \t Knee: %+#7.2f \t Ankle: %+#7.2f   (deg) \n", Rad2Deg*JT.Theta3, Rad2Deg*JT.Theta2, Rad2Deg*JT.Theta1);
			printf(" Theta3: %+#7.2g \t Theta2: %+#7.2g \t Theta1: %+#7.2g  \n", g_DEG2RAD(p_HDynM->get_theta3()), g_DEG2RAD(p_HDynM->get_theta2()), g_DEG2RAD(p_HDynM->get_theta1()));
			
			printf("\n\n Joint Torques (Nm):\n");
			//printf(" Hip: %+#7.2f  \t    \t  Knee: %+#7.2f   (Nm)  \n Ankle_Es: %+#7.2f  \t Ankle_Mea: %+#7.2f   (Nm) \n", JT.Torque_Hip, JT.Torque_Knee, JT.Torque_Ankle, (float)AnkleTorque_sensor);
			printf(" Hip: %+#7.2g  \t    \t  Knee: %+#7.2g  \n Ankle_Es: %+#7.2g  \t Ankle_Mea: %+#7.2g  \n",  p_HDynM->get_torqueAnkle(), p_HDynM->get_torqueKnee(), p_HDynM->get_torqueAnkle(), AnkleTorque_sensor);
		}
	//	if ((controlIsOn == 0) && (torqueControl == ON) && (Posture_Torque_Est == ON))  //Desire torque				//Recent modification...before only: (torqueControl == ON)
		if ( (torqueControl == ON) && (Posture_Torque_Est == ON))  //Desire torque				//Recent modification...before only: (torqueControl == ON)
		{
			//printf("\n\n Assistive Knee torque: %+#7.2f\n", JT.Torque_Knee*0.03);
			printf("\n\n Assistive Knee torque: %+#7.2g\n", p_HDynM->get_torqueKnee()*0.03);
		}
	}
	#endif

	#ifdef IMU_LPMS_USBAL
		//printf("Timestamp=%f, qW=%f, qX=%f, qY=%f, qZ=%f\n",
				//d.timeStamp, d.q[0], d.q[1], d.q[2], d.q[3]);
				//pData_imu_lpms->timeStamp, pData_imu_lpms->q[0], pData_imu_lpms->q[1], pData_imu_lpms->q[2], pData_imu_lpms->q[3]);
		printf("\n IMU shank \n");
		printf("\t Raw data - [0]:Y %+#7.2f \t [1]:X %+#7.2f \t [2]:Z %+#7.2f   (deg) \n", g_RAD2DEG(euler213_lpms[0]), g_RAD2DEG(euler213_lpms[1]), g_RAD2DEG(euler213_lpms[2]));
		//printf("\t theta1 [deg]: %g \t local: %g \n", g_RAD2DEG(p_HDynM->get_theta1()), g_RAD2DEG(euler213_lpms[0] + M_PI));
		//printf("\t theta1 [deg]: %g \n", g_RAD2DEG(euler213_lpms[0] + M_PI));
		//printf("\t theta1 [deg]: %g \n", g_RAD2DEG(l_theta1));
		printf("\t theta1 [deg]: %g \n",g_RAD2DEG(p_HDynM->get_theta1()));

	#endif

	//printf("\n Height computed [m]: %g \n",p_HBparam->get_height());


}

//REMOVE
//void HBParfunction(HumanBodyParametr HB, int weight, int height){
//
//	if ((weight != 0) && (height != 0))		//argrument ... to be developed
//	{
//		printf("ERRORE");
//		return;
//	}
//	else {
//
//		// lenghts in m from me
//		HB.l_shank = 0.515;
//		HB.l_shank_a = 0.360;
//		HB.l_ATIsensor = 0.405;
//
//		HB.l_thigh = 0.405;
//		HB.l_thigh_a = 0.250;
//		HB.l_iTmotor = 0.300;
//
//		HB.l_torso = 0.410;
//		
//		// masses in KG from Nasa Standrd
//		HB.F_torso = 41.32*g_GRAVITYacc;		// N
//		HB.F_pelvis = 6.15*g_GRAVITYacc;
//		HB.F_iTmotor = 1 * g_GRAVITYacc;
//		HB.F_thigh = 10.4*g_GRAVITYacc;
//		HB.F_ATIsensor = 0.5*g_GRAVITYacc;
//		HB.F_shank = 4.04*g_GRAVITYacc;
//
//		//return HB;
//	}
//}

//REMOVE
//JointTorque JointTorquefunction(JointTorque JT, HumanBodyParametr HB, float PitchIMU, float Footsensor_AnkleTorque, float KneeMotorPos){
//
//	double A, B, C, D, E, F, G;
//	float Theta1s1, Theta1s2, KneePos, Baias_AnkleTorque;		//in rad
//	A = 0;	B = 0;	C = 0; D = 0; E = 0; F = 0; G = 0;
//	Theta1s1 = 0; Theta1s2 = 0, KneePos =0;
//	
//	Baias_AnkleTorque = Footsensor_AnkleTorque - 10;			// I don't remember why I put -10...maybe it is like a bies
//	JT.Theta3 = PitchIMU;	//rad
//	//KneePos = -KneeMotorPos + Deg2Rad*(60- 23);
//	KneePos = KneeMotorPos ;	//rad
//
//	A = Baias_AnkleTorque - HB.F_torso*HB.l_torso*cos(JT.Theta3);
//	B = HB.l_shank_a*HB.F_shank + HB.l_ATIsensor*HB.F_ATIsensor + HB.l_shank*(HB.F_thigh + HB.F_iTmotor + HB.F_pelvis + HB.F_torso);
//	C = HB.l_thigh_a*HB.F_thigh + HB.l_iTmotor*HB.F_iTmotor + HB.l_thigh*(HB.F_pelvis + HB.F_torso);
//	D = C*cos(KneePos);
//	E = C*sin(KneePos);
//	G = A / E;
//	F = (B + D) / E;
//
//	if ((KneePos <= Deg2Rad*0.5) && (KneePos >= -Deg2Rad*0.5)){			//DEFINIRE MEGLIOUN RANGE
//		JT.Theta1 = acos(A / (B + C));
//	}
//	else if ((G>sqrt(1 + pow(F, 2))) || (G<-sqrt(1 + pow(F, 2))))
//	{
//		printf("Case: 1+F^2-G^2 < 0; necessary an implementation");
//		Sleep(5000);
//	}
//	else {
//		
//		Theta1s1 = asin((-G + F*sqrt(pow(F, 2) - pow(G, 2) + 1)) / (1 + pow(F, 2)));
//		Theta1s2 = asin((-G - F*sqrt(pow(F, 2) - pow(G, 2) + 1)) / (1 + pow(F, 2)));
//
//		int  count = 0, flagTh1_1 = 0, flagTh1_2 = 0;
//
//		if ((Theta1s1 >= Deg2Rad*0) && (Theta1s1 < Deg2Rad*150))		// METTERE IN RAISNATI
//		{
//			count++;
//			flagTh1_1 = 1;
//		}
//		if ((Theta1s2 >= Deg2Rad*0) && (Theta1s2< Deg2Rad*150))		// METTERE IN RAISNATI
//		{
//			count++;
//			flagTh1_2 = 1;
//		}
//		if (count == 2)
//		{
//			printf("Th1.1 e th1.2 both in the range!!!");
//			Sleep(5000);
//		}
//		if (flagTh1_1 == 1)
//			{ JT.Theta1 = Theta1s1; }
//		else if (flagTh1_2 == 1) 
//			{ JT.Theta1 = Theta1s2; }
//		else
//		{
//			printf("Error!!!");
//			Sleep(5000);
//		}
//	}
//
//	JT.Theta2 = JT.Theta1 + KneePos;
//
//	JT.Torque_Hip = HB.l_torso*cos(JT.Theta3)*HB.F_torso;
//	JT.Torque_Knee = HB.F_torso*(HB.l_torso*cos(JT.Theta3) + HB.l_thigh*cos(JT.Theta2)) + (HB.F_pelvis*HB.l_thigh + HB.F_iTmotor*HB.l_iTmotor + HB.F_thigh*HB.l_thigh_a)*cos(JT.Theta2);
//	JT.Torque_Ankle = HB.F_torso*(HB.l_torso*cos(JT.Theta3) + HB.l_thigh*cos(JT.Theta2) + HB.l_shank*cos(JT.Theta1)) + HB.F_pelvis*(HB.l_thigh*cos(JT.Theta2) + HB.l_shank*cos(JT.Theta1)) + HB.F_iTmotor*(HB.l_iTmotor*cos(JT.Theta2) + HB.l_shank*cos(JT.Theta1)) + HB.F_thigh*(HB.l_thigh_a*cos(JT.Theta2) + HB.l_shank*cos(JT.Theta1)) + cos(JT.Theta1)*(HB.F_ATIsensor*HB.l_ATIsensor + HB.F_shank*HB.l_shank_a);
//
//	if ((abs(JT.Torque_Ankle) - abs(Footsensor_AnkleTorque)) > 5)
//	{
//		//printf("Ankle torque evaluation is too different from the measured");
//	}
//	
//	return JT;
//}

///////////////////////////////////////////////////////////////////////
int _tmain(int argc, _TCHAR* argv[])
{
	
	p_HBparam = new HumanBodyParam(0);//0 for man, 1 for woman
	if(p_HBparam->getGender()==0)
	{
		printf("HumanBodyParam created, gender chosen: MALE \n");
	}
	else
	{
		printf("HumanBodyParam created, gender chosen: FEMALE \n");
	}

	p_HDynM=new HumanDynModel();

	pFile = fopen("../log/logData.txt","w");
	fclose(pFile);

	////////////////////////////////
	#ifdef IMU_AHRS_SENSOR
	errorCode = vn100_connect(&vn100, COM_PORT, BAUD_RATE);

	if (errorCode != VNERR_NO_ERROR)
	{
		printf("\n\n Error encountered when trying to connect to the IMU sensor.\n");
		Sleep(5000);
		return 0;
	}

	/* Configure the VN-100 to output asynchronous data. */
	errorCode = vn100_setAsynchronousDataOutputType(&vn100, VNASYNC_VNQTN, true);
	Sleep(5000);
	#endif

	#ifdef IMU_LPMS_USBAL
	//ImuData d;
	pData_imu_lpms = new ImuData();

	// Gets a LpmsSensorManager instance
	p_LpmsSensorManager = LpmsSensorManagerFactory();

	// Connects to LPMS-B sensor with address 00:11:22:33:44:55 
	//LpmsSensorI* lpms = manager->addSensor(DEVICE_LPMS_B, "00:11:22:33:44:55");//example
	p_LpmsSensor = p_LpmsSensorManager->addSensor(DEVICE_LPMS_U, "AM01DDD2");//cf LpmsDefinition.h: #define DEVICE_LPMS_U 	1 // LPMS-CU (USB)
	Sleep(5000);																  //To get device id: run LpmsControl application
	#endif


	//////###################################################
	
	//// simulation in now running
	//simulationRunning = true;

	Timer2Clock = new cPrecisionClock();
	mainClock = new cPrecisionClock();
	runClock = new cPrecisionClock();

	int res=0;
	int mytemp;
		
	pid_off[0]=0;
	bool start=false;
	starTime = 0;
	

//--------------  Initialise Library and UDP Comms
	// init the comm library
	InitLibrary(MAX_BOARDS);
	// reset all possible active sockets
	CloseSockets();	
	// Open UDP connection 
	res=connectUDP();
	//Sleep(5000);

	// scan for motor controllers
	//res = scanSystem();
	if ((res = scanSystem()) < 0)
	{
		Sleep(2000);
		//exit(0);
	}
	else
		noActiveBoards = res;
	// connect to motor controllers
	connectTCP();
	
	// load joint data (limits, pid gains etc)
	// loadJointDataFromDrivers();

	// initialize joint variables (calibration etc)
	// initJoints();
	
	// load the trajectory from file(this is for ALex)
	// importTrajectory();

	//for(int b=FIRST_LBODY_BOARD;b<LAST_LBODY_BOARD;b++){
	//	setBroadCastRate(b,BCAST_RATE*1000);    //1 = 0.0005sec
	//	setBroadCastPolicy(b,BCAST_POLICY);    
	//} 

		//setBroadCastRate(1,BCAST_RATE);    //Main Motor Board --> 1
		//setBroadCastPolicy(1,BCAST_POLICY); 

	#ifdef KNEEMOTOR_OLD
	setBroadCastPolicy(0,BCAST_POLICY);				//Motor Board--> 0
	setExtraBroadCastPolicy(0, EXTRA_BCAST_POLICY);
	setBroadCastRate(0, BCAST_RATE);    
	#endif

#ifdef KNEEMOTOR_NEW
	setBroadCastPolicy(0,BCAST_POLICY);				//Motor Board--> 0
	setExtraBroadCastPolicy(0, EXTRA_BCAST_POLICY);
	setBroadCastRate(0, BCAST_RATE);    
#endif

	#ifdef FOOT_SENSOR
	setBroadCastPolicy(2, BCAST_POLICY_FT);//iit ftsensot
	setBroadCastRate(2, BCAST_RATE);    //iit ftsensot
	#endif	

	//-------------------------startPositionControl();
	//----------------------------moveToHome();
	
	// wait for few secs
	int mType = GetMotorType(1);
	int newmType = mType | 16;
	newmType = newmType ^ 16;//(mType | 
	 r= SetMotorType(1, (char)newmType);
	
	Sleep(1000);

	//SetAnalogInputs(1, 8);
	//	system("cls");

	//desVel[0]=2000;
	desVel[0]=500;
	desVel[1]=500;
	r=SetDesiredVelocity(desVel);
	//stopPositionControl();
	
	// get simulation run time
	startSimTime = timeObject.getSysTime();
	// run until the defined sim time
	int count=0;

	int bytesReceived=0;
	int boardNo;
	int pktRead=0;

	ClearScreen();

	//declaration of Thread
	cThread* mainThread = new cThread();
	mainThread->set(updateLoop, CHAI_THREAD_PRIORITY_HAPTICS);             //updateLoop,updateLoop,updateLoop,
		
	//while(curSimTime<SIMULATION_TIME && !stopSim)
	while(!stopSim)
	{
		// get the time at thr start of the loop
		//startLoopTime = timeObject.getSysTime();
		curSimTime=timeObject.getSysTime()-startSimTime;
		//sT[loop]=curSimTime;

		loop++;
		Sleep(50);
		//if(!TogglePrint) printf("Frequency = %0.1f Hz\n",1/Timer2);
	}
	closeSim();
	
	//////################################################### Sensor IMU_AHSR
	
	#ifdef IMU_AHRS_SENSOR
	errorCode = vn100_disconnect(&vn100);

	if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error encountered when trying to disconnect from the sensor.\n");
		Sleep(2000);
		return 0;
	}
	#endif

	//////################################################### 

	for(int b=0;b<MAX_BOARDS;b++)
		stopBroadCast(b);     
	Sleep(500);
	// clsoe the network sockets
	CloseSockets();
	Sleep(3000);
	// wait for few seconds
	sampleloop = loop;
	// Save data to file
	//saveData();

	delete p_HBparam;
	delete p_HDynM;
	delete runClock;
	delete Timer2Clock;
	delete mainClock;

	#ifdef IMU_LPMS_USBAL
	// Removes the initialized sensor
	p_LpmsSensorManager->removeSensor(p_LpmsSensor);
	//manager->removeSensor(lpms);

	// Deletes LpmsSensorManager object 
	//delete manager;
	delete p_LpmsSensorManager;
	delete pData_imu_lpms;

	#endif
	return 0;
}

void updateLoop(void)
{
	double time1LoopHap = 0.0;
	int chThread=0;
	char dataTosend[128];
	double looptime = 0;
	// status of the main simulation haptics loop
	bool simulationRunning = false;

	looptime = (float)(BCAST_RATE)*0.0005;//s

	//Bias FTsensor foot
	#ifdef FOOT_SENSOR
		CalibrateOffsets(3);
	#endif
	
	Timer2Clock->reset();
	Timer2Clock->start();

	mainClock->reset();
	mainClock->start();

# ifdef ATISENSOR
	//Setup the ATI sensor
	devMeasurementFns ATI_FTSensor; //9/3/2013  		
	UDP_Communication UDP_ATI; //9/3/2013  		
	UDP_ATI.InitializeWinSock(); //9/3/2013  
#endif


	simulationRunning = true;

	
while(simulationRunning)
{	
		//Tsine=Timer2Clock->getCurrentTimeSeconds();
		Timer2=mainClock->getCurrentTimeSeconds();

	if (Timer2 >= looptime) //1ms loop
	{

		mainClock->reset();
		mainClock->start();

		//////################################################### Sensor IMU_AHSR

		#ifdef IMU_AHRS_SENSOR

		//Lorenzo
		//errorCode = vn100_getCurrentAsyncData(&vn100, &dataa);		/*The library is handling and storing asynchronous data by itself. Calling this function retrieves the most recently processed asynchronous data packet. */

		///*errorCode = vn100_getYawPitchRoll(&vn100, &ypr);
		//errorCode = vn100_getQuabternion(&vn100, &Quat);*/

		//quat_ahrs[0] = dataa.quaternion.x;
		//quat_ahrs[1] = dataa.quaternion.y;
		//quat_ahrs[2] = dataa.quaternion.z;
		//quat_ahrs[3] = dataa.quaternion.w;
		//
		//VN_Quat2Euler132(quat_ahrs, Euler231);
		//Euler231[2] = abs(Euler231[2]);

		////////////////////////////////
		//io
		errorCode = vn100_getQuaternion(&vn100, &quat);

		quat_ahrs[0] = quat.x;
		quat_ahrs[1] = quat.y;
		quat_ahrs[2] = quat.z;
		quat_ahrs[3] = quat.w;

		//VN_Quat2Euler213(quat_ahrs,euler213);
		//euler213[0]=first rotation, here about Y
		//euler213[1]=second rotation, here about X
		//euler213[2]=third rotation, here about Z

		VN_Quat2Euler312(quat_ahrs, euler312_ahrs);//Takes as arg a quat as x,y,z,w (cf example in vn100_windows_basic.c)
		//Numbers are associated with axis: 1=X, 2=Y, 3=Z (cf VN_math.c)
		//312=sequence of rotations
		//euler[0]=rotation about 3==Z
		//euler[1]=rotation about 1==X
		//euler[2]=rotation about 2==Y
		//Hypothesis: sequence of rotation s.t. 312==first about Y, then about 1, then about 3
		//We're interested into rotation about Y st it is not influenced by rotations about other axis, so it should be the first rotation
		
		//If IMU attached on the front of the torso
		double l_offset = 4;//deg
		p_HDynM->set_theta3(-euler312_ahrs[2] + M_PI+ g_DEG2RAD(l_offset));
		//If IMU attached on the back
		//p_HDynM->set_theta3(euler312_ahrs[2]+g_DEG2RAD(l_offset));

		#endif

		#ifdef IMU_LPMS_USBAL
			// Checks, if conncted
			if ( p_LpmsSensor->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED && p_LpmsSensor->hasImuData() )
			{
				/*lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
				lpms->hasImuData()
				) {
*/
				// Reads quaternion data
				//d = lpms->getCurrentData();
				*pData_imu_lpms = p_LpmsSensor->getCurrentData();//sets pData_imu_lpms->q as w,x,y,z - cf C:\OpenMAT\OpenMAT-1.3.4\examples\simple\bin_cmake
				
				//Check order quaternion lpms - fnt
				quat_lpms[0] = pData_imu_lpms->q[1];//qx
				quat_lpms[1] = pData_imu_lpms->q[2];//qy
				quat_lpms[2] = pData_imu_lpms->q[3];//qz
				quat_lpms[3] = pData_imu_lpms->q[0];//qw

				//VN_Quat2Euler312(quat_lpms, euler213_lpms); //Takes as arg a quat as x,y,z,w (cf example in vn100_windows_basic.c)
				VN_Quat2Euler213(quat_lpms, euler213_lpms); //Takes as arg a quat as x,y,z,w (cf example in vn100_windows_basic.c)

				//Set HumanDynModel class member
				double l_offet_imuLpms_deg = 4;//deg
				p_HDynM->set_theta1(euler213_lpms[0] + M_PI+g_DEG2RAD(l_offet_imuLpms_deg));//euler213_lpms[0] + M_PI: rotation about y_sensor such that when IMU upright: 90°, when IMU bended horizontal:0°

				// Shows data
				/*printf("Timestamp=%f, qW=%f, qX=%f, qY=%f, qZ=%f\n",
					d.timeStamp, d.q[0], d.q[1], d.q[2], d.q[3]);*/
			}
		#endif


		////################################################### 

		t=Timer2;
		
		#ifdef KNEEMOTOR_OLD
		getBroadCastData(noActiveBoards);		// MOtor
		#endif
		
		#ifdef KNEEMOTOR_NEW
		getBroadCastData(noActiveBoards);		// MOtor
		alpha2 = Rad2Deg*(joint[0].pos + A0 - Ashift)*0.00001;
		Knee_RelAngle = g_DEG2RAD(180 - alpha2);
		#endif

		#if defined(KNEEMOTOR_NEW) && defined(IMU_LPMS_USBAL)
			p_HDynM->set_theta2(p_HDynM->get_theta1() + M_PI - Knee_RelAngle);
		//double l_theta1 = euler213_lpms[0] + M_PI;
		#endif

		#ifdef FOOT_SENSOR
			getBroadCastFTData(noActiveBoards);	//FT_Sensor Foot Plate
		
			//Add force offset (force recorded when FTplate biased when user standing (to get correct torque))
			//for (int it = 0; it < 3; it++)
			//{
				//FTfoot_data[id_FTfoot].FT[it] += Ffoot_offset[id_FTfoot][it];
			//}
			//Compute ankle torque from sensor reading
			//AnkleTorque_sensor = sgn(FTfoot_data[id_FTfoot].FT[3])*sqrt(pow(FTfoot_data[id_FTfoot].FT[3] / divs, 2) + pow(FTfoot_data[id_FTfoot].FT[4] / divs, 2));
		
			//Compute from FTreading
			AnkleTorque_sensor = FTfoot_data[id_FTfoot].FT[3]/divs*cos(g_DEG2RAD(21))+ FTfoot_data[id_FTfoot].FT[4] / divs*sin(g_DEG2RAD(21));
			AnkleTorque_sensor = -AnkleTorque_sensor; //To fit with positive ankle convention - when increasing weight forward torqueAnkle should be positive (and increase), while the raw reading is negative (and abs value increases)

			//Filter
			for (int it = 0; it < size_bufferAT-1; it++)
			{
				buffer_ankleTorque[it] = buffer_ankleTorque[it+1];
			}
			buffer_ankleTorque[size_bufferAT-1] = AnkleTorque_sensor;
			double l_sum_bufferData = 0;
			for (int it = 0; it < size_bufferAT; it++)
			{
				l_sum_bufferData += buffer_ankleTorque[it];
			}
			AnkleTorque_sensor_filt = l_sum_bufferData / size_bufferAT;
			p_HDynM->set_ankleTorque(AnkleTorque_sensor,AnkleTorque_sensor_filt);

			//p_HDynM->set_weight((double)(FTfoot_data[id_FTfoot].FT[2] + Ffoot_offset[id_FTfoot][2]) / divs);
		#endif

		#ifdef ATISENSOR
		ATI_FTSensor.ReadFTsensorData();
		ATI_FTSensor.GetFTData(FTData);
		#endif
		
	////###################### ATI F/T Sensor Offset, mounted on square shaft ########################################

		FTData[0] = FTData[0] - 14.59;//ATI Fx
		FTData[1] = FTData[1] + 7.46;//ATI Fy
		FTData[2] = FTData[2] - 15.05;//ATI Fz
		FTData[3] = FTData[3] + 0.479;//ATI Tx
		FTData[4] = FTData[4] + 0.162;//ATI Ty
		FTData[5] = FTData[5] - 0.064;//ATI Tz

	//###################### ATI F/T Sensor Offset value on HLR screw ########################################

		FTData[0] = FTData[0] + 12.5;//ATI Fx
		FTData[1] = FTData[1] + 28.0;//ATI Fy
		FTData[2] = FTData[2] + 0.25;//ATI Fz
		FTData[3] = FTData[3] + 1.053;//ATI Txy
		FTData[4] = FTData[4] - 0.300;//ATI Ty
		FTData[5] = FTData[5] - 0.800;//ATI Tz

	//###################### Joint Torque Estimation ########################################

		#ifdef IMU_AHRS_SENSOR
		if (Posture_Torque_Est == ON){
			//AnkleTorque_sensor = sgn(FTfoot_data[id_FTfoot].FT[3])*sqrt(pow(FTfoot_data[id_FTfoot].FT[3] / divs, 2) + pow(FTfoot_data[id_FTfoot].FT[4] / divs, 2));
			//HBParfunction(HB,0,0);
			//KneeMotorPos = Deg2Rad*alpha2;
			//JT = JointTorquefunction(JT, HB, Euler231[2], AnkleTorque_sensor, KneeMotorPos);

			//double l_theta3=Euler231[2];//CHECK
			//p_HDynM->compute_jointTorques( p_HBparam, l_theta3,AnkleTorque_sensor,(double)Knee_RelAngle);

		}
		#endif
	//////////////////////////////////////////////////////////////////////////////////////////////////////

		if (flg_isRunning)
		{
			runTimer = runClock->getCurrentTimeSeconds();
			//Test calibration procedure 
			//double l_theta1 = g_DEG2RAD(theta1_calib_deg);
			//double l_theta2 = g_DEG2RAD(theta1_calib_deg) + M_PI - Knee_RelAngle;
			//double l_theta3 = Euler231[2];
			//double l_Fz = (double)(FTfoot_data[id_FTfoot].FT[2] + Ffoot_offset[id_FTfoot][2]) / divs;

			//p_HBparam->calibrate(l_theta1, l_theta2, p_HDynM->get_theta3(), AnkleTorque_sensor_filt, l_Fz);
			//printf("Gender: %i \n",p_HBparam->getGender());
			//printf("Mass computed: %g kg \n,",p_HBparam->get_mass());
			//printf("Height computed: %g m \n",p_HBparam->get_height());

			//Keep here
			//p_HDynM->compute_jointTorques(p_HBparam, l_theta3, l_ankleTorque, g_DEG2RAD(l_knee_relativeAngle_deg));
			//p_HDynM->compute_jointTorques(p_HBparam, euler312_ahrs, AnkleTorque_sensor_filt, Knee_RelAngle);

			//p_HDynM->compute_torqueKnee(p_HBparam, AnkleTorque_sensor_filt);//Note: no need to feed ankleTorque here, we can use this
			//p_HDynM->compute_torqueKnee(p_HBparam);
			p_HDynM->compute_torqueKnee_withLoad(p_HBparam, 20);


			//printf("Torque knee 2: %g \n",p_HDynM->get_torqueKnee());//check that the 2 fnt give same result, then use this one

			//printf("\n torque knee: %g \n", p_HDynM->get_torqueKnee());//Should be negative and 
			//printf("ankle torque: %g \n", AnkleTorque_sensor_filt);//Should be positive and increase when bending, keeping upperbody same posture
																   //printf("TorqueKnee - TorqueAnkle: %g \n", l_A);
			//printf("theta1 [deg]: %g \n", g_RAD2DEG(p_HDynM->get_theta1()));

			//log
			pFile = fopen("../log/logData.txt", "a");
			if (pFile != NULL)
			{
				//fprintf(pFile, "%g \t %g \t %g \t %g \t %g \n", runTimer, g_RAD2DEG(p_HDynM->get_theta1()),p_HDynM->get_torqueAnkle(), p_HDynM->get_torqueAnkle_filtered(),p_HDynM->get_torqueKnee());
				//fprintf(pFile, "%g \t %g \t %g \t %g \t %g \t %g \t %g \n", runTimer, p_HDynM->get_torqueAnkle(), p_HDynM->get_torqueAnkle_filtered(), p_HDynM->get_torqueKnee(), g_RAD2DEG(p_HDynM->get_theta1()), g_RAD2DEG(p_HDynM->get_theta2()), g_RAD2DEG(p_HDynM->get_theta3()));
				fprintf(pFile, "%g \t %g \t %g \t %g \t %g \t %g \t %g \t %f \n", runTimer, p_HDynM->get_torqueAnkle(), p_HDynM->get_torqueAnkle_filtered(), p_HDynM->get_torqueKnee(), g_RAD2DEG(p_HDynM->get_theta1()), g_RAD2DEG(p_HDynM->get_theta2()), g_RAD2DEG(p_HDynM->get_theta3()), (float)(FTfoot_data[id_FTfoot].FT[2] + Ffoot_offset[id_FTfoot][2]) / divs);

				fclose(pFile);
			}

		}//if()

		

		////////////////////////////////////////////////////////////////

	printTimer++;		
	double printTime = (double)(printTimer)*looptime;
	if (printTimer >10) //0.1s interface and printing loop
	{
	printTimer=0;
	int mytemp=0;


	if (_kbhit())
	{

		chThread = _getch();

			switch (chThread)
			{

				case 'p':// toggle printing on scree // Stamp frequency
				if (TogglePrint>0) TogglePrint=0;
				else TogglePrint=1;	
				ClearScreen();
				break;

				case 'q':
					positionControl = OFF;
					sinewaveON = OFF;
					torqueControl = OFF;
					costVelON = OFF;
					rampON = OFF;
					stopControl();	// startPositionControl and stopPositionControl are responsible for green light flashing
					setTorqueCntrlFlag(0, 0);

					stopSim=1;
					// stop the simulation
					simulationRunning = false;
				break;

				case 'w':
					//GetPidGains(0,1,gains);
					//gotoxy(1,2); printf("p=%d, i=%d, d=%d,\n",gains[0],gains[1],gains[2]);
					//GetPidGains(1,1,gains);
					//gotoxy(1,3); printf("p=%d, i=%d, d=%d,\n",gains[0],gains[1],gains[2]);
					saveData();
				break;
				
				case 'b':			//Bias the Motor Torque sensor 
					CalibrateOffsets(1);

					//FTfoot
					getBroadCastFTData(noActiveBoards);
					//Set force data at bias time
					for (int it = 0; it < 3; it++)
					{
						//Ffoot_offset[id_FTfoot][it] = FTfoot_data[id_FTfoot].FT[it];//Works when biasing for 1st time but then wrong
						Ffoot_offset[id_FTfoot][it] = FTfoot_data[id_FTfoot].FT[it]+ Ffoot_offset[id_FTfoot][it];
						//Ffoot_offset[id_FTfoot][it] = FTfoot_data[id_FTfoot].filt_FT[it];
					}
					//printf("Non filtered F reading: %g \t %g \t %g \n", FTfoot_data[id_FTfoot].FT[0] / divs, FTfoot_data[id_FTfoot].FT[1] / divs, FTfoot_data[id_FTfoot].FT[2] / divs);
					//printf("filtered F reading: %g \t %g \t %g \n", FTfoot_data[id_FTfoot].filt_FT[0]/divs, FTfoot_data[id_FTfoot].filt_FT[1] / divs, FTfoot_data[id_FTfoot].filt_FT[2] / divs);
					//printf("offsets: %g \t %g \t %g \n", Ffoot_offset[id_FTfoot][0] / divs, Ffoot_offset[id_FTfoot][1] / divs, Ffoot_offset[id_FTfoot][2] / divs);
					//Calibrate everything
					CalibrateOffsets(3); //why 3? BoardId supposed to be 2

				break;

				case 'n':			//Bias the ATI_FT sensor 
					//CalibrateOffsets(1);
					printf("Case n \n");
					#ifdef ATISENSOR
					ATI_FTSensor.BiasFTsensor();
					#endif
					//joint[0].anInput1_Offset = joint[0].anInput1;
					//joint[0].anInput2_Offset = joint[0].anInput2;
					break;

				case 't':		//Sinewave reference
					if ((positionControl == ON) && (torqueControl == OFF)) // to avoid to take command when the StartControl is END !!!
					{
						rampON = OFF;
						costVelON = OFF;
						if (sinewaveON == OFF)
						{
							sinewaveON = ON;
							//sineZero = curSimTime;
							sinecounter = 0;  //78500 
							joint[0].pos = joint[0].pos -Ashift;
							InitialPos = (double)joint[0].pos;
							/*	//stopBroadCast(0);
								//int mType = GetMotorType(1);
								//int newmType = mType | 16;
								//int mtype = SetMotorType(1,newmType);
								// mtype = SetMotorType(1, newmType);
								*/
						}
						else
							sinewaveON = OFF; // to stop the sinewave positopn reference
					}
						break;

				case 'y':		//Constant velocity
					if ((positionControl == ON) && (torqueControl == OFF)) // to avoid to take command when the StartControl is END !!!
					{
						rampON = OFF;
						sinewaveON = OFF;
						if (costVelON == OFF)
						{
							costVelON = ON;
							sinecounter = 0;  //78500 
							InitialPos = (double)joint[0].pos;
							AngVel = -AnuglarVelocity / 0.044; // 10*urad/ms	// 4.4 is a coefficient factor find out with test!!  ---> X*10^(-4) rad/s 
						
						}
						else
							costVelON = OFF; // to stop the sinewave positopn reference
					}
					break;

				case 'u'://start recording
				{
					printf("flg_isRunning set to true \n");
					runClock->reset();
					runClock->start();
					flg_isRunning = true;				
				}break;
				case 'r'://stop
				{
					printf("flg_isRunning set to false \n");
					flg_isRunning = false;
					runClock->stop();
				}break;
				
				case 'j':// Force Sine wave: Initiates damper internal sinusoidal force reference 50N p-p
					if (damperSinewaveON==OFF)
						damperSinewaveON=ON;						
					else
						damperSinewaveON=OFF;

					mytemp=desPosThred[0];
					desPosThred[0]=3;	// the value 3 is a flag in the firmware to initiate a deflection sinewave of amplitude 5N
					r=SetDesiredPosition(desPosThred);
					desPosThred[0]=mytemp;
					break;
				case 'm':// Force Sine wave: Initiates damper internal sinusoidal force reference 50N p-p
					if (damperPosSinewaveON==OFF)
						damperPosSinewaveON=ON;						
					else
						damperPosSinewaveON=OFF;

					mytemp=desPosThred[0];
					desPosThred[0]=4;	// the value 3 is a flag in the firmware to initiate a deflection sinewave of amplitude 5N
					r=SetDesiredPosition(desPosThred);
					desPosThred[0]=mytemp;
					break;

				case 'k'://ramp reference
					sinewaveON = OFF;
					if (rampON == OFF)
						rampON = ON;
					else
						rampON = OFF;    // to deactivate ramp positopn reference
					break;

				case 'l'://constant velocity
					if ((positionControl == ON) && (torqueControl == OFF))  // to avoid to take command when the StartControl is END !!!
					{
						rampON = OFF;
						sinewaveON = OFF;
						if (costVelON == OFF)
						{
							costVelON = ON;
							sinecounter = 0;  //78500 
							InitialPos = (double)joint[0].pos;
							AngVel = +AnuglarVelocity / 0.044; // 10*urad/ms	// 4.4 is a coefficient factor find out with test!! ---> X*10^(-4) rad/s 

						}
						else
							costVelON = OFF; // to stop the sinewave positopn reference
					}
					break;
					// this button resets the minimum and contact positions as part of a sequence button press:
					//first press 'o' wait then press 'i'
					//sets also offsets for no contact force and position
					//mytemp = desPosThred[0];
					//desPosThred[0] = 4;
					//r = SetDesiredPosition(desPosThred);
					//printf("\n Direct position Control \n");
					//desPosThred[0] = mytemp;
					//printf("\n Desired position activated \n");
					//desPosThred[0]=0;
					break;					

				case 'z':   // move of -5 degree the motor shaft
					if (((positionControl == ON)) && (torqueControl == OFF))  // to avoid to take command when the StartControl is END !!!
					{
						if (sinewaveON == ON)
						{
							//AmpliduteUD -= Amplincrem;  // A value of sinewave goes down of 2°
						}
						else if (rampON == ON)
						{
							/*if (FunctionOffset >= 5000)
							FunctionOffset -= 5000;*/
						}
						else
						{
							if (joint[0].pos >= (-A0 + Ashift ))		
							{
								desPosThred[0] = joint[0].pos - Amplincrem;  // in this way when you turn all on for the first time it doesnot start to count from zero!
								r = SetDesiredPosition(desPosThred);
							}
						}
					}
				break;

				case 'x':	// move of +5 degree the motor shaft
					if (((positionControl == ON)) && (torqueControl == OFF))  // to avoid to take command when the StartControl is END !!!
					{
						if (sinewaveON == ON)
						{
							//AmpliduteUD += Amplincrem;  // A value of sinewave goes up of 2°
						}
						else if (rampON == ON)
						{
							/*if (FunctionOffset >= 5000)
								FunctionOffset -= 5000;*/
						}
						else    
						{
							if (joint[0].pos <= (A0 + Ashift ))		
							{
								desPosThred[0] = joint[0].pos + Amplincrem;  // in this way when you turn all on for the first time it doesnot start to count from zero!
								r = SetDesiredPosition(desPosThred);
							}
						}
					}
				break;
				case '1'://Calibration HumanBodyParam
				{
					//Needs #define KNEEMOTOR_NEW, #define IMU_AHRS_SENSOR, #define FOOT_SENSOR, Posture_Torque_Est == ON
					//To get Knee_RelAngle and Euler231[2], AnkleTorque_sensor and Vertical force sensor
					
					//Testing
					theta1_calib_deg = 60;
					//double l_knee_relativeAngle_deg=110;//90
					//double l_theta2=g_DEG2RAD(theta1_calib_deg+180- l_knee_relativeAngle_deg);//CHECK
					//double l_theta3=g_DEG2RAD(65); //upright
					//double l_theta3=g_DEG2RAD(70); //upright
					//double l_ankleTorque=(double)AnkleTorque_sensor;//sign?
					double l_ankleVerticalForce=840/2;//550 N
					//double l_ankleVerticalForce = 750 / 2;//550 N

					//Get param
					double l_theta1 = euler213_lpms[0]+M_PI;
					//double l_theta2=g_DEG2RAD(theta1_calib_deg)+M_PI-Knee_RelAngle;//CHECK
					double l_theta2 = l_theta1 + M_PI - Knee_RelAngle;//CHECK
					double l_theta3=-euler312_ahrs[2] + M_PI;//imu on front
					//double l_theta3 = -euler312_ahrs[2] - M_PI+g_DEG2RAD(6);//imu on front
					//double l_theta3 = euler312_ahrs[2];//imu on back
					//double l_theta3 = euler312_ahrs[2]-g_DEG2RAD(15);//imu on back
					
					
					//double l_ankleTorque=AnkleTorque_sensor;//
					//double l_ankleVerticalForce=(double)FTfoot_data[id_FTfoot].FT[2] / divs;//N
					//double l_ankleVerticalForce=(double)(FTfoot_data[id_FTfoot].FT[2] + Ffoot_offset[id_FTfoot][2]) / divs;

					 //TEST and shift as private
					 //p_HBparam->compute_bodyMass(l_ankleVerticalForce);
					 //p_HBparam->compute_linksMasses();
					 //p_HBparam->compute_height( g_DEG2RAD(theta1_calib_deg), l_theta2,l_theta3, l_ankleTorque);
					 //p_HBparam->set_height(1.7); //test
					 //p_HBparam->compute_linksLengths();
					 
					 //	USER FNT	 
					 //p_HBparam->calibrate(g_DEG2RAD(theta1_calib_deg), l_theta2,  l_theta3, l_ankleTorque, l_ankleVerticalForce);
					 
					//p_HBparam->calibrate(g_DEG2RAD(theta1_calib_deg), l_theta2, l_theta3, AnkleTorque_sensor_filt, l_ankleVerticalForce);
					 //p_HBparam->calibrate(l_theta1, l_theta2, l_theta3, AnkleTorque_sensor_filt, l_ankleVerticalForce);
					 p_HBparam->calibrate(p_HDynM->get_theta1(),p_HDynM->get_theta2(),p_HDynM->get_theta3(), AnkleTorque_sensor_filt, l_ankleVerticalForce);


					 printf("\n");
					 printf("Mass computed: %g kg \n,", p_HBparam->get_mass());
					 printf("theta1: %g \t Knee_RelAngle: %g \t theta2: %g \t theta3: %g \n", g_RAD2DEG(l_theta1),g_RAD2DEG(Knee_RelAngle),g_RAD2DEG(l_theta2),g_RAD2DEG(l_theta3));
					 printf("AnkleTorque_sensor_filt: %g \n", AnkleTorque_sensor_filt);
					 printf("l_ankleVerticalForce: %g \n",l_ankleVerticalForce);
					 //printf("Gender: %i \n",p_HBparam->getGender());
	
					 printf("Height computed: %g m \n",p_HBparam->get_height());
					
				}break;
				case '2':	
				{
					//Normally: Compute mass and height from calib procedure (p_HBparam->calibrate)
					//Here test, manual setting Set links param from height and mass
					double l_mass = 85.5;//55;
					double l_height = 1.83;//1.70;
					bool l_gender = 2; //1;
					p_HBparam->set_userParam(l_height, l_mass, l_gender);
					p_HBparam->compute_linksMasses();
					p_HBparam->compute_linksLengths();

					//Test: angles
					//double l_knee_relativeAngle_deg = 90;//90
					//double l_theta2 = g_DEG2RAD(theta1_calib_deg + 180 - l_knee_relativeAngle_deg);//CHECK
					//double l_theta3 = g_DEG2RAD(90.0); //upright
					//double l_ankleTorque = (double)AnkleTorque_sensor;
					//double l_theta1=

					//Test HumanDynModel
					//p_HDynM->compute_jointTorques(p_HBparam, l_theta3, l_ankleTorque, g_DEG2RAD(l_knee_relativeAngle_deg));
					//p_HDynM->compute_jointTorques(p_HBparam, euler312_ahrs, AnkleTorque_sensor_filt, Knee_RelAngle);
					//p_HDynM->compute_torqueKnee(p_HBparam, p_HDynM->get_theta1(), AnkleTorque_sensor_filt);
					//printf("Torque knee 1: %g \n",p_HDynM->get_torqueKnee());
					//p_HDynM->compute_torqueKnee(p_HBparam, AnkleTorque_sensor_filt);
					p_HDynM->compute_torqueKnee(p_HBparam);
					//printf("Torque knee 2: %g \n",p_HDynM->get_torqueKnee());//check that the 2 fnt give same result, then use this one

					printf("\n torque knee: %g \n", p_HDynM->get_torqueKnee());//Should be negative and 
					printf("ankle torque: %g \n", AnkleTorque_sensor_filt);//Should be positive and increase when bending, keeping upperbody same posture
																 //printf("TorqueKnee - TorqueAnkle: %g \n", l_A);
					printf("theta1 [deg]: %g \n", g_RAD2DEG(p_HDynM->get_theta1()));


				}break;

				case 'v':	
							// free space
					break;

				case 's':		// Fake Knee - Gravity Compensation
					if ((positionControl == OFF) && (torqueControl == ON))
					{
						Motor_GravtyCompFlag = OFF;
						if (GravtyCompFlag == OFF)
						{
							GravtyCompFlag = ON;
							printf("\n \n  GravtyComp is ON \n ");
						}
						else
							GravtyCompFlag = OFF;
							printf("\n \n  GravtyComp is OFF \n ");
					}
				break;

				case 'a':		// Human Knee - Gravity Compensation
					//controlIsOn = 0;
					#ifdef IMU_AHRS_SENSOR
					GravtyCompFlag = OFF;
					if (Posture_Torque_Est == OFF)
					{
						Posture_Torque_Est = ON;
						//printf("\n \n  Knee_GravtyCompFlag is ON \n ");
					}
					else{
						Posture_Torque_Est = OFF;
						//printf("\n \n  Motor_GravtyComp is OFF \n ");
					}
					#endif
					/*if (Motor_GravtyCompFlag == OFF)
					{
						Motor_GravtyCompFlag = ON;
						printf("\n \n  Motor_GravtyComp is ON \n ");
					}
					else
						Motor_GravtyCompFlag = OFF;
						printf("\n \n  Motor_GravtyComp is OFF \n ");*/	  // old compensation gravity for the fake knee
				break;

				case 'd':		// Set up Gains for Torque Control mode
					torqueControl = ON;
					positionControl = OFF;

					setupTorqueControlPID();		// inside StopContrl() is called
					setTorqueCntrlFlag(0, 1);
				break;

				case 'f':		// Set up Gains for Position Control mode
					torqueControl = OFF; 
					positionControl = ON;
					GravtyCompFlag = OFF;
					Motor_GravtyCompFlag = OFF;

					setTorqueCntrlFlag(0, 0);
					setupPositionControlPID();		// inside StopContrl() is called
					break;

				case 'e':
					if (storeData==ON)
						storeData=OFF;
					else
						storeData=ON;
						deleteData=OFF;
				break;

				case 5://Press CTRL+'e'to delete stored values in data and FT_data
					storeData=OFF;
					deleteData=ON;
					//gotoxy(1,4);
				//	printf("\n ----------------------- Deleting data --------------------------\n\n\n");
					for(int d=0;d<dataCounter;d++)
					{
						for(int i=0;i<12;i++)
							data_motor[d][i]=0;
						for(int i=0;i<6;i++)
							FT_data[d][i]=0;  //PositionMot1, PositionMot2, Force, torque
						
						sT[d]=0;
					}
					dataCounter=0;
					//gotoxy(1,4);
					//printf("\n --------------------------------------------------------------------\n\n\n");
				break;

				case 'g':
					if (torqueControl == OFF)	//in order to avoid problem to move from setupTorqueCntrl to setupPostionCntrl
					{
						desPos[0] = joint[0].pos;
						desVel[0] = 300;
						r = SetDesiredPosition(desPos);
						r = SetDesiredVelocity(desVel);
					}
					startControl(0);		// startPositionControl and stopPositionControl are responsible for green light flashing
					gotoxy(1, 50); printf("\n \n  startPositionControl STARTS \n ");
				break;
				
				case 'h':
					positionControl = OFF;
					sinewaveON = OFF;
					torqueControl = OFF;
					costVelON = OFF;
					rampON = OFF;
					
					stopControl();	// startPositionControl and stopPositionControl are responsible for green light flashing
					gotoxy(1, 50); printf("\n \n  startControl ENDS \n ");
					setTorqueCntrlFlag(0, 0);
				break;
			
				case 'o':
					if (positionControl == ON)  // to avoid to take command when the StartControl is END !!!
					{	
						moveToHome();
						printf("\n Home_mode setted \n");
						startControl(0);
						gotoxy(1, 20); printf("\n \n  startPositionControl STARTS \n ");	
					}
				break;

				case 'i':
					// this button resets the minimum and contact positions as part of a sequence button press:
					//first press 'o' wait then press 'i'
					//sets also offsets for no contact force and position
					mytemp=desPosThred[0];
					desPosThred[0]=2;				
					r=SetDesiredPosition(desPosThred);
					printf("\n Reset position and force \n");
					desPosThred[0]=mytemp;
					//printf("\n Desired position activated \n");
					//desPosThred[0]=0;
					break;

				case ' ':
					//if (MotorSelect==MAINMOTOR)
					//{
					//	MotorSelect=DAMPERMOTOR;
					//	printf("\n DamperMotor selected   \n");
					//}
					//else
					//{
					//	MotorSelect=MAINMOTOR;
					//	printf("\n MainMotor selected    \n");
					//}
					break;
				default:
				break;
			}		

			chThread=0;
				}

				//	printf("Loop: %0.4f\n",Timer2);		
				//time1LoopHap = 0.0;
			//	printf("desPosThred:%*d  sinefreq:%0.6f  rampPos=%*d  rampSTEPthread=%*d \n",7,desPosThred[0],sinefreq, 7,rampPos,7,rampSTEPthread);	
			//	printf("\n rampPos=%*d  rampSTEPthread=%*d \n", 7,rampPos,7,rampSTEPthread);
				//Timer2Clock->reset();
			//	GetKeyboard();
			if (TogglePrint)
				Myprint();
			
			
		}
	
		////////////////SINEWAVE DEFINITION //////////////////////////////////

		if (sinewaveON==ON)
		{
			phi1 = asin(InitialPos/ A0) + 2.0f * M_PI*(sinefreq)*sinecounter;

			desPosThred[0] = (int)(Ashift + A0*sin(2.0f*M_PI*sinefreq*sinecounter + phi1));//  InitialPos
			
				//Ti = 1 / sinefreq;
				//Tchange = (2 + 1 / 4)*Ti;
				//if (sinecounter <= Tchange)
			//		desPosThred[0] = (int)(A0 * sin(2.0f*M_PI*(sinefreq)*sinecounter + phi1));
				//else
				//	desPosThred[0] = (int)(-A0 * sin(2.0f*M_PI*(sinefreq)*sinecounter + phi1));
				
			r=SetDesiredPosition(desPosThred);
			//printf("desPosThred: %*d  sinefreq:%0.6f\n",7,desPosThred[0],sinefreq);	
			//printf("\n rampPos=%*d  rampSTEPthread=%*d   \n", 7,rampPos,7,rampSTEPthread);
			sinecounter += looptime;
		}
		

		////////////////  costVelON DEFINITION //////////////////////////////////

		if (costVelON == ON)
		{
			if ((joint[0].pos <= (A0 + Ashift)) && (joint[0].pos >= (-A0 + Ashift)))		
			{
				desPosThred[0] = InitialPos + AngVel*sinecounter;// sinecounter is ms!
				r = SetDesiredPosition(desPosThred);
				sinecounter += looptime;
			}
			else
			{
				desPosThred[0] = joint[0].pos;	
				r = SetDesiredPosition(desPosThred);
				costVelON = OFF;
			}
			
		}
		
	
		////////////////Torque DEFINITION //////////////////////////////////


		posindex = (joint[0].pos + M_PI * 100000) / Anglestep;
		//TorqueOff = TorqueOffMtr[posindex];
		TorqueOff =   (0.96*joint[0].vel) + TorqueOffMtr[posindex];
		TorqueOffFK = (0.96*joint[0].vel) + TorqueOffMtrFK[posindex];
		GravityComp = 0.95*GravtyC_Mtr[posindex]+582;
		Motor_GravtyComp = Motor_GC_Mtr[posindex];
		
		if ((positionControl == OFF) && (torqueControl == ON))  //Desire torque				//Recent modification...before only: (torqueControl == ON)
		{
			//Destor[0] = 0;
			if (GravtyCompFlag == ON)
			{		
				Destor[0] = 0 - TorqueOffFK - GravityComp;				// I can not remember but probably the GravityComp value were taken going very slowly...like quasi-static
			}
			/*else if (Motor_GravtyCompFlag == ON)
			{
				Destor[0] = 0 - TorqueOff - Motor_GravtyComp;
			}*/
			if (Posture_Torque_Est == ON)
			{
				//if (abs(JT.Torque_Knee*0.03) < 3){
				//	Destor[0] = JT.Torque_Knee;//0  + 1000 * JT.Torque_Knee*0.2;	// mNm !!		- TorqueOff
				//}
				if (abs(p_HDynM->get_torqueKnee()*0.03) < 3){
					Destor[0] = p_HDynM->get_torqueKnee();//0  + 1000 * JT.Torque_Knee*0.2;	// mNm !!		- TorqueOff
				}
				else
				{
					Destor[0] = 3000; //mNm
				}
			}
			else 
			{	
				#ifdef KNEEMOTOR_OLD
				Destor[0] = 0 - TorqueOff; //b +1000; // -GravityComp;
				#endif;
				#ifdef KNEEMOTOR_NEW
				Destor[0] = 0;			// Initial costant Torque Offset
				#endif
			}
			
			int res=SetDesiredTorque(Destor);
			
			//Virtualwafll		ATTENCION -- The Virtualwall creates problems to move from setupTorqueCntrl to setupPostionCntrl!! ....'g'
			
		/*	int Pwallmin = -A0 + Ashift;
			int Pwallmax = A0 - Ashift;

			if (joint[0].pos > Pwallmax)
				{
				DesOffset[0] = -0.5 * (Pwallmax - joint[0].pos);
				SetPidOffset(DesOffset);
				}
			if (joint[0].pos < Pwallmin)
				{
				DesOffset[0] = -0.5 * (Pwallmin - joint[0].pos);
				SetPidOffset(DesOffset);
				}
			*/
		}

		if (storeData==ON)
			{				
			//comment when calibration data is not needed
			data_motor[dataCounter][0] = joint[0].pos;
			data_motor[dataCounter][1] = joint[0].tpos;
			data_motor[dataCounter][2] = joint[0].temp_tpos;
			data_motor[dataCounter][3] = joint[0].req_tpos;
			data_motor[dataCounter][4] = joint[0].tStamp;
			data_motor[dataCounter][5] = joint[0].vel;
			data_motor[dataCounter][6] = joint[0].tor;
			data_motor[dataCounter][7] = TorqueOff;
			data_motor[dataCounter][8] = joint[0].realCurrent;
			data_motor[dataCounter][9] = joint[0].pidOutput;
			data_motor[dataCounter][10] = joint[0].pidError;
//
//
//					data[dataCounter][0] =joint[0].pos;
////					data[dataCounter][1] =joint[1].pos;
//					data[dataCounter][1] =joint[0].TwinActPos;
//					FT_data[dataCounter][0] =joint[0].Force;
//					FT_data[dataCounter][1] =joint[0].Torque;
//					data[dataCounter][2] =joint[1].vel*10;//in mrad/s
//					data[dataCounter][3] =joint[0].TwinVel*10;//in mrad/s
//					data[dataCounter][4] =joint[0].TwinTargPos;
//					fdata[dataCounter][0]=(float)joint[0].DamperRefNormForce/10.0;
//					fdata[dataCounter][1]=(float)joint[0].DamperRefTorque/1000.0;
//					fdata[dataCounter][2]=(float)joint[0].DamperRefDeflection/100000.0;
//					fdata[dataCounter][3]=(float)joint[0].DamperActDeflection/100000.0;
//					fdata[dataCounter][4]=(float)joint[0].tStamp/1000.0;//time from microcontroler in seconds.
//					fdata[dataCounter][5]=(float)joint[0].TwinActPos/100000.0;//in rad
//					fdata[dataCounter][6]=(float)joint[0].TwinVel/100.0;//in rad/s
//					data[dataCounter][5] =joint[0].pidError;//Damper PID
//					data[dataCounter][6] =joint[1].pidError;//Main motor PID
//					data[dataCounter][7] =joint[1].pos;
//
			
////----------------Sensor calibration data storage ------------------					
//					
//					//comment when calibration data is not needed
//					fCalibFTdata[dataCounter][0]=joint[0].pos;
//					fCalibFTdata[dataCounter][1] = joint[0].tpos;
//					fCalibFTdata[dataCounter][1] = joint[0].temp_tpos;
//					fCalibFTdata[dataCounter][1] = joint[0].req_tpos;
//
//					dCalibFTdata[dataCounter][0]=joint[0].anInput1_Adjusted;//Force channel
//					dCalibFTdata[dataCounter][1]=joint[0].anInput2_Adjusted;//Torque channel
					fCalibFTdata[dataCounter][2]=FTData[0];//ATI Fx
					fCalibFTdata[dataCounter][3]=FTData[1];//ATI Fy
					fCalibFTdata[dataCounter][4]=FTData[2];//ATI Fz
					fCalibFTdata[dataCounter][5]=FTData[3];//ATI Tx
					fCalibFTdata[dataCounter][6]=FTData[4];//ATI Ty
					fCalibFTdata[dataCounter][7]=FTData[5];//ATI Tz

//----------------END of Sensor calibration data storage ------------------	

			dataCounter++;
			}

		}//End of 1ms loop

	}
	stopSim=1;

}

int sgn(float num) {
	if (num >= 0)
		return 1;
	else
	{
		return -1;
	}
}

void gotoxy(int column, int line)
{
	COORD coord;
	coord.X = column;
	coord.Y = line;
	SetConsoleCursorPosition(
		GetStdHandle(STD_OUTPUT_HANDLE),
		coord
	);

}

/******************************************/

float chirpFnc(double currentTime, int f0, int f1, int tf)
{
	float y = 0;
	float omega = 0;

	omega = 2 * M_PI *(f0 + ((f1 - f0) / tf)*currentTime);
	return y = (sin(omega * currentTime));
}

/******************************************/

void setControllerGains()
{
	velPID[0] = 0;
	velPID[1] = 0;
	velPID[2] = 0;

	// sprintf_s is changing gains (long datatypes) to char types for SetPidGains function variable char * gains. 
	sprintf_s(pidGains, sizeof(pidGains) / sizeof(char), "%d,%d,%d", gains[0], gains[1], gains[2]);
	temp = SetPidGains(2, 1, pidGains);


	//sprintf_s(pidGains, sizeof(pidGains)/sizeof(char), "%d,%d,%d", gains[0],gains[1],gains[2]);
	//		temp=SetPidGains(2,2,pidGains);
	GetPidGains(1, 1, gains);
	gotoxy(1, 20); printf("p=%d, i=%d, d=%d \n", gains[0], gains[1], gains[2]);

	//	GetPidGains(2,1,gains);
	//gotoxy(1,21); printf("p=%d, i=%d, d=%d \n",gains[0],gains[1],gains[2]);

}

/******************************************/
void loadJointDataFromDrivers()
/******************************************/
{
	long temp;
	long r[3];

	for (b = FIRST_LBODY_BOARD; b<LAST_LBODY_BOARD; b++) {
		if (joint[b].isConnected) {
			temp = GetPosition(activeBoards[b]);
			joint[b].pos = temp;
			GetVelocity(b);
			joint[b].vel = temp;
			GetTorque(b);
			joint[b].tor = temp;
			temp = GetMinPosition(activeBoards[b]);
			joint[b].motorEncLimit1 = temp;
			temp = GetMaxPosition(activeBoards[b]);
			joint[b].motorEncLimit2 = temp;
			temp = GetMinVelocity(activeBoards[b]);
			joint[b].minvel = temp;
			temp = GetMaxVelocity(activeBoards[b]);
			joint[b].maxvel = temp;
			temp = GetCurrentLimit(activeBoards[b]);
			joint[b].realCurrent = temp;// joint[b].current = temp;  cambiato!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
			temp = GetPidGains(activeBoards[b], 1, r);
			joint[b].pgain = r[0];
			joint[b].igain = r[1];
			joint[b].dgain = r[2];
			temp = GetPidGainScale(activeBoards[b], 1, r);
			joint[b].gainscale = r[0];
			temp = GetILimGain(activeBoards[b], 1);
			joint[b].ilimit = temp;
			temp = GetPidOffset(activeBoards[b]);
			joint[b].offset = temp;

			printf("b%d,p:%d,minp:%d,maxp:%d,mc:%d,(p:%d,i:%d,d:%d, il:%d,o:%d)\n",
				activeBoards[b], joint[b].pos, joint[b].motorEncLimit1, joint[b].motorEncLimit2, joint[b].realCurrent,		// current    cambiato!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
				joint[b].pgain, joint[b].igain, joint[b].dgain, joint[b].ilimit, joint[b].offset);

		}
	}
	printf("\n");
}

/******************************************/

int scanSystem()
/******************************************/
{
	int r = 0;
	if ((r = ScanForActiveBoards())>0) {
		noActiveBoards = r;
		GetActiveBoards(activeBoards);
	}
	if (r == -1)
		printf("Scanning broadcast request: Sent failed!\n");
	else if (r == -2)
		printf("Broadcast packet read failed!\n");
	else if (r == -3)
		printf("Scan completed: No active boards found!\n");
	else {
		printf("Scan completed: (%d) Boards found !\n", r);
		for (b = FIRST_LBODY_BOARD; b<LAST_LBODY_BOARD; b++)
			if (activeBoards[b]>0)
				printf("BoardID:%d found at [IP:169.254.89.%d]\n", activeBoards[b], BASEADDR + activeBoards[b]);
	}
	printf("\n");
	return r;
}

/******************************************/

int connectUDP()
/******************************************/
{
	r = ConnectUDP();
	if (r<0)
		printf("UDP Connection failed!, Error:%d\n", r);
	else
		printf("UDP Connected, SocketID:%d\n", r);
	printf("\n");
	return r;
}

/******************************************/

void connectTCP()
/******************************************/
{
	printf("Creating TCP Connection....\n");
	for (b = FIRST_LBODY_BOARD; b<MAX_BOARDS; b++)
		if (activeBoards[b]>0) {
			if ((r = ConnectTCP(activeBoards[b]))>0) {
				printf("BoardID:%d ->TCP Connection established[IP:169.254.89.%d, SocketID:%d ]\n", activeBoards[b], BASEADDR + activeBoards[b], r);
				joint[b].isConnected = 1;
			}
			else {
				printf("BoardID:%d ->TCP Connection failed[IP:169.254.89.%d, SocketID:%d ]\n", activeBoards[b], BASEADDR + activeBoards[b], r);
				joint[b].isConnected = 0;
				Sleep(5000);
				exit(0);
			}
		}
	printf("\n");
}

/******************************************/

void startControlAll()
{
	for (b = FIRST_LBODY_BOARD; b<LAST_LBODY_BOARD; b++)
	{
		if (activeBoards[b]>0) {
			r = StartBoardPositionControl(activeBoards[b]);
			if (r != -1)
				printf("Board %d :Start Position Control succeded, %d\n", activeBoards[b], r);
			else
				printf("Board %d :Start Position Control failed! Error:%d\n", activeBoards[b], r);
		}
		printf("\n");
	}
}

/******************************************/

void startControl(int BoardNumber)
{

	r = StartBoardPositionControl(activeBoards[BoardNumber]);
	if (r != -1)
		printf("Board %d :Start Position Control succeded, %d\n", activeBoards[BoardNumber], r);
	else
		printf("Board %d :Start Position Control failed! Error:%d\n", activeBoards[BoardNumber], r);

	printf("\n");
}


void stopControl()
{
	r = StopPositionControl();
	if (r != -1)
		printf("All Active Boards:Stop Position Control succeded\n");
	else
		printf("All Active Boards:Stop Position Control failed! Error:%d\n", r);
	printf("\n");
}

/******************************************/

void setTorqueCntrlFlag(int BoardNumber, int torqueFlag)
{

	r = SetTorqueOnOff(activeBoards[BoardNumber], torqueFlag);
	if (r != -1)
		printf("Boardg %d :SetTorqueOnOff Control succeded, %d\n", activeBoards[BoardNumber], r);
	else
		printf("Board %d :SetTorqueOnOff Control failed! Error:%d\n", activeBoards[BoardNumber], r);

	printf("\n");

}

/******************************************/

void setBroadCastRate(int boardNumber, int BCastRate)
/******************************************/
{
	r = SetBCastRate(activeBoards[boardNumber], BCastRate);  // 1=0.5msec 
															 //r=SetBCastRate(activeBoards[boardNumber], 2*BCastRate, TwinSatus);  // 1=0.5msec 
	if (r != -1)
		printf("Board %d :Broadcast request sent succed, Rate:%5.1fHz", activeBoards[boardNumber], 1 / (float)((float)(BCastRate)*0.0005));
	else
		printf("Board %d :Broadcast request sent failed!, Rate:%5.1fHz", activeBoards[boardNumber], 1 / (float)(BCastRate*0.0005));
}

/******************************************/

void setBroadCastPolicy(int boardNumber, int BCastPolicy)
/******************************************/
{
	r = SetBCastPolicy(activeBoards[boardNumber], BCastPolicy);
	if (r != -1)
		printf("Board %d :Set Broad Cast Policy succeded\n", activeBoards[boardNumber]);
	else
		printf("Board %d :Get Broad Cast Policy failed! Error:%d\n", activeBoards[boardNumber], r);
}

/******************************************/

void setExtraBroadCastPolicy(int boardNumber, int ExtraPolicy)
/******************************************/
{
	r = SetExtraBCastPolicy(activeBoards[boardNumber], ExtraPolicy);
	if (r != -1)
		printf("Board %d :Set Broad Cast Policy succeded\n", activeBoards[boardNumber]);
	else
		printf("Board %d :Get Broad Cast Policy failed! Error:%d\n", activeBoards[boardNumber], r);
}

/******************************************/

void stopBroadCast(int BoardNumber)
/******************************************/
{
	r = StopBCast(activeBoards[BoardNumber]);
	if (r != -1)
		printf("Board %d :Stop Broadcast request sent succed\n", activeBoards[BoardNumber]);
	else
		printf("Board %d :Stop Broadcast request sent failed!\n", activeBoards[BoardNumber]);
}

/******************************************/

void saveData()
/******************************************/
{
	time_t now = time(0);
	tm *ltm = localtime(&now);
	// print various components of tm structure.
	//cout << "Year: "<< 1900 + ltm->tm_year << endl;
	//cout << "Month: "<< 1 + ltm->tm_mon<< endl;
	//cout << "Day: "<<  ltm->tm_mday << endl;
	//cout << "Time: "<< 1 + ltm->tm_hour << ":";
	//cout << 1 + ltm->tm_min << ":";
	//cout << 1 + ltm->tm_sec << endl;
	char Filename_date[256];
	char Filename_date_FT[256];
	sprintf(Filename_date, "%d_%d_%d_%d_%d_%d_Saved_Data.txt", 1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
	sprintf(Filename_date_FT, "%d_%d_%d_%d_%d_%d_Saved_Data_FTCaliData.txt", 1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
	printf("Saving in file: %s \n", Filename_date);

	FILE *fp;
	char filename[256];
	//if (_getch()==13)
	//	fp=fopen("adata.txt","w");
	//else
	//{	
	//	cin>>filename;	
	//	fp=fopen(filename,"w");
	//}

	//fp=fopen("FTcalibrationData.txt","w");
	//		//fp=fopen("adata.txt","w");
	//		//fprintf(fp, "time, M1Despos, M1RealPos, M1vel, M1tor, M2Despos, M2RealPos, M2vel, M2tor\n");
	//for(int d=0;d<dataCounter;d++)
	//		//fprintf(fp, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",sT[d], data[d][1], data[d][2], data[d][3], data[d][4], data[d][5], 
	//		//data[d][6], data[d][7], data[d][8]);
	//		//fprintf(fp,"%.4f \t %.4f \t %.4f\t %.4f\t %10.4f\t %10.4f\t  %10.4f\t %10.4f\t \n",sT[d], data[d][2],data[d][3],data[d][4],data[d][7], data[d][9], data[d][10], data[d][11]); //added by Yogesh	
	//		//fprintf(fp,"%.4f \t %.4f \t %.4f\t  \n",sT[d], data[d][2],data[d][3]); //added by Yogesh	
	//		// to print Fz, Tz, Analog i/p 1 and 2	
	//		//fprintf(fp,"%.4f \t %.4f \t %.4f \t %.1f \t %.1f\t \n",sT[d], data[d][8],data[d][9],data[d][10],data[d][11]); //added by Yogesh	
	//		//fprintf(fp,"%.4f \t %.4f \t %.4f \t %.1f \t %.1f\t \n",sT[d], data[d][2],data[d][9],data[d][10],data[d][11]); //time,position,force(counts),force(N),torque //added by Yogesh	
	//		//fprintf(fp,"%.4f \t %.4f \t %.4f \t  %.4f \t %.1f \t %d \t %d \t %d \t \n",sT[d], data[d][2],data[d][3],data[d][9],data[d][10], data[d][5], data[d][6], data[d][7]); //time,damper motor position,main motor position, force(counts),force(N) //added by Yogesh	

	//		//11/3/2013fprintf(fp,"%.4f \t %.4f \t %.4f \t  %.4f \t %.1f \t %d \t %d \t %d \t \n",sT[d], data[d][2],data[d][3],data[d][9],data[d][10], accn[d][0], accn[d][1], accn[d][2]); //time,damper motor position,main motor position, force(counts),force(N), accns //added by Yogesh on 13-2-13		
	//fclose(fp);

	//
	//////FT sensor Calibration using ATI in the assembly.  No spring element in this case
	////fp=fopen("FTCaliData.txt","w");
	////for(int d=0;d<dataCounter;d++)
	////fprintf(fp,"%.4f \t %d \t %d \t %d \t %d \t %.4f \t %.4f \t %.4f \t %.4f \t %.4f \t %.4f\n",sT[d], data[d][0],data[d][1],data[d][2],data[d][3],FT_data[d][0],FT_data[d][1],FT_data[d][2],FT_data[d][3],FT_data[d][4],FT_data[d][5]);//FT sensor Calibration data
	////fclose(fp);

	//FT sensor Calibration using ATI in the assembly.  No spring element in this case
	fp = fopen(Filename_date_FT, "w");
	for (int d = 0; d<dataCounter; d++)
		fprintf(fp, "%.4f \t %.4f \t %.4f \t %ld \t %ld \t %.4f \t %.4f \t %.4f \t %.4f \t %.4f \t %.4f\n", sT[d], fCalibFTdata[d][0], fCalibFTdata[d][1], dCalibFTdata[d][0], dCalibFTdata[d][1], fCalibFTdata[d][2], fCalibFTdata[d][3], fCalibFTdata[d][4], fCalibFTdata[d][5], fCalibFTdata[d][6], fCalibFTdata[d][7]);//FT sensor Calibration data
	fclose(fp);

	//////Stiffness of the system with spring element in the assembly
	////fp=fopen("StiffnessData_15_9_2013_.txt","w+");
	////fp=fopen("Data_18_11_2013_.txt","w+");
	////for(int d=0;d<dataCounter;d++)
	////fprintf(fp,"%d \t %d \t %.4f \t %.4f \t %d \t %d \t %d \t %.4f \t %.4f \t %.4f \t %.4f \t %.3f \t %d \n", data[d][0],data[d][1],FT_data[d][0],FT_data[d][1],data[d][2],data[d][3],data[d][4],fdata[d][0],fdata[d][1],fdata[d][2],fdata[d][3],fdata[d][4],data[d][5]);  //PositionMot1, PositionMot2, Force, torque
	////fclose(fp);

	//////data[dataCounter][0] =joint[0].pos;
	//////data[dataCounter][1] =joint[0].TwinActPos;
	//////FT_data[dataCounter][0] =joint[0].Force;
	//////FT_data[dataCounter][1] =joint[0].Torque;
	//////data[dataCounter][2] =joint[1].vel;
	//////data[dataCounter][3] =joint[0].TwinVel;
	//////data[dataCounter][4] =joint[0].TwinTargPos;

	////fp=fopen("PosVelData_18_11_2013_.txt","w");
	////for(int d=0;d<dataCounter;d++)
	////	fprintf(fp,"%.4f \t %*d \t %*d \t %*d \t %*d \n",fdata[d][4],7, data[d][7],7, data[d][1],7,data[d][2],7,data[d][3]);//main motor position velocity data
	////fclose(fp);

	//////fdata[dataCounter][4]//timestamp
	//////data[dataCounter7][7]//pos
	//////data[dataCounter][1] =joint[0].TwinActPos;
	//////data[dataCounter][2] =joint[1].vel*10;//in mrad/s
	//////data[dataCounter][3] =joint[0].TwinVel*10;//in mrad/s

	//fp=fopen(Filename_date,"w");
	//for(int d=0;d<dataCounter;d++)
	//	fprintf(fp,"%.3f \t %.3f \t %.3f \t %.3f \t %.3f \t %.3f\n",fdata[d][4], fdata[d][5], fdata[d][6],fCalibFTdata[d][2],fCalibFTdata[d][3],fCalibFTdata[d][4]);//main motor position velocity data
	//fclose(fp);

	fp = fopen(Filename_date, "w");
	for (int d = 0; d<dataCounter; d++)
		//	data[dataCounter][0] = joint[0].pos;
		//data[dataCounter][1] = joint[0].tpos;
		//data[dataCounter][2] = joint[0].temp_tpos;
		//data[dataCounter][3] = joint[0].req_tpos;
		fprintf(fp, "%d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d\t %d \t %d\n", data_motor[d][4], data_motor[d][0], data_motor[d][1], data_motor[d][2], data_motor[d][3], data_motor[d][5], data_motor[d][6], data_motor[d][7], data_motor[d][8], data_motor[d][9], data_motor[d][10]);
	////fclose(fp);
	fclose(fp);
}

/******************************************/

void ClearScreen(void)
{
	int n;
	for (n = 0; n < 100; n++)
	{
		gotoxy(0, n);
		printf("                                                                                                           ");
	}
	gotoxy(0, 1);
}

//---------------------------------------------------------------------------

void closeSim(void)
{

	Timer2Clock->stop();
	mainClock->stop();
	// wait for graphics and haptics loops to terminate
	//while (!simulationFinished) { cSleepMs(100); }
}

/******************************************/

int getBroadCastData(int numActiveBoards)
{
	int bytesReceived = 0;
	int boardNo;
	int pktRead = 0;
	int PcCnt = 0;

	noActiveBoards = numActiveBoards;
	for (int pkt = 0; pkt<noActiveBoards; pkt++)	//for (int pkt = 0; pkt<noActiveBoards + 1; pkt++)
	{
		bytesReceived = GetBCastData(packetToReceive);
		if (bytesReceived >0)
		{
			if (packetToReceive.content[2] == (char)0xbb)   // this is a boradcast packet....here is define wich kind of hardware are you talking with
			{
				PcCnt = 3;
				boardNo = packetToReceive.content[3];

				//if (boardNo - 1 == 1)// This reads the damper motor board which has a modified protocol
				//{
				//	if (BCAST_POLICY & 0x1){//check if bit 0 of the BCAST_POLICY is asserted
				//		joint[boardNo - 1].pos = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)
				//			+ ((packetToReceive.content[PcCnt + 3] << 16) & 0xff0000) + ((packetToReceive.content[PcCnt + 4] << 24) & 0xff000000);
				//		PcCnt = PcCnt + 4;
				//	}
				//	if (BCAST_POLICY & 0x2){//check if bit 1 of the BCAST_POLICY is asserted 
				//		joint[boardNo - 1].DamperRefNormForce = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00);
				//		PcCnt = PcCnt + 2;
				//	}
				//	if (BCAST_POLICY & 0x4){//check if bit 2 of the BCAST_POLICY is asserted
				//		joint[boardNo - 1].DamperRefTorque = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 1] << 8) & 0xff00);
				//		PcCnt = PcCnt + 2;
				//	}
				//	if (BCAST_POLICY & 0x8){//check if bit 3 of the BCAST_POLICY is asserted
				//		joint[boardNo - 1].pidOutput = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 1] << 8) & 0xff00);
				//		PcCnt = PcCnt + 2;
				//	}
				//	if (BCAST_POLICY & 0x10){//check if bit 4 of the BCAST_POLICY is asserted
				//		joint[boardNo - 1].DamperRefDeflection = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)
				//			+ ((packetToReceive.content[PcCnt + 3] << 16) & 0xff0000) + ((packetToReceive.content[PcCnt + 4] << 24) & 0xff000000);
				//		PcCnt = PcCnt + 4;
				//	}
				//	if (BCAST_POLICY & 0x20){//check if bit 5 of the BCAST_POLICY is asserted
				//		joint[boardNo - 1].DamperActDeflection = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)
				//			+ ((packetToReceive.content[PcCnt + 3] << 16) & 0xff0000) + ((packetToReceive.content[PcCnt + 4] << 24) & 0xff000000);
				//		PcCnt = PcCnt + 4;
				//	}
				//	if (BCAST_POLICY & 0x40){//check if bit 6 of the BCAST_POLICY is asserted
				//		joint[boardNo - 1].temp_Vdc = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)
				//			+ ((packetToReceive.content[PcCnt + 3] << 16) & 0xff0000) + ((packetToReceive.content[PcCnt + 4] << 24) & 0xff000000);
				//		PcCnt = PcCnt + 4;
				//	}
				//	if (BCAST_POLICY & 0x80){//check if bit 7 of the BCAST_POLICY is asserted
				//		joint[boardNo - 1].tStamp = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)
				//			+ ((packetToReceive.content[PcCnt + 3] << 16) & 0xff0000) + ((packetToReceive.content[PcCnt + 4] << 24) & 0xff000000);
				//		PcCnt = PcCnt + 4;
				//	}
				//	if (BCAST_POLICY & 0x100){//check if bit 8 of the BCAST_POLICY is asserted
				//		joint[boardNo - 1].fault = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 1] << 8) & 0xff00);
				//		PcCnt = PcCnt + 2;
				//	}
				//	//Construct analogue input selection
				//	int AB = 2 * ((BCAST_POLICY & 0x400) >> 10) + ((BCAST_POLICY & 0x200) >> 9);
				//	switch (AB)
				//	{
				//	case 0:
				//		break;
				//	case 1:
				//		break;
				//	case 2:
				//		break;
				//	case 3:
				//	{
				//			  joint[boardNo - 1].anInput1 = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00));
				//			  PcCnt = PcCnt + 2;
				//			  joint[boardNo - 1].anInput2 = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00));
				//			  PcCnt = PcCnt + 2;
				//			  // anInput3 and anInput4 receive force and torque in decaN and mNm 
				//			  //for fitting  positive and negative forces. Force up to 327.67N with a resolution of 10mN. Torque up to 32.767Nm with a resolution of 1mNm.
				//			  if (((packetToReceive.content[PcCnt + 2]) & 0x80) > 0) joint[boardNo - 1].anInput3 = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)) - 0xffff;
				//			  else joint[boardNo - 1].anInput3 = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00));
				//			  joint[boardNo - 1].Force = joint[boardNo - 1].anInput3 / 10.0;// convert to Newtons from decaNewtons
				//			  PcCnt = PcCnt + 2;
				//			  // decaN and mNm for fitting  positive and negative forces up to 327.67N with a resolution of 10mN.
				//			  if (((packetToReceive.content[PcCnt + 2]) & 0x80) > 0) joint[boardNo - 1].anInput4 = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)) - 0xffff;
				//			  else joint[boardNo - 1].anInput4 = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00));
				//			  joint[boardNo - 1].Torque = joint[boardNo - 1].anInput4 / 1000.0;// convert to Nm from mNm
				//			  PcCnt = PcCnt + 2;
				//	}
				//		break;
				//	}
				//	if (EXTRA_BCAST_POLICY & 0x1){//check if bit 0 of the EXTRA_BCAST_POLICY is asserted
				//		PcCnt = PcCnt + 4;
				//	}
				//	if (EXTRA_BCAST_POLICY & 0x2){//check if bit 1 of the EXTRA_BCAST_POLICY is asserted 						
				//		PcCnt = PcCnt + 4;
				//	}
				//	if (EXTRA_BCAST_POLICY & 0x4){//check if bit 2 of the EXTRA_BCAST_POLICY is asserted
				//		PcCnt = PcCnt + 4;
				//	}
				//	if (EXTRA_BCAST_POLICY & 0x8){//check if bit 3 of the EXTRA_BCAST_POLICY is asserted
				//		PcCnt = PcCnt + 4;
				//	}
				//	if (EXTRA_BCAST_POLICY & 0x10){//check if bit 4 of the EXTRA_BCAST_POLICY is asserted
				//		PcCnt = PcCnt + 2;
				//	}
				//	if (EXTRA_BCAST_POLICY & 0x20){//check if bit 5 of the EXTRA_BCAST_POLICY is asserted
				//		PcCnt = PcCnt + 2;
				//	}
				//	if (EXTRA_BCAST_POLICY & 0x40){//check if bit 6 of the EXTRA_BCAST_POLICY is asserted
				//		joint[boardNo - 1].TwinActPos = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)
				//			+ ((packetToReceive.content[PcCnt + 3] << 16) & 0xff0000) + ((packetToReceive.content[PcCnt + 4] << 24) & 0xff000000);
				//		PcCnt = PcCnt + 4;
				//	}
				//	if (EXTRA_BCAST_POLICY & 0x80){//check if bit 7 of the EXTRA_BCAST_POLICY is asserted
				//		joint[boardNo - 1].TwinTargPos = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)
				//			+ ((packetToReceive.content[PcCnt + 3] << 16) & 0xff0000) + ((packetToReceive.content[PcCnt + 4] << 24) & 0xff000000);
				//		PcCnt = PcCnt + 4;
				//	}
				//	if (EXTRA_BCAST_POLICY & 0x100){//check if bit 8 of the EXTRA_BCAST_POLICY is asserted
				//		joint[boardNo - 1].TwinVel = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00);
				//		PcCnt = PcCnt + 2;
				//	}
				//	if (EXTRA_BCAST_POLICY & 0x200){//check if bit 9 of the EXTRA_BCAST_POLICY is asserted
				//		if (((packetToReceive.content[PcCnt + 2]) & 0x80) > 0) joint[boardNo - 1].Xaccleration = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)) - 0xffff;
				//		else joint[boardNo - 1].Xaccleration = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00));
				//		PcCnt = PcCnt + 2;
				//	}
				//	if (EXTRA_BCAST_POLICY & 0x400){//check if bit 10 of the EXTRA_BCAST_POLICY is asserted
				//		if (((packetToReceive.content[PcCnt + 2]) & 0x80) > 0) joint[boardNo - 1].Yaccleration = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)) - 0xffff;
				//		else joint[boardNo - 1].Yaccleration = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00));
				//		PcCnt = PcCnt + 2;
				//	}
				//	if (EXTRA_BCAST_POLICY & 0x800){//check if bit 11 of the EXTRA_BCAST_POLICY is asserted
				//		if (((packetToReceive.content[PcCnt + 2]) & 0x80) > 0) joint[boardNo - 1].Zaccleration = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)) - 0xffff;
				//		else joint[boardNo - 1].Zaccleration = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00));
				//		PcCnt = PcCnt + 2;
				//	}
				//}
				if (boardNo - 1 == 0)      ////////////////////////This reads the main motor board
				{
					if (BCAST_POLICY & 0x1) {//check if bit 0 of the BCAST_POLICY is asserted
						joint[boardNo - 1].pos = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)
							+ ((packetToReceive.content[PcCnt + 3] << 16) & 0xff0000) + ((packetToReceive.content[PcCnt + 4] << 24) & 0xff000000);
						PcCnt = PcCnt + 4;
					}
					if (BCAST_POLICY & 0x2) {//check if bit 1 of the BCAST_POLICY is asserted
											 //if (((packetToReceive.content[PcCnt + 2]) & 0x80) > 0) joint[boardNo - 1].vel = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)) - 0xffff;
											 //else joint[boardNo - 1].vel = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00));
											 //PcCnt = PcCnt + 2;
						joint[boardNo - 1].vel = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00));
						PcCnt = PcCnt + 2;
					}
					if (BCAST_POLICY & 0x4) {//check if bit 2 of the BCAST_POLICY is asserted
											 //joint[boardNo - 1].tobbr = packetToReceive.content[PcCnt + 1] + (packetToReceive.content[PcCnt + 2] * 256);
						joint[boardNo - 1].tor = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00);
						PcCnt = PcCnt + 2;
					}
					if (BCAST_POLICY & 0x8) {//check if bit 3 of the BCAST_POLICY is asserted
						joint[boardNo - 1].pidOutput = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00);
						PcCnt = PcCnt + 2;
					}
					if (BCAST_POLICY & 0x10) {//check if bit 4 of the BCAST_POLICY is asserted
						joint[boardNo - 1].pidError = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)
							+ ((packetToReceive.content[PcCnt + 3] << 16) & 0xff0000) + ((packetToReceive.content[PcCnt + 4] << 24) & 0xff000000);
						PcCnt = PcCnt + 4;
					}
					if (BCAST_POLICY & 0x20) {//check if bit 5 of the BCAST_POLICY is asserted
						joint[boardNo - 1].realCurrent = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)
							+ ((packetToReceive.content[PcCnt + 3] << 16) & 0xff0000) + ((packetToReceive.content[PcCnt + 4] << 24) & 0xff000000);
						PcCnt = PcCnt + 4;
						//	if (((packetToReceive.content[PcCnt+4])& 0x80)>0)
						//joint[boardNo-1].realCurrent=(unsigned char)packetToReceive.content[PcCnt+1]+((packetToReceive.content[PcCnt+2]<<8)& 0xff00) 
						//	+((packetToReceive.content[PcCnt+3]<<16)& 0xff0000)+((packetToReceive.content[PcCnt+4]<<24)& 0xff000000)-0xffff;
						//else
						//	joint[boardNo-1].realCurrent=(unsigned char)packetToReceive.content[PcCnt+1]+((packetToReceive.content[PcCnt+2]<<8)& 0xff00) 
						//	+((packetToReceive.content[PcCnt+3]<<16)& 0xff0000)+((packetToReceive.content[PcCnt+4]<<24)& 0xff000000);
						//PcCnt=PcCnt+4;
					}
					if (BCAST_POLICY & 0x40) {//check if bit 6 of the BCAST_POLICY is asserted
						joint[boardNo - 1].temp_Vdc = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)
							+ ((packetToReceive.content[PcCnt + 3] << 16) & 0xff0000) + ((packetToReceive.content[PcCnt + 4] << 24) & 0xff000000);
						PcCnt = PcCnt + 4;
					}
					if (BCAST_POLICY & 0x80) {//check if bit 7 of the BCAST_POLICY is asserted
						joint[boardNo - 1].tStamp = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)
							+ ((packetToReceive.content[PcCnt + 3] << 16) & 0xff0000) + ((packetToReceive.content[PcCnt + 4] << 24) & 0xff000000);
						PcCnt = PcCnt + 4;
					}
					if (BCAST_POLICY & 0x100) {//check if bit 8 of the BCAST_POLICY is asserted
						joint[boardNo - 1].fault = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 1] << 8) & 0xff00);
						PcCnt = PcCnt + 2;
					}
					//Construct analogue input selection
					int AB = 2 * ((BCAST_POLICY & 0x400) >> 10) + ((BCAST_POLICY & 0x200) >> 9);
					switch (AB)
					{
					case 0:
						break;
					case 1:
						break;
					case 2:
						break;
					case 3:
					{
						joint[boardNo - 1].anInput1 = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00));
						PcCnt = PcCnt + 2;
						joint[boardNo - 1].anInput2 = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00));
						PcCnt = PcCnt + 2;

						// anInput3 and anInput4 receive force and torque in decaN and mNm 
						//for fitting  positive and negative forces. Force up to 327.67N with a resolution of 10mN. Torque up to 32.767Nm with a resolution of 1mNm.
						if (((packetToReceive.content[PcCnt + 2]) & 0x80) > 0) joint[boardNo - 1].anInput3 = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)) - 0xffff;
						else joint[boardNo - 1].anInput3 = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00));
						joint[boardNo - 1].Force = joint[boardNo - 1].anInput3 / 10.0;// convert to Newtons from decaNewtons
						PcCnt = PcCnt + 2;
						// decaN and mNm for fitting  positive and negative forces up to 327.67N with a resolution of 10mN.
						if (((packetToReceive.content[PcCnt + 2]) & 0x80) > 0) joint[boardNo - 1].anInput4 = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)) - 0xffff;
						else joint[boardNo - 1].anInput4 = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00));
						joint[boardNo - 1].Torque = joint[boardNo - 1].anInput4 / 1000.0;// convert to Nm from mNm
						PcCnt = PcCnt + 2;
					}
					break;
					}
					if (EXTRA_BCAST_POLICY & 0x1) {//check if bit 0 of the EXTRA_BCAST_POLICY is asserted
						joint[boardNo - 1].tpos = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)
							+ ((packetToReceive.content[PcCnt + 3] << 16) & 0xff0000) + ((packetToReceive.content[PcCnt + 4] << 24) & 0xff000000);
						PcCnt = PcCnt + 4;
					}
					if (EXTRA_BCAST_POLICY & 0x2) {//check if bit 1 of the EXTRA_BCAST_POLICY is asserted 						
						joint[boardNo - 1].temp_tpos = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)
							+ ((packetToReceive.content[PcCnt + 3] << 16) & 0xff0000) + ((packetToReceive.content[PcCnt + 4] << 24) & 0xff000000);
						PcCnt = PcCnt + 4;
					}
					if (EXTRA_BCAST_POLICY & 0x4) {//check if bit 2 of the EXTRA_BCAST_POLICY is asserted
						joint[boardNo - 1].req_tpos = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)
							+ ((packetToReceive.content[PcCnt + 3] << 16) & 0xff0000) + ((packetToReceive.content[PcCnt + 4] << 24) & 0xff000000);
						PcCnt = PcCnt + 4;
					}
					if (EXTRA_BCAST_POLICY & 0x8) {//check if bit 3 of the EXTRA_BCAST_POLICY is asserted
						joint[boardNo - 1].TwinPos = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)
							+ ((packetToReceive.content[PcCnt + 3] << 16) & 0xff0000) + ((packetToReceive.content[PcCnt + 4] << 24) & 0xff000000);
						PcCnt = PcCnt + 4;
					}
					if (EXTRA_BCAST_POLICY & 0x10) {//check if bit 4 of the EXTRA_BCAST_POLICY is asserted
						joint[boardNo - 1].angle = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00);
						PcCnt = PcCnt + 2;
					}
					if (EXTRA_BCAST_POLICY & 0x20) {//check if bit 5 of the EXTRA_BCAST_POLICY is asserted
						joint[boardNo - 1].targAngle = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00);
						PcCnt = PcCnt + 2;
					}
					if (EXTRA_BCAST_POLICY & 0x40) {//check if bit 6 of the EXTRA_BCAST_POLICY is asserted
						joint[boardNo - 1].TwinActPos = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)
							+ ((packetToReceive.content[PcCnt + 3] << 16) & 0xff0000) + ((packetToReceive.content[PcCnt + 4] << 24) & 0xff000000);
						PcCnt = PcCnt + 4;
					}
					if (EXTRA_BCAST_POLICY & 0x80) {//check if bit 7 of the EXTRA_BCAST_POLICY is asserted
						joint[boardNo - 1].TwinTargPos = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)
							+ ((packetToReceive.content[PcCnt + 3] << 16) & 0xff0000) + ((packetToReceive.content[PcCnt + 4] << 24) & 0xff000000);
						PcCnt = PcCnt + 4;
					}
					if (EXTRA_BCAST_POLICY & 0x100) {//check if bit 8 of the EXTRA_BCAST_POLICY is asserted
						joint[boardNo - 1].TwinVel = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00);
						PcCnt = PcCnt + 2;
					}
					if (EXTRA_BCAST_POLICY & 0x200) {//check if bit 9 of the EXTRA_BCAST_POLICY is asserted
						if (((packetToReceive.content[PcCnt + 2]) & 0x80) > 0) joint[boardNo - 1].Xaccleration = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)) - 0xffff;
						else joint[boardNo - 1].Xaccleration = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00));
						PcCnt = PcCnt + 2;
					}
					if (EXTRA_BCAST_POLICY & 0x400) {//check if bit 10 of the EXTRA_BCAST_POLICY is asserted
						if (((packetToReceive.content[PcCnt + 2]) & 0x80) > 0) joint[boardNo - 1].Yaccleration = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)) - 0xffff;
						else joint[boardNo - 1].Yaccleration = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00));
						PcCnt = PcCnt + 2;
					}
					if (EXTRA_BCAST_POLICY & 0x800) {//check if bit 11 of the EXTRA_BCAST_POLICY is asserted
						if (((packetToReceive.content[PcCnt + 2]) & 0x80) > 0) joint[boardNo - 1].Zaccleration = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)) - 0xffff;
						else joint[boardNo - 1].Zaccleration = ((unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00));
						PcCnt = PcCnt + 2;
					}
					if (EXTRA_BCAST_POLICY & 0x1000) {//check if bit 11 of the EXTRA_BCAST_POLICY is asserted
						joint[boardNo - 1].LinkVel = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00);
						PcCnt = PcCnt + 2;
					}
				}

			}
			/*else
			{
			printf("No match with ID Motor");
			Sleep(3000);
			return 0;
			}*/

		}
	}
	bytesReceived = 0;

	if ((desTor & 0x8000)>0) desTor = desTor - 0xffff;

	//	//---------------Calculation of force from the Calibration matrix------------------------------------------------------
	//	
	//	//The next two lines use the calibration matris to generate the forcein N and the torque in Nm from the raw data in counts
	//	//works only when the incoming values are in counts.  the firmware is now changed for anInput3 and anInput4 to send
	//	// decaN and mNm for fitting  positive and negative forces up to 327.67N with a resolution of 10mN.
	joint[0].anInput1_Adjusted = joint[0].anInput1 - joint[0].anInput1_Offset;
	joint[0].anInput2_Adjusted = joint[0].anInput2 - joint[0].anInput2_Offset;

	joint[0].Force2 = joint[0].anInput1_Adjusted*Cmatrix[0][0] + joint[0].anInput2_Adjusted*Cmatrix[1][0];
	joint[0].Torque2 = joint[0].anInput1_Adjusted*Cmatrix[0][1] + joint[0].anInput2_Adjusted*Cmatrix[1][1];
	//	
	//	//---------------END of Calculation of force from the Calibration matrix-------------------------------------------------
	return 0;
}

/******************************************/ //FT_Sensor Foot Plate

int getBroadCastFTData(int numActiveBoards)
{
	int bytesReceived = 0;
	int boardNo;
	int pktRead = 0;
	int PcCnt = 0;
	noActiveBoards = numActiveBoards;
	for (int pkt = 0; pkt<noActiveBoards + 1; pkt++)					// DA MODIIIFICAREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
		bytesReceived = GetBCastData(packetToReceive);
	{
		if (bytesReceived >0)
		{
			if (packetToReceive.content[2] == (char)0xbc)   // this is a boradcast packet bc define that is a sensor
			{
				PcCnt = 3;
				boardNo = packetToReceive.content[3];

				if (BCAST_POLICY_FT & 0x1) {//check if bit 0 of the BCAST_POLICY is asserted
					for (int i = 0; i < 6; i++)
					{
						if (((packetToReceive.content[PcCnt + 2]) & 0x80)>1)
						{
							FTfoot_data[boardNo - 1].ChRaw_Offs[i] = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00) - 0xffff;
							PcCnt = PcCnt + 2;
						}
						else
						{
							FTfoot_data[boardNo - 1].ChRaw_Offs[i] = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00);
							PcCnt = PcCnt + 2;
						}
					}
				}
				if (BCAST_POLICY_FT & 0x2) {//check if bit 1 of the BCAST_POLICY is asserted
					for (int i = 0; i < 6; i++)
					{
						FTfoot_data[boardNo - 1].FT[i] = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)
							+ ((packetToReceive.content[PcCnt + 3] << 16) & 0xff0000) + ((packetToReceive.content[PcCnt + 4] << 24) & 0xff000000);

						PcCnt = PcCnt + 4;
					}
				}
				if (BCAST_POLICY_FT & 0x4) {//check if bit 2 of the BCAST_POLICY is asserted
					for (int i = 0; i < 6; i++)
					{
						FTfoot_data[boardNo - 1].ChRaw[i] = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00);
						PcCnt = PcCnt + 2;
					}
				}
				if (BCAST_POLICY_FT & 0x8) {//check if bit 3 of the BCAST_POLICY is asserted
					FTfoot_data[boardNo - 1].temp_Vdc = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)
						+ ((packetToReceive.content[PcCnt + 3] << 16) & 0xff0000) + ((packetToReceive.content[PcCnt + 4] << 24) & 0xff000000);
					PcCnt = PcCnt + 4;
				}
				if (BCAST_POLICY_FT & 0x10) {//check if bit 4 of the BCAST_POLICY is asserted
					FTfoot_data[boardNo - 1].tStamp = (unsigned char)packetToReceive.content[PcCnt + 1] + ((packetToReceive.content[PcCnt + 2] << 8) & 0xff00)
						+ ((packetToReceive.content[PcCnt + 3] << 16) & 0xff0000) + ((packetToReceive.content[PcCnt + 4] << 24) & 0xff000000);
					PcCnt = PcCnt + 4;
				}

			}

		}
	}

	bytesReceived = 0;
	return 0;
}


void setupPositionControlPID()
{
	stopControl();
	gotoxy(1, 20); printf("\n \n  startPositionControl-POSITION STARS \n ");
	//startTorqueControl(0, 0);

#ifdef KNEEMOTOR_OLD
	sprintf_s(mConfig, sizeof(mConfig) / sizeof(char), "%d", 0x0201);
#endif

#ifdef KNEEMOTOR_NEW
	sprintf_s(mConfig, sizeof(mConfig) / sizeof(char), "%d", 0x0200);		//new motor
#endif

	posPID[0] = 100000;
	posPID[1] = 5;
	posPID[2] = 5000;

	// sprintf_s is changing gains (long datatypes) to char types for SetPidGains function variable char * gains. 
	sprintf_s(pidGains, sizeof(pidGains) / sizeof(char), "%d,%d,%d", posPID[0], posPID[1], posPID[2]);
	temp = SetPidGains(1, 1, pidGains);

	GetPidGains(1, 1, gains);
	gotoxy(1, 20); printf("Pos Gains: p=%d, i=%d, d=%d \n", gains[0], gains[1], gains[2]);


	// In order to have only a position control settingto null the others PID Gains...but maybe this can be avoid with the TorqueFlag

	torPID[0] = 0; //1500;
	torPID[1] = 0;
	torPID[2] = 0;//1500

				  // sprintf_s is changing gains (long datatypes) to char types for SetPidGains function variable char * gains. 
	sprintf_s(pidGains, sizeof(pidGains) / sizeof(char), "%d,%d,%d", torPID[0], torPID[1], torPID[2]);
	temp = SetPidGains(1, 2, pidGains);
	GetPidGains(1, 2, gains);
	//gotoxy(1, 20); printf("torque Gains: p=%d, i=%d, d=%d \n", gains[0], gains[1], gains[2]);

	Vwallk[0] = 0;//445; // K
	Vwallk[1] = 0;//22;		// range of virtual wall from the limit
	Vwallk[2] = 0;

	// sprintf_s is changing gains (long datatypes) to char types for SetPidGains function variable char * gains. 
	sprintf_s(pidGains, sizeof(pidGains) / sizeof(char), "%d,%d,%d", Vwallk[0], Vwallk[1], Vwallk[2]);
	temp = SetPidGains(1, 3, pidGains);
	GetPidGains(1, 3, gains);
	//gotoxy(1, 50); printf("wall Gains: p=%d, i=%d, d=%d \n", gains[0], gains[1], gains[2]);

}

/******************************************/

void setupTorqueControlPID()
{
	stopControl();
	gotoxy(1, 45); printf("\n \n  startPositionControl-TORQUE STARS \n ");

#ifdef KNEEMOTOR_OLD
	sprintf_s(mConfig, sizeof(mConfig) / sizeof(char), "%d", 0x4a03);
	SetMotorConfig(1, mConfig);
#endif

#ifdef KNEEMOTOR_NEW
	sprintf_s(mConfig, sizeof(mConfig) / sizeof(char), "%d", 0x4a03);// 0x4a02);
	SetMotorConfig(1, mConfig);
#endif

	posPID[0] = 0;
	posPID[1] = 0;
	posPID[2] = 0;

	// sprintf_s is changing gains (long datatypes) to char types for SetPidGains function variable char * gains. 
	sprintf_s(pidGains, sizeof(pidGains) / sizeof(char), "%d,%d,%d", posPID[0], posPID[1], posPID[2]);
	temp = SetPidGains(1, 1, pidGains);

#ifdef KNEEMOTOR_OLD
	torPID[0] = 1500; //1500;
	torPID[1] = 20;
	torPID[2] = 1;
#endif

#ifdef KNEEMOTOR_NEW
	torPID[0] = 1000; //10000; 1500;
	torPID[1] = 10;// 200;
	torPID[2] = 800; //85000;
#endif

					 // sprintf_s is changing gains (long datatypes) to char types for SetPidGains function variable char * gains. 
	sprintf_s(pidGains, sizeof(pidGains) / sizeof(char), "%d,%d,%d", torPID[0], torPID[1], torPID[2]);
	temp = SetPidGains(1, 2, pidGains);
	GetPidGains(1, 2, gains);
	gotoxy(1, 20); printf("torque Gains: p=%d, i=%d, d=%d \n", gains[0], gains[1], gains[2]);

	Vwallk[0] = 10;//445; // K
	Vwallk[1] = 20000;//22;		// range of virtual wall from the limit
	Vwallk[2] = 0;

	// sprintf_s is changing gains (long datatypes) to char types for SetPidGains function variable char * gains. 
	sprintf_s(pidGains, sizeof(pidGains) / sizeof(char), "%d,%d,%d", Vwallk[0], Vwallk[1], Vwallk[2]);
	temp = SetPidGains(1, 3, pidGains);
	GetPidGains(1, 3, gains);
	//gotoxy(1, 20); printf("wall Gains: p=%d, i=%d, d=%d \n", gains[0], gains[1], gains[2]);
}

/******************************************/

void moveToHome()
/******************************************/
{
	setupPositionControlPID();
	//enable profiling function in firmware
	/*
	int mType = GetMotorType(1);
	int newmType = mType | 16;
	newmType = newmType ^ 16;//(mType |
	r = SetMotorType(1, (char)newmType);
	*/

	desPos[0] = 0;//30000;
	desVel[0] = 250;

	r = SetDesiredVelocity(desVel);
	if (r != -1) printf("Set Desired Velocity succeded\n");
	else printf("Set Desired Velocity failed! Error:%d\n", r);

	r = SetDesiredPosition(desPos);
	if (r != -1) printf("Set Desired Position succeded\n");
	else printf("Set Desired Position failed! Error:%d\n", r);

	printf("\n");


}
