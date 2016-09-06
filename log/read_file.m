clear all;close all;clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%     Read data from file    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Read data from file and store it
%id_file_A = fopen('logData.txt','r'); % open file and return file handle
id_file_A = fopen('1st_sept_16\stillTrunk_kneeBending_load\logData_4_10kg_alongBodySym.txt','r'); % open file and return file handle

%format_file_A = '%f %f %f %f %f %f %f'; %format of the data - each line - within the file
%size_buffer_A =[7 Inf]; % size of the transpose of the data file==[numColsFile numLinesFile]
format_file_A = '%f %f %f %f %f %f %f %f'; %format of the data - each line - within the file
size_buffer_A =[8 Inf]; % size of the transpose of the data file==[numColsFile numLinesFile]

%this is filled by column whereas the file is read by line, so this corresponds to the transpose of the file
%[m n] is an mxn matrix, n can be Inf, m cannot
buffer_A = fscanf(id_file_A,format_file_A,size_buffer_A);
fclose(id_file_A);
data_file_A=buffer_A'; %transpose to get it identical to the file

runTime=data_file_A(:,1);
torqueAnkle=data_file_A(:,2);
torqueAnkle_filtered=data_file_A(:,3);
torqueKnee=data_file_A(:,4);
theta1=data_file_A(:,5);
theta2=data_file_A(:,6);
theta3=data_file_A(:,7);
Fz=data_file_A(:,8);

figure('Color',[1 1 1])
plot(runTime,torqueAnkle_filtered,'b',runTime,torqueKnee,'r');
legend 'torqueAnkle' 'torqueKnee'
xlabel 'Time (s)'
ylabel 'Joint Torques (Nm)'
title 'Knee torque estimation - sit to stand'

figure('Color',[1 1 1])
plot(runTime,theta1,'b',runTime,theta2,'r',runTime,theta3,'g');
legend 'theta1' 'theta2' 'theta3'
xlabel 'Time (s)'
ylabel 'Theta (deg)'
title 'Knee torque estimation - sit to stand'

figure('Color',[1 1 1])
plot(runTime,Fz)
title 'Fz'

% figure('Color',[1 1 1])
% plot(data_file_A(:,1),'r');
% figure('Color',[1 1 1])
% plot(data_file_A(:,2),'b');
% figure('Color',[1 1 1])
% plot(data_file_A(:,3),'g');
% figure('Color',[1 1 1])
% plot(data_file_A(:,4),'k');
% figure('Color',[1 1 1])
% plot(data_file_A(:,5),'c');



% i_start=1;
% i_stop=size(data_file_K,1);
% figure
% plot(data_file_K(i_start:i_stop,1),data_file_K(i_start:i_stop,2),'r');
% hold on
% plot(data_file_K(i_start:i_stop,1),data_file_K(i_start:i_stop,3),'b');
% figure
% plot(data_file_K(i_start:i_stop,1),data_file_K(i_start:i_stop,4),'r');
% hold on
% plot(data_file_K(i_start:i_stop,1),data_file_K(i_start:i_stop,5),'b');
% 
% figure
% plot(data_file_K(i_start:size(data_file_K,1),1),data_file_K(:,7))
% hold on
% plot(data_file_K(i_start:size(data_file_K,1),1),data_file_K(:,8),'r')
% 
% figure
% plot(data_file_C(:,1),data_file_C(:,5))
% plot(data_file_C(:,1),data_file_C(:,6))
% plot(data_file_C(:,1),data_file_C(:,7))
% plot3(data_file_C(:,5),data_file_C(:,6),data_file_C(:,7))
% xlabel 'x'
% ylabel 'y'
% zlabel 'z'

% figure
% plot3(data_file_C(:,2),data_file_C(:,3),data_file_C(:,4),'o')
% hold on
% plot3(data_file_C(:,5),data_file_C(:,6),data_file_C(:,7),'o')
% hold on 
% plot3(data_file_C(:,8),data_file_C(:,9),data_file_C(:,10),'o')
% xlabel 'x'
% ylabel 'y'
% zlabel 'z'





