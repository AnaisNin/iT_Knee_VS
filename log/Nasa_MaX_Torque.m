
clear all
close all

% It is a Matrix. Import all the data in matlab, but whitout letter or comma

ActPos1 = zeros(53,1);

for i=1:53
 if (i>=2)
  ActPos1(i)=ActPos1(i-1)+2.5;
 end
end

Torque1 =[68.32,74.65,80.83,86.86,92.72,98.41,103.91,109.21,114.31,119.19,123.84,128.25,132.42,136.34,140.00,143.39,146.51,149.35,151.91,154.18,156.15,157.83,159.20,160.28,161.04,161.50,161.66,161.50,161.04,160.28,159.20,157.83,156.15,154.18,151.91,149.35,146.51,143.39,140.00,136.34,132.42,128.25,123.84,119.19,114.31,109.21,103.91,98.41,92.72,86.86,80.83,74.65,68.32
]; % From Exel

Tot_time_esecution=3.5;
Time_acquisition = 0.05;
n_samples = Tot_time_esecution/Time_acquisition;
n_samples_difetto = floor(n_samples); 
efficency_global = 0.9*0.8;
k = 160 ; % Inverse of trasmission ratio

% Generate the sample data.
X=[0,  0.5,  1, 1.5,  2, 2.5,  3, 3.4, Tot_time_esecution];
Y=[140,141,135, 120,100,  80, 50, 25,                  5];
Y = Y/efficency_global;  
Y = Y/k; % Torque at motor level
Y = Y*1000; % from Nm to mNm 

% Make a finer sampling so we can see what it
% does in between the training points.
coeffs = polyfit(X, Y, length(Y)+1);
interpolatedX_T = linspace(min(X), max(X), n_samples_difetto+1);
interpolatedY_T = polyval(coeffs, interpolatedX_T);


% Setting

h=newplot

Lwidth=0.5;
Lcolor='r';
fontSize=12;
FontSize=12;

titleFontsize=14;
subtitleFontsize=14;
axisFontSize=11;
axisFontWeight='bold';%'normal';
Lwidth=1.5;

dy   = 0.08; %space in y
dx   = 0.075; %space in x
Lc1  = 0.07;  %Left Col 1
Lr1  = 0.07;  %Top Row 1
h    = (0.95-2*dy-1*Lr1)/3;  %height
w    = (1.05-dx-2*Lc1)/2;  %width
Lc2  = Lc1 + dx + w;  %Left Col 2
Br = [Lr1+2*dy+2*h Lr1+dy+h Lr1];

C1='r';  %[0.45 0.45 0.45];% C1='b';
C2='g'; %[0 0 0];% C2='r';
C3='b';
C4='c';
C5='m';  %[0.45 0.45 0.45];% C1='b';
C6='y'; %[0 0 0];% C2='r';
C7='k';
%C8='w'; % white!!!

C1Style='--'; %tratteggiato
C2Style='-';
C3Style='--'; %tratteggiato
C4Style='-';
C5Style='--'; %tratteggiato
C6Style='-';
C7Style='--'; %tratteggiato
C8Style='-';
% C3Style=':' ;%puntini
% C4Style='*' ;%puntini
C5Style='o'; %tratteggiato
% C6Style='.';
% C7Style='x' ;%puntini
% C8Style='s' ;%puntini

% Find the coefficients.
fig2 = figure('Name','Torque_Up','NumberTitle','off');  hold on
title('Motor Torque Up', 'FontSize', fontSize);
xlabel('Time(s)', 'FontSize', fontSize);
ylabel('Torque(mNm)', 'FontSize', fontSize);
hold on;
plot(X, Y, 'bo', 'MarkerSize', 10, 'LineWidth', 3);
% Plot the interpolated points.
hold on;
plot(interpolatedX_T, interpolatedY_T, ':ks'); %);
grid on;



interpolatedY_ActPos =linspace(90, 0,size(interpolatedY_T,2));
%interpolatedY_ActPos =interpolatedX_T;
interpolatedY_T = interpolatedY_T/10;


% ------------------------------------------------------------------%
% ------------------------------------------------------------------%


 


figure(1); %sinewave tracking position-force
%set(gcf,'position',[50 50 700 400]);  %for 2x1 figures
set(gcf,'position',[50 50 600 250]);  %for 3x1 figures
% set(gcf,'position',[50 50 600 900]);%for 4x1 figures
    % gcf = figure holder
    % [50 50 ...  ] = postion on the screen
    % 700 400] = figure dimension
set(gcf,'Color','white');


hold on;box on;
plot(ActPos1,Torque1,'Color',C1,'LineWidth',Lwidth,'LineStyle',C2Style);hold on;
plot(interpolatedY_ActPos,interpolatedY_T,'Color',C3,'LineWidth',Lwidth,'LineStyle',C1Style);hold on;
title ('Torque Comparison','FontSize',titleFontsize)
xlabel('Knee Flexion(degree)','FontSize',axisFontSize,'FontWeight',axisFontWeight);
ylabel('Torque(Nm)','FontSize',axisFontSize,'FontWeight',axisFontWeight);%,'Position',[-7,50,1]);
axis([-5 140 0 180]);
% set(f2,'pos',[Lc1,Br(3),w,h]); 
% ylim([-1 4]);
set(gca,'FontSize',axisFontSize,'FontWeight',axisFontWeight);
h_legend=legend('iT-Knee Torque ','StandUp Torque')
set(h_legend,'FontSize',axisFontSize);
% 
