
%%
close all
clc
time = data(:,1);
pos = data(:,2);
vel = data(:,3);
tor = data(:,4);
des_tor = data(:,5);
pidE = data(:,6);
pidO = data(:,7);


vel2 = zeros(length(data(:,1)),1);
acc = zeros(length(data(:,1)),1);
for i=2:length(data(:,1))-1,
    acc(i) = (pos(i+1)-2*pos(i) + pos(i-1))/(1e-3)^2;
    vel2(i) = (pos(i+1) - pos(i))*1000;%%./(time(i+1)-time(i))
end
vel2(1)=vel(2);
acc(1) = acc(2);
acc(length(acc)) = acc(length(acc)-1);

[b a] = butter(5, 10/500);
Gdb = tf(b,a,1e-3)
acc_f = filter(b,a,acc);
vel2_f = filter(b,a,vel);

inertia = tor./acc_f;
I= 1.7e+4;
%%
figure(1)
plot(time, tor, time, des_tor, 'g')

tor_error = des_tor - tor;
figure(2)
plot(time, tor_error)

figure(3)
plot(pos, tor)

figure(4)
plot(time, pos)

figure(5)
plot(time, vel)

figure(6)
plot(time, vel, time, vel2_f, 'g')

close all
figure(7)
plot(time, acc)

%%
s = tf('s');

Jm = 8.47e-06;
JG = 0.054e-4;

bm = 2.09e-06 /60*2*pi;

Gm = (Jm + JG)*s^2 + bm *s
%%
[b a] = butter(2, 10/500,'low');%butter(2, 5/500);

tor_f = filter(b,a,data(:,4));
plot(data(:,1), tor_f,data(:,1), data(:,4),'r',data(:,1), data(:,3),'g')

Gdb = tf(b,a,1e-3);
b
a

%%

[b a] = butter(2, 2.5/500,'low');%butter(2, 5/500);

tor_f = filter(b,a,data(:,4));
figure
plot(data(:,1), tor_f,data(:,1), data(:,4),'r',data(:,1), data(:,3),'g')

b
a

%%

[b a] = butter(2, 2.5/500,'low');%butter(2, 5/500);

tor_f = filter(b,a,data1(:,4));
figure
plot(data1(:,1), tor_f,data1(:,1), data1(:,4),'r',data1(:,1), data1(:,3),'g')

b
a
%%

[b a] = butter(2, 2.5/500,'low');%butter(2, 5/500);

tor_f2 = filter(b,a,data2(:,4));
figure
plot(data2(:,1), tor_f2,data2(:,1), data2(:,4),'r',data2(:,1), data2(:,3),'g')

b
a

%%

[b a] = butter(2, 2.5/500,'low');%butter(2, 5/500);

tor_f3 = filter(b,a,data3(:,4));
figure
plot(data3(:,1), tor_f3,data3(:,1), data3(:,4),'r',data3(:,1), data3(:,3),'g')

b
a

%%
Fs = 1000;                    % Sampling frequency
T = 1/Fs;                     % Sample time
L = length(data2(:,1));                     % Length of signal
t = (0:L-1)*T;                % Time vector

y = tor_f2;%data2(:,5);     % Sinusoids plus noise
plot(Fs*t,y)


NFFT = 2^nextpow2(L); % Next power of 2 from length of y
Y = fft(y,NFFT)/L;
f = Fs/2*linspace(0,1,NFFT/2+1);

% Plot single-sided amplitude spectrum.
plot(f,2*abs(Y(1:NFFT/2+1))) 
title('Single-Sided Amplitude Spectrum of y(t)')
xlabel('Frequency (Hz)')
ylabel('|Y(f)|')

%%      Frequency analysis
clc
ts = 5e-4;

u = data(:,5);
y = data(:,4);

u=detrend(u);
y=detrend(y);

%%for i=1:length(y)-1%(Tmax/ts-1)
%%%    v(i+1,1) =(y(i+1) -y(i))/ts;
%%end
%%v(1,1)=v(2,1);

%%[b a] = butter(10, 200/1000,'low');%butter(2, 5/500);

%%vf = filter(b,a,v);

z=iddata(y,u,ts); 
yh2=spafdr(z,2);

[Cxy,Fcoher]=MSCOHERE(u,y,hanning(1024),512,1024,500);

%%

figure(1);
grid on
w=logspace(-1,5,1000);
bode(yh2,'k',w);
figure(2)
plot(Fcoher,Cxy )