function jointPlot()

clear all
close all
clf

load adata.txt
for i=1:length(adata(:,6))
  k(i)=(2*10000*adata(i,6)*adata(i,6)*0.015^2)/(0.015-adata(i,6))^2;
  l(i)=(2*10000*adata(i,7)*adata(i,7)*0.015^2)/(0.015-adata(i,7))^2;  
end


figure(1);
subplot(2,2,1);
plot(adata(:,1),adata(:,2));hold on;
plot(adata(:,1),adata(:,3),'r');
xlabel('Time (sec)');
ylabel('M1 Angle (rad)');

subplot(2,2,2);
plot(adata(:,1),k);hold on;
plot(adata(:,1),l, 'r');
xlabel('Time (sec)');
ylabel('delta1 (m)');

subplot(2,2,3);
plot(adata(:,1),adata(:,5));hold on;
xlabel('Time (sec)');
ylabel('M1 Torque (Nm)');

subplot(2,2,4);
plot(adata(:,1),adata(:,6));hold on;
plot(adata(:,1),adata(:,7),'r');
xlabel('Time (sec)');
ylabel('delta1 (m)');