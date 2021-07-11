close all;
clc;
clear all;
N=100;  % number of samples
a=0.1; % acceleration


sigmaPsi=1;
sigmaEta=20;
k=1:N;
x=k;
x(1)=0;
x_wn(1)=0;
z(1)=x(1)+normrnd(0,sigmaEta);
for t=1:(N-1)
  x(t+1) = x(t) + a*t + normrnd(0,sigmaPsi);
  x_wn(t+1) = x_wn(t) + a*t;
  z(t+1) = x(t+1) + normrnd(0,sigmaEta);
end
%kalman filter
xOpt(1) = z(1);     % Optimal values
eOpt(1) = sigmaEta; % eOpt(t) is a square root of the error dispersion (variance).
% It's not a random variable. 
sl = 5;
for t=1:(N-1)
    
  eOpt(t+1)=sqrt((sigmaEta^2)*(eOpt(t)^2+sigmaPsi^2)/(sigmaEta^2+eOpt(t)^2+sigmaPsi^2));
  
  K(t+1)=(eOpt(t+1))^2/sigmaEta^2;
  
 xOpt(t+1)=(xOpt(t)+a*t)*(1-K(t+1))+K(t+1)*z(t+1);
%     
%     K(t) = sigmaPsi^2/ (sigmaEta^2 + sigmaPsi^2);
%     xOpt(t+1) = xOpt(t) + K(t) * (z(t) - x(t));
%     eOpt(t+1) = (1 - K(t)) * eOpt(t);
%  
end
plot(k,xOpt,'r',k,z,'b',k,x,'g')
legend('Kalman', 'Measurement', 'Prediction');
figure
subplot(211);
x1 = -3*sigmaPsi:0.1:3*sigmaPsi;
y1 = pdf('Normal', x1, 0, sigmaPsi);
plot(x1, y1);
title('Sigma Psi');
subplot(212);
x2 = -3*sigmaEta:0.1:3*sigmaEta;
y2 = pdf('Normal', x2, 0, sigmaEta);
plot(x2, y2);
title('Sigma Eta');
figure(3)
subplot(2,1,1)
plot(x_wn)
hold on
plot(x)
legend('Physical reading without noise', 'Physical reading with noise');
subplot(2,1,2)
plot(x)
hold on
plot(z)
legend('Sensor reading without noise', 'Sensor reading with noise');