close all;
clc;
clear all;


% Initial Conditions
%States = 2;



States = 3;

x(:,1) =  [0.1;0.1;0.1];   
% Our real plant initial condition       
x_(:,1) = rand(1,3);      % Our estimate initial conidition 


figure
plot(x_(:,1)); hold on;

xc = x_;                 % Set the ideal model as we think it should start

figure(1)
plot(xc); hold on;
title('xc');

P = eye(States);

w=5;                   % process noise
v=100;                 % measurement noise
Q=w^2*eye(States);     % covariance of process
R=v^2*eye(States);                 % covariance of measurement  
% 
G = [1; 1; 1];            % G is the Jacobian of the plant tranfer functions due to the error.
H = eye(3,3);             % H is the Jacobian of the sensor transfer functions due to the variables involved
 steps = 50 ;          % length of simulation
% % 
for i =2:steps          %start @ time=2 
%   % the real plant
%   x(:,i) = [cos(x(2,i-1)*(i-1));sin(x(2,i-1)*(i-1));atan(sin(x(2,i-1)*(i-1))/cos(x(2,i-1)*(i-1)))]
    x(:,i) = [cos(x(2,i-1)*(i-1));sin(x(2,i-1)*(i-1));atan(sin(x(2,i-1)*(i-1))/cos(x(2,i-1)*(i-1)))]
  z(:,i) = x(:,i)+ randn*v;

  %z(i) = x(1,i) 
% 
%   % prediction
  x_(:,i) = [cos(x(2,i-1)*(i-1));sin(x(2,i-1)*(i-1));atan(sin(x(2,i-1)*(i-1))/cos(x(2,i-1)*(i-1)))];
  z_(:,i) = x_(:,i);
% 
%   % compute F
  F = [ i*sin(i*x_(2,i)),0,0;0,i*cos(i*x_(2,i)),0; 0,0, 1];
%   % Prediction of the plant covariance
   P = F*P*F' + G*w*G';
%   % Innovation Covariance
  S = H*P*H'+R
%   % Kalman's gain
 K = P*H'*inv(S)
%   % State check up and update
  x_(:,i) = x_(:,i) + K * (z(:,i)-z_(:,i));
%   
%   % Covariance check up and update
 P = (eye(States)-K*H)*P;
 end
% 
%%
figure
plot(x(1,:),'-b');hold on;                  %plot the real plant behavior
plot(x_(1,:),'--r');hold on;  %plot the Kalman filter prediction over the plant
plot(z(1,:),'-.g'); %plot the observations over this plant 
%%
% figure
%plot(z,'-.g');  hold on;                     
% plot(x_(1,:),'--r');                 

%
figure
plot(x(2,:),'-b');hold on;                  %Frecuency estimation
plot(x_(2,:),'-g');                 %Frecuency filtered by Kalman

  