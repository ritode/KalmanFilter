close all;
clear all;
clc;
gearratio=1/21.5;
r2d=180/pi;

wheelodom=load('vechicle_status.txt');
gpsrtk_gps=load('groundtruth.txt');

%%vehicle status%%
wheelodomt=wheelodom(:,1)-wheelodom(1,1);
throttle=wheelodom(:,2)-wheelodom(1,2);
motorrpm=wheelodom(:,3);
brakepot=wheelodom(:,4);
brakepos=wheelodom(:,5);
steerang=wheelodom(:,6);
direction=wheelodom(:,7);
steerang_1=wheelodom(:,6)-wheelodom(1,6);
vspeed_vehicle=(motorrpm*2*pi*30*gearratio)/(60*100);

%%%gpsins RTK enable wheel enable%%
gpsrtk_gpst=gpsrtk_gps(:,1)-gpsrtk_gps(1,1);
gpsrtk_gpsx=gpsrtk_gps(:,2)-gpsrtk_gps(1,2);      
gpsrtk_gpsy=gpsrtk_gps(:,3)-gpsrtk_gps(1,3);
%%roll pitc yaw%%
q2=[gpsrtk_gps(:,4) gpsrtk_gps(:,5) gpsrtk_gps(:,6) gpsrtk_gps(:,7)];
rpy2 = quat2eul(q2);
gpsrtk_gps_roll=rpy2(:,1);
gpsrtk_gps_pitch=rpy2(:,2);
gpsrtk_gps_yaw=rpy2(:,3)-rpy2(1,3);
% gpsrtk_gps_yaw=rpy2(:,3);

%%vx,vtheta%%
gpsrtk_gpsvx=gpsrtk_gps(:,8);
gpsrtk_gps_vtheta=gpsrtk_gps(:,9);

%%%%interpolation%%%%%
gpsrtk_gps_yaw=interp1(gpsrtk_gpst,gpsrtk_gps_yaw,wheelodomt);
gpsrtk_gpsx=interp1(gpsrtk_gpst,gpsrtk_gpsx,wheelodomt);
gpsrtk_gpsy=interp1(gpsrtk_gpst,gpsrtk_gpsy,wheelodomt);
%%%%%%%%%%%%%%%%%%%%%%%%

[X,Y]=ll2utm(12.986392,77.668466);

%IMU based solution%%
% initial pose
x_rear_curr=X;       % initial x-coordinate
y_rear_curr=Y;       % initial y-coordinate

% pose
x_rear=[];
y_rear=[];

p_pre = [0.01 0 0; 0 0.01 0; 0 0 0.001];
r_pre = [0.01 0 0; 0 0.01 0; 0 0 0.001];
x_upd(1) = X;
y_upd(1) = Y;
var_x_gps = gpsrtk_gps(:,10);
var_y_gps = gpsrtk_gps(:,11);
figure(4);
for i=2:length(wheelodomt)
    dt= wheelodom(i,1)- wheelodom(i-1,1);
    dx_rear=vspeed_vehicle(i).*dt.*cos(gpsrtk_gps_yaw(i));
    dy_rear=vspeed_vehicle(i).*dt.*sin(gpsrtk_gps_yaw(i));  %Prediction
    
    x_rear_curr=x_rear_curr+dx_rear;
    y_rear_curr=y_rear_curr+dy_rear;
    x_rear=[x_rear; x_rear_curr];
    y_rear=[y_rear; y_rear_curr];
    
    A = [1 0 -vspeed_vehicle(i)*sin((gpsrtk_gps_yaw(i))); 0 1 vspeed_vehicle(i)*cos((gpsrtk_gps_yaw(i))); 0 0 1];
    B = [1 1 0; -vspeed_vehicle(i)*sin(gpsrtk_gps_yaw(i)) vspeed_vehicle(i)*cos(gpsrtk_gps_yaw(i)) 1];
    W = [0.001*vspeed_vehicle(i) 0; 0 0.0001*(gpsrtk_gps_yaw(i))];
    
    P=A*p_pre*A'+B'*W*B; %Covariance matrix for Prediction model
    p_pre=P;
    var_x_imu(i) = p_pre(1,1);
    var_y_imu(i) = p_pre(2,2);
    
    cov_xy_gps = sqrt(var_x_gps(i)) * sqrt(var_y_gps(i));
    R = [var_x_gps(i) cov_xy_gps 0.0001; cov_xy_gps var_y_gps(i) 0.0001; 0.0001 0.0001 0.0001];
       
%     AR = [1 0 (gpsrtk_gpsx(i)); 0 1 (gpsrtk_gpsy(i)); 0 0 1];
%     BR = [1 1 0;(gpsrtk_gpsx(i)) (gpsrtk_gpsy(i)) 1];
%     WR = [0.001*(gpsrtk_gpsx(i)) 0; 0 0.001*(gpsrtk_gpsy(i))];
%     R = AR*r_pre*AR'+BR'*WR*BR; %Covariance matrix for measurement model
%     var_xx_gps(i) = r_pre(1,1);
%     var_yy_gps(i) = r_pre(2,2);
    r_pre = R;
    C = [gpsrtk_gpsx(i),gpsrtk_gpsy(i),1];
    K = (P*C')/(C*P*C') + R;
    x_upd(i) = x_upd(i-1) + K(1,1)*(gpsrtk_gpsx(i) - x_rear_curr);
    y_upd(i) = y_upd(i-1) + K(2,2)*(gpsrtk_gpsy(i) - y_rear_curr);
    if i==2
        plot(x_upd(i),y_upd(i),'*');
        hold on;
    end
    plot(x_upd(i),y_upd(i));hold on;
end
figure(1)
plot(x_rear,y_rear,'r--');hold on;grid on;
xlabel('X(m)');
ylabel('Y(m)');
title('Odometry');
 
figure(2)
plot(gpsrtk_gpsy(1),-gpsrtk_gpsx(1),'b*',gpsrtk_gpsy,-gpsrtk_gpsx,'--',gpsrtk_gpsy(end),-gpsrtk_gpsx(end),'g*');grid on;
xlabel('X(m)');
ylabel('Y(m)');
title('GPS');
axis equal;

figure(3)
plot(gpsrtk_gpst,var_x_gps);hold on; grid on;
plot(gpsrtk_gpst,var_y_gps);hold on; grid on;
xlabel('t(s)');
ylabel('DOP');
legend('X-DOP','Y-DOP');

% figure(4)
% plot(x_upd,y_upd);

% X = zeros(2,1);
% 
% % covariance matrix
% P = zeros(2,2);
% 
% % kalman filter output through the whole time
% X_arr = zeros(length(xy_gps), 2);
% 
% % transition matrix
% F = [1 dt;
%      0 1]; 
% 
% % observation matrix 
% H = [1 0];
% 
% %noise matrix
% Q = [0.04 0;
%      0 1];
% 
% % fusion
% for i = 1:length(xy_gps)
%     if (i == 1)
%         [X, P] = init_kalman(X, xy_gps(i,1)); % initialize the state using the 1st sensor
%     else
%         [X, P] = prediction(X, P, Q, F);
% 
%         [X, P] = update(X, P, xy_gps(i, 1), xy_gps(i, 2), H);
%         [X, P] = update(X, P, xy_imu(i, 1), xy_imu(i, 2), H);
%     end
% 
%     X_arr(i, :) = X;
% end
% 
% figure(4)
% plot(X_arr(:, 1),X_arr(:, 2),'.');
% 
% function [X, P] = init_kalman(X, y)
%     X(1,1) = y;
%     X(2,1) = 0;
% 
%     P = [100 0;
%          0 300];
% end
% 
% function [X, P] = prediction(X, P, Q, F)
%     X = F*X;
%     P = F*P*F' + Q;
% end
% 
% function [X, P] = update(X, P, y, R, H)
%     Inn = y - H*X;
%     S = H*P*H' + R;
%     K = P*H'/S;
% 
%     X = X + K*Inn;
%     P = P - K*H*P;
% end