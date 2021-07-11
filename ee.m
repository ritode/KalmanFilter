close all;
clc;
clear all;
% l2 = [1 2 3];
% l1 = [3 2 1];
% for i = 1:length(l1)
%     unc_imu = (@(x,y)((((x-i).^2)/(l1(i)))+(((y).^2)/(l2(i)))-1));
%     fimplicit(unc_imu); hold on;grid on;
%     xlim([-10 10]);
%     ylim([-10 10]);
% end
clc;clearvars;close all;
t = 0:0.1:(60*60);
v = 3;  % in m/s
theta = 0;
theta = ((1/(60 * 60)) * (pi/180)) * t;
bias = 0;
%%IMU based solution%%

%% IMU based localization
% initial pose
x_rear_curr=0;       % initial x-coordinate
y_rear_curr=0;       % initial y-coordinate

% pose
x_rear=[];
y_rear=[];

p_pre=[0.01 0 0; 0 0.01 0; 0 0 0.001];

for i=2:length(t)
    
   % dt= gpsrtk(i,1)- gpsrtk(i-1,1);
    dt = 0.01;
   %dtheta_rear=angularspeed_vehicle_orient_inter(i).*dt_rear;
    
    %dx_rear=vspeed_vehicle(i).*dt_rear.*cos(theta_rear_curr+dtheta_rear);
    dx=v*dt*cos(theta(i) + bias);
% 
    %dy_rear=vspeed_vehicle(i).*dt_rear.*sin(theta_rear_curr+dtheta_rear);
    dy=v*dt*sin(theta(i) + bias);
%     
    x_rear_curr=x_rear_curr+dx;
    y_rear_curr=y_rear_curr+dy;
    x_rear=[x_rear; x_rear_curr];
    y_rear=[y_rear; y_rear_curr];

    A=[1 0 -v*sin((theta(i))); 0 1 v*cos((theta(i))); 0 0 1];
    B = [1 1 0; -v*sin(theta(i)) v*cos(theta(i)) 1];

    W=[0.001*v 0; 0 0.0001*(theta(i))];

    P=A*p_pre*A'+B'*W*B;
    p_pre=P;
    var_x(i) = p_pre(1,1);
    var_y(i) = p_pre(2,2);  
   
end
push = 1;
for i = 1:2000:length(var_x)
    eqn = (@(x,y)((((x-push).^2)/(var_x(i)))+(((y).^2)/(var_y(i)))-1));
    ellipse = fimplicit(eqn);
    x_ellipse = get(ellipse,'XData');
    y_ellipse = get(ellipse,'YData');
    push = push+100;
    plot(x_ellipse,y_ellipse); hold on; grid on
%     x1 = min(x_ellipse);
%     x2 = max(x_ellipse);
%     xlim([x1 x2]);
%     y1 = min(y_ellipse);
%     y2 = max(y_ellipse);
%     ylim([y1 y2]);
    xlim([-10 2000]);
    ylim([-10000 10000]);
end

% figure
% plot(x_rear, y_rear,'b--'); grid on; hold on;
% xlabel('X (m)');
% ylabel('Y (m)');
% 



























































% plot(gpsrtkx,gpsrtky,'r--'); grid on; hold on;
% bias1 = 0;
% theta1 = ((1/(60 * 60)) * (pi/180)) * t;    % 1 deg per hour deviation (datasheet) = 0.001666 deg per minute = 0.00002777 deg per second
% 
% 
% x_rear_curr1=0;       % initial x-coordinate
% y_rear_curr1=0;       % initial y-coordinate
% 
% % pose
% x_rear1=[];
% y_rear1=[];
% for i=2:length(t)
%     
%    % dt_rear= wheelodom(i,1)- wheelodom(i-1,1);
%    dt = 0.01;
%  
%    %dtheta_rear=angularspeed_vehicle_orient_inter(i).*dt_rear;
%     
%     %dx_rear=vspeed_vehicle(i).*dt_rear.*cos(theta_rear_curr+dtheta_rear);
%     dx_rear1=v*dt*cos(theta1(i) + bias1);
% % 
%     %dy_rear=vspeed_vehicle(i).*dt_rear.*sin(theta_rear_curr+dtheta_rear);
%     dy_rear1=v*dt*sin(theta1(i) + bias1);
% %     
%       x_rear_curr1=x_rear_curr1+dx_rear1;
%       y_rear_curr1=y_rear_curr1+dy_rear1;
%       x_rear1=[x_rear1; x_rear_curr1];
%       y_rear1=[y_rear1; y_rear_curr1];
%    
% end
% plot(x_rear1, y_rear1,'r--'); grid on; hold on;
% legend('Ideal', 'With IMU error');
% xlabel('X (m)');
% ylabel('Y (m)');
% 
% % Plot for theta
% figure
% plot(t, theta * ones(size(t))* (180 / pi), 'b'); hold on; grid on;
% % plot(t, theta1 * (180 / pi), 'r--'); hold on; grid on;
% % legend('Ideal', 'Actual');
% xlabel('time (s)');
% ylabel('theta (deg)');
% % 
% disp('Standard Deviation in y_rear (with noise vs without noise)');
% data_deviation_y = y_rear1 - y_rear;
% std = sqrt((data_deviation_y - mean(data_deviation_y))' * (data_deviation_y - mean(data_deviation_y))/size(data_deviation_y,1));
% disp(std);
% 
% 
% data_deviation_x = x_rear1 - x_rear;
% std1 = sqrt((data_deviation_x - mean(data_deviation_x))' * (data_deviation_x - mean(data_deviation_x))/size(data_deviation_x,1));
% disp(std1);

