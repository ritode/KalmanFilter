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

%%var_x,var_y%%
var_x_gps = gpsrtk_gps(:,10);
var_y_gps = gpsrtk_gps(:,11);


%%%%interpolation%%%%%
gpsrtk_gps_yaw=interp1(gpsrtk_gpst,gpsrtk_gps_yaw,wheelodomt);
%imu_y=interp1(imut,imu_y,wheelodomt,'spline');

%%%%%%%%%%%%%%%%%%%%%%%%

%[X,Y]=ll2utm(12.98624028,77.668548677);

% [X,Y]=ll2utm(12.986392,77.668466);
X = 789482.58;
Y = 1437146.53;
%IMU based solution%%

%% IMU based localization
% initial pose
x_rear_curr=X;       % initial x-coordinate
y_rear_curr=Y;       % initial y-coordinate

% pose
x_rear=[];
y_rear=[];

p_pre=[0.01 0 0; 0 0.01 0; 0 0 0.001];
for i=2:length(wheelodomt)
    
   dt= wheelodom(i,1)- wheelodom(i-1,1);
 
   %dtheta_rear=angularspeed_vehicle_orient_inter(i).*dt_rear;
    
    %dx_rear=vspeed_vehicle(i).*dt_rear.*cos(theta_rear_curr+dtheta_rear);
    dx_rear=vspeed_vehicle(i).*dt.*cos(gpsrtk_gps_yaw(i));
% 
    %dy_rear=vspeed_vehicle(i).*dt_rear.*sin(theta_rear_curr+dtheta_rear);
    dy_rear=vspeed_vehicle(i).*dt.*sin(gpsrtk_gps_yaw(i));
%     
      x_rear_curr=x_rear_curr+dx_rear;
      y_rear_curr=y_rear_curr+dy_rear;
      x_rear=[x_rear; x_rear_curr];
      y_rear=[y_rear; y_rear_curr];
      A = [1 0 -vspeed_vehicle(i)*sin((gpsrtk_gps_yaw(i))); 0 1 vspeed_vehicle(i)*cos((gpsrtk_gps_yaw(i))); 0 0 1];
      B = [1 1 0; -vspeed_vehicle(i)*sin(gpsrtk_gps_yaw(i)) vspeed_vehicle(i)*cos(gpsrtk_gps_yaw(i)) 1];
      W=[0.001*vspeed_vehicle(i) 0; 0 0.0001*(gpsrtk_gps_yaw(i))];

      P=A*p_pre*A'+B'*W*B;
      p_pre=P;
      var_x_imu(i) = p_pre(1,1);
      var_y_imu(i) = p_pre(2,2);
   
end
figure(1)
plot(x_rear,y_rear,'r--');hold on;grid on;
%plot(-gpsrtk_gpsy(1),gpsrtk_gpsx(1),'r*',-gpsrtk_gpsy,gpsrtk_gpsx,'b--',-gpsrtk_gpsy(end),gpsrtk_gpsx(end),'c*');grid on;
xlabel('X(m)');
ylabel('Y(m)');

%legend('Highcost','Model');
% 
figure(2)
%plot(x_rear(1),y_rear(1),'b*',x_rear,y_rear,'r--',x_rear(end),y_rear(end),'g*');hold on;grid on;
plot(gpsrtk_gpsy(1),-gpsrtk_gpsx(1),'b*',gpsrtk_gpsy,-gpsrtk_gpsx,'--',gpsrtk_gpsy(end),-gpsrtk_gpsx(end),'g*');grid on;
xlabel('X(m)');
ylabel('Y(m)');
axis equal;

figure(3)
plot(gpsrtk_gpst,var_x_gps);hold on; grid on;
plot(gpsrtk_gpst,var_y_gps);hold on; grid on;
xlabel('t(s)');
ylabel('DOP');

legend('X-DOP','Y-DOP');

[LAT,LON]=utm2ll(x_rear, y_rear,43); 
dlmwrite('underparking.txt',[LAT,LON],'precision','%.8f');

[LAT1,LON1]=utm2ll(gpsrtk_gps(:,2), gpsrtk_gps(:,3),43); 
dlmwrite('GPSINSunderparking.txt',[LAT1,LON1],'precision','%.8f');

xerror = x_rear(1)-x_rear(end);
yerror = y_rear(1)-y_rear(end);

figure(4)
push = 0;
for i = 658:165:length(var_x_imu)
    eqnimu = (@(x,y)((((x-push).^2)/(var_x_imu(i)))+(((y).^2)/(var_y_imu(i)))-1));
    fimplicit(eqnimu); 
    imu_ellipse = fimplicit(eqnimu); 
    imu_ellipse_x = get(imu_ellipse, 'XData');
    imu_ellipse_y = get(imu_ellipse, 'YData');
    plot(imu_ellipse_x,imu_ellipse_y); hold on;grid on;
    push = push+900;
%     xmin = min(imu_ellipse_x);
%     if isempty(xmin)
%         xmin = -100;
%         xmax = 100;
%         ymin = -100;
%         ymax = 100;
%     else
%         xmin = min(imu_ellipse_x);
%         xmax = max(imu_ellipse_x);
%         ymin = min(imu_ellipse_y);
%         ymax = max(imu_ellipse_y);
%     end
%     xlim([xmin xmax]);
%     ylim([ymin ymax]);
    xlim([-1000 15000]);
    ylim([-1000 1000]);
end

% figure(5)
% push1 = 0;
% for i = 1:329:length(var_x_gps)
%     eqngps = (@(x,y)((((x-push1).^2)/(var_x_gps(i)))+(((y).^2)/(var_y_gps(i)))-1));
%     gps_ellipse = fimplicit(eqngps); 
%     gps_ellipse_x = get(gps_ellipse, 'XData');
%     gps_ellipse_y = get(gps_ellipse, 'YData');
%     plot(gps_ellipse_x,gps_ellipse_y); hold on;grid on;
%     push1 = push1+100;
%     xlim([-10000 10000]);
%     ylim([-10000 10000]);
% end

% for i=2:length(x_rear)
%     if (i<=5)
%         covariance = cov(x_rear(2:i),y_rear(2:i));
%         var_x(i) = covariance(1,1);
%         var_y(i) = covariance(2,2);
%         cov_xy(i) = covariance(1,2);
%     else
%         covariance = cov(x_rear(i-5:i),y_rear(i-5:i));
%         var_x(i) = covariance(1,1);
%         var_y(i) = covariance(2,2);
%         cov_xy(i) = covariance(1,2);
%     end
% end

% figure(4)
% unc_imu_x_array = [];
% unc_imu_y_array = [];
% 
% for i=1:length(var_x)
%     lambda_1(i) = 100*(var_x(i) + var_y(i))./2 + sqrt(((var_x(i) - var_y(i))./2).^2 + cov_xy(i).^2);
%     lambda_2(i) = 100*(var_x(i) + var_y(i))./2 - sqrt(((var_x(i) - var_y(i))./2).^2 + cov_xy(i).^2);
% end
% 
% for i= 1:length(lambda_1)
%     unc_imu = (@(x,y)((((x-x_rear(i)).^2)/(lambda_1(i)))+(((y).^2)/(lambda_2(i)))-1));
%     fimplicit(unc_imu);hold on
% %     unc_imu_x = get(uncertainty_imu, 'XData');
% %     unc_imu_y = get(uncertainty_imu, 'YData');
% %     unc_imu_x_array = [unc_imu_x_array, unc_imu_x];
% %     unc_imu_y_array = [unc_imu_y_array, unc_imu_y];
% end