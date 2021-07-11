clc;clearvars;close all;

N=100;  
a=0.2;
time = -N:N;
sigma_1=50;
sigma_2_plot = [];
xn(1)=0;
x(1)=0;
z(1)=normrnd(0,1);
for t=1:(N-1)
  sigma_2 = randi([0 t],1,1);
  sigma_2_plot = [sigma_2_plot;sigma_2];
  x(t+1) = x(t) + a*t;
  xn(t+1) = x(t+1) + normrnd(0,sigma_1);
  z(t+1) = xn(t+1) + normrnd(0,sigma_2);
end

K = 1;
K_out1(1) = 0;
K_out2(1) = 0;
K_out3(1) = 0;
e_out(1) = sigma_1;
for t = 1:(N-1)
    e_out(t+1) = sqrt(((1-K)*(e_out(t)+sigma_2+sigma_1)));
    K = e_out(t+1)/(e_out(t+1) + (sigma_1));
    K_out1(t+1) = K_out1(t)+a*t+ (K*(z(t+1) - (K_out1(t)+a*t)));
end
plot(K_out1)
hold on
plot(z)
hold on
plot(xn)
hold on
plot(x)
legend('Kalman','Measured','Physical','Ideal');
sum_k = 0;
sum_s = 0;
sum3 = 0;
for i=1:N
    diff_k(i) = (K_out1(i) - x(i))^2;
    sum_k = sum_k+diff_k(i);
    diff_s(i) = (z(i) - x(i))^2;
    sum_s = sum_s+diff_s(i);
%     diff3(i) = (K_out3(i) - x(i))^2;
%     sum3 = sum3+diff3(i);
end
sigma_kout1 = sqrt((sum_k)/(N-1)); 
sigma_z = sqrt((sum_s)/(N-1)); 
sigma_kout3 = sqrt((sum3)/(N-1)); 
figure(2)
subplot(2,1,1)
e_xn = pdf('Normal',time,0,sigma_1);
plot(time,e_xn); hold on
e_z = pdf('Normal',time,0,sqrt(sigma_z));
plot(time,e_z); hold on
e_kout1 = pdf('Normal',time,0,sigma_kout1);
plot(time,e_kout1); hold on
legend('Physical','Measured','Kalman1','K2','K3');
title('PDF errors')
subplot(2,1,2)
plot(sigma_2_plot);
title('Changing Sigma_2');