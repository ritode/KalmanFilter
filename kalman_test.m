clc;close all
t = 0:0.1:50;
pdf_x = -70:70;
phy(1) = 0;
meas(1) = 0;
acc = 2;
p_sigma = 10;
m_sigma = 30;
p_noise = randn(1,length(t));
m_noise = randn(1,length(t));
p_noise_pdf = pdf('Normal',pdf_x,0,p_sigma);
m_noise_pdf = pdf('Normal',pdf_x,0,m_sigma);
subplot(2,1,1);
plot(pdf_x,p_noise_pdf)
subplot(2,1,2);
plot(pdf_x,m_noise_pdf)

for i = 1:(length(t)-1)
    phy(t+1) = acc*phy(t) + p_noise;
    
end
