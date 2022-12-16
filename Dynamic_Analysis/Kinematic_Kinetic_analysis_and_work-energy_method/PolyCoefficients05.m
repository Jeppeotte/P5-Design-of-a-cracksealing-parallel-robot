function PolyCo5 = PolyCoefficients05(xi,xf,yi,yf,dxi,dxf,dyi,dyf,ti,t_f,T_sample)
% clear
% close all
% clc

% yi = 0.4500;
% xi = -0.1;
% yf = 0.2000;
% xf = 0.1;
% dyi = 0;
% dxi = 0;
% dyf = 0;
% dxf = 0;
% ti = 0;
% t_f = 2;

syms a0x a0y a1x a1y a2x a2y a3x a3y 

eqxi = a0x + a1x*(a0y + a1y*ti + a2y*ti^2 + a3y*ti^3) + a2x*(a0y + a1y*ti + a2y*ti^2 + a3y*ti^3)^2 + a3x*(a0y + a1y*ti + a2y*ti^2 + a3y*ti^3)^3 == xi;
eqxf = a0x + a1x*(a0y + a1y*t_f + a2y*t_f^2 + a3y*t_f^3) + a2x*(a0y + a1y*t_f + a2y*t_f^2 + a3y*t_f^3)^2 + a3x*(a0y + a1y*t_f + a2y*t_f^2 + a3y*t_f^3)^3 == xf;
eqyi = a0y + a1y*ti + a2y*ti^2 + a3y*ti^3 == yi;
eqyf = a0y + a1y*t_f + a2y*t_f^2 + a3y*t_f^3 == yf;
d_eqxi = a1x + 2*a2x*(a1y + 2*a2y*ti + 3*a3y*ti^2)*(a0y + a1y*ti + a2y*ti^2 + a3y*ti^3) + 3*a3x*(a1y + 2*a2y*ti + 3*a3y*ti^2)*(a0y + a1y*ti + a2y*ti^2 + a3y*ti^3)^2 == dxi;
d_eqxf = a1x + 2*a2x*(a1y + 2*a2y*t_f + 3*a3y*t_f^2)*(a0y + a1y*t_f + a2y*t_f^2 + a3y*t_f^3) + 3*a3x*(a1y + 2*a2y*t_f + 3*a3y*t_f^2)*(a0y + a1y*t_f + a2y*t_f^2 + a3y*t_f^3)^2 == dxf;
d_eqyi = a1y + 2*a2y*ti + 3*a3y*ti^2 == dyi;
d_eqyf = a1y + 2*a2y*t_f + 3*a3y*t_f^2 == dyf;


S = solve([eqxi,eqxf,eqyi,eqyf,d_eqxi,d_eqxf,d_eqyi,d_eqyf],[a0x,a0y,a1x,a1y,a2x,a2y,a3x,a3y])

a0x = double(S.a0x);
a0y = double(S.a0y);
a1x = double(S.a1x);
a1y = double(S.a1y);
a2x = double(S.a2x);
a2y = double(S.a2y);
a3x = double(S.a3x);
a3y = double(S.a3y);


t = 0:T_sample:t_f; 
n_t = length(t);

x=zeros(n_t,1);
y=zeros(n_t,1);
dx = zeros(n_t,1);
dy = zeros(n_t,1);
ddx = zeros(n_t,1);
ddy = zeros(n_t,1);


for n = 1:n_t
    x(n,1) = a0x + a1x*(a0y + a1y*t(1,n) + a2y*t(1,n)^2 + a3y*t(1,n)^3)...
        + a2x*((a0y + a1y*t(1,n) + a2y*t(1,n)^2 + a3y*t(1,n)^3)^2)...
            + a3x*(a0y + a1y*t(1,n) + a2y*t(1,n)^2 + a3y*t(1,n)^3)^3;
    y(n,1) = a0y + a1y*t(1,n) + a2y*t(1,n)^2 + a3y*t(1,n)^3;

    dx(n,1) = a1x*(a1y + 2*a2y*t(1,n) + 3*a3y*t(1,n)^2)...
        + 2*a2x*(a1y + 2*a2y*t(1,n)...
        + 3*a3y*t(1,n)^2)*(a0y + a1y*t(1,n) + a2y*t(1,n)^2 + a3y*t(1,n)^3)...
        + 3*a3x*(a1y + 2*a2y*t(1,n)...
        + 3*a3y*t(1,n)^2)*(a0y + a1y*t(1,n) + a2y*t(1,n)^2 + a3y*t(1,n)^3)^2;
    dy(n,1) = a1y + 2*a2y*t(1,n) + 3*a3y*t(1,n)^2;

    ddx(n,1) = a1x*(2*a2y + 6*a3y*t(1,n)) + 2*a2x*((a1y + 2*a2y*t(1,n) + 3*a3y*t(1,n)^2)^2)...
        + 2*a2x*(a0y + a1y*t(1,n) + a2y*t(1,n)^2 + a3y*t(1,n)^3)*(2*a2y + 6*a3y*t(1,n))...
        + 6*a3x*(a0y + a1y*t(1,n) + a2y*t(1,n)^2 + a3y*t(1,n)^3)*((3*a3y*t(1,n)^2 + 2*a2y*t(1,n)+a1y)^2)...
        + 3*a3x*(a0y + a1y*t(1,n) + a2y*t(1,n)^2 + a3y*t(1,n)^3)*(2*a2y + 6*a3y*t(1,n));
    ddy(n,1) = 2*a2y + 6*a3y*t(1,n);
end

PolyCo5(:,1) = x(:,1);
PolyCo5(:,2) = y(:,1);
PolyCo5(:,3) = dx(:,1);
PolyCo5(:,4) = dy(:,1);
PolyCo5(:,5) = ddx(:,1);
PolyCo5(:,6) = ddy(:,1);

% %%%%%%%% For plotting use code below %%%%%%%%%%
% figure(1)
% plot(x,y)
% grid on
% xlim([xi-0.1 xf+0.1])
% ylim([0 yf])
% 
% figure(2)
% plot(t,x)
% grid on
% xlabel('tid')
% ylabel('x')
% xlim([0 t_f+0.1])
% ylim([xi-0.1 xf+0.1])
% 
% 
% clear t
% syms t
% 
% xt = a0x + a1x*(a0y + a1y*t + a2y*t^2 + a3y*t^3) + a2x*(a0y + a1y*t + a2y*t^2 + a3y*t^3)^2 + a3x*(a0y + a1y*t + a2y*t^2 + a3y*t^3)^3;
% yt = a0y + a1y*t + a2y*t^2 + a3y*t^3;
% zt = t;
% 
% dxt = a1x + 2*a2x*(a1y + 2*a2y*t + 3*a3y*t^2)*(a0y + a1y*t + a2y*t^2 + a3y*t^3) + 3*a3x*(a1y + 2*a2y*t + 3*a3y*t^2)*(a0y + a1y*t + a2y*t^2 + a3y*t^3)^2;
% dyt = a1y + 2*a2y*t + 3*a3y*t^2;
% 
% 
% figure(3)
% fplot3(xt,yt,zt,[0 t_f])
% xlabel('x')
% ylabel('y')
% zlabel('t')
% 
% figure(4)
% fplot3(dxt,dyt,zt,[0 t_f])
% xlabel('dx')
% ylabel('dy')
% zlabel('t')
% 
