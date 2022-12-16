```matlab:Code
clc
clear
close all
syms L
```

Inertimomenter

```matlab:Code
%Højde, bredde, længde og tykkelse af profilet:
a = 40/1000; %m
b = 40/1000; %m
c = L-40/1000; %m
t = 4/1000; %m

%Dimoensioner på nozzel:
a_2 = 120/1000; %m
b_2 = 40/1000; %m
c_2 = 40/1000; %m
V_nozzel = a_2*b_2*c_2; %Volumen for nozzel
M_nozzel = 0.5; %kg - Vægt af nozzel, det havle af den faktiske grundet beregningerne er på 1 arm

%Volumen for den ydre volumen:
V_M = a*b*c;

%Volumen for den indre volumen
V_m = (a-2*t)*(b-2*t)*c;

%Densitet af materialet:
rho = 2700; %kg/m^3

%Vægt af profilet ud fra volumen og den sitet
M_profil = (V_M - V_m) * rho; %kg

%Inertimoment for armen med omdrejning omkring monteringspunkt
I_1 = 1/12*(V_M*rho)*(b^2 + c^2) - 1/12*(V_m*rho)*((b - 2*t)^2 + c^2) + M_profil*(c/2)^2;

%Inertimoment for nozzel med omdrejning omkring monteringspunkt
I_2 = 1/12*M_nozzel*(b_2^2 + c_2^2) + M_nozzel*(L+20/1000)^2;

%Samlet inertimoment
I = I_1 + I_2;

%Maksimale torque fra motoren
tau_max = 177*43/1000; %Nm

%Ligningen for torque ud fra inertimoment og vinkelacceleration
tau = I * a;

%Den højeste vinkel accelerationen hvis motoren skal bevæge sig 180grader
%på 1 sekund, hvor den accelerere og decelerere
a = 17.61 %rad/s^2

%assume(L,'real')
S = solve(tau_max == I * a,L);
vpa(S)

```
