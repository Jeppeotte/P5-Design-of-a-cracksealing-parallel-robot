# Kinematics

```matlab:Code
clear
clc
close all
```

Lengths:

```matlab:Code
L_0=0.1;
L_1=0.3325;
L_2=L_1;
L_3=0.3825;
L_4=L_3;

L=[L_0,L_1,L_2,L_3,L_4];
```

## Inverse kinematics

Generating x and y coordinates of the tool point

PolyCo1 is a 3rd degree polynomial in x wrt y and y is linear in time i.e. constant velocity in y. 

PolyCo4 - x and y are linear wrt to each other, but are both 3rd degree polynomials wrt time.

PloyCo5 - y is a 3rd degree polynomial wrt time and x is a 9th degree polynomial wrt time duo to x being a 3rd degree polynomial wrt y.

```matlab:Code
yi = 0.4500;
xi = -0.1;
yf = 0.2000;
xf = 0.1;
dyi = 0;
dxi = 0;
dyf = 0;
dxf = 0;
ti = 0;
t_f = 2;

T_sample = 1/19*t_f;
t = 0:T_sample:t_f; 
n_t = length(t);

move_var = PolyCoefficients05(xi,xf,yi,yf,dxi,dxf,dyi,dyf,ti,t_f,T_sample);
x(:,1) = move_var(:,1);
y(:,1) = move_var(:,2);
dx(:,1) = move_var(:,3);
dy(:,1) = move_var(:,4);
ddx(:,1)= move_var(:,5);
ddy(:,1)= move_var(:,6);

```

Plotting tool-path for visual check.

```matlab:Code
plot(x,y)
grid on
xlim([-0.5 0.5])
ylim([0 0.9])

```

### Position

\hfill \break

```matlab:Code
Theta=InversePosition(x,y,n_t,L)
```

Arrays for each configuration

```matlab:Code
 theta_pm=Theta(:,1:2);
% theta_mp=Theta(:,3:4);
% theta_mm=Theta(:,5:6);
% theta_pp=Theta(:,7:8);
```

Plot configurations:

- The black line is the tool-path

```matlab:Code
xlimit=[-0.5 0.5];
ylimit=[-0.3 1];
%Config. pm:
% ConfigPlot(n_t,x,y,L,theta_pm(:,1),theta_pm(:,2),xlimit,ylimit)
% hold off
% %Config. mp
% ConfigPlot(n_t,x,y,L,theta_mp(:,1),theta_mp(:,2),xlimit,ylimit)
% hold off
% %Config. mm
% ConfigPlot(n_t,x,y,L,theta_mm(:,1),theta_mm(:,2),xlimit,ylimit)
% hold off
% %Config. pp
% ConfigPlot(n_t,x,y,L,theta_pp(:,1),theta_pp(:,2),xlimit,ylimit)
% hold off
% for p=1:n_t
%     B1_x=L(1,2)*cos(Theta(p,1))-L(1,1)/2;
%     B1_y=L(1,2)*sin(Theta(p,1));
%     B2_x=L(1,2)*cos(Theta(p,2))+L(1,1)/2;
%     B2_y=L(1,2)*sin(Theta(p,2));
%     P_x = x(p,1);
%     P_y = y(p,1);
% 
%     B1P = [B1_x - P_x;B1_y - P_y];
%     L_B1P=sqrt(B1P(1,1)^2 + B1P(2,1)^2)
% end

```

  

Plotting angular position w.r.t time

```matlab:Code
plot(t,Theta(:,1),t,Theta(:,2))
legend('\theta_1','\theta_2')
grid on
title('Angular Position')
hold off
```

Singularity analysis

```matlab:Code
%Til en prototype? næ nej du
```

### Velocity

\hfill \break

```matlab:Code
dTheta = InverseVelocity(L,x,y,dx,dy,Theta,n_t)

plot(t,dTheta(:,1),t,dTheta(:,2))
title('Angular Velocity')
legend('d\theta_1','d\theta_2')
grid on
hold off
```

### Acceleration

\hfill \break

```matlab:Code
ddTheta = InverseAcceleration(x,y,dx,dy,ddx,ddy,L,Theta,dTheta,n_t)

plot(t,ddTheta(:,1),t,ddTheta(:,2))
title('Angular Acceleration')
legend('dd\theta_1','dd\theta_2')
grid on
hold off
```

## Forward Kinematics

```matlab:Code
x_F = zeros(n_t,1);
y_F1 = zeros(n_t,1);
y_F2 = zeros(n_t,1);
x_error = zeros(n_t,1);
y_error1 = zeros(n_t,1);
y_error2 = zeros(n_t,1);

for n=1:n_t
    e = (L_1*(sin(Theta(n,1)) - sin(Theta(n,2))))/(L_0 + L_1*cos(Theta(n,2)) - L_1*cos(Theta(n,1)));
    f = (L_1*L_0/2*(cos(Theta(n,2)) + cos(Theta(n,1))))/(L_0 + L_1*cos(Theta(n,2)) - L_1*cos(Theta(n,1)));
    d = 1 + e^2;
    g = 2*(e*f - e*L_1*cos(Theta(n,1)) + e*L_0/2 - L_1*sin(Theta(n,1)));
    h = f^2 - 2*f*(L_1*cos(Theta(n,1)) - L_0/2) - 2*L_1*L_0/2*cos(Theta(n,1)) + (L_0/2)^2 + L_1^2 - L_3^2;
    
    y_F1(n,1) = (-g - 1*sqrt(g^2 - 4*d*h))/(2*d);
    y_F2(n,1) = (-g + 1*sqrt(g^2 - 4*d*h))/(2*d);
    x_F(n,1) = e*y(n,1) + f;
    
    x_error(n,1) = (x_F(n,1) - x(n,1))/x(n,1)*100;
    y_error1(n,1) = (y_F1(n,1) - y(n,1))/y(n,1)*100;
    y_error2(n,1) = (y_F2(n,1) - y(n,1))/y(n,1)*100; 
end

```

  
# Kinetics

Mass in kg and mass moment of inertia in m*kg^2.  (Notice the point of rotation given in the subscript.)

Mass matrix:

```matlab:Code
m1 = 1.56*L_1; J1_A1 = 19.16943*10^-3; m2 = 1.56*L_2; J2_A2=J1_A1; 
m3=1.56*L_3; J3_G=17.9963*10^-3; m4=1.56*L_4; J4_G=J3_G;

% Vector with diagonal elements in mass matrix
q_m=[m1,m1,J1_A1,m2,m2,J2_A2,m3,m3,J3_G,m4,m4,J4_G].';
% Mass matrix
M=diag(q_m);
```

Size of vectors and accelerations:

```matlab:Code
% Local vectors
A1G1 = zeros(2,n_t);
h_A1G1 = zeros(2,n_t);
A2G2 = zeros(2,n_t);
h_A2G2 = zeros(2,n_t);

A1B1 = zeros(2,n_t);
h_A1B1 = zeros(2,n_t);
A2B2 = zeros(2,n_t);
h_A2B2 = zeros(2,n_t);

theta3 =zeros(n_t,1);
theta4 = zeros(n_t,1);
B1P  = zeros(2,n_t);
h_B1P = zeros(2,n_t);
B2P  = zeros(2,n_t);
h_B2P = zeros(2,n_t);

G3B1 = zeros(2,n_t);
G3P  = zeros(2,n_t);

G4B2 = zeros(2,n_t);
G4P  = zeros(2,n_t);

PG3  = zeros(2,n_t);
h_PG3= zeros(2,n_t);

PG4  = zeros(2,n_t);
h_PG4= zeros(2,n_t);

% Acceleration:
aG1 = zeros(2,n_t);
aG2 = zeros(2,n_t);

aB1 = zeros(2,n_t);
aB2 = zeros(2,n_t);

aG3 = zeros(2,n_t);
aG4 = zeros(2,n_t);

aP  = zeros(2,n_t);

dd_q = zeros(12,n_t);

% Force-vector
g = zeros(12,n_t);

% Equivalent radius of gyration for body 3 and 4 from point B
r_eq = 0.3173678954; %[m]
```

Calculating 

```matlab:Code
for n=1:n_t
%Local vectors: (h_xx indicates a CCW rotated vector by pi/2)
    A1G1(:,n) = [L_1/2*cos(Theta(n,1));L_1/2*sin(Theta(n,1))];
    h_A1G1(:,n) = [-A1G1(2,n);A1G1(1,n)];
    A2G2(:,n) = [L_2/2*cos(Theta(n,2));L_2/2*sin(Theta(n,2))];
    h_A2G2(:,n) = [-A2G2(2,n);A2G2(1,n)];
    
    A1B1(:,n) = [L_1*cos(Theta(n,1));L_1*sin(Theta(n,1))];
    h_A1B1(:,n) = [-A1B1(2,n);A1B1(1,n)];
    A2B2(:,n) = [L_2*cos(Theta(n,2));L_2*sin(Theta(n,2))];
    h_A2B2(:,n) = [-A2B2(2,n);A2B2(1,n)];

    TempVar1 = L_0+L_1*(cos(Theta(n,2)) - cos(Theta(n,1)));
    TempVar2 = L_1*abs(sin(Theta(n,1)) - sin(Theta(n,2)));
    Beta1 = acos(sqrt(TempVar1^2+TempVar2^2)/(2*L_3));
    Beta2 = atan(TempVar2/TempVar1);
    theta3(n,1) = Beta1+Beta2;
    theta4(n,1) = pi-Beta1+Beta2;
    B1P(:,n)  = [L_3*cos(theta3(n,1));L_3*sin(theta3(n,1))];
    h_B1P(:,n)= [-B1P(2,n);B1P(1,n)];
    B2P(:,n)  = [L_4*cos(theta4(n,1));L_4*sin(theta4(n,1))];
    h_B2P(:,n)= [-B2P(2,n);B2P(1,n)];
    
    G3B1(:,n) = [-r_eq*cos(theta3(n,1));-r_eq*sin(theta3(n,1))];
    G3P(:,n)  = [(L_3-r_eq)*cos(theta3(n,1));(L_3-r_eq)*sin(theta3(n,1))];
    PG3(:,n)  = -G3P(:,n);
    h_PG3(:,n)= [-PG3(2,n);PG3(1,n)];

    G4B2(:,n) = [-r_eq*cos(theta4(n,1));-r_eq*sin(theta4(n,1))];
    G4P(:,n)  = [(L_4-r_eq)*cos(theta4(n,1));(L_4-r_eq)*sin(theta4(n,1))];
    PG4(:,n)  = -G4P(:,n);
    h_PG4(:,n)= [-PG4(2,n);PG4(1,n)];
```

Acceleration

```matlab:Code
%Acceleration of B joints and P
    aB1(:,n) = ddTheta(n,1)*h_A1B1(:,n) - dTheta(n,1)*A1B1(:,n);
    aB2(:,n) = ddTheta(n,2)*h_A2B2(:,n) - dTheta(n,2)*A2B2(:,n);
    aP(:,n)  = [ddx(n,1);ddy(n,1)];

%Acceleration of center of mass:
    aG1(:,n) = ddTheta(n,1)*h_A1G1(:,n) - dTheta(n,1)*A1G1(:,n);
    aG2(:,n) = ddTheta(n,2)*h_A2G2(:,n) - dTheta(n,2)*A2G2(:,n);

    %Solving for acceleration og center of mass and angular acceleration.
    %The vectors have a [ddx;ddy;ddTheta] data structure.
    aG3 = AccelerationCenter(ddx(n,1),ddy(n,1),aB1(:,n),B1P(:,n),h_B1P(:,n),PG3(:,n),h_PG3(:,n));
    aG4 = AccelerationCenter(ddx(n,1),ddy(n,1),aB2(:,n),B2P(:,n),h_B2P(:,n),PG4(:,n),h_PG4(:,n));

%Matrix with accelerations (each coloum is one time-step) 
    dd_q(:,n) = [aG1(:,n);ddTheta(n,1);aG2(:,n);ddTheta(n,2);aG3;aG4];

```

Mass * acceleration in a vector: 

```matlab:Code
    g0(:,n) = M*dd_q(:,n);
```

System of equation to calculate forces:

```matlab:Code
syms fA1x fA1y fB1x fB1y fA2x fA2y fB2x fB2y fPx fPy tauA1 tauA2
g1 = fA1x - fB1x - g0(1,n) == 0;
g2 = fA1y - fB1y - g0(2,n) == 0;
g3 = tauA1 - A1B1(1,n)*fB1y - A1B1(2,n)*fB1x - g0(3,n) == 0;

g4 = fA2x - fB2x - g0(4,n) == 0;
g5 = fA2y - fB2y - g0(5,n) == 0;
g6 = tauA2 - A2B2(1,n)*fB2y - A2B2(2,n)*fB2x - g0(6,n) == 0;

g7 = fB1x + fPx - g0(7,n) == 0;
g8 = fB1y + fPy - g0(8,n) == 0;
g9 = G3B1(1,n)*fB1y - G3B1(2,n)*fB1x + G3P(1,n)*fPy - G3P(1,n)*fPx - g0(9,n) == 0;

g10 = fB2x - fPx - g0(10,n) == 0;
g11 = fB1y - fPy - g0(11,n) == 0;
g12 = G4B2(1,n)*fB2y - G4B2(2,n)*fB2x - G4P(1,n)*fPy - G4P(1,n)*fPx - g0(12,n) == 0;

g_eq = [g1,g2,g3,g4,g5,g6,g7,g8,g9,g10,g11,g12];
g_var = [fA1x, fA1y, fB1x, fB1y, fA2x, fA2y, fB2x, fB2y, fPx, fPy, tauA1, tauA2];

%Solving for forces:
S_f = solve(g_eq,g_var);

% Forces is given as double-precision numbers i g, each coloum is a
% time-step
g(:,n) = [double(S_f.fA1x); double(S_f.fA1y); double(S_f.fB1x);
double(S_f.fB1y); double(S_f.fA2x); double(S_f.fA2y); double(S_f.fB2x);
double(S_f.fB2y); double(S_f.fPx); double(S_f.fPy); double(S_f.tauA1);
double(S_f.tauA2)];
end
```

# Work-Energy validation

This will be prefomed for a specific amount of time, which is specifed by the sampling time

```matlab:Code
clc
ni = n_t/2; %Inital time step
nf = ni+1;  % Final time step
```

Size of energy and work vector: 

```matlab:Code
E1 = zeros(n_t,1);
E2 = zeros(n_t,1);
E3 = zeros(n_t,1);
E4 = zeros(n_t,1);
W1 = zeros(n_t,1);
W2 = zeros(n_t,1);
```

Position vectors for Instantaneous Center of Zero Velocity (ICoZV)

```matlab:Code
for n = ni:nf
    OB1 = [L_1*cos(Theta(n,1)) - L_0/2; L_1*sin(Theta(n,1))];
    OB2 = [L_2*cos(Theta(n,2)) + L_0/2; L_2*sin(Theta(n,2))];
    OP = [x(n,1);y(n,1)];

    OG3 = OB1 + Az(theta3(n,1))*[r_eq;0];
    OG4 = OB2 + Az(theta4(n,1))*[r_eq;0];

    %h_A1B1(:,ni) is defined in the kinetics calculations.
    %h_A2B2(:,ni) is defined in the kinetics calculations.

    vB1 = dTheta(n,1)*h_A1B1(:,n);
    vB2 = dTheta(n,1)*h_A2B2(:,n);

    h_vB1 = [-vB1(2,1);vB1(1,1)];
    h_vB2 = [-vB2(2,1);vB2(1,1)];
    h_vP = [-dy(n,1);dx(n,1)];

    syms rB1 rB2 rP 

    VCenter3 = solve([OB1(1,1)+rB1*h_vB1(1,1)==OP(1,1)+rP*h_vP(1,1), OB1(2,1)+rB1*h_vB1(2,1)==OP(2,1)+rP*h_vP(2,1)],[rB1,rP]);
    VCenter4 = solve([OB2(1,1)+rB2*h_vB2(1,1)==OP(1,1)+rP*h_vP(1,1), OB2(2,1)+rB2*h_vB2(2,1)==OP(2,1)+rP*h_vP(2,1)],[rB2,rP]);

    %Body 3
    rB1 = double(VCenter3.rB1);
    rP3 = double(VCenter3.rP);

    %Body 4
    rB2 = double(VCenter4.rB2);
    rP4 = double(VCenter4.rP);
```

Plotting to visualize

```matlab:Code
%     % Position vecotrs are red
%     plot([0;OP(1,1);0;OB1(1,1);0;OB2(1,1)],[0;OP(2,1);0;OB1(2,1);0;OB2(2,1)],'r')
%     hold on
%     % Velocities are green
%     plot([OB1(1,1);OB1(1,1)+vB1(1,1)],[OB1(2,1);OB1(2,1)+vB1(2,1)],'g')
%     plot([OB2(1,1);OB2(1,1)+vB2(1,1)],[OB2(2,1);OB2(2,1)+vB2(2,1)],'g')
%     plot([OP(1,1);OP(1,1)+dx(ni,1)],[OP(2,1);OP(2,1)+dy(ni,1)],'g')
%     
%     %Orthogonal velocity vectors are magenta
%     plot([OB1(1,1);OB1(1,1)-h_vB1(1,1)],[OB1(2,1);OB1(2,1)-h_vB1(2,1)],'m')
%     plot([OB1(1,1);OB1(1,1)+h_vB1(1,1)],[OB1(2,1);OB1(2,1)+h_vB1(2,1)],'m')
%    
%     plot([OB2(1,1);OB2(1,1)-h_vB2(1,1)],[OB2(2,1);OB2(2,1)-h_vB2(2,1)],'m')
%     plot([OB2(1,1);OB2(1,1)+h_vB2(1,1)],[OB2(2,1);OB2(2,1)+h_vB2(2,1)],'m')
%     
%     plot([OP(1,1);OP(1,1)-h_vP(1,1)],[OP(2,1);OP(2,1)-h_vP(2,1)],'m')
%     plot([OP(1,1);OP(1,1)+h_vP(1,1)],[OP(2,1);OP(2,1)+h_vP(2,1)],'m')
%     
%     %Point C (ICoZV) is where the magenta lines inersect
%     plot(OB1(1,1)+rB1*h_vB1(1,1),OB1(2,1)+rB1*h_vB1(2,1),'o')
%     plot(OP(1,1)+rP3*h_vP(1,1),OP(2,1)+rP3*h_vP(2,1),'x')
%     
%     plot(OB2(1,1)+rB2*h_vB2(1,1),OB2(2,1)+rB2*h_vB2(2,1),'o')
%     plot(OP(1,1)+rP4*h_vP(1,1),OP(2,1)+rP4*h_vP(2,1),'x')
%     
%     xlim([-1 1])
%     ylim([-1 1])
%     grid on
%     hold off
```

Calculating distance between ICoZV and center of mass.

```matlab:Code
    %Coordinates of ICoZV:
    C3=OP+rP3*h_vP;
    C4=OP+rP4*h_vP;

    %Distance:
    d3=norm(OG3-C3);
    d4=norm(OG4-C4);
```

Mass moment of inertia

```matlab:Code
    % J1_A1 and J2_A2 is calculated in Maple and given in the mass matrix of
    % the kinetic calculations.

    %For body 3 and 4, the mass moment of inertia changes in time, so we
    %calculate an inertia for the two time steps for body 3 and 4

    J3_C3 = J3_G + m3*d3^2;
    J4_C4 = J4_G + m4*d4^2;

```

Angular velocity of body 3 and 4

```matlab:Code
    dTheta3 = AngVelocity(ddx(n,1),ddy(n,1),aB1(:,n),B1P(:,n),h_B1P(:,n));
    dTheta4 = AngVelocity(ddx(n,1),ddy(n,1),aB2(:,n),B2P(:,n),h_B2P(:,n));
```

Change in energy of the system:

```matlab:Code
    E1(n,1) = 1/2*J1_A1*dTheta(n,1)^2;
    E2(n,1) = 1/2*J2_A2*dTheta(n,2)^2;
    E3(n,1) = 1/2*J3_C3*dTheta3^2;
    E4(n,1) = 1/2*J4_C4*dTheta4^2;
    
    W1(n,1) = g(11,n)*Theta(n,1);
    W2(n,1) = g(12,n)*Theta(n,2);
end

E_Totali = (E1(ni,1)+E2(ni,1) + E3(ni,1)+E4(ni,1)) % Initial energy of the system i Joule
E_Totalf = (E1(nf,1)+E2(nf,1) + E3(nf,1)+E4(nf,1)) % Final energy of system in Joule
Delta_E = (E1(nf,1)+E2(nf,1) + E3(nf,1)+E4(nf,1)) - (E1(ni,1)+E2(ni,1) + E3(ni,1)+E4(ni,1)) % Change of energy

W_Totali = (W1(ni,1)+W2(ni,1)) % Inital Work
W_Totalf = (W1(nf,1)+W2(nf,1)) % Final work 
Delta_W = (W1(nf,1)+W2(nf,1)) - (W1(ni,1)+W2(ni,1)) % Change of work by motors

% "Inital energy + Change of work - Final energy = 0" for no error. W_E_Error in Joules  
W_E_Error = (E1(ni,1)+E2(ni,1)+E3(ni,1)+E4(ni,1)) + (W1(nf,1)+W2(nf,1))-(W1(ni,1)+W2(ni,1)) - (E1(nf,1)+E2(nf,1)+E3(nf,1)+E4(nf,1))

% Percent error
W_E_ErrorPercent = W_E_Error/E_Totalf * 100

```

We see that the percent error is small (<1%), this validates the relation between kinematics and kinetics in the sense that the movement in terms of velocity and postion is correctly related to the active forces in the system. Therefore an assumption is made, that the internal forces in the system is correct like the external forces. 

Plotting relationship between time-discretisation and percent error of the work-energy approach:

```matlab:Code
% The data below is calculated for different time discretization.
T_discret = [2,4,6,8,10,20,40,100,200,600,1200]
WE_PercentError = [612.02, 809.67, 215.67, 118.90, 80.699, 31.165, 14.141,5.3586,2.6330,0.8677,0.4326]

p = loglog(T_discret,WE_PercentError);
grid on
xlim([0 2000])
ylim([0 1000])
p.LineStyle = ":";
p.Color = "red";
p.Marker = "O";
p.LineWidth = 1.5;
title('Relation between time discretization and percent error')
xlabel('t_f / \Deltat')
ylabel('%-error')
legend("Circels mark data points")
```
