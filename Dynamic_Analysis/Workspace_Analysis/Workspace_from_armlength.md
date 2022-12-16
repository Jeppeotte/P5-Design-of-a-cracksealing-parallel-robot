```matlab:Code
clc
clear
close all
syms l1 l2 h l x y

%Vælger afstand mellem armenes monteringspunkter
l_0 = 0.1; %m
%l_0 = y; %Hvis man vil have en funktion for længden mellem monteringspunkterne

%Længden af den samlet arm udregnet fra inertimoment
l_tot = 0.715; %m
hyp_2 = l_tot;

%Et kvadratisk workspace gør at man kan bruge trigonometri til at udregne
%længden på de to led.
%Workspace kan definers som højden gange grundlinje:
w_area = h*l; %m^2
l = x; %m
h = l; %m
%Trekant 1 som er inde i workspacet, har en hyp som kan beskrives ved:
%hyp_1 = sqrt(h^2+(l/2)^2);

%Trekant 2, den store trekant som er ud over workspacet.
%Det vil sige at grundlinjen er givet ved:
l_trekant2 = (l/2)+(l_0/2);
%Forholdet mellem de to grundlinjer kan så gives ved:
f = (l/2)/l_trekant2;
%Højde på storetrekant, kan udregnes ud fra hypotinusen:
h_trekant2 = sqrt(hyp_2^2-l_trekant2^2);

%Da vi allerede kender hypotinusen på den store trekant kan vi beskrive
%hyp_1 ud fra forholdet mellem de to trekanter.
hyp_1 = hyp_2/f;
%Højden kan beskrives ud fra forholdet mellem de

%Afstanden fra monteringen og op til det kvadratiske workspace kan
%beskrives ved:
%Dette kan grundet ydre positionerne for armene vil have denne højde.
b = cos(45*pi/180)*l1;

assume(l,'positive')
assume(l1,'positive')
assume(l2,'positive')
assume(x,'positive')
%Solver for de forskellige længder
s = solve(l1 + l2 == sqrt((l/2-l_0/2+l_0)^2+((l2-b)+h)^2),l1 + l2 == hyp_2,l1*2+l_0==l2*2,l1,l2,l,'ReturnConditions', true);

%Proximal link
l1 = eval(s.l1(1,1));
l3 = l1;
%Distal link
l2 = eval(s.l2(1,1));
l4 = l2;
%højden af workspace
l = eval(s.x(1,1));
h = l;
% %Comment ud hvis man skal plotte reachable workspace
% %plotting length of the two links
% %plotter længden af indre link
% fplot(l1)
% hold on
% %plotter længden af ydre link
% fplot(l2)
% xlim([0 0.5])
% ylim([0.2 0.5])
% legend('Proximal link length','Distal link length')
% xlabel('Length in m')
% ylabel('Length in m')
% title('Proximal and distal length compared to the distance between the motors')
% hold off
% 
% %Plotting area compared to length between mounting points
% %plotter højden af workspacet
% %fplot(h)
% 
% %plotter workspace arealet
% area = h^2;
% fplot(area)
% hold on
% %l_0=l2-l1;
% %plotter afstanden mellem de 2 monteringspunkter.
% %fplot(l_0)
% xlim([0 0.5])
% ylim([0 0.5])
% legend('Area of the incribed square')
% xlabel('Length in m')
% ylabel('Area in m^2')
% title('Area of the incribed square compared to the distance between the motors')
% hold off

%Plotter workspace
y_up = l2-cos(45*pi/180)*l1;
theta1 = -pi/4:0.1:3*pi/2; % all possible theta1 values
theta2 = 0:0.1:pi; % all possible theta2 values
theta3 = -pi/4:0.1:3*pi/2; % all possible theta3 values
theta4 = 0:0.1:pi; % all possible theta4 values

[THETA1,THETA2] = meshgrid(theta1,theta2); % generate grid of angle values
[THETA3,THETA4] = meshgrid(theta3,theta4); % generate grid of angle values

X = l1 * cos(THETA1) + l2 * cos(THETA1 + THETA2)+l_0/2; % compute x coordinates
Y = l1 * sin(THETA1) + l2 * sin(THETA1 + THETA2); % compute y coordinates

X_2 = l3 * -cos(THETA3) + l4 * -cos(THETA3 + THETA4)-l_0/2; % compute x coordinates
Y_2 = l3 * sin(THETA3) + l4 * sin(THETA3 + THETA4); % compute y coordinates

data1 = [X(:) Y(:) THETA1(:)]; % create x-y-theta1 dataset
data2 = [X(:) Y(:) THETA2(:)]; % create x-y-theta2 dataset

data3 = [X_2(:) Y_2(:) THETA3(:)]; % create x-y-theta1 dataset
data4 = [X_2(:) Y_2(:) THETA4(:)]; % create x-y-theta2 dataset

plot(X(12:32,1),Y(12:32,1),'r');
hold on
plot(X(1,25:42),Y(1,25:42),'r');

plot(X_2(12:32,1),Y_2(12:32,1),'b');
plot(X_2(1,25:42),Y_2(1,25:42),'b');
axis equal;
plot(l_0/2,0,'s')
plot(-l_0/2,0,'s')

x1=-l/2;
x2=l/2;
y1=0+y_up;
y2=h+y_up;
x = [x1, x2, x2, x1, x1];
y = [y1, y1, y2, y2, y1];
plot(x, y, 'b-', 'LineWidth', 2);
xlim([-0.7 0.7])
ylim([-0.15 0.75])
xlabel('Length in m')
ylabel('Length in m')
title('Reachable workspace with the largest realisable inscribed square')
grid on

```
