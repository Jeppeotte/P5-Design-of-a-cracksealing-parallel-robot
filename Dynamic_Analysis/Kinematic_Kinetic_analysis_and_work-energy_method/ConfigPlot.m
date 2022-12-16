function ConfigPlot = ConfigPlot(n_t,x,y,L,theta_1,theta_2,xlimit,ylimit)

A1_x = -L(1,1)/2;
A1_y = 0;
A2_x = L(1,1)/2;
A2_y = 0;
plotcolor = ['r';'b';'g';'m']; % this needs to be a n_tx1 vector, it can be added for few time-steps.

for p=1:n_t
    B1_x=L(1,2)*cos(theta_1(p,1))-L(1,1)/2;
    B1_y=L(1,2)*sin(theta_1(p,1));
    B2_x=L(1,2)*cos(theta_2(p,1))+L(1,1)/2;
    B2_y=L(1,2)*sin(theta_2(p,1));
    P_x = x(p,1);
    P_y = y(p,1);
    
    X1=[A1_x,B1_x,P_x];
    X2=[A2_x,B2_x,P_x];
    Y1=[A1_y,B1_y,P_y];
    Y2=[A2_y,B2_y,P_y];
    plot(X1,Y1,plotcolor(p,1),X2,Y2,plotcolor(p,1),x,y,'k') % X1,Y1,plotcolor(p,:)
    xlabel('[m]')
    ylabel('[m]')
    xlim(xlimit)
    ylim(ylimit)
    grid on
    hold on
end

legend('Step 1', 'Step 2', 'Step 3', 'Step 4')
