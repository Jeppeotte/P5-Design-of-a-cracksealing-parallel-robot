function aG = AccelerationCenter(ddx,ddy,aB,BP,h_BP,PG,h_PG)

syms ddTheta dTheta

aG_eq1 = ddx == aB(1,1) + (ddTheta*h_BP(1,1)) - (dTheta^2*BP(1,1));
aG_eq2 = ddy == aB(2,1) + (ddTheta*h_BP(2,1)) - (dTheta^2*BP(2,1));

S_a34 = solve([aG_eq1,aG_eq2],[dTheta,ddTheta]);

dTheta = double(S_a34.dTheta(1,1));
ddTheta= double(S_a34.ddTheta(1,1));
    
aG = [[ddx;ddy] + (ddTheta*h_PG) - (dTheta^2*PG); ddTheta];
