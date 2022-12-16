function InvPos = InversePosition(x,y,n_t,L)

k=[1,2;3,4;5,6;7,8];
psi_1=[1;-1;-1;1];
psi_2=[-1;1;-1;1];
InvPos=zeros(n_t,2); %Change this n_t,2*number of config.
L_0=L(1);
L_1=L(2);
L_3=L(4);


for i=1  %i=1:4 for all configurations
    for n=1:n_t
        a_1      = (L_1^2)+(y(n,1)^2)+((x(n,1)+L_0/2)^2)-(L_3^2)+2*(x(n,1)+L_0/2)*L_1;
        b_1      = -4*y(n,1)*L_1;
        c_1      = (L_1^2)+(y(n,1)^2)+((x(n,1)+L_0/2)^2)-(L_3^2)-2*(x(n,1)+L_0/2)*L_1;
        a_2      = (L_1^2)+(y(n,1)^2)+((x(n,1)-L_0/2)^2)-(L_3^2)+2*(x(n,1)-L_0/2)*L_1;
        b_2      = b_1;
        c_2      = (L_1^2)+(y(n,1)^2)+((x(n,1)-L_0/2)^2)-(L_3^2)-2*(x(n,1)-L_0/2)*L_1;
        %Opsætter deskiminanten:
        z_i1     = (-b_1+psi_1(i,1)*sqrt((b_1^2)-4*a_1*c_1)) / (2*a_1) ;
        z_i2     = (-b_2+psi_2(i,1)*sqrt((b_2^2)-4*a_2*c_2)) / (2*a_2) ;
        
        %Nu kan vinklerne theta_1 og theta_2 findes:
        InvPos(n,k(i,1))   =((2*atan(z_i1))); %[rad]
        %theta_1=(((2*atan(z_i1))*180)/pi);
        %theta_test_1 = (theta1*180)/pi

        InvPos(n,k(i,2))   =(2*atan(z_i2)); %[rad]
        %theta_2= ((2*atan(z_i2))*180)/pi;
        %theta_test_2 = (theta2*180)/pi
    end
end


