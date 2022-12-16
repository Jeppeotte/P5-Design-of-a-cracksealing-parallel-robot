function InvVel = InverseVelocity(L,x,y,d_x,d_y,Theta,n_t)

%Defining vairables
k=[1,2;3,4;5,6;7,8]; %This matrix is for bookkeeping
InvVel=zeros(n_t,2); %Change this n_t,2*number of config.
L0=L(1);
L1=L(2);


nTheta(:,1) = Theta(:,1); %-Theta(1,1);
nTheta(:,2) = Theta(:,2); %-Theta(1,2);
nx = x(:,1); %- x(1,1);
ny = y(:,1); %- y(1,1);
nd_x = d_x(:,1); %-d_x(1,1);
nd_y = d_y(:,1); %-d_y(1,1);

% Calculating the angular velocity in a n_tx2*c matrix for c config. 
for i = 1 %i=1:4 for all configurations
    for n = 1:n_t
        
        InvVel(n,k(i,1)) = (-2*(nx(n,1) + L0/2 - L1*cos(nTheta(n,k(i,1))))...
            *nd_x(n,1) - 2*(ny(n,1) - L1*sin(nTheta(n,k(i,1))))*nd_y(n,1))...
            /(2*L1*(sin(nTheta(n,k(i,1)))*(nx(n,1) + L0/2) - ny(n,1)*cos(nTheta(n,k(i,1)))));

        InvVel(n,k(i,2)) = (-2*(nx(n,1) + L0/2 - L1*cos(nTheta(n,k(i,2))))...
            *d_x(n,1) - 2*(ny(n,1) - L1*sin(nTheta(n,k(i,2))))*d_y(n,1))...
            /(2*L1*(sin(nTheta(n,k(i,2)))*(nx(n,1) - L0/2) - ny(n,1)*cos(nTheta(n,k(i,2)))));
    end
end
