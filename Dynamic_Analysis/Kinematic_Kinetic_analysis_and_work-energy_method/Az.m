function A = Az(theta)

A(1:2,1:2) = 0;
A(1,1) = cos(theta);
A(2,1) = sin(theta);
A(1,2) = -A(2,1);
A(2,2) = A(1,1);