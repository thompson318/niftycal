function [flag,dxx,dyy] = quad_subpixel(values);

% this function takes in a 3X3 array of values, can be viewed as a surface
% and fits a 2D quadratic surface to it. it then finds the subpixel
% location of the peak. Given integer peak is assumed to be in the middle
% of the array.

A = [];
B = [];

for y = -1:1
    for x = -1:1
        A = [A; x^2 y^2 x y x*y 1];
        B = [B; values(y+2,x+2)];
    end
end

X = A\B;

H = [X(1) X(2)/2; 
     X(2)/2 X(3)];
 
[V,D] = eig(H);

flag = 0;

% convex in both directions
if (V(1) < 0 & V(2) < 0 )
    flag = 1;
    
    a = X(1); b = X(2); c = X(3);
    d = X(4); e = X(5); f = X(6);
    
    dxx = -1*(2*b*c - d*e)/(4*a*b - e^2);
    
    dyy = -1*(2*a*d - c*e)/(4*a*b - e^2);
else
    dxx = 0;
    dyy = 0;
end

