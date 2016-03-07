function F = curve2d(a, data)
%curve2d 
%Created by Mirai Higuchi
%
%calclates coefficients of quadratic surface.
%
%INPUT: 
% a - coefficients of quadratic surface
% data - observed data
%
%OUTPUT: 
% F
%
x=data(1,:);
y=data(2,:);

F=(a(1).*(x-a(4)).^2+a(2)*(y-a(5)).^2+a(6).*(x-a(4)).*(y-a(5))+a(3));