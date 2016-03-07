function [weight] = calcWeight( uncertainty, dx, dy)
%calcWeight
%Created by Mirai Higuchi
%
%Calclates weights of each control points
%
%INPUT: 
% uncertainty: uncertainties of control point localization
% dx - number of control points (x-axis)
% dy - number of control points (y-axis)
%
%OUTPUT: 
% weight: Weights of each control points
%

weight = zeros(3, dx*dy);

for loop_pnt = 1: dy*dx
    sigma = [ uncertainty(1, loop_pnt), uncertainty(3, loop_pnt); uncertainty(3, loop_pnt), uncertainty(2, loop_pnt) ];
    % Calc Weight
    sigma = sigma*sigma;        
    weight(1,loop_pnt) = sigma(1,1);
    weight(2,loop_pnt) = sigma(2,2);
    weight(3,loop_pnt) = sigma(1,2);
end
end

