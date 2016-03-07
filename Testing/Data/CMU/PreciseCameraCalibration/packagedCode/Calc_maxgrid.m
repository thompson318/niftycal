function [maxdist] = Calc_maxgrid(detectx, dy, dx, fc, cc, kc, alpha_c, outlier_map)
%Calc_maxgrid 
%Created by Mirai Higuchi
%
%Calclates maximum distance between any adjacent control points on images.
%
%INPUT: 
% detectx - 2D coordinates of the detected control points on image planes.
% dx - number of control points (x-axis)
% dy - number of control points (y-axis)
% fc - focal length of the intrinsic parameters
% cc - principle point of the intrinsic parameters
% kc - lens distortion parameters
% alpha_c - skew of the intrinsic parameters
% outlier_map - outlier map - 1 for inlier, 0 for outlier
%
%OUTPUT: 
% maxdist: maximum distance between any adjacent control points on images
%


% correct distortion
undistort_x = zeros(2, dx*dy);
for loop_pnt = 1:dx*dy
    x_kk = [ detectx(1,loop_pnt), detectx(2,loop_pnt) ]';
    x_new = normalize(x_kk, fc, cc, kc, alpha_c);
    u = x_new(1)*fc(1)+cc(1);
    v = x_new(2)*fc(2)+cc(2);
    undistort_x(1,loop_pnt) = u;
    undistort_x(2,loop_pnt) = v;
end

% calculate max distance between any adjacent points
distance_x = zeros(2, (dx)*(dy));
for loop_dy = 1:dy
    for loop_dx = 1:dx
        if loop_dx < dx &&  loop_dy < dy
            if outlier_map((loop_dy-1)*(dx) + loop_dx) == 1 && outlier_map((loop_dy-1)*(dx) + loop_dx+1) == 1 && outlier_map((loop_dy)*(dx) + loop_dx) == 1
                distance_x(1, (loop_dy-1)*(dx) + loop_dx) = sqrt((undistort_x(1,(loop_dy-1)*(dx) + loop_dx+1) - undistort_x(1, (loop_dy-1)*(dx) + loop_dx))^2+(undistort_x(2,(loop_dy-1)*(dx) + loop_dx+1) - undistort_x(2, (loop_dy-1)*(dx) + loop_dx))^2);
                distance_x(2, (loop_dy-1)*(dx) + loop_dx) = sqrt((undistort_x(1,(loop_dy)*(dx) + loop_dx) - undistort_x(1, (loop_dy-1)*(dx) + loop_dx))^2+(undistort_x(2,(loop_dy)*(dx) + loop_dx) - undistort_x(2, (loop_dy-1)*(dx) + loop_dx))^2);
            end
        elseif loop_dy < dy
            if outlier_map((loop_dy-1)*(dx) + loop_dx) == 1 && outlier_map((loop_dy)*(dx) + loop_dx) == 1
                distance_x(2, (loop_dy-1)*(dx) + loop_dx) = sqrt((undistort_x(1,(loop_dy)*(dx) + loop_dx) - undistort_x(1, (loop_dy-1)*(dx) + loop_dx))^2+(undistort_x(2,(loop_dy)*(dx) + loop_dx) - undistort_x(2, (loop_dy-1)*(dx) + loop_dx))^2);
            end
        elseif loop_dx < dx
            if outlier_map((loop_dy-1)*(dx) + loop_dx) == 1 && outlier_map((loop_dy-1)*(dx) + loop_dx+1) == 1           
                distance_x(1, (loop_dy-1)*(dx) + loop_dx) = sqrt((undistort_x(1,(loop_dy-1)*(dx) + loop_dx+1) - undistort_x(1, (loop_dy-1)*(dx) + loop_dx))^2+(undistort_x(2,(loop_dy-1)*(dx) + loop_dx+1) - undistort_x(2, (loop_dy-1)*(dx) + loop_dx))^2);
            end
        end
    end
end
maxdist = ceil(max(max(distance_x)));

