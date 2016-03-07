function [uncertainty] = calcUncertainty(I, point2D, dx, dy, maxdisx, maxdisy, outlier_map)
%calcUncertainty
%Created by Mirai Higuchi
%
%calclates uncertainty of control point localization.
%
%INPUT: 
% I - input image
% pints2D - 2D coordinates of control points on a rectified image plane
% dx - number of control points (x-axis)
% dy - number of control points (y-axis)
% maxdisx - distance between any adjacent control points on a rectified image plane 
% maxdisy - distance between any adjacent control points on a rectified image plane 
% outlier_map - outlier map - 1 for inlier, 0 for outlier%
%
%OUTPUT: 
% uncertainty: uncertainties of control point localization
%

% Edge Filter (Sobel Filter)
edge_filter_h = [-1,0,1; -2,0,2; -1,0,1]/8;
edge_filter_v = [-1,-2,-1; 0,0,0; 1,2,1]/8;

[m,n] = size(point2D);
margin = 0;

uncertainty = zeros(3, dx*dy);
%counter1 = 1;
[imsize_y, imsize_x] = size(I);

% Edge Detection
iI = 255 - I;
counter1 = 10;


for loop_y = 1: dy
    for loop_x = 1: dx
        if outlier_map((loop_y-1)*dx+loop_x) == 0 
            continue;
        end
        radius_x = maxdisx/2;
        radius_y = maxdisy/2;        
        sx = point2D(1, (loop_y-1)*dx+loop_x)-radius_x-margin;
        ex = point2D(1, (loop_y-1)*dx+loop_x)+radius_x+margin;
        sy = point2D(2, (loop_y-1)*dx+loop_x)-radius_y-margin;
        ey = point2D(2, (loop_y-1)*dx+loop_x)+radius_y+margin;
        Icroped_h = iI( floor(sy):ceil(ey), floor(sx):ceil(ex));
        Icroped_v = iI( floor(sy):ceil(ey), floor(sx):ceil(ex));
        Icroped_h = filter2( edge_filter_h,Icroped_h, 'same' );
        Icroped_v = filter2( edge_filter_v,Icroped_v, 'same' );
        [size_y, size_x] = size(Icroped_h);
        
        % calc variance
        var_xx = double(0);
        var_yy = double(0);
        var_xy = double(0);
        counter = 0;
        for loop_yy = 2: size_y-1
            for loop_xx = 2: size_x-1
                var_xx = var_xx+Icroped_h(loop_yy, loop_xx)^2;
                var_yy = var_yy+Icroped_v(loop_yy, loop_xx)^2;
                var_xy = var_xy+Icroped_h(loop_yy, loop_xx)*Icroped_v(loop_yy, loop_xx);
            end
        end
        var_xx = var_xx;
        var_yy = var_yy;
        var_xy = var_xy;
        var_mat = [ var_xx var_xy; var_xy var_yy];
                    
        uncertainty( 1, (loop_y-1)*dx+loop_x ) = var_mat(1,1);
        uncertainty( 2, (loop_y-1)*dx+loop_x ) = var_mat(2,2);
        uncertainty( 3, (loop_y-1)*dx+loop_x ) = var_mat(1,2);
    end
end
end

