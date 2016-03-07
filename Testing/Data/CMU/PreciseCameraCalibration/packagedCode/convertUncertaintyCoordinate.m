function [uncertainty] = convertUncertaintyCoordinate( uncertainty_in, point2D, dx, dy, homo, kc, cc, fc)
%convertUncertaintyCoordinate
%Created by Mirai Higuchi
%
%convert coordinate of uncertainty from rectified image coordinate to input image coordinate.
%
%INPUT: 
% uncertainty_in: uncertainties of control point localization
% pints2D - 2D coordinates of control points on a rectified image plane
% dx - number of control points (x-axis)
% dy - number of control points (y-axis)
% homo - homography matrix between rectified image and input image 
% kc - lens distortion parameters
% cc - principle point of the intrinsic parameters
% fc - focal length of the intrinsic parameters
%
%OUTPUT: 
% uncertainty: un-rectified uncertainties of control point localization
%

margin = 0;

uncertainty = zeros(3, dx*dy);

for loop_y = 1: dy
    for loop_x = 1: dx
        if loop_x < dx
            radius_x = abs(point2D(1, (loop_y-1)*dx+loop_x+1)- point2D(1, (loop_y-1)*dx+loop_x))/2;
        else
            radius_x = abs(point2D(1, (loop_y-1)*dx+loop_x)- point2D(1, (loop_y-1)*dx+loop_x-1))/2;
        end
        if loop_y < dy
            radius_y = abs(point2D(2, (loop_y)*dx+loop_x)- point2D(2, (loop_y-1)*dx+loop_x))/2;
        else
            radius_y = abs(point2D(2, (loop_y-1)*dx+loop_x)- point2D(2, (loop_y-2)*dx+loop_x))/2;
        end
        sx = point2D(1, (loop_y-1)*dx+loop_x)-radius_x-margin;
        ex = point2D(1, (loop_y-1)*dx+loop_x)+radius_x+margin;
        sy = point2D(2, (loop_y-1)*dx+loop_x)-radius_y-margin;
        ey = point2D(2, (loop_y-1)*dx+loop_x)+radius_y+margin;
        
        var_mat = [ uncertainty_in(1,(loop_y-1)*dx+loop_x), uncertainty_in(3,(loop_y-1)*dx+loop_x); uncertainty_in(3,(loop_y-1)*dx+loop_x) uncertainty_in(2,(loop_y-1)*dx+loop_x)];
        % affine
        x = [ sx ex ex sx;
              sy sy ey ey;
              1  1  1  1];
        x2 = homo*x;
        x2(1,:) = x2(1,:)./x2(3,:);
        x2(2,:) = x2(2,:)./x2(3,:);        
        for loop_pnt = 1:4
            xd = (x2(1,loop_pnt)-cc(1))/fc(1);
            yd = (x2(2,loop_pnt)-cc(2))/fc(2);
            r2 = xd^2+yd^2;
            xdd = xd*(1+kc(1)*r2+kc(2)*r2^2)+2*kc(3)*xd*yd+kc(4)*(r2+2*xd^2);
            ydd = yd*(1+kc(1)*r2+kc(2)*r2^2)+2*kc(4)*xd*yd+kc(3)*(r2+2*yd^2);
            x2(1,loop_pnt) = xdd*fc(1)+cc(1);
            x2(2,loop_pnt) = ydd*fc(2)+cc(2);
        end
        x_mean = sum(x(1,1:4));
        x_mean = x_mean/4;
        y_mean = sum(x(2,1:4));
        y_mean = y_mean/4;
        x(1,1:4) = x(1,1:4)-x_mean;
        x(2,1:4) = x(2,1:4)-y_mean;
        x_mean = sum(x2(1,1:4));
        x_mean = x_mean/4;
        y_mean = sum(x2(2,1:4));
        y_mean = y_mean/4;
        x2(1,1:4) = x2(1,1:4)-x_mean;
        x2(2,1:4) = x2(2,1:4)-y_mean;
        x2(3,1:4) = 1;
        affine = vgg_Haffine_from_x_MLE(x(1:3,1:4), x2(1:3,1:4));
        affine2x2 = affine(1:2,1:2);
        
        var_mat = affine2x2*var_mat*affine2x2';
        uncertainty( 1, (loop_y-1)*dx+loop_x ) = var_mat(1,1);
        uncertainty( 2, (loop_y-1)*dx+loop_x ) = var_mat(2,2);
        uncertainty( 3, (loop_y-1)*dx+loop_x ) = var_mat(1,2);
    end
end
end

