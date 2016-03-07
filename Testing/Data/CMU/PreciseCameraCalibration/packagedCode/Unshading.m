function [image] = Unshading(I, dx, dy, maxdist, outlier_map);
% Unshading
% Created by Mirai Higuchi
%
% shading correction 
%
% INPUT:
% I - rectified image
% dx - number of control points (x-axis)
% dy - number of control points (y-axis)
% maxdist - distance between adjacent control points on a rectified image.
% outlier_map - outlier map - 1 for inlier, 0 for outlier
%
%OUTPUT: 
% image - corrected image

[imsizey, imsizex] = size(I);

Filtered_img = medfilt2(I, [ 10 10 ]);
Itmp = medfilt2(Filtered_img, [ 10 10 ]);

[y, x] = meshgrid(maxdist:maxdist:maxdist*(dy-1), maxdist:maxdist:maxdist*(dx-1));
x=x(:)+floor(maxdist/2);
y=y(:)+floor(maxdist/2);
%data=[x';y'];
%b=zeros(1,(dy+1)* (dx+1));
data_counter = 1;
for loop_y=1:(dy-1)
    for loop_x=1:(dx-1)
        if  outlier_map( (loop_y-1)*dx+loop_x ) == 0 || outlier_map( (loop_y-1)*dx+loop_x+1 ) == 0 || outlier_map( (loop_y)*dx+loop_x ) == 0 || outlier_map( (loop_y)*dx+loop_x+1 ) == 0
            continue;
        else
            b(data_counter)=Itmp(y((loop_y-1)*(dx-1)+1),x(loop_x));
            xx(data_counter) = x(loop_x);
            yy(data_counter) = y((loop_y-1)*(dx-1)+1);
            data_counter = data_counter+1;
        end
    end
end

if data_counter > 10
    data=[xx;yy];
    a0 = [-0.1,-0.1,100,imsizex/2,imsizey/2,0];
    [a, resnorm] = lsqcurvefit(@curve2d, a0, data, b);
    bresult = curve2d(a, data);


    [xx,yy] = meshgrid(1:imsizex,1:imsizey);
    xx=xx(:);
    yy=yy(:);
    data_all=[xx';yy'];
    bresult_all = curve2d(a, data_all);
    bresult_all2d = reshape(bresult_all(:), [imsizey imsizex]);

    maxb = max(bresult_all(:));
    image=I.*(maxb./bresult_all2d);


    max_val = max(max(image));
    min_val = min(min(image));
    image(:) = image(:)-min_val;
    image(:) = image(:)*255/(max_val-min_val);
else
    image = I;
end