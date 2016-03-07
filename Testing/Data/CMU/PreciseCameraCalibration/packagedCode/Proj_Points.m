function [distort_x] = Proj_Points(points_cor,homography,omc,Tc,fc,cc,kc,alpha_c,dx,dy);

% [x,y] = meshgrid(-dX:dX*dx,-dY:dY*dy);
% x=x(:);
% y=y(:);
% 
% X = [x';y';zeros(1,prod(size(x)))];
% x2 = project_points2(X,omc,Tc,fc,cc,kc,alpha_c);
% 
% x3d = reshape(x2(1,:),[length(-dY:dY*dy) length(-dX:dX*dx)]);
% y3d = reshape(x2(2,:),[length(-dY:dY*dy) length(-dX:dX*dx)]);
% 
% [xim,yim] = meshgrid(1:size(I,2),1:size(I,1));
% Z = interp2(xim,yim,double(I),x3d,y3d);
% image = uint8(Z);

points_cor(3,:)= 1;

% perspective transform
rectified_x = zeros(3, dx*dy);
rectified_x = homography*points_cor;
rectified_x(1,:) = rectified_x(1,:)./rectified_x(3,:);
rectified_x(2,:) = rectified_x(2,:)./rectified_x(3,:);

% distortion
distort_x = zeros(2, dx*dy);
for loop_pnt = 1:dx*dy
    xd = (rectified_x(1,loop_pnt)-cc(1))/fc(1);
    yd = (rectified_x(2,loop_pnt)-cc(2))/fc(2);
    r2 = xd^2+yd^2;
    xdd = xd*(1+kc(1)*r2+kc(2)*r2^2)+2*kc(3)*xd*yd+kc(4)*(r2+2*xd^2);
    ydd = yd*(1+kc(1)*r2+kc(2)*r2^2)+2*kc(4)*xd*yd+kc(3)*(r2+2*yd^2);
    distort_x(1,loop_pnt) = xdd*fc(1)+cc(1);
    distort_x(2,loop_pnt) = ydd*fc(2)+cc(2);
end

