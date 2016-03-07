function [image, homography] = Proj_Undistort3(I,detectx, dY,dX,dy,dx,omc,Tc,fc,cc,kc,alpha_c, maxdist)
% Proj_Undistort3
% Created by Mirai Higuchi
%
% rectify input image to front-parallel image.
%
% INPUT:
% I - input image
% detectx - 2D coordinates of the detected control points on image planes.
% dX - distance between control points (x-axis)[mm]
% dY - distance between control points (y-axis)[mm]
% dx - number of control points (x-axis)
% dy - number of control points (y-axis)
% omc - rotation vector
% Tc- translation
% fc - focal length
% cc - principal point coordinates
% kc - lens distortion parameters
% alpha_c - skew coefficient
% maxdist - maximum distance between any adjacent control points on an image
%
%OUTPUT: 
% image - rectified image
% homography - homography matrix


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

% calculate Homography
A=[fc(1), alpha_c, cc(1);
    0, fc(2), cc(2);
    0, 0, 1];
R=rodrigues(omc);
H = A*[R, Tc];
H1 = [H(:,1),H(:,2),H(:,4)];
tt= [0;0;fc(1)*dX/maxdist];
H = A*[eye(3), tt];
H2 = [H(:,1),H(:,2),H(:,4)];
homography=H2*inv(H1);

dx_strt = maxdist;
dy_strt = maxdist;
X = [undistort_x(1,:);undistort_x(2,:);ones(1,dx*dy)];
W = homography * X;
W(1,:) = W(1,:)./W(3,:);
W(2,:) = W(2,:)./W(3,:);
dx_strt_bf = W(1,1);
dy_strt_bf = W(2,1);
homography(:)=homography(:)/homography(3,3);
trans = eye(3);
trans(1,3)=-dx_strt_bf+dx_strt;
trans(2,3)=-dy_strt_bf+dy_strt;
homography = trans*homography;
homography = inv(homography);

% create output image
sizex = maxdist*(dx+1);
sizey = maxdist*(dy+1);

% project image to new coordinate
[x,y] = meshgrid(1:sizex,1:sizey);
x=x(:);
y=y(:);
X = [x';y';ones(1,prod(size(x)))];
W = homography * X;
x2(1,:) = W(1,:)./W(3,:);
x2(2,:) = W(2,:)./W(3,:);
for loop_pnt = 1:prod(size(x))
    xd = (x2(1,loop_pnt)-cc(1))/fc(1);
    yd = (x2(2,loop_pnt)-cc(2))/fc(2);
    r2 = xd^2+yd^2;
    xdd = xd*(1+kc(1)*r2+kc(2)*r2^2)+2*kc(3)*xd*yd+kc(4)*(r2+2*xd^2);
    ydd = yd*(1+kc(1)*r2+kc(2)*r2^2)+2*kc(4)*xd*yd+kc(3)*(r2+2*yd^2);
    x2(1,loop_pnt) = xdd*fc(1)+cc(1);
    x2(2,loop_pnt) = ydd*fc(2)+cc(2);
end
x3d = reshape(x2(1,:),[sizey sizex]);
y3d = reshape(x2(2,:),[sizey sizex]);

[xim,yim] = meshgrid(1:size(I,2),1:size(I,1));
Z = interp2(xim,yim,double(I),x3d,y3d,'linear');
image = Z;
