function [imagex, WorldX] = corner_grid_detect(I, y, x, dy, dx, dY, dX);

% Get corner grid subpix pos
n_sq_x = dx-1;
n_sq_y = dy-1;

wintx = 7;
winty = 7;
[Xc,good,bad,type] = cornerfinder([x';y'],I,winty,wintx); % the four corners
warning off;

%[Xc,good,bad,type] = cornerfinder([x;y],I,winty,wintx); % the four corners

if (isnan(Xc(1,1))), Xc(1,1) = x(1); end;
if (isnan(Xc(1,2))), Xc(1,2) = x(2); end;
if (isnan(Xc(1,3))), Xc(1,3) = x(3); end;
if (isnan(Xc(1,4))), Xc(1,4) = x(4); end;

if (isnan(Xc(2,1))), Xc(2,1) = y(1); end;
if (isnan(Xc(2,2))), Xc(2,2) = y(2); end;
if (isnan(Xc(2,3))), Xc(2,3) = y(3); end;
if (isnan(Xc(2,4))), Xc(2,4) = y(4); end;

x = Xc(1,:)';
y = Xc(2,:)';

% Compute the inside points through computation of the planar homography (collineation)

a00 = [x(1);y(1);1];
a10 = [x(2);y(2);1];
a11 = [x(3);y(3);1];
a01 = [x(4);y(4);1];


% Compute the planar collineation: (return the normalization matrix as well)

[Homo,Hnorm,inv_Hnorm] = compute_homography([a00 a10 a11 a01],[0 1 1 0;0 0 1 1;1 1 1 1]);

% Build the grid using the planar collineation:

x_l = ((0:n_sq_x)'*ones(1,n_sq_y+1))/n_sq_x;
y_l = (ones(n_sq_x+1,1)*(0:n_sq_y))/n_sq_y;
pts = [x_l(:) y_l(:) ones((n_sq_x+1)*(n_sq_y+1),1)]';

XX = Homo*pts;
XX = XX(1:2,:) ./ (ones(2,1)*XX(3,:));


% Complete size of the rectangle

W = n_sq_x*dX;
L = n_sq_y*dY;




% %%%%%%%%%%%%%%%%%%%%%%%% ADDITIONAL STUFF IN THE CASE OF HIGHLY DISTORTED IMAGES %%%%%%%%%%%%%
% figure(2);
% hold on;
% plot(XX(1,:),XX(2,:),'r+');
% title('The red crosses should be close to the image corners');
% hold off;
% 
% disp('If the guessed grid corners (red crosses on the image) are not close to the actual corners,');
% disp('it is necessary to enter an initial guess for the radial distortion factor kc (useful for subpixel detection)');
% quest_distort = input('Need of an initial guess for distortion? ([]=no, other=yes) ');
% 
% quest_distort = ~isempty(quest_distort);
% 
% if quest_distort,
%     % Estimation of focal length:
%     c_g = [size(I,2);size(I,1)]/2 + .5;
%     f_g = Distor2Calib(0,[[x(1) x(2) x(4) x(3)] - c_g(1);[y(1) y(2) y(4) y(3)] - c_g(2)],1,1,4,W,L,[-W/2 W/2 W/2 -W/2;L/2 L/2 -L/2 -L/2; 0 0 0 0],100,1,1);
%     f_g = mean(f_g);
%     script_fit_distortion;
% end;
% %%%%%%%%%%%%%%%%%%%%% END ADDITIONAL STUFF IN THE CASE OF HIGHLY DISTORTED IMAGES %%%%%%%%%%%%%





Np = (n_sq_x+1)*(n_sq_y+1);

disp('Corner extraction...');

grid_pts = cornerfinder(XX,double(I),winty,wintx); %%% Finds the exact corners at every points!

%save all_corners x y grid_pts

%grid_pts = grid_pts - 1; % subtract 1 to bring the origin to (0,0) instead
%of (1,1) in matlab (not necessary in C)


Xi = reshape(([0:n_sq_x]*dX)'*ones(1,n_sq_y+1),Np,1)';
%Yi = reshape(ones(n_sq_x+1,1)*[n_sq_y:-1:0]*dY,Np,1)';
Yi = reshape(ones(n_sq_x+1,1)*[0:n_sq_y]*dY,Np,1)';
Zi = zeros(1,Np);

Xgrid = [Xi;Yi;Zi];


% All the point coordinates (on the image, and in 3D) - for global optimization:

% store result
% x = grid_pts;
% X = Xgrid;
imagex = grid_pts;
WorldX = Xgrid;
        
%display corners
figure(1);
imshow(uint8(I));
hold on;
c = round([grid_pts(1,:)]);
r = round([grid_pts(2,:)]);
for i=1:size(grid_pts,2)
    plot(c(i),r(i),'g+');
end
hold off;
drawnow;