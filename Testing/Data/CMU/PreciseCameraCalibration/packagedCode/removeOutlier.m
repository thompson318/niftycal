function [outlier_map] = removeOutlier(point2D, dx, dy, distancex, distancey, thr)
%removeOutlier
%Created by Mirai Higuchi
%
%fit grid by RANSAC and remove outlier
%
%INPUT: 
%pints2D - 2D coordinates of control points on a rectified image plane
% dx - number of control points (x-axis)
% dy - number of control points (y-axis)
% distancex - distance between any adjacent control points on a rectified image plane 
% distancey - distance between any adjacent control points on a rectified image plane 
% thr - threshold for RANSAC
%
%OUTPUT: 
%outlier_map - outlier map - 1 for inlier, 0 for outlier
%

outlier_map = ones(dx*dy,1);

%create ideal control points
[y,x] = meshgrid(distancex:distancex:distancex*dy,distancey:distancey:distancey*dx);
x=x(:);
y=y(:);
ipoints = [x';y';ones(1,dx*dy)];

%initial parameters
alpha = 1;
tx = 0;
ty = 0;
W = 1;

%parameters for optimization
iter_fitting = 5;
rand_data_num = ceil(dx*dy/2);
nonrand_data_num = dx*dy-rand_data_num;
iter_ransac = 5;
error = zeros(1, dx*dy);
error_arr = zeros(dx*dy, iter_ransac);
score_arr = zeros(1, iter_ransac);
param_arr = zeros(3, iter_ransac);
f=ipoints(1,dx*dy);
ipointsf=ipoints/f;
ipointsf(3,:)=1;
for loop_ransac = 1:iter_ransac
    rand_val = rand(1, rand_data_num);
    rand_val = ceil(rand_val.*(dx*dy));
    nonrand_val = zeros(1,nonrand_data_num);

    nonrand_cnt = 1;
    for loop_pnt = 1:dx*dy
        index = find(rand_val == loop_pnt);
        if index > 0
        elseif nonrand_cnt <= nonrand_data_num
            nonrand_val(nonrand_cnt) = loop_pnt;
            nonrand_cnt = nonrand_cnt + 1;
        end
    end    
    for loop_ite=1:iter_fitting       
        tr_arr = [alpha 0 tx; 0 alpha ty];
        points = tr_arr*ipointsf;
        Hesse = zeros(3);
        b = zeros(3,1);        
        for loop_data=1: rand_data_num
            pnt_index = rand_val(loop_data);
            p_obs = point2D(1:2, pnt_index)/f;
            p = points(1:2, pnt_index);
            pi = ipointsf(1:2, pnt_index);
            J = [ pi(1) 1 0; pi(2) 0 1];
            Hesse = Hesse + J'*J;
            b = b + J'*(p_obs-p);            
        end
        delta_p = inv(Hesse)*b;
        alpha = alpha + delta_p(1);
        tx = tx + delta_p(2);
        ty = ty + delta_p(3);        
    end
    tr_arr = [alpha 0 tx*f; 0 alpha ty*f];
    points = tr_arr*(ipoints);
    error = ((point2D(1,:)-points(1,:)).^2+(point2D(2,:)-points(2,:)).^2).^(1/2);

    % evaluate fitting result
    score = 0;
    for loop_data=1: nonrand_data_num
        pnt_index = nonrand_val(loop_data);
        if error(pnt_index) < thr 
            score = score + 1;
        end
    end
    score_arr(loop_ransac) = score;
    param_arr(1, loop_ransac) = alpha;
    param_arr(2, loop_ransac) = tx;
    param_arr(3, loop_ransac) = ty;
    error_arr(:, loop_ransac) = error';
end

max_score_index = find(score_arr==max(score_arr));
max_score_error = error_arr(:, max_score_index(1));
% fitting using only inlier
for loop_ite=1:iter_fitting       
    tr_arr = [alpha 0 tx; 0 alpha ty];
    points = tr_arr*ipointsf;
    Hesse = zeros(3);
    b = zeros(3,1);        
    for loop_data=1: dx*dy
        if( max_score_error(loop_data) < thr )
            pnt_index = loop_data;
            p_obs = point2D(1:2, pnt_index)/f;
            p = points(1:2, pnt_index);
            pi = ipointsf(1:2, pnt_index);
            J = [ pi(1) 1 0; pi(2) 0 1];
            Hesse = Hesse + J'*J;
            b = b + J'*(p_obs-p);            
        end
    end
    delta_p = inv(Hesse)*b;
    alpha = alpha + delta_p(1);
    tx = tx + delta_p(2);
    ty = ty + delta_p(3);        
end
tr_arr = [alpha 0 tx*f; 0 alpha ty*f];
points = tr_arr*(ipoints);
error = ((point2D(1,:)-points(1,:)).^2+(point2D(2,:)-points(2,:)).^2).^(1/2);

% evaluate fitting result
cnt = 0;
for loop_pnt=1: dx*dy
    if error(loop_pnt) > thr
        outlier_map(loop_pnt) = 0;
        cnt = cnt + 1;
    end
end
if cnt > 30
    cnt=0;
end
end