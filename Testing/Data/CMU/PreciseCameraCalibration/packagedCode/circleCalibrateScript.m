function [error, storageParam] = circleCalibrateScript(dX, dY, dx, dy, directoryName, outputname, IM_Cell,FP_Cell, boundingbox, numIter, concentric_flag,optimization_flag, data_flag, outlier_flag)
% circleCalibrateScript
% Created by Ankur Datta
% Modified by Mirai Higuchi
%
% Calls calibration as a script for running a battery of tests
%
% INPUT:
% dX - distance between control points (x-axis)[mm]
% dY - distance between control points (y-axis)[mm]
% dx - number of control points (x-axis)
% dy - number of control points (y-axis)
% directryName - name of the base directory
% outputname- name of output files
% IM_Cell - cell array of images
% FP_Cell - cell array of the gound truth of control point location on input images
% boundingbox - coordinate of boundingbox of grid
% numIter - number of iterations to perform
% concentric_flag - flag for chosing the pattern - %0 for circle pattern; 1 for ring pattern; % -1 for square
% optimization_flag - flag for optimization method - %0 for Bundle Adjustment without weight; 1 for Weighted Bundle Ajustment
% data_flag - flag for input data - %0 for Synthetic dataset; 1 for real image dataset
% outlier_flag - flag for removing outlier - %0 for invalid; 1 for valid
%
%OUTPUT: 
% error - array of RMS error
% storageParam - array of parameters

% number of images
N_ima = length(IM_Cell);

% buffer for debug and evaluation
fc_buf = zeros(2,numIter);%higuchi
cc_buf = zeros(2,numIter);%higuchi
kc_buf = zeros(5,numIter);%higuchi
omc_buf = zeros(3,numIter);%higuchi
tc_buf = zeros(3,numIter);%higuchi
pixerror_buf = zeros(2,numIter);%higuchi
ave_error_arr = zeros(1, numIter);
ave_error_whole_arr = zeros(1, numIter);
ave_error_localization_arr = zeros(1, numIter);
error = zeros(1,numIter); % buffer of RMS error
storageParam = zeros(6,numIter);

string = sprintf('dX and dY set at: %f %f', dX, dY);
disp(string);

%only estimate part of dist
est_dist    = [1;1;1;1;0];%do not estimate k3
est_alpha   = 0; %do not estimate skew
%initial two different value focals for x/y instead of a single value
two_focals_init = 1;
if data_flag == 0 % for synthetic images
    outlier_thr = 0.5;%1; %threshold for removing outlier
elseif data_flag == 1 % for real images
    outlier_thr = 1;%1; %threshold for removing outlier
end

%Outlier map
outlier_map_arr = ones(dx*dy, N_ima);

%create Filters for control point localization
createFilter2;

rectName = [];
for loopIterate = 1:numIter
    % some variable correction
    n_ima = N_ima;
    for kk = 1:n_ima
        if (loopIterate == 1)
            % get input image
            I = IM_Cell{kk}; 
            %%% convert to grayscale if color image
            if (size(I,3) ~= 1)
                I = rgb2gray(I);
            end

            % save the image
            eval(['IOrig_' num2str(kk) ' = I;']);
        else
            % reading the image to the memory
            string = sprintf('Reading image: %d',kk);
            disp(string);
            I = Rect(kk).im; %eval(['I = ' '_rect' num2str(kk) ';']);
            homo = Rect(kk).homo;
            maxdis = Rect(kk).maxdist;
        end        

        if (loopIterate == 1) %%% fit an ellipse for all of them
            % get boundigbox
            x = boundingbox(1:4,kk);
            y = boundingbox(5:8,kk);
            if (concentric_flag>=0)
                % get the bounding box for each ellipse with corresponding 2D and 3D points
                [x,X] = getROI(directoryName, loopIterate, kk, I, y,x, dy, dx, dY, dX);
                % extract each region and fit ellipse
                %[Param] = computeEllipse(directoryName, loopIterate, I,kk,x,dy,dx,concentric_flag);
                [Param] = computeEllipse2(directoryName, loopIterate, I,kk,x,dy,dx,concentric_flag);
                % extract data
                x = [Param(:,:).x; Param(:,:).y];
                % save the data.
                eval(['x_' num2str(kk) ' = x;']);
                eval(['X_' num2str(kk) ' = X;']);
            else
                %extract corners for square pattern
                [imgx, X] = corner_grid_detect(I, y, x, dy, dx, dY, dX);
                % save the data.
                eval(['x_' num2str(kk) ' = imgx;']);
                eval(['X_' num2str(kk) ' = X;']);
            end
            eval(['XXX_' num2str(kk) ' = X;']);
            % calc uncertainty & calc coordinate of uncertainty & calc weight
            weight = ones(3, dx*dy);
            weight(3,:) = 0;
            eval(['weight_', num2str(kk),' = weight;']);            
        else %if (loopIterate >1)
            if (concentric_flag==0) 
                %%% for just circles
                circleFilter_resized = imresize(circleFilter,[maxdis,maxdis],'bicubic');
                %circleFilter_resized = imresize2small(circleFilter,maxdis, maxdis);
                [Param] = computeCorrelationSSD(I, dy, dx,circleFilter_resized,maxdis,maxdis);
            else
                if (concentric_flag==1)
                    %%% for concentric circles
                    ringFilter_resized = imresize(ringFilter,[maxdis,maxdis],'bicubic');
                    %ringFilter_resized = imresize2small(ringFilter,maxdis,maxdis);
                    [Param] = computeCorrelationSSD(I, dy, dx,ringFilter_resized,maxdis,maxdis);
                    %[Param] = computeCorrelationNCC(directoryName, loopIterate, I, kk, dy, dx,ringFilter_resized,maxdis,maxdis);                    
                else
                    %%%for squares
                    [Param] = computeSquareCorrelation(directoryName, loopIterate, I, kk, dy, dx,crossFilterBlackTL, crossFilterWhiteTL,maxdis,maxdis);
                end
            end
                        
            % extract data
            x = [Param(:,:).x; Param(:,:).y];

            % remove outlier
            if outlier_flag == 1 
                outlier_map = removeOutlier(x, dx, dy, maxdis, maxdis, outlier_thr);
                outlier_map_arr(1:dx*dy, kk) = outlier_map;
            end
            
            % calc uncertainty & calc coordinate of uncertainty & calc weight
            if optimization_flag == 1
                [uncertainty] = calcUncertainty(I, x, dx, dy, maxdis, maxdis, outlier_map);
                [uncertainty_org] = convertUncertaintyCoordinate( uncertainty, x, dx, dy, homo, kc, cc, fc);
                [weight] = calcWeight(uncertainty_org, dx, dy);
            else
                weight = ones(3, dx*dy);
                weight(3,:) = 0;
            end
            eval(['weight_', num2str(kk),' = weight;']);

            % if we are working with unprojected view then project the points.
            %x(1,:) = x(1,:) + (-dX-1);
            %x(2,:) = x(2,:) + (-dY-1);
            %x(1,:) = x(1,:) +1;
            %x(2,:) = x(2,:) +1;

            XX = [x(1,:); x(2,:); zeros(1,size(x,2))];
            XXX = [x(1,:); x(2,:)];
            eval(['omc = ' 'omc_' num2str(kk) ';']);
            eval(['Tc = ' 'Tc_' num2str(kk) ';']);

            %xx = project_points2(XX,omc,Tc,fc,cc,kc,alpha_c);
            xx = Proj_Points(XX,homo,omc,Tc,fc,cc,kc,alpha_c,dx,dy);
            x = xx;
            % save the data.
            eval(['x_' num2str(kk) ' = x;']);
            eval(['X_' num2str(kk) ' = X;']);
            eval(['XX_' num2str(kk) ' = XX;']);
            eval(['XXX_' num2str(kk) ' = XXX;']);
            
        end % if (loopIterate == 1)
    end %for kk = 1:n_ima
    
    % setting variables
    nx = size(I,2);
    ny = size(I,1);

    % call calibration now
    go_calib_optim_iter

    % store the result parameters
    storage(1:2,loopIterate) = fc;
    storageParam(3:4,loopIterate) = cc;
    storageParam(5:6,loopIterate) = [kc(1); kc(2)];

    % store the error - Average error
    %error(loopIterate) = sum(sqrt(sum(ex.^2,1)))/size(ex,2);%sqrt(sum(err_std.^2)/2);
    sigma_sum = 0;
    for kk = 1:n_ima
        eval(['weight = weight_' num2str(kk) ';']);
        outlier_map = outlier_map_arr(:,kk);        
        for loop_pt=1:dx*dy
            if outlier_flag == 0
                % get weight of a point
                sigma = [weight(1,loop_pt) weight(3,loop_pt); weight(3,loop_pt) weight(2,loop_pt)];
            elseif outlier_flag == 1
                if outlier_map(loop_pt) == 0
                    sigma = zeros(2);
                else
                    sigma = [weight(1,loop_pt) weight(3,loop_pt); weight(3,loop_pt) weight(2,loop_pt)];
                end
            end
            sigma_sum = sigma_sum + sum(sigma(:),1)/2;
            pt = ex(1:2, (kk-1)*dx*dy+loop_pt);
            error(loopIterate) = error(loopIterate)+(pt'*sigma*pt);
        end
    end
    error(loopIterate) = sqrt(error(loopIterate)/(sigma_sum));
    string = sprintf('Iteration: %d    Average Error: %f', loopIterate,error(loopIterate));
    disp(string);

    % analyse error
    scope = 0.1;

    %axis_limit_values = [-1.0 1.0];

    analyse_error;

    % undistort and unproject images
    distdxdy_arr = zeros(1, n_ima);
    for kk = 1:n_ima
        % undistort the original
        eval(['I = ' 'IOrig_' num2str(kk) ';']);


        eval(['omc = ' 'omc_' num2str(kk) ';']);
        eval(['Tc = ' 'Tc_' num2str(kk) ';']);

        % project world control points to image view
        eval(['XX =', 'X_', num2str(kk), ';']);
        eval(['detectx =', 'x_', num2str(kk), ';']);
        projx = project_points2(XX,omc,Tc,fc,cc,kc,alpha_c);
        
        if (loopIterate==numIter)
            continue;
        end
        outlier_map = outlier_map_arr(:,kk);        
        maxdist = Calc_maxgrid(detectx, dy,dx,fc,cc,kc,alpha_c,outlier_map);
        distdxdy_arr(kk) = maxdist;
    end    
    maxdist = max(distdxdy_arr(:));   
    
    for kk = 1:n_ima
        % undistort the original
        eval(['I = ' 'IOrig_' num2str(kk) ';']);


        eval(['omc = ' 'omc_' num2str(kk) ';']);
        eval(['Tc = ' 'Tc_' num2str(kk) ';']);

        % project world control points to image view
        eval(['XX =', 'X_', num2str(kk), ';']);
        eval(['detectx =', 'x_', num2str(kk), ';']);
        projx = project_points2(XX,omc,Tc,fc,cc,kc,alpha_c);
        
        %figure(3);
        %imshow(uint8(I));
        %hold on;
        %%plot(detectx(1,:),detectx(2,:), 'r+');
        %plot(projx(1,:),projx(2,:), 'g+');
        %hold off;
        %drawnow;
        %%pause;

        if (loopIterate==numIter)
            continue;
        end

        % make front-parallel image
        string = sprintf('Unproj image %d',kk);
        disp(string);
        [image homography] = Proj_Undistort3(I,detectx, dY,dX,dy,dx,omc,Tc,fc,cc,kc,alpha_c, maxdist);

        % execute unshading step.
        if loopIterate > 1
            outlier_map = outlier_map_arr(:,kk);        
            image = Unshading(image, dx, dy, maxdist, outlier_map);
            figure(100+kk), imshow(uint8(image));
        end

        % saving the image to the memory
        if loopIterate > 1
            Rect_old = Rect;
        end
        Rect(kk).im = image; %eval(['_rect' num2str(kk) ' = ' image ';']);
        Rect(kk).homo = homography;
        Rect(kk).maxdist = maxdist;        

        % add the rect name to the images
        rectName = '_rect';

        %writing the image
        file = [directoryName rectName num2str(kk) '.bmp'];
        %string = sprintf('Writing %s',file);
        %disp(string);
        %imwrite(uint8(image), file); %imwrite(uint8(round(image)), file);
    end
    evaluator2;    
%    display2;
end
evaluator3;

if (0)
    figure, plot([1:numIter],error);
    title('Average Error Vs. Iterations');
    xlabel('Iteration');
    ylabel('Average Error in pixel');

    string = sprintf('%sConvergence_plot.bmp',directoryName);
    %saveas(gcf,string,'bmp');
end

disp('done');

%%%%%%%%%%%%%%%%%%%%%%%


function [x,X] = getROI(directoryName, current_loop, iter_no, im,y,x,dy,dx,dY,dX);
%%% this function returns the grid spaced using dx, dy for the image

% Compute the inside points through computation of the planar homography (collineation)

a00 = [x(1);y(1);1];
a10 = [x(2);y(2);1];
a11 = [x(3);y(3);1];
a01 = [x(4);y(4);1];


% Compute the planar collineation: (return the normalization matrix as well)

[Homo,Hnorm,inv_Hnorm] = compute_homography([a00 a10 a11 a01],[0 1 1 0;0 0 1 1;1 1 1 1]);


% Build the grid using the planar collineation:
n_sq_x = dx;
n_sq_y = dy;

% world points
x_l = ((0:n_sq_x)'*ones(1,n_sq_y+1))/n_sq_x;
y_l = (ones(n_sq_x+1,1)*(0:n_sq_y))/n_sq_y;
pts = [x_l(:) y_l(:) ones((n_sq_x+1)*(n_sq_y+1),1)]';

% image points
XX = Homo*pts;
XX = XX(1:2,:) ./ (ones(2,1)*XX(3,:));

% since we deal with # of ellipses and not corners, so modifying the code
% belows

% Np = (n_sq_x+1)*(n_sq_y+1);
% Xi = reshape(([0:n_sq_x]*dX)'*ones(1,n_sq_y+1),Np,1)';
% Yi = reshape(ones(n_sq_x+1,1)*[n_sq_y:-1:0]*dY,Np,1)';
Np = (n_sq_x)*(n_sq_y);
Xi = reshape(([0:n_sq_x-1]*dX)'*ones(1,n_sq_y),Np,1)';
%Yi = reshape(ones(n_sq_x,1)*[n_sq_y-1:-1:0]*dY,Np,1)';
Yi = reshape(ones(n_sq_x,1)*[0:n_sq_y-1]*dY,Np,1)';
Zi = zeros(1,Np);

Xgrid = [Xi;Yi;Zi];

%display
if (0)
    figure(1), imshow(uint8(im));
    hold on;
    for row=1:dy
        for col = 1:dx
            pt11 = XX(:,(row-1)*(dx+1)+col);
            pt12 = XX(:,(row-1)*(dx+1)+col+1);
            pt21 = XX(:,row*(dx+1)+col);
            pt22 = XX(:,row*(dx+1)+col+1);

            % top
            line([pt11(1) pt12(1)], [pt11(2) pt12(2)]);
            % bottom
            line([pt21(1) pt22(1)], [pt21(2) pt22(2)]);
            % left
            line([pt11(1) pt21(1)], [pt11(2) pt21(2)]);
            %right
            line([pt12(1) pt22(1)], [pt12(2) pt22(2)]);
        end
    end
    hold off;
    % pause;
    drawnow;
end


string = sprintf('%sGrid_image_%d_%d.jpg',directoryName, current_loop,iter_no);
% saveas(1,string);

x = XX;
X = Xgrid;

function [pt] = findLineIntersection(line1, line2);

A = [line1(1) line1(2); line2(1) line2(2)];
B = [line1(3) line2(3)];

pt = inv(A)*B';

% make it a column vector
pt = pt(:);

function [line] = findLine(x1, y1, x2, y2);

line(1) = y1-y2;
line(2) = x2-x1;

line(3) = line(1)*x1 + line(2)*y1;

line = line./norm(line);


