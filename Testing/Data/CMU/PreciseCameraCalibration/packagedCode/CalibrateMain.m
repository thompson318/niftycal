function CalibrateMain();
%CalibrateMain
%Created by Ankur Datta
%Modified by Mirai Higuchi
%
%Main calibration function to load input data.
%Please custumize this function.

close all;
clear all;

addpath(genpath('TOOLBOX_calib'));

% setting appropriate ring flag:
data_flag = 0; % 1 for real datasets; % 0 for synthetic datasets

% setting variable
N_ima = 10; % number of images
numIter = 5;% number of iterations

dX = 48; %distance between control points (x-axis)[mm]
dY = 48; %distance between control points (x-axis)[mm]
string = sprintf('dX and dY set at: %f %f', dX, dY);
disp(string);
dx = 10; %number of control points (x-axis)
dy = 7;  %number of control points (y-axis)

%create Filters for circle and ring
createFilter2;

%buffer for input images
img_arr = cell(1,N_ima);
%buffer for ground truth of control points
fp_coordinate_arr = cell(1,N_ima);
%Bounding box
boundingbox = zeros(8, N_ima);
for loop_pt = 1:1
    if loop_pt == 1
        filePrefix = 'checker_';
        %directoryName1 = '/Users/mirai/Data/SourceCode/Calibration/images/checker1/'; %for Mac
        %directoryName2 = '/Users/mirai/Data/SourceCode/Calibration/images/checker2/'; %for Mac
        directoryName1 = 'C:\user\higuchi\SyntheticData_work\checker1\'; %for windows
        directoryName2 = 'C:\user\higuchi\SyntheticData_work\checker2\'; %for windows
        %0 for circle pattern; 1 for ring pattern; % -1 for square
        concentric_flag = -1;
        %0 for Bundle Adjustment; 1 for Weighted Bundle Adjustment         
        optimization_flag = 0;% Only 0
        %0 for don't remove outlier; 1 for remove outlier        
        outlier_flag = 0;
        outputname = 'ankur'; % output filename for debug & evaluation
    elseif loop_pt == 2
        filePrefix = 'circle_';
        %directoryName1 = '/Users/mirai/Data/SourceCode/Calibration/images/circle1/'; %for Mac
        %directoryName2 = '/Users/mirai/Data/SourceCode/Calibration/images/circle2/'; %for Mac
        directoryName1 = 'C:\user\higuchi\SyntheticData_work\circle1\'; %for windows
        directoryName2 = 'C:\user\higuchi\SyntheticData_work\circle2\'; %for windows
        %0 for circle pattern; 1 for ring pattern; % -1 for square
        concentric_flag = 0;
        %0 for Bundle Adjustment; 1 for Weighted Bundle Adjustment        
        optimization_flag = 1;
        %0 for don't remove outlier; 1 for remove outlier        
        outlier_flag = 0;
        outputname = 'weighted'; % output filename for debug & evaluation
    elseif loop_pt == 3
        filePrefix = 'ring_';
        directoryName1 = '/Users/mirai/Data/SourceCode/Sample/Ankur/AnkurCalibration/images/data8/ring1/'; %for Mac
        directoryName2 = '/Users/mirai/Data/SourceCode/Sample/Ankur/AnkurCalibration/images/data8/ring2/'; %for Mac
        directoryName1 = 'C:\user\higuchi\SyntheticData_work\ring1\'; %for windows
        directoryName2 = 'C:\user\higuchi\SyntheticData_work\ring2\'; %for windows
        %0 for circle pattern; 1 for ring pattern; % -1 for square
        concentric_flag = 1;
        %0 for Bundle Adjustment; 1 for Weighted Bundle Adjustment        
        optimization_flag = 1;
        %0 for don't remove outlier; 1 for remove outlier        
        outlier_flag = 0;
        outputname = 'weighted'; % output filename for debug & evaluation
    end

    for loop_deg = 45:-15:15
        for loop_rz = -5:5:5
            for loop_pose =0: 1      
                if loop_deg == 45 && loop_pose == 1
                    continue;
                end
                for loop_img=1: N_ima;
                    if loop_img <=5
                        directoryName0 = directoryName1;
                    else
                        directoryName0 = directoryName2;
                    end
                    
                    foldername = [num2str(loop_deg),'_rz',num2str(loop_rz),'/',num2str(loop_pose),'/'];
                    if loop_pose == 0                   
                        if loop_img == 1 || loop_img == 6
                            first = '0';
                            second = [num2str(loop_deg)];
                        elseif loop_img == 2 || loop_img == 7
                            first = '0';
                            second = [num2str(-1*loop_deg)];
                        elseif loop_img == 3 || loop_img == 8
                            second = '0';
                            first = [num2str(loop_deg)];                       
                        elseif loop_img == 4 || loop_img == 9 
                            second = '0';
                            first = [num2str(-1*loop_deg)];                       
                        else 
                            first = '0';
                            second ='0';
                        end
                        filename = [ filePrefix, first, '_' second,'_', num2str(loop_rz)];
                    else
                        if loop_img == 1 || loop_img == 6
                            first = [num2str(loop_deg)];
                            second = [num2str(loop_deg)];
                        elseif loop_img == 2 || loop_img == 7
                            first = [num2str(loop_deg)];
                            second = [num2str(-1*loop_deg)];
                        elseif loop_img == 3 || loop_img == 8
                            first = ['-',num2str(loop_deg)];
                            second = [num2str(loop_deg)];
                        elseif loop_img == 4 || loop_img == 9 
                            first = ['-',num2str(loop_deg)];
                            second = [num2str(-1*loop_deg)];
                        else 
                            first = '0';
                            second ='0';
                        end
                        filename = [ filePrefix, first, '_' second,'_', num2str(loop_rz)];
                    end

                    string = [directoryName0, foldername, filename, '_.bmp'];
                    disp(string);
                    I = imread(string);
                    %%% convert to grayscale if color image
                    if (size(I,3) ~= 1)
                        I = rgb2gray(I);
                    end
                    img_arr{1, loop_img} = I;

                    % read the ground truth of control points coordinates
                    string = [directoryName0, foldername, filename, '_.txt'];
                    fid=fopen(string, 'rt');
                    fp_coordinate = fscanf(fid, '%f\t%f\n', [2 inf]);
                    fclose(fid);
                    fp_coordinate_arr{1, loop_img} = fp_coordinate;
                    % set bounding box
                    if concentric_flag >= 0
                        x(1) = fp_coordinate(1, 1)-dX/3;
                        y(1) = fp_coordinate(2, 1)-dY/3;
                        x(2) = fp_coordinate(1, dx)+dX/3;
                        y(2) = fp_coordinate(2, dx)-dY/3;
                        x(3) = fp_coordinate(1, dy*dx)+dX/3;
                        y(3) = fp_coordinate(2, dy*dx)+dY/3;
                        x(4) = fp_coordinate(1, (dy-1)*dx+1)-dX/3;
                        y(4) = fp_coordinate(2, (dy-1)*dx+1)+dY/3;
                    else
                        x(1) = fp_coordinate(1, 1);
                        y(1) = fp_coordinate(2, 1);
                        x(2) = fp_coordinate(1, dx);
                        y(2) = fp_coordinate(2, dx);
                        x(3) = fp_coordinate(1, dy*dx);
                        y(3) = fp_coordinate(2, dy*dx);
                        x(4) = fp_coordinate(1, (dy-1)*dx+1);
                        y(4) = fp_coordinate(2, (dy-1)*dx+1);
                    end
                    boundingbox(1:8,loop_img) = [ x'; y'];
                    
                    % save the input image
                    eval(['IOrig_' num2str(loop_img) ' = I;']);
                    
                    % display the input image
                    figure(10+loop_img);
                    imshow(I);
                end

                [error, storageParam] = circleCalibrateScript(dX, dY, dx, dy, [directoryName2,foldername],outputname, img_arr,fp_coordinate_arr, boundingbox, numIter, concentric_flag,optimization_flag, data_flag, outlier_flag);

            end
        end
    end
end