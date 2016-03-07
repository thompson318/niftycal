function CalibrateMainReal();
%CalibrateMainReal
%Created by Ankur Datta
%Modified by Mirai Higuchi
%
%Main calibration function. Load input data.
%Please custumize this function.

close all;
clear all;

addpath(genpath('TOOLBOX_calib'));
fileprefix ='img';
% setting appropriate ring flag:
data_flag = 0; % 1 for real datasets; % 0 for synthetic datasets

% setting variable
N_ima = 10; % number of images
numIter = 5;% number of iterations

dX = 23; %distance between control points (x-axis)[mm]
dY = 23; %distance between control points (x-axis)[mm]
string = sprintf('dX and dY set at: %f %f', dX, dY);
disp(string);
dx = 10; %number of control points (x-axis)
dy = 7;  %number of control points (y-axis)

%create Filters for circle and ring
createFilter2;

%Read Images
img_arr = cell(1,N_ima);
%Read Ground truth data
fp_coordinate_arr = cell(1,N_ima); % dummy
%Bounding box
boundingbox = zeros(8, N_ima);
for loop_pt = 1:1
    if loop_pt == 1
        filePrefix = 'img'; %checker
        %directoryName1 = '/Users/mirai/Data/SourceCode/Sample/Ankur/AnkurCalibration/images_real/data3/checker/2/'; %for mac
        %directoryName2 = '/Users/mirai/Data/SourceCode/Sample/Ankur/AnkurCalibration/images_real/data3/checker/3/'; %for mac
        directoryName1 = 'C:\user\higuchi\RealData\checker\1\'; %for windows
        directoryName2 = 'C:\user\higuchi\RealData\checker\2\'; %for windows
        %0 for circle pattern; 1 for ring pattern; % -1 for square
        concentric_flag = -1; 
        %0 for Bundle Adjustment; 1 for Weighted Bundle Adjustment         
        optimization_flag = 0; % Only 0
        %0 for don't remove outlier; 1 for remove outlier        
        outlier_flag = 0;
        outputname = 'ankur'; % output filename for debug & evaluation
    elseif loop_pt == 2
        filePrefix = 'img'; %circle
        %directoryName1 = '/Users/mirai/Data/SourceCode/Sample/Ankur/AnkurCalibration/images_real/data3/circle/2/'; %for mac
        %directoryName2 = '/Users/mirai/Data/SourceCode/Sample/Ankur/AnkurCalibration/images_real/data3/circle/3/'; %for mac
        directoryName1 = 'C:\user\higuchi\RealData\circle\1\'; %for windows
        directoryName2 = 'C:\user\higuchi\RealData\circle\2\'; %for windows
        %0 for circle pattern; 1 for ring pattern; % -1 for square
        concentric_flag = 0;
        %0 for Bundle Adjustment; 1 for Weighted Bundle Adjustment         
        optimization_flag = 0;
        %0 for don't remove outlier; 1 for remove outlier        
        outlier_flag = 0;
        %outputname = 'weighted'; % output filename for debug & evaluation
        outputname = 'ankur'; % output filename for debug & evaluation
    elseif loop_pt == 3
        filePrefix = 'img'; %ring
        %directoryName1 = '/Users/mirai/Data/SourceCode/Sample/Ankur/AnkurCalibration/images_real/data3/ring/3/'; %for mac
        %directoryName2 = '/Users/mirai/Data/SourceCode/Sample/Ankur/AnkurCalibration/images_real/data3/ring/4/'; %for mac
        directoryName1 = 'C:\user\higuchi\RealData\ring\1\'; %for windows
        directoryName2 = 'C:\user\higuchi\RealData\ring\2\'; %for windows
        %0 for circle pattern; 1 for ring pattern; % -1 for square
        concentric_flag = 1;
        %0 for Bundle Adjustment; 1 for Weighted Bundle Adjustment         
        optimization_flag = 1;
        %0 for don't remove outlier; 1 for remove outlier        
        outlier_flag = 1;
        %outputname = 'weighted'; % output filename for debug & evaluation
        %outputname = 'weighted'; % output filename for debug & evaluation
        outputname = 'weightedRANSAC'; % output filename for debug & evaluation
    end
    img_cnt = 1;
    for loop_img=1: N_ima;
        if loop_img <= 5
            directoryName0 = directoryName1;
        elseif loop_img == 6
            img_cnt=1;
            directoryName0 = directoryName2;
        else
            directoryName0 = directoryName2;
        end

        filename = [fileprefix, num2str(img_cnt)];
        string = [directoryName0, filename, '.bmp'];
        disp(string);
        I = imread(string);
        %%% convert to grayscale if color image
        if (size(I,3) ~= 1)
            I = rgb2gray(I);
        end
        img_arr{1, loop_img} = I;

        % read control points coordinates (Dummy)
        fp_coordinate = zeros(2,dx*dy);
        fp_coordinate_arr{1, loop_img} = fp_coordinate;
        % set bounding box
        if loop_img <=5
            directoryName = directoryName1;
            ptFile = sprintf('%scornerPts_%d.mat',directoryName,loop_img);
        elseif loop_img <=10
            directoryName = directoryName2;
            ptFile = sprintf('%scornerPts_%d.mat',directoryName,loop_img-5);
        elseif loop_img <=15
            directoryName = directoryName3;
            ptFile = sprintf('%scornerPts_%d.mat',directoryName,loop_img-10);
        elseif loop_img <=20
            directoryName = directoryName4;
            ptFile = sprintf('%scornerPts_%d.mat',directoryName,loop_img-15);
        end
        if (~exist(ptFile))
            %prompt
            if (concentric_flag>=0)
                % click on 4 points bounding circle/ring pattern
                disp('Select the image region containing the ellipses.');
                disp('Click the bounding box of the pattern grid');
            else
                %click 4 corners of square grid
                disp('Follow the same point clicking procedure as the matlab toolbox.');
            end

            figure(1), imshow(I);
            hold on;
            for i = 1:4
                [x(i),y(i)] = ginput(1);
                plot(x(i),y(i),'r+');
                drawnow;
            end
            hold off;
            % save points
            if loop_img <=5
                file = sprintf('%scornerPts_%d.mat',directoryName, loop_img);
                save(file, 'x', 'y');
            elseif loop_img <=10
                file = sprintf('%scornerPts_%d.mat',directoryName, loop_img-5);
                save(file, 'x', 'y');
            elseif loop_img <=15
                file = sprintf('%scornerPts_%d.mat',directoryName, loop_img-10);
                save(file, 'x', 'y');
            elseif loop_img <=15
                file = sprintf('%scornerPts_%d.mat',directoryName, loop_img-15);
                save(file, 'x', 'y');
            end
        else
            load(ptFile);
        end
        boundingbox(1:8,loop_img) = [ x'; y'];

        % save the image
        eval(['IOrig_' num2str(loop_img) ' = I;']);
        figure(10+loop_img);
        imshow(I);
        
        img_cnt = img_cnt+1;
    end

    foldername = [''];
    [error, storageParam] = circleCalibrateScript(dX, dY, dx, dy, [directoryName2,foldername], outputname, img_arr, fp_coordinate_arr, boundingbox, numIter, concentric_flag,optimization_flag, data_flag, outlier_flag);
end