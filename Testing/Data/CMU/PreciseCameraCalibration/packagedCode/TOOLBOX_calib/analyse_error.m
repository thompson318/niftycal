% Color code for each image:

if ~exist('n_ima')|~exist('fc'),
    fprintf(1,'No calibration data available.\n');
    return;
end;

check_active_images;

if n_ima ~=0,
if ~exist(['ex_' num2str(ind_active(1)) ]),
    fprintf(1,'Need to calibrate before analysing reprojection error. Maybe need to load Calib_Results.mat file.\n');
    return;
end;
end;


%if ~exist('no_grid'),
no_grid = 0;
%end;

colors = 'brgkcm';

%h = figure;
h = figure(5);

for kk = 1:n_ima,
    if exist(['y_' num2str(kk)]),
        if active_images(kk) & eval(['~isnan(y_' num2str(kk) '(1,1))']),
            
            if ~no_grid,
                eval(['XX_kk = X_' num2str(kk) ';']);
                N_kk = size(XX_kk,2);
                
                if ~exist(['n_sq_x_' num2str(kk)]),
                    no_grid = 1;
                end;
                
                if ~no_grid,
                    eval(['n_sq_x = n_sq_x_' num2str(kk) ';']);
                    eval(['n_sq_y = n_sq_y_' num2str(kk) ';']);
                    if (N_kk ~= ((n_sq_x+1)*(n_sq_y+1))),
                        no_grid = 1;
                    end;
                end;
            end;
            
            eval(['plot(ex_' num2str(kk) '(1,:),ex_' num2str(kk) '(2,:),''' colors(rem(kk-1,6)+1) '+'');']);
            hold on; 
        end;
    end;
end;

if (exist('axis_limit_values'))
    %if (loopIterate >= 2)
     %   axis_limit_values = [-0.5 0.5];
    %end
        
    axis_limit_values
    xlim([axis_limit_values(1) axis_limit_values(2)]);
    set(gca,'XTick',[axis_limit_values(1):0.1:axis_limit_values(2)]);
    ylim([axis_limit_values(1) axis_limit_values(2)]);
    set(gca,'YTick',[axis_limit_values(1):0.1:axis_limit_values(2)]);    
    grid on;
end


hold off;
% axis('equal');
iter_err = sum(sqrt(sum(ex.^2,1)))/size(ex,2);

string1 = sprintf('Reprojection error (in pixel): %f - To exit: right button',iter_err);
string2 = sprintf('Reprojection error (in pixel): %f',iter_err);

if 1, %~no_grid,
    title(string1);
else
    title(string2);   
end;
xlabel('x');
ylabel('y');

set(5,'color',[1 1 1]);
set(5,'Name','error','NumberTitle','off');

if n_ima == 0,
    
        text(.5,.5,'No image data available','fontsize',24,'horizontalalignment' ,'center');

else

err_std = std(ex')';

fprintf(1,'Pixel error:          err = [ %3.5f   %3.5f] (all active images)\n\n',err_std); 

% saving the error plot
if (exist('loopIterate') & exist('directoryName'))
    string = sprintf('%sError_plot_%d.bmp',directoryName, loopIterate);
    saveas(gcf, string, 'jpg');
end

b=0;

while b==1,
    
    [xp,yp,b] = ginput3(1);
    
    if b==1,
        ddd = (ex(1,:)-xp).^2 + (ex(2,:)-yp).^2;
        
        [mind,indmin] = min(ddd);
        
        
        done = 0;
        kk_ima = 1;
        while (~done)&(kk_ima<=n_ima),
            %fprintf(1,'%d...',kk_ima);
            eval(['ex_kk = ex_' num2str(kk_ima) ';']);
            sol_kk = find((ex_kk(1,:) == ex(1,indmin))&(ex_kk(2,:) == ex(2,indmin)));
            if isempty(sol_kk),
                kk_ima = kk_ima + 1;
            else
                done = 1;
            end;
        end;
        
        eval(['x_kk = x_' num2str(kk_ima) ';']);    
        xpt = x_kk(:,sol_kk);
        eval(['y_kk = y_' num2str(kk_ima) ';']);    
        ypt = y_kk(:,sol_kk);
        
        if ~no_grid,
            
            eval(['n_sq_x = n_sq_x_' num2str(kk_ima) ';']);
            eval(['n_sq_y = n_sq_y_' num2str(kk_ima) ';']);
            
            Nx = n_sq_x+1;
            Ny = n_sq_y+1;
            
            y1 = floor((sol_kk-1)./Nx);
            x1 = sol_kk - 1 - Nx*y1; %rem(sol_kk-1,Nx);
            
            y1 = (n_sq_y+1) - y1;
            x1 = x1 + 1;
            
            
            fprintf(1,'\n');
            fprintf(1,'Selected image: %d\n',kk_ima);
            fprintf(1,'Selected point index: %d\n',sol_kk);
            fprintf(1,'Pattern coordinates (in units of (dX,dY)): (X,Y)=(%d,%d)\n',[x1-1 y1-1]);
            fprintf(1,'Image coordinates (in pixel): (%3.2f,%3.2f)\n',[xpt']);
            fprintf(1,'Pixel error = (%3.5f,%3.5f)\n',[ex(1,indmin) ex(2,indmin)]);
            
                        
        else
            
            fprintf(1,'\n');
            fprintf(1,'Selected image: %d\n',kk_ima);
            fprintf(1,'Selected point index: %d\n',sol_kk);
            fprintf(1,'Detected coordinates (in pixel): (%3.2f,%3.2f)\n',[xpt']);
            fprintf(1,'True coordinates (in pixel): (%3.2f,%3.2f)\n',[ypt']);
            
            fprintf(1,'Pixel error = (%3.5f,%3.5f)\n',[ex(1,indmin) ex(2,indmin)]);
            
            if (loopIterate ==1)
                eval(['I = IOrig_' num2str(kk_ima) ';']);
            else
                eval(['I = I_' num2str(kk_ima) ';']);
            end
            
            I2 = I(floor(xpt(2)-1):floor(xpt(2)+1),floor(xpt(1)-1):floor(xpt(1)+1));
            
            figure(1), imshow(uint8(I));
            hold on;
            plot(xpt(1), xpt(2), 'r+');
            plot(ypt(1), ypt(2), 'g+');
            title('Image showing the detected center');
            hold off;
            figure(2), imshow(uint8(I2),'InitialMagnification',10000);
            hold on;
            plot(xpt(1)-floor((xpt(1)-1))+0.5, xpt(2)-floor((xpt(2)-1))+0.5, 'r+');
            plot(ypt(1)-floor((xpt(1)-1))+0.5, ypt(2)-floor((xpt(2)-1))+0.5, 'g+');
            title('Zoomed Image');
            hold off;
            
%             if (exist(directoryName))
%                 string = sprintf('%sEllipse_center_%d.jpg',directoryName, kk_ima);
%                 I = imread(string);
%                 figure(2), imshow(I);
%                 title('Corresponding Ellipse image');
%             end

            figure(5);
            
            
        end;
        
        
        if exist(['wintx_' num2str(kk_ima)]),
            
            eval(['wintx = wintx_' num2str(kk_ima) ';']);
            eval(['winty = winty_' num2str(kk_ima) ';']);
            
            fprintf(1,'Window size: (wintx,winty) = (%d,%d)\n',[wintx winty]);
        end;
        
        
    end;
    
end;

disp('done');

end;
