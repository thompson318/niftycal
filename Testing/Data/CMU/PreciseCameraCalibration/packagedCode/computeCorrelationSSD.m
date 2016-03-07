function [Param] = computeCorrelationSSD(im,dy,dx,filter,distancex,distancey)
%computeCorrelationSSD
%Created by Ankur Datta
%Modified by Mirai Higuchi
%
% extract each region, find the peak of ssf and refine it at sub-pixel level.
%
%INPUT: 
% im - rectified image
% dx - number of control points (x-axis)
% dy - number of control points (y-axis)
% filter - template image for SSD 
% distancex - distance between any adjacent control points on a rectified image plane 
% distancey - distance between any adjacent control points on a rectified image plane 
%
%OUTPUT: 
%Param - 2D coordinates of refined control points on a rectified image plane
%

%assign input filter
circle = filter;

% do NCC and save the result
% change isnan or isinf to zero
im(isnan(im)) = 0; im(isinf(im)) = 0;


% the distance between circle centers
counter = 0;
for row=1:dy
    for col = 1:dx        
        %predicted circle window in un-proj orthogonal view
        pt11 = round([distancex*(col-1)+1,distancey*(row-1)+1]);
        pt22 = round([distancex*(col+1), distancey*(row+1)]);
        
        % extract square region - decent approximation
        I = im(pt11(2):pt22(2),pt11(1):pt22(1));
               
        %flip image
        I = 255 - I;
        
%         %%% check to see the sizes of the images make sense
%         if (size(I,1) < size(circle,1) | size(I,2) < size(circle,2))
%             ty = max([ size(circle,1)-size(I,1) 0]);
%             tx = max([ size(circle,2)-size(I,2) 0]);
%             
%             I = im(pt11(2):pt22(2)+ty,pt11(1):pt22(1)+tx);
%         end

        % SSD
        [sizey sizex] = size(I);
        ssd_arr = ones(sizey, sizex)*999999999999;
        for loop_y=1:sizey
            for loop_x=1:sizex
                if (loop_y+distancey-1) < sizey && (loop_x+distancex-1) < sizex
                    Icroped = I(loop_y:loop_y+distancey-1, loop_x:loop_x+distancex-1);
                    ssd_arr(loop_y,loop_x) = sum((Icroped(:)-circle(:)).^2);
                end
            end
        end

%         keyboard;
        
%         % store the result
%         im2(pt11(2):pt22(2),pt11(1):pt22(1)) = C(floor(size(C,1)/2)-floor(size(I,1)/2):floor(size(C,1)/2)+floor(size(I,1)/2),...
%             floor(size(C,2)/2)-floor(size(I,2)/2):floor(size(C,2)/2)+floor(size(I,2)/2));
        
        % subpixel refine the peak         
        [yc,xc] = find(ssd_arr==min(min(ssd_arr)));
        %take the first peak if there is multiple
        if size(yc,1)>1
            yc = yc(1);
            xc = xc(1);
        end

        dxx = 0; dyy = 0;     
        if yc > 1 && yc < sizey && xc > 1 && xc < sizex
            [flag, dxx, dyy] = quad_subpixelssd([ssd_arr(yc-1:yc+1,xc-1:xc+1)]);        
        end
%{
        if iter_no == 5 && row == 4 && col ==5
            tmp = ssd_arr(yc, xc-5:xc+5);
            figure(5000);
            plot(tmp);
            axis([0,12,0,50000000]);        
        end
%}        
        %[dxx, dyy] = subpixel(C,xc,yc);
        
%         yc = yc - floor(size(circle,1)/2);
%         xc = xc - floor(size(circle,2)/2);
        yc = yc + (size(circle,1)-1)/2;
        xc = xc + (size(circle,2)-1)/2;
        
        %transfer back to whole board coordinates
        detectx = pt11(1) + xc + dxx - 1;
        detecty = pt11(2) + yc + dyy - 1;
        
        % store the data
        counter = counter + 1;
        Param(counter).x = detectx;
        Param(counter).y = detecty;                
    end
end

%display detection results
if 1
figure(2)    
imshow(uint8(im));
hold on;
% c = roundn([Param(:).x]',0);
% r = roundn([Param(:).y]',0);
c = [Param(:).x]';
r = [Param(:).y]';
plot(c,r,'g+');
hold off;
drawnow;
end
