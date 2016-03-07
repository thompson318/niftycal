function [Param] = computeSquareCorrelation(directoryName, current_loop, im,iter_no,dy,dx,filterBlackTL,filterWhiteTL,distancex,distancey);
% extract each region, find the peak of correlation and refine it at sub-pixel level.

% do NCC and save the result
% change isnan or isinf to zero
im(isnan(im)) = 0; im(isinf(im)) = 0;

% C = normxcorr2(circle,im);
% string = sprintf('%sCorrelation_%d_%d.bmp',directoryName,current_loop, iter_no);
% imwrite(C, string);

% the distance between circle centers
counter = 0;
for row=1:dy
    for col = 1:dx        
        %predicted circle window in un-proj orthogonal view
        pt11 = round([distancex*(col-0.5),distancey*(row-0.5)]);
        pt22 = round([distancex*(col+0.5), distancey*(row+0.5)]);
        
        % extract square region - decent approximation
        I = im(pt11(2):pt22(2),pt11(1):pt22(1));
               
        %check Black/Whilte
        [nr nc] = size(I);
        pixTL = I(round(nr/4), round(nc/4)); 
        pixTR = I(round(nr/4), round(3*nc/4)); 
        pixBL = I(round(3*nr/4), round(nc/4)); 
        pixBR = I(round(3*nr/4), round(3*nc/4)); 
        
        if (pixTL+pixBR<pixTR+pixBL)
            crossFilter = filterBlackTL;
        else
            crossFilter = filterWhiteTL;
        end
        
        % Normalized cross-correlation
        C = normxcorr2(crossFilter,I);
               
        [yc,xc] = find(C==max(max(C)));
        %take the first peak if there is multiple
        if size(yc,1)>1
            yc = yc(1);
            xc = xc(1);
        end

        dxx = 0; dyy = 0;        
        [flag, dxx, dyy] = quad_subpixel([C(yc-1:yc+1,xc-1:xc+1)]);
       
        %[dxx, dyy] = subpixel(C,xc,yc);
        
%         yc = yc - floor(size(circle,1)/2);
%         xc = xc - floor(size(circle,2)/2);
        yc = yc - (size(crossFilter,1)-1)/2;
        xc = xc - (size(crossFilter,2)-1)/2;
        
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
c = round([[Param(:).x]']);
r = round([[Param(:).y]']);
plot(c,r,'g+');
hold off;
drawnow;
end
