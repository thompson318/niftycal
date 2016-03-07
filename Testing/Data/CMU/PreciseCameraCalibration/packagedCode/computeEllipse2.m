function [Param] = computeEllipse2(directoryName, current_loop, im,iter_no,imagePts,dy,dx,ring_flag);
%%% extract each region, threshold it, extract the largest connected
%%% component, find canny edges, subpixel refine edges, fit ellipse.
%%% Use gravity center instead of ellipse fitting, which may not be consistant

% threshold = 125;
imagePts = round(imagePts);
counter = 0;

% %display flag
% displayFlag = 1;
% 
% if (displayFlag)
% figure(1); 
% h = imshow(im);
% hold on;
% end;

% %smooth with gaussian
% im2 = double(im);
% im2 = max(max(im2))-im2;
% sigma = 4;
% dist = [-5 -4 -3 -2 -1 0 1 2 3 4 5];
% gau = (1/(sqrt(2*pi)*sigma))*exp(-dist.^2/(2*sigma^2)); %guassian (5*5)
% im2 = conv2(im2, gau, 'same');
% im2 = conv2(im2, gau', 'same');
% %imshow(uint8(im2));
% im2 = max(max(im2)) - im2;
% im2 = 255*(im2-min(min(im2)))./(max(max(im2))-min(min(im2)));
% imshow(uint8(im2));
% im = uint8(im2);

% extract each region.
for row=1:dy
    for col = 1:dx
        pt11 = imagePts(:,(row-1)*(dx+1)+col);
        pt12 = imagePts(:,(row-1)*(dx+1)+col+1);
        pt21 = imagePts(:,row*(dx+1)+col);
        pt22 = imagePts(:,row*(dx+1)+col+1);

        % extract square region - decent approximation
        Imgcell = double(im(pt11(2):pt22(2),pt11(1):pt22(1)));
        Imgcell = 255 - Imgcell;
        
        % get size
        [nr nc] = size(Imgcell);        
        
        % compute adaptive threshold
        cweight = 0.5;
        threshold = min(min(Imgcell))*cweight + max(max(Imgcell))*(1-cweight) ;
        
        % threshold it
        Ibw = zeros(nr,nc);
        for ri=1:nr
            for ci=1:nc
                if (Imgcell(ri,ci)>=threshold)
                    Ibw(ri,ci) = 1;
                else
                    Ibw(ri,ci) = 0;
                end
            end
        end

        % largest connected component
        X = bwlabel(Ibw,8);
        num_comp = max(max(X));        
        if (num_comp>2)
%             num_comp = num_comp;    
%             disp('more than 1 comp');
%             pause;
        end
        region_size = 0;
        ind = 0;
        maxPixCount = 0;
        for cmpidx = 1:num_comp
            pixCount = sum(sum(X==cmpidx));
            if (pixCount > region_size)
                region_size = pixCount;
                maxPixCount = pixCount;
                ind = cmpidx;
            end
        end
        region = (X==ind);

        %if ring, fille the inner holes
%         if (ring_flag)
%             region_ring = region;
%             region = bwfill(region,'holes',8);
%             region_inner = xor(region, region_ring);
%         end
        
        %get gravity center as initial ellipse center   
        [y2, x2] = find(region==1);
        x3 = mean(x2);
        y3 = mean(y2);
        
        gravx = x3;
        gravy = y3;
        % if concentric circles, find the inner center and take average
%         if (ring_flag)
%             %get gravity center of inner ellipse center
%             count = 0;
%             sumx = 0;
%             sumy = 0;
%             for ri=1:nr
%                 for ci=1:nc
%                     if (region_inner(ri,ci)==0) continue; end
%                     sumx = sumx + ci;
%                     sumy = sumy + ri;
%                     count = count+1;
%              end
%            end
%            gravx2 = sumx/count;
%            gravy2 = sumy/count;  
%            %take average of the two centers
%            gravx = (gravx + gravx2)/2;
%            gravy = (gravy + gravy2)/2;                     
%        end
        
        % find canny edge
        %[edgeim,thresh] = edge(double(region),'canny');

%       edgeim=adaptivethreshold(region,35,0.03,0);
                
        % plot the ellipse
%         if (displayFlag)
%             [r,c] = find(edgeim~=0);
%             plot(c+pt11(1)-1, r+pt11(2)-1,'r.');
%         end       

        %use gravity center as ellipse center
        param(1) = gravx;
        param(2) = gravy;

        % store result
        counter = counter+1;
        %Param(counter).data = [pt11' pt12' pt22' pt21' param];
        Param(counter).x = pt11(1) + param(1)-1;
        Param(counter).y = pt11(2) + param(2)-1;
    end
end

%display ellipse centers
figure(1)
imshow(uint8(im));
hold on;
% c = roundn([Param(:).x]',0);
% r = roundn([Param(:).y]',0);
c = [Param(:).x]';
r = [Param(:).y]';
for i=1:size(c)
    plot(c(i),r(i),'g+');
end
hold off;
drawnow;

%save image
%string = sprintf('%sEllipse_center_%d.jpg',directoryName, iter_no);
%saveas(1,string);
