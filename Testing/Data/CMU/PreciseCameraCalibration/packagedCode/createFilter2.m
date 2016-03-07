%create optimal corelation fliter 
% using digital integration of signal+gaussian
% Created by Ankudr Datta
% Modified by Mirai Higuchi

%assign parameters
kernelwid = min(dX,dY);
sigma = 3;
%inner_rad = radius/3;

%create filter
center = [(kernelwid+1)/2, (kernelwid+1)/2];
mid = kernelwid/2+0.5; %kernelwid has to be even

%load filter image
if (~exist('patt'))
    patt = 3;   %2/6 = 1/3 ring
else
    if (patt==0)
        patt = 2;
    end
end
circleFilter = double(imread('gcircleFilter.bmp'));
ringFilter = double(imread(['gringFilter.bmp']));

%flip filter
circleFilter = 255 - circleFilter;
ringFilter = 255 - ringFilter;

%smooth kernel
dist = zeros(sigma*2+1, sigma*2+1);
for loop_x = -sigma: sigma
    for loop_y = -sigma: sigma
%    dist = [-3 -2 -1 0 1 2 3];
        dist(loop_y+sigma+1, loop_x+sigma+1) = sqrt(loop_y^2+loop_x^2);
    end
end
gau = (1/(sqrt(2*pi)*sigma))*exp(-dist.^2/(2*sigma^2)); %guassian (7*7)

%define cross filter for square correlation
for row=1:kernelwid
    for col = 1:kernelwid
        if (row < mid)
            if (col<mid)
                crossFilterBlackTL(row,col) = 0;
                crossFilterWhiteTL(row,col) = 255;
            else
                crossFilterBlackTL(row,col) = 255;
                crossFilterWhiteTL(row,col) = 0;
            end
        else 
            if (col<mid)
                crossFilterBlackTL(row,col) = 255;
                crossFilterWhiteTL(row,col) = 0;                
            else
                crossFilterBlackTL(row,col) = 0;
                crossFilterWhiteTL(row,col) = 255;                
            end
        end
    end
end
crossFilterBlackTL = conv2(crossFilterBlackTL, gau, 'same');
crossFilterBlackTL = conv2(crossFilterBlackTL, gau', 'same');
crossFilterWhiteTL = conv2(crossFilterWhiteTL, gau, 'same');
crossFilterWhiteTL = conv2(crossFilterWhiteTL, gau', 'same');

        
% %display filter
% figure;
% imagesc(circleBoard);
% pixval;
% figure;
% imagesc(ringBoard);
% pixval;