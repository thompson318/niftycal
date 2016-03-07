function [smallim] = imresize2small(input_img, newsizex, newsizey);
% imresize2small
% Created by Mirai Higuchi
%
% resize input image.
%
% INPUT:
% input_img - input image
% newsizex - new image size
% newsizey - new image size
%
% OUTPUT:
% smallim - resized image
%

[insizey, insizex] = size(input_img);
smallim = zeros(newsizey, newsizex);
for loop_y=0: newsizey-1
    for loop_x =0: newsizex-1
        sx = loop_x*insizex/newsizex+1;
        ex = (loop_x+1)*insizex/newsizex;
        sy = loop_y*insizey/newsizey+1;
        ey = (loop_y+1)*insizey/newsizey;
        %imgRgsmall(loop_y+1, loop_x+1) = imgRgsmall(loop_y+1, loop_x+1) + imgRg(floor(sy),floor(sx))*(1-(sx-floor(sx)))*(1-(sy-floor(sy)));
        for loop_yy = floor(sy): ceil(ey)
            for loop_xx = floor(sx): ceil(ex)
                if loop_xx <= insizex && loop_yy <= insizey
                    if loop_xx < sx && loop_yy < sy
                        smallim(loop_y+1, loop_x+1) = smallim(loop_y+1, loop_x+1) + input_img(loop_yy,loop_xx)*(1-(sx-loop_xx))*(1-(sy-loop_yy));
                    elseif loop_xx < sx && loop_yy > floor(ey)
                        smallim(loop_y+1, loop_x+1) = smallim(loop_y+1, loop_x+1) + input_img(loop_yy,loop_xx)*(1-(sx-loop_xx))*(1-(loop_yy-ey));
                    elseif loop_xx > floor(ex) && loop_yy < sy
                        smallim(loop_y+1, loop_x+1) = smallim(loop_y+1, loop_x+1) + input_img(loop_yy,loop_xx)*(1-(loop_xx-ex))*(1-(sy-loop_yy));
                    elseif loop_xx > floor(ex) && loop_yy > floor(ey)
                        smallim(loop_y+1, loop_x+1) = smallim(loop_y+1, loop_x+1) + input_img(loop_yy,loop_xx)*(1-(loop_xx-ex))*(1-(loop_yy-ey));
                    elseif loop_xx < sx
                        smallim(loop_y+1, loop_x+1) = smallim(loop_y+1, loop_x+1) + input_img(loop_yy,loop_xx)*(1-(sx-loop_xx));
                    elseif loop_xx > floor(ex)
                        smallim(loop_y+1, loop_x+1) = smallim(loop_y+1, loop_x+1) + input_img(loop_yy,loop_xx)*(1-(loop_xx-ex));
                    elseif loop_yy < sy
                        smallim(loop_y+1, loop_x+1) = smallim(loop_y+1, loop_x+1) + input_img(loop_yy,loop_xx)*(1-(sy-loop_yy));
                    elseif loop_yy > floor(ey)
                        smallim(loop_y+1, loop_x+1) = smallim(loop_y+1, loop_x+1) + input_img(loop_yy,loop_xx)*(1-(loop_yy-ey));
                    else
                        smallim(loop_y+1, loop_x+1) = smallim(loop_y+1, loop_x+1) + input_img(loop_yy,loop_xx);
                    end
                end
            end
        end
    end
end

smallim = smallim/(insizex/newsizex*insizey/newsizey);