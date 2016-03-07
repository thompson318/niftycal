% for debug
% higuchi
sx = 1024;%2800

for kk = 1:n_ima
    % result of feature points localization on input images.
    eval(['I = ' 'IOrig_' num2str(kk) ';']);
    eval(['omc = ' 'omc_' num2str(kk) ';']);
    eval(['Tc = ' 'Tc_' num2str(kk) ';']);

    % project world features to image view
    eval(['XX =', 'X_', num2str(kk), ';']);
    eval(['detectx =', 'x_', num2str(kk), ';']);
    projx = project_points2(XX,omc,Tc,fc,cc,kc,alpha_c);

    hdl = figure(10+kk);
    if kk <= 4
        set(hdl,'Position',[sx-720/2*(kk-1),1000,720/2,480/2]);
    else
        set(hdl,'Position',[sx-720/2*(kk-1),1000-480/2,720/2,480/2]);
    end
    imshow(uint8(I));
    hold on;
    plot(detectx(1,:),detectx(2,:), 'r+');
    plot(projx(1,:),projx(2,:), 'g+');
    hold off;
    %drawnow;
    %%pause;
    

    % error of feature points localization on input images.
    error_of_fp2d = zeros(dy,dx);
    fp_arr = FP_Cell{kk};
    fp_arr = fp_arr ;
    eval(['x_result =', 'x_', num2str(kk), ';']);
    %error_of_fp = xx - fp_arr';
    counter_fp = 0;
    for loop_dy=1:dy
        for loop_dx=1:dx
            counter_fp = counter_fp+1;
            error_of_fp2d(loop_dy, loop_dx ) = sqrt((fp_arr(1, counter_fp)-x_result(1, counter_fp))^2+(fp_arr(2, counter_fp)-x_result(2, counter_fp))^2);
           
        end
    end
    XXX=1:1:dx;
    YYY=1:1:dy;
    [X2, Y2] = meshgrid(XXX,YYY);
    hdl=figure(30+kk);
    if kk <= 4
        set(hdl,'Position',[sx-720/2*(kk-1),1000-480*1,720/3,480/3]);
    else
        set(hdl,'Position',[sx-+720/2*(kk-1),1000-480*2,720/3,480/3]);
    end
    plot3(X2,Y2,error_of_fp2d);
    axis([0,10,0,8,0,0.1]);

    
    % result of feature points localization on front-parallel images.
    hdl = figure(20+kk);
    if kk <= 4
        set(hdl,'Position',[sx-720/2*(kk-1),1000-380*2,720/2,480/2]);
    else
        set(hdl,'Position',[sx-720/2*(kk-1),1000-380*3,720/2,480/2]);
    end
    if (loopIterate>1)
        I = Rect_old(kk).im; %eval(['I = ' '_rect' num2str(kk) ';']);
        imshow(uint8(I));
        
        eval(['XX =', 'XX_', num2str(kk), ';']);
        hold on;
        plot(XX(1,:),XX(2,:), 'r+');
        hold off;
        %drawnow;
    end
    %%pause;

    
    % result of wegiht on front-parallel images.
    hdl = figure(40+kk);
    if kk <= 4
        set(hdl,'Position',[sx-720/2*(kk-1),1000-345*3,720/2,480/2]);
    else
        set(hdl,'Position',[sx-720/2*(kk-1),1000-345*4,720/2,480/2]);
    end
%%{
    if (loopIterate>1)
        eval(['weight = weight_' num2str(kk) ';']);                
        eval(['weight_pre = weight_pre' num2str(kk) ';']);   
        weight_img = zeros(size(I));
        weight_pre_img = zeros(size(I));
        eval(['XX =', 'XX_', num2str(kk), ';']);
        [sizey,sizex]=size(I);
        crcle = zeros(2, dX+1);
        for loop_pnt = 1: dx*dy
            for loop_x = XX(1, loop_pnt)-dX:0.1:XX(1, loop_pnt)+dX
                if loop_x < 1
                    loop_x = 1;
                elseif loop_x > sizex
                    loop_x = sizex;
                end
                var = [weight_pre(1,loop_pnt),weight_pre(3,loop_pnt);
                    weight_pre(3,loop_pnt),weight_pre(2,loop_pnt)];
                var = var*var/1000000;
                var = inv(var);
                CC = var(1,1)*(loop_x-XX(1,loop_pnt))^2 - 0.03^2;
                BB = 2*var(1,2)*(loop_x-XX(1,loop_pnt));
                AA = var(2,2);
                if(BB^2-4*AA*CC>=0)
                    y1 = (-BB+sqrt(BB^2-4*AA*CC))/(2*AA);
                    y2 = (-BB-sqrt(BB^2-4*AA*CC))/(2*AA);
                    y=uint32(y1+XX(2,loop_pnt));
                    if y>=1 && y <=sizey
                        weight_img(y,uint32(loop_x)) = 255;
                    end
                    y=uint32(y2+XX(2,loop_pnt));
                    if y>=1 && y <=sizey
                        weight_img(y,uint32(loop_x)) = 255;                    
                    end
                end
            end
        end
        imshow(uint8(weight_img));        
        hold on;
        plot(XX(1,:),XX(2,:), 'r+');
        hold off;
    end
%}    
%{    
    if (loopIterate>1)
        eval(['weight = weight_' num2str(kk) ';']);                
        eval(['weight_pre = weight_pre' num2str(kk) ';']);                
        weight_img = zeros(size(I));
        weight_pre_img = zeros(size(I));
        eval(['XX =', 'XX_', num2str(kk), ';']);
        [sizey,sizex]=size(I);
        crcle = zeros(2, dX+1);
        for loop_pnt = 1: dx*dy
            for loop_x = XX(1, loop_pnt)-dX:0.1:XX(1, loop_pnt)+dX
                if loop_x < 1
                    loop_x = 1;
                elseif loop_x > sizex
                    loop_x = sizex;
                end
                var = [weight(1,loop_pnt),weight(3,loop_pnt);
                    weight(3,loop_pnt),weight(2,loop_pnt)];
                var = inv(var);
                CC = var(1,1)*(loop_x-XX(1,loop_pnt))^2 - 0.03^2;
                BB = 2*var(1,2)*(loop_x-XX(1,loop_pnt));
                AA = var(2,2);
                if(BB^2-4*AA*CC>=0)
                    y1 = (-BB+sqrt(BB^2-4*AA*CC))/(2*AA);
                    y2 = (-BB-sqrt(BB^2-4*AA*CC))/(2*AA);
                    y=uint32(y1+XX(2,loop_pnt));
                    if y>=1 && y <=sizey
                        weight_img(y,uint32(loop_x)) = 255;
                    end
                    y=uint32(y2+XX(2,loop_pnt));
                    if y>=1 && y <=sizey
                        weight_img(y,uint32(loop_x)) = 255;                    
                    end
                end
            end
        end
        imshow(uint8(weight_img));        
        hold on;
        plot(XX(1,:),XX(2,:), 'r+');
        hold off;
    end    
%}    
    
end

%error_map = sqrt( error_mapx.*error_mapx + error_mapy.*error_mapy);% - sqrt( result_mapx.*result_mapx + result_mapy.*result_mapy));
%%{
XX=1:1:IMAGE_WIDTH/10;
YY=1:1:IMAGE_HEIGHT/10;
[XXX, YYY] = meshgrid(XX,YY);
%figure(200);
%plot3(X,Y,error_mapx);
%figure(201);
%plot3(X,Y,error_mapy);
hdl = figure(202);
%set(hdl,'Position',[2800,0,720/3,480/3]);
set(hdl,'Position',[sx-720/2*4,1000,720/2,480/2]);

plot3(XXX,YYY,error_map);
axis([0,80,0,60,0,0.35]);
%figure(204);
%plot3(X,Y,error_map);
%axis([0,80,0,60,0,0.05]);

XXXX=16:1:610/10;
YYYY=8:1:400/10;
[X2, Y2] = meshgrid(XXXX,YYYY);
hdl =figure(203);
%set(hdl,'Position',[2800-720/3,0,720/3,480/3]);
set(hdl,'Position',[sx-720/2*4,1000-380*2,720/2,480/2]);
plot3(X2,Y2,error_map(8:40,16:61));
axis([0,80,0,60,0,0.35]);
%%}
