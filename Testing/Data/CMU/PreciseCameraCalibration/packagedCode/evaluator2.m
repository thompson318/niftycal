% for debug
% higuchi
IMAGE_WIDTH = 720;
IMAGE_HEIGHT = 480;
x_grid = zeros(1, IMAGE_WIDTH/10);
y_grid = zeros(1, IMAGE_HEIGHT/10);
for loop = 1: IMAGE_WIDTH/10
    x_grid(1, loop) = loop*10;
end
for loop = 1: IMAGE_HEIGHT/10
    y_grid(1, loop) = loop*10;
end


% set the ground truth of parameters
fx_truth = 5.75/0.005;
fy_truth = fx_truth;
u0_truth = 360.5;
v0_truth = 240.5;
distortion_truth = [ -0.7, -0.3, 0.0004, 0.00005, 8 ];

truth_mapx = zeros(IMAGE_HEIGHT/10, IMAGE_WIDTH/10);
truth_mapy = zeros(IMAGE_HEIGHT/10, IMAGE_WIDTH/10);
result_mapx = zeros(IMAGE_HEIGHT/10, IMAGE_WIDTH/10);
result_mapy = zeros(IMAGE_HEIGHT/10, IMAGE_WIDTH/10);

for loop_x = 1: IMAGE_WIDTH/10
    for loop_y =1: IMAGE_HEIGHT/10
        % grand truth
        loop_u = x_grid(1, loop_x);
        loop_v = y_grid(1, loop_y);
        %loop_u = loop_u-0.005*(loop_u-u0_truth);
        %loop_v = loop_v-0.005*(loop_v-v0_truth);
        x = (loop_u-u0_truth)/fx_truth;
        y = (loop_v-v0_truth)/fy_truth;
        W = eye(3) * [x, y, 1]';
        x_d = W(1)/W(3);
        y_d = W(2)/W(3);
        r2 = x_d*x_d + y_d*y_d;
        x_dd = x_d*(1+distortion_truth(1)*r2 + distortion_truth(2)*r2*r2 + distortion_truth(5)*r2*r2*r2) + 2*distortion_truth(3)*x_d*y_d + distortion_truth(4)*(r2+2*x_d*x_d);
        y_dd = y_d*(1+distortion_truth(1)*r2 + distortion_truth(2)*r2*r2 + distortion_truth(5)*r2*r2*r2) + 2*distortion_truth(4)*x_d*y_d + distortion_truth(3)*(r2+2*y_d*y_d);        
        u = x_dd*fx_truth+u0_truth;
        v = y_dd*fy_truth+v0_truth;
        truth_mapx(loop_y, loop_x) = u;
        truth_mapy(loop_y, loop_x) = v;
        
        % result
        %u0_result = u0_truth;%cc(1);
        %v0_result = v0_truth;%cc(2);
        u0_result = cc(1);
        v0_result = cc(2);
        fx_result = fc(1);
        fy_result = fc(2);
        distortion_result = kc;
        
        loop_u = x_grid(1, loop_x);
        loop_v = y_grid(1, loop_y);
        x = (loop_u-u0_truth)/fx_truth;
        y = (loop_v-v0_truth)/fy_truth;
        W = eye(3) * [x, y, 1]';
        x_d = W(1)/W(3);
        y_d = W(2)/W(3);
        r2 = x_d*x_d + y_d*y_d;
        x_dd = x_d*(1+distortion_result(1)*r2 + distortion_result(2)*r2*r2) + 2*distortion_result(3)*x_d*y_d + distortion_result(4)*(r2+2*x_d*x_d);
        y_dd = y_d*(1+distortion_result(1)*r2 + distortion_result(2)*r2*r2) + 2*distortion_result(4)*x_d*y_d + distortion_result(3)*(r2+2*y_d*y_d);
        u = x_dd*fx_result+u0_result;
        v = y_dd*fy_result+v0_result;
        result_mapx(loop_y, loop_x) = u;
        result_mapy(loop_y, loop_x) = v;
        
    end
end

error_mapx = abs(result_mapx - truth_mapx);
error_mapy = abs(result_mapy - truth_mapy);
error_map = sqrt( error_mapx.*error_mapx + error_mapy.*error_mapy);% - sqrt( result_mapx.*result_mapx + result_mapy.*result_mapy));

ave_error = sum(sum(error_map(8:40,16:61)))/(33*46)
ave_error_whole = sum(sum(error_map(:,:)))/(73*49)
ave_error_arr(loopIterate) = ave_error;
ave_error_whole_arr(loopIterate) = ave_error_whole;

error_of_fp2d = zeros(dy,dx);
fp_ave = 0;
for loop_img=1: n_ima
    fp_arr = FP_Cell{loop_img};
    fp_arr = fp_arr ;
    eval(['x_result =', 'x_', num2str(loop_img), ';']);
    eval(['X_result =', 'XXX_', num2str(loop_img), ';']);
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
    figure(300+loop_img);
    plot3(X2,Y2,error_of_fp2d);
    axis([0,10,0,8,0,0.1]);
    fp_ave = fp_ave + sum(sum(error_of_fp2d))/(dx*dy);
   
    string = [directoryName, 'Points',num2str(loop_img),'_',outputname,num2str(loopIterate),'.txt'];
    fid=fopen(string, 'wt');
    fprintf(fid, '%4.12e\t%4.12e\n', x_result);
    fclose(fid);
    string = [directoryName, 'RectifiedPoints',num2str(loop_img),'_',outputname,num2str(loopIterate),'.txt'];
    fid=fopen(string, 'wt');
    fprintf(fid, '%4.12e\t%4.12e\n', X_result);
    fclose(fid);
end
ave_error_localization_arr(loopIterate) = fp_ave/n_ima

string = [directoryName, 'Parameters_',outputname,num2str(loopIterate),'.txt'];
fid=fopen(string, 'wt');
fprintf(fid, '%4.12e\n', fc(1));
fprintf(fid, '%4.12e\n', fc(2));
fprintf(fid, '%4.12e\n', cc(1));
fprintf(fid, '%4.12e\n', cc(2));
fprintf(fid, '%4.12e\n', kc(1));
fprintf(fid, '%4.12e\n', kc(2));
fprintf(fid, '%4.12e\n', kc(3));
fprintf(fid, '%4.12e\n\n', kc(4));
for loop_img=1: n_ima
    eval([ 'omckk=omc_' num2str(kk),';']);
    eval(['Tckk=Tc_' num2str(kk), ';']);
    fprintf(fid, '%4.12e\n', omckk(1));
    fprintf(fid, '%4.12e\n', omckk(2));
    fprintf(fid, '%4.12e\n', omckk(3));
    fprintf(fid, '%4.12e\n', Tckk(1));
    fprintf(fid, '%4.12e\n', Tckk(2));
    fprintf(fid, '%4.12e\n\n', Tckk(3));
end
fclose(fid);

