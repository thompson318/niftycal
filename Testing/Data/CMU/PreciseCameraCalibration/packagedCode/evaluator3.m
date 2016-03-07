%for debug
%higuchi
% save error of LUT (part of image)
sum(ave_error_arr(1:numIter))/(numIter);
string = [directoryName, 'local_error_',outputname '.txt'];
fid=fopen(string, 'wt');
fprintf(fid, '%4.12e\n', ave_error_arr);
fclose(fid);
figure(400);
plot(ave_error_arr);
axis([1,numIter,0,0.2]);
figure(401);
plot(ave_error_arr);
axis([1,numIter,0,5]);

% save error of LUT (whole image)
string = [directoryName, 'Whole_error_',outputname, '.txt'];
fid=fopen(string, 'wt');
fprintf(fid, '%4.12e\n', ave_error_whole_arr);
fclose(fid);
figure(402);
plot(ave_error_whole_arr);
axis([1,numIter,0,0.2]);
figure(403);
plot(ave_error_whole_arr);
axis([1,numIter,0,5]);

% save error of RMS error
string = [directoryName, 'RMS_',outputname,'.txt'];
fid=fopen(string, 'wt');
fprintf(fid, '%4.12e\n', error);
fclose(fid);
figure(404);
plot(error);
axis([1,numIter,0,0.02]);
figure(405);
plot(error);
axis([1,numIter,0,0.5]);

% save error of feature point localization
string = [directoryName, 'Localization_error_',outputname,'.txt'];
fid=fopen(string, 'wt');
fprintf(fid, '%4.12e\n', ave_error_localization_arr);
fclose(fid);
figure(406);
plot(ave_error_localization_arr);
axis([1,numIter,0,0.02]);
figure(407);
plot(ave_error_localization_arr);
axis([1,numIter,0,0.5]);

% save estimated parameters.
%{
string = [directoryName, 'EstimatedParam_',outputname,'.txt'];
fid=fopen(string, 'wt');
fprintf(fid, '%4.12e\n ', [fc_buf(1, 5);fc_buf(2, 5);cc_buf(1, 5);cc_buf(2, 5);kc_buf(1, 5);kc_buf(2, 5);kc_buf(3, 5);kc_buf(4, 5);kc_buf(5, 5);omc_buf(1, 5);omc_buf(2, 5);omc_buf(3, 5);tc_buf(1, 5);tc_buf(2, 5);tc_buf(3, 5));
fclose(fid);
%}