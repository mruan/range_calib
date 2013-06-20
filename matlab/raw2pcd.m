% Author: Ming Ruan
% Data:   June 18
% Discription:
% Convert raw data to pcd file. Refer to the wiki page to find the exact
% format of the raw output from different sensors.
% (Current version only tested for swissranger)

clear % clear variables in current workspace

% Change the file name accordingly
path = '../data/';
filename = 'sr1.txt'; % This file must exist in current directory
data = dlmread(strcat(path, filename));

% Some sensor specific data, change accordingly
sensor = 'sr1';
width  = 176;
height = 144;
stride = width*height;

% You normally don't have to touch the rest
[row, col] = size(data);
num_frames = int32(row/stride);
for i=1:num_frames
    name = sprintf('%s%s_%u.pcd', path, sensor, i-1);
    fileID = fopen(name,'w');
    
    % Print pcd file header (metadata)
    fprintf('Writing to file %s\n', name);
    fprintf(fileID, 'VERSION .7\n');
    fprintf(fileID, 'FIELDS x y z\n');
    fprintf(fileID, 'SIZE 4 4 4\n');
    fprintf(fileID, 'TYPE F F F\n');
    fprintf(fileID, 'COUNT 1 1 1\n');
    fprintf(fileID, 'WIDTH %d\n', width);
    fprintf(fileID, 'HEIGHT %d\n', height);
    fprintf(fileID, 'VIEWPOINT 0 0 0 1 0 0 0\n');
    fprintf(fileID, 'POINTS %d\n', width*height);
    fprintf(fileID, 'DATA ascii\n');

    % then the points
    offset = (i-1) * stride;
    for j=offset+1:offset+stride
        fprintf(fileID, '%f %f %f\n', data(j,1), data(j,2), data(j,3));
    end

    fclose(fileID);
end