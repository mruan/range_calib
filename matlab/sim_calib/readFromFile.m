function [Xv, Zv] = readFromFile(filename)
fd = fopen(filename, 'r');

num_srs = fscanf(fd ,'NumSensors: %d', 1);
fgetl(fd);
num_lms = fscanf(fd ,'NumLandmarks: %d', 1);
fgetl(fd);

Xv = []; Zv = [];
for i=1:num_srs
  [x, z, fd] = readSensors(fd);
  Xv = [Xv; x];
  Zv = [Zv; z];
end

[x, fd] = readLandmarks(fd, num_lms);
Xv = [Xv; x];

fclose(fd);
end

function [x, z, fd] = readSensors(fd)

fgetl(fd);
fgetl(fd);
x = fscanf(fd, 'Quaternion: %f %f %f %f', 4);
fgetl(fd);
x = x/norm(x);
if x(1)<0
  x = -x(2:4);
else
  x = x(2:4);
end

t = fscanf(fd, 'Translation: %f %f %f', 3);
fgetl(fd);

x = [t; x];

numobs = fscanf(fd, 'NumObservations: %d', 1);
fgetl(fd);

z = fscanf(fd, '%f %f %f %f', [4, numobs]); 
fgetl(fd);

z = z(2:end,:);
z = z(:);
end

function [x, fd] = readLandmarks(fd, num_lms)
fgetl(fd);

x = fscanf(fd, '%f %f %f', [3, num_lms]); 
x = x(:);
end