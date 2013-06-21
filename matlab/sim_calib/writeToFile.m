function [] = writeToFile(filename, Xv, Zv)
fd = fopen(filename, 'w');

num_sensor = 4;
num_obs = 25;

fprintf(fd, 'NumSensors: %d\n', num_sensor);
fprintf(fd, 'NumLandmarks: %d\n', num_obs);
fd = WriteSensor(fd, 'RangeSensor', 'g0', Xv(1 :6),  Zv(1  :75));
fd = WriteSensor(fd, 'RangeSensor', 'g1', Xv(7 :12), Zv(76 :150));
fd = WriteSensor(fd, 'RangeSensor', 'g2', Xv(13:18), Zv(151:225));
fd = WriteSensor(fd, 'RangeSensor', 'g3', Xv(19:24), Zv(226:300));
fd = WriteLandmarks(fd, Xv(25:end));

fclose(fd);
end

function [fd] = WriteSensor(fd, type, name, X, Z)
[num_obs, ~] = size(Z);

num_obs = int32(num_obs /3);

t = X(1:3);
q = [sqrt(1 - X(4:6).^2); X(4:6)];

fprintf(fd, 'Type: %s\n', type);
fprintf(fd, 'Name: %s\n', name);
fprintf(fd, 'Quaternion: %f %f %f %f\n', q(1), q(2), q(3), q(4));
fprintf(fd, 'Translation: %f %f %f\n', t(1), t(2), t(3));
fprintf(fd, 'NumObservations: %d\n', num_obs);

for i=0:num_obs-1
  fprintf(fd, '%d %f %f %f\n', i, Z(3*i+1), Z(3*i+2), Z(3*i+3));
end

end

function [fd] = WriteLandmarks(fd, P)
[num_obs, ~] = size(P);
num_obs = int32(num_obs /3);

fprintf(fd, 'Landmarks:\n');
for i=0:num_obs-1
  fprintf(fd, '%f %f %f\n', P(3*i+1), P(3*i+2), P(3*i+3));
end

end
