function [] = plotResult(Gt, Xf, Zv)

figure(1); clf;
hold on;
% plot ground truth
x = Gt(25:3:end);
y = Gt(26:3:end);
z = Gt(27:3:end);
plot3(x, y, z, 'ro');

% plot where I think they should be
x = Xf(25:3:end);
y = Xf(26:3:end);
z = Xf(27:3:end);
plot3(x, y, z, 'bx');

numPts = numel(x);
cl = ['y.', 'm.', 'c.', 'g.'];
for i=0:3
[X, Y, Z] = projPtsBk(Xf(i*6+1:i*6+6), Zv(i*3*numPts+1:i*3*numPts+3*numPts));
plot3(X, Y, Z, cl(i*2+1:i*2+2));
end
hold off;
end

function [X, Y, Z] = projPtsBk(x, P)
tran = x(1:3);
if numel(x) ==6
  rot  = x(4:6);
  w = sqrt(1- dot(rot,rot));
  dcm = quat2dcm([w, rot']);
else
  dcm = quat2dcm(x(4:7));
end

[m, ~] = size(P);
m = int32(m/3);
P = reshape(P, 3, m);
Pts = dcm*P + repmat(tran, 1, m);
X = Pts(1,:);
Y = Pts(2,:);
Z = Pts(3,:);
end