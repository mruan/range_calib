function [] = plotResult(Xf, Zv, varargin)

figure(1); clf;
hold on;

nVarargs = length(varargin);
if nVarargs >0
  Gt = varargin{1};
  % plot ground truth
  x = Gt(25:3:end);
  y = Gt(26:3:end);
  z = Gt(27:3:end);
  plot3(x, y, z, 'ro');
end  
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

addCamera(Xf(i*6+1:i*6+3), Xf(i*6+4:i*6+6), cl(i*2+1));

% If ground truth is supplied
if nVarargs >0
  addCamera(Gt(i*6+1:i*6+3), Gt(i*6+4:i*6+6), 'r');
end

end
hold off;
end

function [X, Y, Z] = projPtsBk(x, P)
tran = x(1:3);
if numel(x) ==6
  rot  = x(4:6);
  w = sqrt(1- dot(rot,rot));
  R = q2r([w, rot']);
else
  R = q2r(x(4:7));
end

[m, ~] = size(P);
m = int32(m/3);
P = reshape(P, 3, m);
Pts = R*P + repmat(tran, 1, m);
X = Pts(1,:);
Y = Pts(2,:);
Z = Pts(3,:);
end