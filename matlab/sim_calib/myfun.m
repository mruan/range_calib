function [F, J] = myfun(x, X0, Z)
% X0 specifies the operating point (initial guess)
% x  specifies the current pertubation around X0
% This turns a constrained optimization (quaternion=1) to a unconstrained
% version by claiming w = sqrt(1-x^2-y^2-z^2)

% num of params per camera
np = 6;

% num of cameras
nc = 4;

% num of dimensions of each meansurement
nd = 3;

% num of measurements
nm = (numel(x) - np*nc) / nd;

stride = nm*nd;
som = np*nc+1; % first element of the points/measurements

% Pre-allocate memory
F = zeros(stride, 1);
J = zeros(stride, numel(x));

% follow c conventions
for i=0:nc-1 % for each camera
  tf = state2Tf( x(i*np+1:i*np+6));
  Tf = state2Tf(X0(i*np+1:i*np+6));
  
  sr = i*nm*nd; % starting row
  temp = projPts(tf*Tf, Z(sr+1:sr+stride));
  res  = temp - x(som:end) - X0(som:end);
  F(sr+1:sr+stride) = res;
  
  % [4:6] [10:12] [16:18] [22:24]
  [J1, J2, J3] = computeJacforQuat(x(i*np+4:i*np+np));
  
  sc = i*np; % starting column to access camera params

  % precompute projected points
  zp = projPts(Tf, Z(sr+1:sr+stride));
  
  for j=0:nm-1 % for each measurement
    tof = sr+j*nd; % total offset
    % [x,y,z] -> Jac = I
    J(tof+1:tof+3, sc+1:sc+3) = eye(3);
    
    % [a,b,c] -> Jac quite complecated
    z = zp(j*nd+1:j*nd+3); % the jth measurement as a column vector
    jac = [z'*J1; z'*J2; z'*J3];
    J(tof+1:tof+3, sc+4:sc+6) = zeros(size(jac));
%    J(tof+2, sc+4:sc+6) = z'*J2;
%    J(tof+3, sc+4:sc+6) = z'*J3;
    
    % [px, py, pz] -> Jac = -I
    J(tof+1:tof+3, 4*np+j*nd+1:4*np+j*nd+3) = -eye(3);
  end
end
r = sqrt(F'*F);
fprintf('Error %f\n', r);
end

function [Tf] = state2Tf(x)
tran = x(1:3);
if numel(x) ==6
  rot  = x(4:6);
  w = sqrt(1- dot(rot,rot));
  R = q2r([w, rot']);
else
  R = q2r(x(4:7));
end
Tf = [R, tran; 0 0 0 1];
end

function [P] = projPts(tf, Pz)
[m,~] = size(Pz);
m = int32(m/3);
Pz= reshape(Pz, 3, m);
Pz= [Pz; ones(1, m)];

P = tf*Pz;
P = P(1:3,:);
P = P(:);
end

function [J1, J2, J3] = computeJacforQuat(v)
a=v(1); b=v(2); c=v(3);
w=sqrt(1-a*a-b*b-c*c);

aaw = 2*a*a/w;
abw = 2*a*b/w;
acw = 2*a*c/w;
bbw = 2*b*b/w;
bcw = 2*b*c/w;
ccw = 2*c*c/w;

J1 =[   0      2*b+acw  2*c-abw; ...
     -4*b      2*a+bcw  2*w-bbw; ...
     -4*c     -2*w+ccw  2*a-bcw]';
J2 =[ 2*b-acw -4*a     -2*w+aaw; ...
      2*a-bcw    0      2*c+abw; ...
      2*w-ccw -4*c      2*b+acw]';
J3 =[ 2*c+abw  2*w-aaw -4*a; ...
     -2*w+bbw  2*c-abw -4*b; ...
     -2*a+bcw  2*b-acw 0]';
end