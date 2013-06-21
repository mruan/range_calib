function [R, q] = simpleRot(origin, lookat)
% origin and lookat must be column vectors
% Note: The quaternion is parametrized by [a,b,c] by dropping w.

assert(numel(origin) == 3 && numel(lookat)==3);

r = lookat - origin;
hz = r/norm(r);

hy = [0; 0; -1];
hy = hy - dot(hy, hz)*hz;
hy = hy/norm(hy);

hx = cross(hy, hz);

R = [hx, hy, hz];

q = dcm2quat(R);
if q(1) < 0
  q = -q;
end
q = q(2:4)';
end