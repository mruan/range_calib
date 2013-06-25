function [R, q] = simpleRot(origin, lookat)
% origin and lookat must be column vectors
% Note: The quaternion is parametrized by [a,b,c] by dropping w.

assert(numel(origin) == 3 && numel(lookat)==3);

r = lookat - origin;
hz = r/norm(r);

hx = cross([0;0;-1], hz);
hx = hx/norm(hx);

hy = cross(hz, hx);
hy = hy/norm(hx);

R = [hx, hy, hz];

% Notice this is DCM, so we have to transpose the Rotation Matrix
q = r2q(R);

q = q(2:4)';
end