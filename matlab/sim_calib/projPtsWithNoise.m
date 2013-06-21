function [Pts] = projPtsWithNoise(x, P)
% unpack x into R and T -> tf
tf = x;

[m,n] = size(P);
Pts = tf\[P; ones(1,n)];

noise = randn([m, n]);

nsr = 0.00; % noise to signal ratio
Pts = Pts(1:3,:) + nsr*noise;
end