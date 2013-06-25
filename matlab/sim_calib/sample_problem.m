function [Xv, Zv, Gt] = sample_problem()

C1t = [-2, 2, 3]';
C2t = [-2,-2, 3]';
C3t = [ 2,-2, 3]';
C4t = [ 2, 2, 3]';

% q is parameterized by [a,b,c];
[R1, q1] = simpleRot(C1t, [ 2,-2, 0]');
[R2, q2] = simpleRot(C2t, [ 2, 2, 0]');
[R3, q3] = simpleRot(C3t, [-2, 2, 0]');
[R4, q4] = simpleRot(C4t, [-2,-2, 0]');

[X, Y] = meshgrid(-1:0.5:1, -1:0.5:1);
Z = 2.0*exp(-X.^2 - Y.^2 - X.*Y);
%surf(X,Y,Z);
P = [X(:)'; Y(:)'; Z(:)'];

Z1 = projPtsWithNoise([R1, C1t; 0 0 0 1], P);
Z2 = projPtsWithNoise([R2, C2t; 0 0 0 1], P);
Z3 = projPtsWithNoise([R3, C3t; 0 0 0 1], P);
Z4 = projPtsWithNoise([R4, C4t; 0 0 0 1], P);

Zv = [Z1(:); Z2(:); Z3(:); Z4(:)];
Xv = [C1t; q1; ...
      C2t; q2; ...
      C3t; q3; ...
      C4t; q4; ...
      P(:)];

% For simplicity, only add noise to translation
nsr= 0.00;

noise = [nsr*randn(size(C1t)); 0;0;0; ...
         nsr*randn(size(C1t)); 0;0;0; ...
         nsr*randn(size(C1t)); 0;0;0; ...
         nsr*randn(size(C1t)); 0;0;0; ...
         nsr*randn(numel(P),1)];
Xv = Xv + noise;


% ground truth
Gt = [C1t; q1; C2t; q2; C3t; q3; C4t; q4; P(:)];

end