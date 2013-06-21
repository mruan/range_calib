clear;

s = RandStream('mcg16807','Seed',0);
RandStream.setGlobalStream(s);

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
Z = 2*exp(-X.^2 - Y.^2 - X.*Y);
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
nsr= 0.05;
noise = [nsr*randn(size(C1t)); 0;0;0; ...
         nsr*randn(size(C1t)); 0;0;0; ...
         nsr*randn(size(C1t)); 0;0;0; ...
         nsr*randn(size(C1t)); 0;0;0; ...
         nsr*randn(numel(P),1)];
Xv = Xv + noise;
x0 = zeros(size(Xv));

options = optimset('Jacobian','on', ...
                   'Algorithm', {'levenberg-marquardt',.001},...
                   'TolX', 1e-14);
[xp, resn,res,f] = lsqnonlin(@(x) myfun(x, Xv, Zv), x0, [], [], options);
%[F, J] = myfun(x, Xv, Zv);

xf = getFinalX(Xv, xp);
Gt = [C1t; q1; C2t; q2; C3t; q3; C4t; q4; P(:)];

r = norm(xf- Gt);
plotResult(Gt, xf, Zv);
