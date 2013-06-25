clear;

% Initialize random seed (fixed)
s = RandStream('mcg16807','Seed',0);
RandStream.setGlobalStream(s);

% Create a sample problem
[Xv, Zv, Gt] = sample_problem();
writeToFile('sim.txt', Xv, Zv);
[Xv, Zv] = readFromFile('sim.txt');

% Initial guess (zero +- operation point)
x0 = zeros(size(Xv));

options = optimset('Jacobian','on', ...
                   'Algorithm', {'levenberg-marquardt',.001},...
                   'TolX', 1e-14);
[xp, resn,res,f] = lsqnonlin(@(x) myfun(x, Xv, Zv), x0, [], [], options);
%[F, J] = myfun(x, Xv, Zv);

xf = getFinalX(Xv, xp);

%r = norm(xf- Gt);

plotResult(xf, Zv, Gt);
