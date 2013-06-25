
% The synthetic problem that we are trying to solve
% I just need the ground truth to display
% MAKE SURE these are the same problem
[Xv, Zv, Gt] = sample_problem();

%writeToFile('sim.txt', Xv, Zv);
% The problem solved by BA
[Xv, Zv] = readFromFile('out.txt');

plotResult(Xv, Zv);