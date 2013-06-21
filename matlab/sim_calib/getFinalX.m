function [final] = getFinalX(X, x)
final = zeros(size(X));

for i=0:3
  final(i*6+1:i*6+3) = X(i*6+1:i*6+3) + x(i*6+1:i*6+3);
  q0 = X(i*6+4:i*6+6)';
  r0 = quat2dcm([sqrt(1-q0*q0'), q0]);
  q1 = x(i*6+4:i*6+6)';
  r1 = quat2dcm([sqrt(1-q1*q1'), q1]);
  
  rx = r1*r0;
  qx = dcm2quat(rx);
  if (qx(1) < 0)
    qx = -qx;
  end
  
  final(i*6+4:i*6+6) = qx(2:end)';
end

final(4*6+1:end) = X(4*6+1:end) + x(4*6+1:end);
end