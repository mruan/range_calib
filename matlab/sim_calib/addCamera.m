function [] = addCamera(t, q, color)

% width, height, depth control how the camera box look like
w = 0.16;
h = 0.12;
d = 0.32;

% vertices
P = [w, h, d; ...
     w,-h, d; ...
    -w,-h, d; ...
    -w, h, d]';

if iscolumn(q)
  q = q';
end
if isrow(t)
  t = t';
end

if numel(q) == 3
  q = [sqrt(1-q*q'), q];
end
R = q2r(q);

Pp = R*P;

alpha = 0:0.05:1;

OC1 = zeros(3,21);
OC2 = zeros(3,21);
OC3 = zeros(3,21);
OC4 = zeros(3,21);
C12 = zeros(3,21);
C23 = zeros(3,21);
C34 = zeros(3,21);
C41 = zeros(3,21);
for i=1:21
  OC1(:,i) = t + alpha(i)*Pp(:,1);
  OC2(:,i) = t + alpha(i)*Pp(:,2);
  OC3(:,i) = t + alpha(i)*Pp(:,3);
  OC4(:,i) = t + alpha(i)*Pp(:,4);
  
  C12(:,i) = t+Pp(:,1) + alpha(i)*(Pp(:,2)-Pp(:,1));
  C23(:,i) = t+Pp(:,2) + alpha(i)*(Pp(:,3)-Pp(:,2));
  C34(:,i) = t+Pp(:,3) + alpha(i)*(Pp(:,4)-Pp(:,3));
  C41(:,i) = t+Pp(:,4) + alpha(i)*(Pp(:,1)-Pp(:,4));
end

plot3(OC1(1,:), OC1(2,:), OC1(3,:), color);
plot3(OC2(1,:), OC2(2,:), OC2(3,:), color);
plot3(OC3(1,:), OC3(2,:), OC3(3,:), color);
plot3(OC4(1,:), OC4(2,:), OC4(3,:), color);

plot3(C12(1,:), C12(2,:), C12(3,:), color);
plot3(C23(1,:), C23(2,:), C23(3,:), color);
plot3(C34(1,:), C34(2,:), C34(3,:), color);
plot3(C41(1,:), C41(2,:), C41(3,:), color);

end