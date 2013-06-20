
filepath = '../data/sr0_0.pcd';
data = dlmread(filepath, ' ', 10, 0);

cloud = reshape(data(:,3), 176, 144)';
cloud = fliplr(cloud); % flip left to right
surf(cloud, 'EdgeColor', 'none');
%axis equal
colormap gray
view(0, 90);
