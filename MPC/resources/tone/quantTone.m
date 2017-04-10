clear all
close all
clc

%% Display Experimental Data

filename = 'stiffness.png';
img = imread(filename);

xMin = 0; % MAS score, where 1.5 = 1+
xMax = 4;
if ~isempty(strfind(filename,'stiff'))
    yMin = 0;
    yMax = 15;
else
    yMin = 0;
    yMax = 45;
end

figure()
imagesc([xMin xMax], [yMin yMax], img);

%% Convert from Figure to Numbers

n = 17; % number of data points
disp(' ');
points = zeros(2,n);
for i = 1:n
    points(:,i) = ginput(1)';
end
disp(points)
