clear all
close all
clc

% NOTE: The following script extracts elbow parameters only, specifically
% ----  slope & intercept for MAS vs. gamma, mu, k, b.
fitsE = zeros(4,2);

% enter spasticity score vs. stretch reflex threshold data from (Levin &
% Feldman, 2003)
spast_LF = [7 8 8 9 11 11 11 11 12 12 14];                                        % [0-16] (8 pts from MAS)
static_LF = [33.19 58.28 87.56 60.98 41.84 52.20 55.85 74.04 57.63 71.80 42.70];  % gamma [deg]
dynamic_LF = [0.327 0.238 0.294 0.356 0.316 0.178 0.127 0.225 0.197 0.257 0.236]; % mu [sec]

% map unique spasticity score from (Levin & Feldman, 2003) to MAS (assuming
% that individual jerk and clonus scores correlate 1:1 with eachother, as
% well as with total spasticity score)
MAS_LF = round((spast_LF - spast_LF/2)/2); % [0-4]

% fit lines (just average in the case of mu) to MAS vs. threshold data
fitsE(1,:) = polyfit(MAS_LF,static_LF,1);
fitsE(2,:) = [0 mean(dynamic_LF)];

% extract graphical data for MAS vs. stiffness/damping from (McCrea, 2003)
n = 17;   % number of observations
xMin = 0; % MAS score, where 1.5 = 1+
xMax = 4;
stiff_Mc = zeros(n,2);
damp_Mc = zeros(n,2);

filenames = {'stiffness.png','damping.png'};
for i = 1:length(filenames)
    
    % display plot (image) with correct axis limits
    file = filenames{i};
    img = imread(file);
    if ~isempty(strfind(file,'stiff'))
        yMin = 0;
        yMax = 15;
    else
        yMin = 0;
        yMax = 45;
    end
    figure()
    imagesc([xMin xMax], [yMin yMax], flipud(img));
    
    % get points from plot & fit line to data
    for j = 1:n
        point = ginput(1);
        MAS = round(point(1),1);
        if ~isempty(strfind(file,'stiff'))
            k = round(point(2),2)*10^-4;
            stiff_Mc(j,:) = [MAS k];
            fitsE(i+2,:) = polyfit(stiff_Mc(:,1),stiff_Mc(:,2),1);
        else
            b = round(point(2),2)*10^-5;
            damp_Mc(j,:) = [MAS b];
            fitsE(i+2,:) = polyfit(damp_Mc(:,1),damp_Mc(:,2),1);
        end
    end
end
