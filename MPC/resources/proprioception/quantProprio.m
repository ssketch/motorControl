clear all
close all
clc

%% Display Experimental Data

img = imread('control_Yousif.png');

xMin = -15.5; % cm
xMax =  15.5;
yMin = -14;
yMax =  14;

figure()
imagesc([xMin xMax], [yMin yMax], img);

%% Get Task-Space Noise Vectors

toShoulder = [-5;50]; % replace with actual value when hear back from N. Yousif

numTarg = input('Specify the number of targets: ');
disp(' ');
targets = zeros(2,numTarg);
centers = zeros(2,numTarg);
majAxes = zeros(2,numTarg);

for i = 1:numTarg
    disp(['Pick TARGET point #',num2str(i),'...']);
    targets(:,i) = ginput(1)' + toShoulder;
    
    disp(' ');
    disp(['Pick CENTER of distribution #',num2str(i),'...']);
    centers(:,i) = ginput(1)' + toShoulder;
    
    disp(' ');
    disp(['Specify MAJOR axis #',num2str(i),'...']);
    majAxes(:,i) = ginput(1)' + toShoulder;
    disp(' ');
end

%% Transform Task-Space to Joint-Space

% specify subject anatomy (assuming average anthropometry)
H = 170; % cm
L1 = 0.188*H;
L2 = 0.253*H;

Th_targs = zeros(2,numTarg);
dTh_bias = zeros(2,numTarg);
dTh_nois = zeros(2,numTarg);

for i = 1:numTarg
    [Th_targs(:,i),dTh_bias(:,i),dTh_nois(:,i)] = ...
        tsErr2jsErr(targets(:,i),centers(:,i),majAxes(:,i),L1,L2);
end

%% Plot Bias & Noise as Function of Joint Angles

% split data for shoulder and elbow joints
shouldData = [Th_targs(1,:)' dTh_bias(1,:)' abs(dTh_nois(1,:))'];
shouldData = sortrows(shouldData,1);
[mShould,bShould] = fit_data(shouldData(:,1),shouldData(:,2));
fitShould = mShould * shouldData(:,1) + bShould; % for bias
avgShould = mean(shouldData(:,3));               % for noise

elbowData = [Th_targs(2,:)' dTh_bias(2,:)' abs(dTh_nois(2,:))'];
elbowData = sortrows(elbowData,1);
[mElbow,bElbow] = fit_data(elbowData(:,1),elbowData(:,2));
fitElbow = mElbow * elbowData(:,1) + bElbow;
avgElbow = mean(elbowData(:,3));

% first, for the shoulder
figure()
subplot(1,2,1); plot(shouldData(:,1),shouldData(:,2),'bo-','LineWidth',3); xlabel('Shoulder Angle'); ylabel('Bias'); grid on;
hold on; plot(shouldData(:,1),fitShould,'k--','LineWidth',5); hold off;
subplot(1,2,2); plot(shouldData(:,1),shouldData(:,3),'bx','LineWidth',3,'MarkerSize',12); xlabel('Shoulder Angle'); ylabel('Noise_m_a_x'); grid on;
hold on; plot(shouldData(:,1),avgShould*ones(numTarg),'k--','LineWidth',5); xlim([min(shouldData(:,1)) max(shouldData(:,1))]); hold off;

% second, for the elbow
figure()
subplot(1,2,1); plot(elbowData(:,1),elbowData(:,2),'ro-','LineWidth',3); xlabel('Elbow Angle'); ylabel('Bias'); grid on;
hold on; plot(elbowData(:,1),fitElbow,'k--','LineWidth',5); hold off;
subplot(1,2,2); plot(elbowData(:,1),elbowData(:,3),'rx','LineWidth',3,'MarkerSize',12); xlabel('Elbow Angle'); ylabel('Noise_m_a_x'); grid on;
hold on; plot(elbowData(:,1),avgElbow*ones(numTarg),'k--','LineWidth',5); xlim([min(elbowData(:,1)) max(elbowData(:,1))]); hold off;
