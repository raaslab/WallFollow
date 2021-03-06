% specificRuns
% this will run a specific case for if there's an error within "time"
% variable
clc; clear all; close all;

% numBC = 20;

max_Distance = 2000;   % if max_Distance == j then discharge is unit rate per distance (budget)
j = 1;				% battery levels
tTO = 5;           % take off cost
tL = 45;            % landing cost
rRate = 2;         % rate of recharge
UGVS = 1;          % time to travel one unit for the UGV (greater than 1 means UGV is slower)
method = 1;        % 1 = GLNS, 0 = concorde
timeI = [];
% changing number of input BC

% filename4 = sprintf('inputs/timeVSi/%d_%d.txt',trial,numBC);
% polygonCreater(filename4,numBC,100,0,0) % creates random polygons.

% use 'testInput.txt' if you want the file from polygonCreater
data = readData('/home/klyu/lab/WallFollow/gtspCode/input/bridge60.txt'); % get the size and shape from the data (this will tell you number of clusters points and so on)
[numClusters, ~] = size(data);
x = [data(:,1), data(:,4)];
y = [data(:,2), data(:,5)];
UGVCapable = [data(:,3), data(:,6)];

G = 0;
i = numClusters*2; % number of vertices needed to be multiplied by battery levels


filename1 = sprintf('bridge61');
filename2 = sprintf('/home/klyu/lab/WallFollow/gtspCode/output/bridge62.gtsp');
filename3 = sprintf('bridge63');

tic
pathName = '/home/klyu/lab/WallFollow/gtspCode/output';
% pathName = '/home/klyu/lab/coverageWork/testForCoverage/errorInstance'; % for error instances
[ansTime,gtspMatrix,gtspTime, v_Cluster] = testGeneral(i, j, filename1, tTO, tL, rRate, UGVS, G, x, y, method, max_Distance, pathName,UGVCapable);

% making GLNS matrix input
roundedGtspMatrix = round(gtspMatrix);
roundedGtspMatrix(roundedGtspMatrix == -1) = 999999;
roundedGtspMatrix(roundedGtspMatrix == Inf) = 999999;
createGTSPFile(filename2,roundedGtspMatrix, i, j, v_Cluster) % creating GLNS file
f = fullfile(pathName, filename3);
trialTime = toc
save(f);



