% look at testing this in ros
clear
close all
bag = rosbag('../../../googleDrive/WallFollow/2019-06-18-13-16-07.bag')

% load('/home/klyu/lab/WallFollow/gtspCode/output/bridge51.mat')
% load('/home/klyu/lab/WallFollow/gtspCode/output/bridge53.mat')
load('/home/user01/Kevin_Yu/WallFollow/gtspCode/output/bridge51.mat')
load('/home/user01/Kevin_Yu/WallFollow/gtspCode/output/bridge53.mat')


bSel = select(bag,'Topic','/mavros/global_position/local');
msgStructs = readMessages(bSel,'DataFormat','struct');
msgStructs{[1,1]};

endTime = 27500;

xPoints = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs);
zPoints = cellfun(@(m) double(m.Pose.Pose.Position.Z),msgStructs);

xPoints = xPoints(1:endTime);
zPoints = zPoints(1:endTime);

% figure(3)
% plot(xPoints,zPoints)
% axis([-20 200 -2 30])
% title('UAV Flight')
% xlabel('X Position')
% ylabel('Altitude')
% hold on
%
% data = readData('/home/klyu/lab/WallFollow/gtspCode/input/bridge5Outline.txt');
% plot(data(:,1),data(:,2))
% hold off

tour = [22, 21, 20, 19, 7, 17, 5, 15, 14, 13, 1, 23];
S = [22,21,20,19,7,17,5,15,14,13];
T = [21, 20, 19, 7, 17, 5, 15, 14, 13, 1];

    SInit = [];
    TInit = [];
for i = 1:numPointsInit
    for j = 1:numPointsInit
        temp = gtspMatrix(i,j);
        if temp ~= -1 && i == 2
            SInit = [SInit, i];
            TInit = [TInit, j];
        end
    end
end
SInit
TInit

% graphingClusterOrdered(x, y, numPointsInit, numBatteryLevels, S, T, v_Cluster, tour)
graphingClusterOrdered(x, y, numPointsInit, numBatteryLevels, SInit, TInit, v_Cluster, tour)

