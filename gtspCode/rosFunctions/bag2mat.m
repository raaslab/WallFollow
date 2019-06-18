
% look at testing this in ros
% clear
close all
% bag = rosbag('2019-06-17-12-44-21.bag')
rosbag info '2019-06-17-12-44-21.bag';

bSel = select(bag,'Topic','/mavros/global_position/local');
msgStructs = readMessages(bSel,'DataFormat','struct');
msgStructs{[1,1]};
% ￼
% ￼

xPoints = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs);
zPoints = cellfun(@(m) double(m.Pose.Pose.Position.Z),msgStructs);

xPoints = xPoints(1:27000);
zPoints = zPoints(1:27000);

figure(3)
plot(xPoints,zPoints)
axis([-20 200 -2 30])
title('UAV Flight')
xlabel('X Position')
ylabel('Altitude')
hold on

data = readData('/home/klyu/lab/WallFollow/gtspCode/input/bridge5Outline.txt');
plot(data(:,1),data(:,2))
hold off


function [data] = readData(file)

fileID = fopen(file,'r');
% formatSpec = '%f';
formatSpec = '%f %f';
sizeData = [2 Inf];
data = fscanf(fileID,formatSpec,sizeData);
data = data';
fclose(fileID);
end
