% readsDataImpossible
% reads data for impossible edges

function [data] = readDataImpossible(file)

fileID = fopen(file,'r');
% formatSpec = '%f';
formatSpec = '%f %f %f %f %f %f';
sizeData = [6 Inf];
data = fscanf(fileID,formatSpec,sizeData);
data = data';
fclose(fileID);
end
