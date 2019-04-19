% flying
% this will make a v_Adj with only UAV flights on it
% INPUTS

% OUTPUTS


function [v_AdjNew, distances] = flying(maxDistance, x, y, numPoints, numLevels, v_Cluster, v_ClusterLevel, groupedPoints)

[v_AdjNew, distances] =  makingSTWv_Adj(maxDistance, x, y, numPoints, numLevels, v_Cluster, groupedPoints);
for i = 1:numPoints
    for j = 1:numPoints
        if i==j
            v_AdjNew(i,j) = Inf;
        end
    end
end

end