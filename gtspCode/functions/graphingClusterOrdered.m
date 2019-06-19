% graphingClusterOrdered
% this creates the cluster graph of the tour, but plots it in order of what
% ever cluster is next
% INPUTS
% x = x corrdinates for GTSP points
% y = y corrdinates corresponding to "x"
% numPoints = number of initial points in the graph
% numLevels = number of initial level given for the graph
% s = starting node for an edge
% t = corresponding ending node for an edge with "s"
% v_Cluster = tells you points corresponding to clusters
% tour = this is the final tour from concorde
% OUTPUTS
% this function just plots the cluster so no output is needed


function [] = graphingClusterOrdered(x, y, numPoints, numLevels, s, t, v_Cluster, tour)

xOut = [];
yOut = [];

G = digraph;
for i = 1:numPoints
	if i<((numPoints/2)+1)
    	xOut(end+1) = i;
    	yOut(end+1) = 1;
    else
    	xOut(end+1) = i - (numPoints/2);
    	yOut(end+1) = 2;
    end
end
%-------------------------------------------------------------------------%
% need to make id variable
id = [];
numOfTour = numel(tour);
newV_Cluster = cell2mat(v_Cluster);
for i = 1:numOfTour-1
    locationOfNodes = find(newV_Cluster == newV_Cluster(tour(i)));
    numOfLocationOfNodes = numel(locationOfNodes);
    if tour(i) == locationOfNodes(1)
        for j = 1:numOfLocationOfNodes
            id(end+1) = locationOfNodes(j);
        end
    else
        for j = numOfLocationOfNodes:-1:1
            id(end+1) = locationOfNodes(j);
        end
    end
end
id(end+1) = numPoints+1
idNew = [];
for i = 1:numPoints-1
    idNew(end+1) = i;
end
%-------------------------------------------------------------------------%
[G, xOut, yOut] = graphMakingWPoints(xOut, yOut, G, idNew);
[G] = createEdges(G, s, t);

figure(4)                                                            % plots graph
plot(G, 'XData', xOut, 'YData', yOut, 'LineStyle', '-.', 'LineWidth', 4, 'MarkerSize', 6);

yRec = 0.5;
wRec = 1;
hRec = numLevels+1;
for i = 1:numPoints/2                                                 % clusters the points
    xRec = i-(wRec/2);                                              % parameters for creating clusters
    pos = [xRec, yRec, wRec, hRec];                                 % creates oval, but needs to be fixed
    rectangle('Position', pos, 'Curvature', [1 1])
    xStr = num2str(i);
    str = sprintf('%s', xStr);
    text(xRec+(wRec/4), yRec+hRec/2, str, 'Color', 'red', 'Fontsize', 14);
end

title('GTSP Output')


end

