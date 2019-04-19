% typeA
% creates fly, fly edge

function [edges] = typeA(v_Cluster, distances, levels, sites, clusterLevels, maxDistance, groupedPoints)

uniqueClusters = max(unique(v_Cluster));
edges = zeros(sites);
for i = 1:sites
    if i > uniqueClusters
        tempFind = (i - uniqueClusters);
        sameCluster = find(v_Cluster == tempFind);
        point2 = sameCluster(levels)/levels;
        point1 = sameCluster(end)/levels;
        costTemp = distances(point1, point2);
        for j = 1:sites
            if point1 == j || point2 == j
                edges(point1, j) = Inf;
            else
                edges(point1, j) = costTemp + distances(point2, j);
            end
        end
    else
        tempFind = i;
        sameCluster = find(v_Cluster == tempFind);
        point2 = sameCluster(levels)/levels;
        point1 = sameCluster(end)/levels;
        costTemp = distances(point1, point2);
        for j = 1:sites
            if point1 == j || point2 == j
                edges(point2, j) = Inf;
            else
                edges(point2, j) = costTemp + distances(point1, j);
            end
        end
    end
end


end
