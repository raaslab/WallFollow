% removeImpossibleEdges
% removes edges from matrix edges, which is a cost matrix
% INPUT
% edges: cost matrix of graph (nXn)
% removeEdges: matrix of edges to remove (nX2)
% OUTPUT
% newEdges: new cost matrix with edges removed


function [newEdges] = removeImpossibleEdges(edges,removeEdges)

s = removeEdges(:,1);
t = removeEdges(:,2);
newEdges = edges;

sizeOfEdges = size(edges);
for i = 1:sizeOfEdges(1)
    locationOfi = find(s==i);
    for j = 1:sizeOfEdges(2)
        remove = 0;
        for k = 1:length(locationOfi)
            if t(locationOfi(k)) == j
                remove = 1;
            end
        end
        if remove == 1
            newEdges(i,j) = Inf;
        end
    end
end






end