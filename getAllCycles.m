function [cycles] = getAllCycles(E,V)
%GETALLCYCLES Return a matrix whos columns correspond to all possible
%  simple cycles given a set of edges from a CONNECTED graph. 
% Requires the Graph Theory Toolbox (grTheory) and the Deep Learning Toolbox.
% Input:     E       - Set of edges as a matrix with rows [vi vj]
%            V       - Set of vertices
% Output:    Cycles  - Matrix of all possible cycles

Cycles_Basis = grCycleBasis(E);   % all independent cycles
cycles = Cycles_Basis;            % all cycles

for i=1:size(Cycles_Basis,2)-1
    C = cycles;
    comb = combvec(Cycles_Basis,C);
    C = xor(comb(1:length(E),:),comb(length(E)+1:end,:));
    index = find(sum(C));           % indexes of non zero columns
    
    for k=index(1:end)  
        % Check if the combination is a cycle
        eindex = logical(C(:,k));
        G = graph(E(eindex,1),E(eindex,2),[],size(V,1));
        [~,compsize] = conncomp(G);
        compsize = sort(compsize);
        if(length(compsize)==1)
            if(sum(ismember(cycles',C(:,k)','rows'))==0)  % if NOT PRESENT
                cycles = [cycles C(:,k)];                 % Add
            end
        else
            if(compsize(end-1)==1 && sum(ismember(cycles',C(:,k)','rows'))==0)  % if CYCLE AND NOT PRESENT
                cycles = [cycles C(:,k)];                                       % Add
            end
        end
    end
end

end

