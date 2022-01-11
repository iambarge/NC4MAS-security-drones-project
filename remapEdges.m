% Copyright (c) 2020, Nicolò Bargellesi, Luca Facin & Lorenzo Marchini
%
% This source code is licensed under the MIT-style license found in the
% LICENSE file in the root directory of this source tree.
%

function E = remapEdges(E,Vmap)
%REMAPEDGES Returns the set of edges associated to the connected components
%Input:     E   - Set of edges
%           Vm  - Map of connected vertices
%Output:    Em  - Set of remapped edges

% Edges to Movement
% Em = E;
% for i=1:length(E)
%     Em(i) = [find(ismember(Vmap,E(i,1))) find(ismember(Vmap,E(i,2)))];
% end

% Movement 2 Edges
Em = [Vmap(E(:,1)), Vmap(E(:,2))];

E = Em;

end

