function [V,Vm,Vmap] = envCell(B,NFZ,res)
%ENVCELL Given the environment borders and No Fly Zones returns a
%  cellularized version of the environment with the desired resolution.
% Input:    B       - Environment Borders
%           NFZ     - No Fly Zones as list of borders {[...],[...]}
%           res     - Cell resolution
% Output:   V       - Cells vertices
%           Vm      - Movement (connected) vertices
%           Vmap    - Map of movement (connected) vertices

% Consider the whole rectangle containing the environment
sB = sort(B);
X = sB(1,1):res:sB(end,1);
Y = sB(1,2):res:sB(end,2);
comb = combvec(X,Y)';

% Check if inside NFZ
NFin = zeros(length(comb),1);
NFon = zeros(length(comb),1);
for i=1:size(NFZ,2)
    [in,on] = inpolygon(comb(:,1),comb(:,2),NFZ{i}(:,1),NFZ{i}(:,2));
    NFin = sign(NFin+in);
    NFon = sign(NFon+on);
end

% Check if inside Borders
[in,on] = inpolygon(comb(:,1),comb(:,2),B(:,1),B(:,2));

% Construct Vertices
V = [];         % vertices           
Vm = [];        % movement vertices
Vmap = [];      % movement vertices map

for i = 1:length(comb)
    if(in(i) && ~(NFin(i)-NFon(i)))
        V = [V; comb(i,:)];
        if(~on(i) && ~NFin(i))
            Vm = [Vm; comb(i,:)];
            Vmap = [Vmap; length(V)];
        end
    end
end

end

