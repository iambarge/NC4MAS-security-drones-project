function [Em,Am,A] = envEdges(V,Vm,Vmap,res)
%ENVEDGES Build edges of the graph

% Construct Edges
Em =[];         % movement edges
Ev =[];         % vision edges
    
for i = 1:size(Vm,1)
    % MOVEMENT
    index = find(ismembertol(Vm,[Vm(i,1)+res Vm(i,2)],'ByRows',true));
    if(sum(index) > 0)        
        Em = [Em; i index];
    end
    index = find(ismembertol(Vm,[Vm(i,1) Vm(i,2)+res],'ByRows',true));
    if(sum(index) > 0)
        Em = [Em; i index];
    end
        
    % VISION
    index = find(ismembertol(V,[Vm(i,1)-res Vm(i,2)],'ByRows',true));
    if(sum(index) > 0)
        Ev = [Ev; Vmap(i) index];
    end
    index = find(ismembertol(V,[Vm(i,1)+res Vm(i,2)],'ByRows',true));
    if(sum(index) > 0)
        Ev = [Ev; Vmap(i) index];
    end
    index = find(ismembertol(V,[Vm(i,1) Vm(i,2)-res],'ByRows',true));
    if(sum(index) > 0)
        Ev = [Ev; Vmap(i) index];
    end
    index = find(ismembertol(V,[Vm(i,1) Vm(i,2)+res],'ByRows',true));
    if(sum(index) > 0)
        Ev = [Ev; Vmap(i) index];
    end
    index = find(ismembertol(V,[Vm(i,1)-res Vm(i,2)-res],'ByRows',true));
    if(sum(index) > 0)
        Ev = [Ev; Vmap(i) index];
    end
    index = find(ismembertol(V,[Vm(i,1)-res Vm(i,2)+res],'ByRows',true));
    if(sum(index) > 0)
        Ev = [Ev; Vmap(i) index];
    end
    index = find(ismembertol(V,[Vm(i,1)+res Vm(i,2)+res],'ByRows',true));
    if(sum(index) > 0)
        Ev = [Ev; Vmap(i) index];
    end
    index = find(ismembertol(V,[Vm(i,1)+1 Vm(i,2)-1],'ByRows',true));
    if(sum(index) > 0)
        Ev = [Ev; Vmap(i) index];
    end
end

% Get adjacency matrices
 G = graph(Ev(:,1),Ev(:,2));
 Gm = graph(Em(:,1),Em(:,2));
 
 A = adjacency(G);
 Am = adjacency(Gm);

end

