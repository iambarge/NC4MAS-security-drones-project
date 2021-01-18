function paths = splitCycles(env)
%SPLITCYCLES Get subpaths from cycles

    paths = zeros(size(env.cycles,1),1);
    N=0;
    for c = env.cycles(:,1:end)
          skip = false;
          N = N + 1;
          e = find(c);
          E = zeros(1,length(e));
          v = zeros(1,length(e)+1);
          for j = 1:length(e)
              if j == 1
                E(j) = e(j);
                v(j) = env.Em(e(j),1);
                v(j+1) = env.Em(e(j),2);
              else
                A1 = env.Em(e,1) == v(j);
                A2 = env.Em(e,2) == v(j);
                A = logical(A1+A2);
                if sum(A) > 2
                    skip = true;
                    break
                end
                B = [env.Em(e(A),1); env.Em(e(A),2)];
                C1 = B ~= v(j);
                C2 = B ~= v(j-1);
                C = logical(C1.*C2);
                v(j+1) = B(C);
                D1 = env.Em(e(A),1) == v(j+1);
                D2 = env.Em(e(A),2) == v(j+1);
                D = logical(D1+D2);
                A = find(A);
                E(j) = e(A(D));
              end
          end
          
          if ~skip
          for j = 1:length(E)
              path1 = zeros(size(env.cycles,1),1);
              path1(E(j)) = 1;
              path2 = c - path1;
              if (sum(ismember(paths',path1','rows'))==0)  % if NOT PRESENT
                  paths = [paths path1];
              end
              if (sum(ismember(paths',path2','rows'))==0)  % if NOT PRESENT
                  paths = [paths path2];
              end
              
              path = path2;
              for k = 1:sum(c)/2-1
                  k1 = k+j;
                  if k1 > length(E)
                      k1 = k1-length(E); 
                  end
                  path21 = path;
                  path21(E(k1)) = 0;
                  path22 = c - path21;
                  if (sum(ismember(paths',path21','rows'))==0)  % if NOT PRESENT
                     paths = [paths path21];
                  end
                  if (sum(ismember(paths',path22','rows'))==0)  % if NOT PRESENT
                     paths = [paths path22];
                  end
                  path = path21;              
              end
          end
          end
    end
    paths(:,1) = [];    % remove the empty path
end

