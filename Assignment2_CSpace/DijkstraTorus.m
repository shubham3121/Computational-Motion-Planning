function route = DijkstraTorus (input_map, start_coords, dest_coords)
% Run Dijkstra's algorithm on a grid.
% Inputs : 
%   input_map : a logical array where the freespace cells are false or 0 and
%      the obstacles are true or 1
%   start_coords and dest_coords : Coordinates of the start and end cell
%       respectively, the first entry is the row and the second the column.
% Output :
%   route : An array containing the linear indices of the cells along the
%    shortest route from start to dest or an empty array if there is no
%    route.

% set up color map for display
% 1 - white - clear cell
% 2 - black - obstacle
% 3 - red = visited
% 4 - blue  - on list
% 5 - green - start
% 6 - yellow - destination

cmap = [1 1 1; ...
        0 0 0; ...
        1 0 0; ...
        0 0 1; ...
        0 1 0; ...
        1 1 0];

colormap(cmap);


[nrows, ncols] = size(input_map);

% map - a table that keeps track of the state of each grid cell
map = zeros(nrows,ncols);

map(~input_map) = 1;  % Mark free cells
map(input_map)  = 2;  % Mark obstacle cells

% Generate linear indices of start and dest nodes
start_node = sub2ind(size(map), start_coords(1), start_coords(2));
dest_node  = sub2ind(size(map), dest_coords(1),  dest_coords(2));

map(start_node) = 5;
map(dest_node)  = 6;

% Initialize distance array
distances = Inf(nrows,ncols);

% For each grid cell this array holds the index of its parent
parent = zeros(nrows,ncols);

distances(start_node) = 0;
% [X, Y] = meshgrid (1:ncols, 1:nrows);
% 
% xd = dest_coords(1);
% yd = dest_coords(2);

% Evaluate Heuristic function, H, for each grid cell
% Manhattan distance
% H = abs(X - xd) + abs(Y - yd);
% H = H';
% % Initialize cost arrays
% f = Inf(nrows,ncols);
% g = Inf(nrows,ncols);
% 
% g(start_node) = 0;
% f(start_node) = H(start_node);


% Main Loop
while true
    
    % Draw current map
    map(start_node) = 5;
    map(dest_node) = 6;
    
    image(1.5, 1.5, map);
    grid on;
    axis image;
    drawnow;
    
    % Find the node with the minimum distance
    [min_dist, current] = min(distances(:));
    %[min_f, current] = min(f(:));
    
    if ((current == dest_node) || isinf(min_dist))
        break;
    end;
    
    % Update map
    map(current) = 3;         % mark current node as visited
   distances(current) = Inf;% f(current) = Inf; % remove this node from further consideration
    
    % Compute row, column coordinates of current node
    [i, j] = ind2sub(size(distances), current);
    %[i, j] = ind2sub(size(f), current);
    
    % Visit each neighbor of the current node and update the map, distances
    % and parent tables appropriately.
   
    %%% All of your code should be between the two lines of stars. 
    % *******************************************************************
    neighbors = [i-1 j;i+1 j;i j-1;i j+1];
    if(i == 1 && j == 1)
        neighbors(1,:) = ''; neighbors(2,:) = '';
    elseif(i == 1 && j < ncols)
        neighbors(1,:) = [nrows j];
    elseif(j == 1 && i < nrows)
        neighbors(3,:) = [i ncols];
    elseif(i == nrows && j == 1)
        neighbors(2,:) = [1 j]; neighbors(3,:) = [i ncols];
    elseif(i == nrows && j < ncols)
        neighbors(2,:) = [1 j];
    elseif(i == 1 && j == ncols)
        neighbors(1,:) = [nrows j]; neighbors(4,:) = [i 1];
    elseif(i < nrows && j == ncols)
        neighbors(4,:) = '';
    elseif(i == nrows && j == ncols)
        neighbors(2,:) = [1 j]; neighbors(4,:) = [i 1];
    end
    neigh = sub2ind(size(map), neighbors(:,1), neighbors(:,2));
    for i=1:length(neigh)
        if(map(neigh(i))==1 || map(neigh(i))==4 || map(neigh(i))==6 && map(neigh(1))~=2)
            if(distances(neigh(i)) > min_dist + 1)
                distances(neigh(i)) = min_dist + 1;
                map(neigh(i)) = 4;
                parent(neigh(i)) = current;
            end
        end
    end  
%     for i=1:length(neigh)
%         if(map(neigh(i))==1 || map(neigh(i))==4 || map(neigh(i))==6 && map(neigh(1))~=2)
%             if(g(neigh(i)) > g(current) + 1)
%                 g(neigh(i)) = g(current) + 1;
%                 f(neigh(i)) = g(neigh(i)) + H(neigh(i));
%                 map(neigh(i)) = 4;
%                 parent(neigh(i)) = current;
%             end
%         end
%     end

    % *******************************************************************
end

if (isinf(distances(dest_node)))
    route = [];
else
    route = [dest_node];
    
    while (parent(route(1)) ~= 0)
        route = [parent(route(1)), route];
    end
end

    function update (i,j,d,p)
        if ( (map(i,j) ~= 2) && (map(i,j) ~= 3) && (map(i,j) ~= 5) && (distances(i,j) > d) )
            distances(i,j) = d;
            map(i,j) = 4;
            parent(i,j) = p;
        end
    end

end
