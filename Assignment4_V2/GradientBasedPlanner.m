function route = GradientBasedPlanner (f, start_coords, end_coords, max_its)
% GradientBasedPlanner : This function plans a path through a 2D
% environment from a start to a destination based on the gradient of the
% function f which is passed in as a 2D array. The two arguments
% start_coords and end_coords denote the coordinates of the start and end
% positions respectively in the array while max_its indicates an upper
% bound on the number of iterations that the system can use before giving
% up.
% The output, route, is an array with 2 columns and n rows where the rows
% correspond to the coordinates of the robot as it moves along the route.
% The first column corresponds to the x coordinate and the second to the y coordinate

[gx, gy] = gradient (-f);

%%% All of your code should be between the two lines of stars.
% *******************************************************************
route = start_coords;
P1 = start_coords;
while(max_its > 0)
     if(norm(end_coords - P1) < 2)
         break;
     end
    delta = [gx(floor(P1(2)), floor(P1(1))), ...
            gy(floor(P1(2)), floor(P1(1)))];
    mag = delta/norm(delta);
    P1 = P1 + [mag(1) mag(2)];
    route = [route; P1];
    max_its = max_its - 1;
end
% *******************************************************************
end
