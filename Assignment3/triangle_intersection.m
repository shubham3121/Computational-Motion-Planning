function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************
flag1= false; flag2 = false;
A = threed(P1(1,:)); B = threed(P1(2,:)); C = threed(P1(3,:));
% Checking P2 in triangle P1
for i =1:length(P2)
    P = threed(P2(i,:));
    if(point_in_triangle(P, A, B, C))
        flag1  = true;
        break;
    end
end
% Checking P2 in triangle P1

A = threed(P2(1,:)); B = threed(P2(2,:)); C = threed(P2(3,:));
 
for i =1:length(P1)
    P = threed(P1(i,:));
    if(point_in_triangle(P, A, B, C))
        flag2  = true;
        break;
    end
end

flag = flag1 || flag2;
% *******************************************************************
end

% function status = same_side(p,a,b,c)
% status  = false;
% cp1 = cross(b-a, p-a);
% cp2 = cross(b-a, c-a);
% if(dot(cp1,cp2) >= 0)
%     status  = true;
% end
% end

function status = point_in_triangle(p,a,b,c)
% status  = true if point in the triangle and false if point outside of the
% triangle

v0 = c - a;
v1 = b - a;
v2 = p - a;
dot00 = dot(v0,v0);
dot01 = dot(v0,v1);
dot02 = dot(v0,v2);
dot11 = dot(v1,v1);
dot12 = dot(v1,v2);

invDenom = 1/(dot00*dot11 - dot01*dot01);
u = (dot11*dot02 - dot01*dot12)*invDenom;
v = (dot00*dot12 - dot01*dot02)*invDenom;

status = (u>=0)&&(v>=0)&&(u+v)<=1;
end

function coord = threed(P)
% convert a 2D corrdinate to a 3D coordinate
coord = [P 0];
end