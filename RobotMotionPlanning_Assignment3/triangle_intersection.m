function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************
P1=[P1,[0;0;0]];
P2=[P2,[0;0;0]];
if(triTriContact(P1,P2))
    flag = true;
    return
end
flag=false;
% *******************************************************************
end