function [ contact ] = triTriContact( t1, t2 )
% detect collision between two triangles
%   translated from the UNC-CH V-Collide RAPID code
% Triangles are of the form
% t1 = [p11 p12 p13; p21 p22; p23; p31 p32 p33];



% shift t1 and t2 by p1#

p = t1 - [t1(1,:); t1(1,:); t1(1,:)];
q = t2 - [t1(1,:); t1(1,:); t1(1,:)];

% compute triangle edges
e = [p(2,:) - p(1,:); 
      p(3,:) - p(2,:);
      p(1,:) - p(3,:)];

f = [q(2,:) - q(1,:); 
     q(3,:) - q(2,:);
     q(1,:) - q(3,:)];

 % compute normals
 n = cross( e(1,:), e(2,:) );
 m = cross( f(1,:), f(2,:) );
 
 g = [cross( e(1,:), n);
      cross( e(2,:), n);
      cross( e(3,:), n)];
      
 h = [cross( f(1,:), m);
      cross( f(2,:), m);
      cross( f(3,:), m)];
 
  e1f = [cross(e(1,:), f(1,:));
         cross(e(1,:), f(2,:));
         cross(e(1,:), f(3,:))];
     
  e2f = [cross(e(2,:), f(1,:));
         cross(e(2,:), f(2,:));
         cross(e(2,:), f(3,:))];
 
  e3f = [cross(e(3,:), f(1,:));
         cross(e(3,:), f(2,:));
         cross(e(3,:), f(3,:))];
  
  % now begin the series of tests

  contact = 0;
  if (~project6(n, p(1,:), p(2,:), p(3,:), q(1,:), q(2,:), q(3,:))) return; end;
  if (~project6(m, p(1,:), p(2,:), p(3,:), q(1,:), q(2,:), q(3,:))) return; end;
  
  if (~project6(e1f(1,:), p(1,:), p(2,:), p(3,:), q(1,:), q(2,:), q(3,:))) return; end;
  if (~project6(e1f(2,:), p(1,:), p(2,:), p(3,:), q(1,:), q(2,:), q(3,:))) return; end;
  if (~project6(e1f(3,:), p(1,:), p(2,:), p(3,:), q(1,:), q(2,:), q(3,:))) return; end;

  if (~project6(e2f(1,:), p(1,:), p(2,:), p(3,:), q(1,:), q(2,:), q(3,:))) return; end;
  if (~project6(e2f(2,:), p(1,:), p(2,:), p(3,:), q(1,:), q(2,:), q(3,:))) return; end;
  if (~project6(e2f(3,:), p(1,:), p(2,:), p(3,:), q(1,:), q(2,:), q(3,:))) return; end;
  if (~project6(e3f(1,:), p(1,:), p(2,:), p(3,:), q(1,:), q(2,:), q(3,:))) return; end;
  if (~project6(e3f(2,:), p(1,:), p(2,:), p(3,:), q(1,:), q(2,:), q(3,:))) return; end;
  if (~project6(e3f(3,:), p(1,:), p(2,:), p(3,:), q(1,:), q(2,:), q(3,:))) return; end;

  if (~project6(g(1,:), p(1,:), p(2,:), p(3,:), q(1,:), q(2,:), q(3,:))) return; end;
  if (~project6(g(2,:), p(1,:), p(2,:), p(3,:), q(1,:), q(2,:), q(3,:))) return; end;
  if (~project6(g(3,:), p(1,:), p(2,:), p(3,:), q(1,:), q(2,:), q(3,:))) return; end;

  if (~project6(h(1,:), p(1,:), p(2,:), p(3,:), q(1,:), q(2,:), q(3,:))) return; end;
  if (~project6(h(2,:), p(1,:), p(2,:), p(3,:), q(1,:), q(2,:), q(3,:))) return; end;
  if (~project6(h(3,:), p(1,:), p(2,:), p(3,:), q(1,:), q(2,:), q(3,:))) return; end;

  contact = 1;
  return;

end

function [ project ] = project6( ax, p1, p2, p3, q1 ,q2, q3 )

P1 = dot( ax, p1 );
P2 = dot( ax, p2 );
P3 = dot( ax, p3 );

Q1 = dot( ax, q1 );
Q2 = dot( ax, q2 );
Q3 = dot( ax, q3 );

mx1 = max( [P1, P2, P3] );
mn1 = min( [P1, P2, P3] );

mx2 = max( [Q1, Q2, Q3] );
mn2 = min( [Q1, Q2, Q3] );

project = 0;
if ( mn1 > mx2 ) return; end;
if ( mn2 > mx1 ) return; end;

project = 1;
return;
end
