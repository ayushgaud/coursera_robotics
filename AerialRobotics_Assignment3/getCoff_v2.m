    
  %{   
   For making Ax=B
   
   A is the result matrix of the equation [1 t t^2 t^3....][a0 a1 a2
   a4....] at various points for which B matrix is the result that we know
   B = [a b b c 0 0....]
   A*Coefficients=B
   1. 2n equations for position constraints for each way point.
   2. 6 equations for vel, acceleration and jerk = 0 for initial and final
   point.
   3. 7(n-1) equations for all the medieval points for all the derivatives.(The
   continuity equations.)
   
   Assumptions
   -----------
   1. All the way points are equidistant.
   2. We approximately know the time taken to go between two points.
   We take that as 1sec for every piece.
   
   If we are to use variable time then we have to normalise it between 0
   and 1.And have to remember that data.
    

waypoints =[

     1     2     3     5     6;
     1     3     2     5     4;
     1     1     1     1     1;];

%}
        
function[coeff]=getCoff_v2(waypoints)
   
   
   if(nargin==0)
   %waypoints =[0 1 2 3 4;0 1 0 -1 0;0 1 2 1 0];
   %waypoints =[0 1 2 3 4;6 8 10 12 14;5 8 11 14 17]
   %waypoints =[0 1 2 3 4;6 8 10 12 14;0 0 0 0 0]
   end
   
   n=size(waypoints,2)-1;
   A=zeros(8*n,8*n);
 
   % Making B matrix 
   b=zeros(3,8*n);
   for i=1:n
       b(:,i)=waypoints(:,i);
       b(:,i+n)=waypoints(:,i+1);
   end
   
   row=1;
   
   % For Position constraint at t=0 for each path 
   % Here first instance is a0
   for i=1:n
       A(row,((i-1)*8)+1:i*8)=polyT(8,0,0);
       row=row+1;
   end
   
   % For Position constraint at t=1 for each path 
   for i=1:n
       A(row,((i-1)*8)+1:i*8)=polyT(8,0,1);
       row=row+1;
   end
   
   % For Velocity, Acceleration And Jerk constraint at first and last
   % points for first point at t=0 and for last point t=1
   i=1;
   for k=1:3
       %A(row,((i-1)*8)+1:i*8)=polyT(8,k,0);
       A(row,((i-1)*8)+1:i*8)=polyT(8,k,0)/max(polyT(8,k,0));
       row=row+1;
   end
   i=n;
   for k=1:3
       A(row,((i-1)*8)+1:i*8)=polyT(8,k,1);
       row=row+1;
   end
   
   % For continuity, at every medieval points
   for i=1:(n-1)
       for k=1:6           
           A(row,((i-1)*8)+1:((i-1)*8+16))=horzcat(polyT(8,k,1),-polyT(8,k,0)); 
%            A(row,((i-1)*8)+1:((i-1)*8+8))= polyT(8,k,1),-polyT(8,k,0); 
%            A(row,((i-1)*8)+9:((i-1)*8+16))= 0; 
            row=row+1;
       end
   end

   coeff=pinv(A)*b';
   
   %plotTrajectory(coff,waypoints);
end


%{
syms t;
eqn = [1 t t^2 t^3 t^4 t^5 t^6 t^7]
n=(size(waypoints,2)-1);
hold on;
grid on;

%plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'*')
for i=1:n   
    hold on
    coe = coeff((i-1)*8+1:i*8,:);
    
    ex = vpa(eqn*coe(:,1));
    ey = vpa(eqn*coe(:,2));
    ez = vpa(eqn*coe(:,3));
    x=subs(ex,t,0:0.01:1);
    y=subs(ey,t,0:0.01:1);
    z=subs(ez,t,0:0.01:1);
    h=plot3(x,y,z,'-')
   
    hold on;
    if(i==1)
        set (h,'Color','blue');
    end
    if(i==2)
        set (h,'Color','yellow');
    end
    if(i==3)
        set (h,'Color','green');
    end
    if(i==4)
        set (h,'Color','red');
    end
   
    hold on;    
end
hold off;

for i=1:4
    temp = coeff((i-1)*8+1:i*8,:);

end
%}
