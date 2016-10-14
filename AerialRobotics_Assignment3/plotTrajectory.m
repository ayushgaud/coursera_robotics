%{
This function plots the trajectory of the waypoints given the coefficients
of each segment, for each axes.
%}

function [] = plotTrajectory(coeff,waypoints)

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
    h=plot3(x,y,z,'-');
   
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
end
