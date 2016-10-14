function [ occupied ] = ray_tracing( x,y,theta,map,scanAngles,origin,resol )
i_ocx = zeros(size(scanAngles,1),1);
i_ocy = zeros(size(scanAngles,1),1);
for k = 1:size(scanAngles,1)
    deg = mod((theta + scanAngles(k))*180/pi,360);
    slope = tan((theta + scanAngles(k)));
    x0 = max(ceil(resol*x) + origin(1),1);
    y0 = max(ceil(resol*y) + origin(2),1);
    x0 = min(x0,size(map,2));
    y0 = min(y0,size(map,1));
    if(deg>=  - 45 && deg<45)
        x1 = size(map,2) - origin(1);
        y1 = y0 + (x1 - x0)*slope;
    elseif(deg>= 45 && deg<135)
        y1 = size(map,1) - origin(2);
        x1 = x0 + (y1 - y0)*slope;
    elseif(deg>= 135 && deg<225)
        x1 = 1;
        y1 = y0 + (x1 - x0)*slope;
    else
        y1 = 1;
        x1 = x0 + (y1 - y0)*slope;
    end
    dx = abs(x1 - x0);
    dy = abs(y1 - y0);
    ini_x = x0;
    ini_y = y0;
    if(x1>x0)
        x_inc = 1;
    else
        x_inc =  - 1;
    end
    if(y1>y0)
        y_inc = 1;
    else
        y_inc =  - 1;
    end
    n = 1 + dx + dy;
    error = dx - dy;
    dx = dx*2;
    dy = dy*2;
    
    cond=(n>0 && map(ini_y,ini_x)<0.5);
    while(cond)
        if (error > 0)
            ini_x = ini_x  +  x_inc;
            error = error  -  dy;
        else
            ini_y = ini_y  +  y_inc;
            error = error  +  dx;
        end
        n = n - 1;
        ini_x = max(ini_x,1);
        ini_y = max(ini_y,1);
        ini_x = min(ini_x,size(map,2));
        ini_y = min(ini_y,size(map,1));
        cond = (n>0 && map(ini_y,ini_x)<0.5);
    end
    i_ocx(k) = ini_x;
    i_ocy(k) = ini_y;
end
occupied=[i_ocx,i_ocy];
end

