% This Function created the k'th derivative of the polynomial of degree n.
function [T] = polyT(n,k,t)
     
        T = zeros(n,1);
        D = zeros(n,1);
        
        % Initial
        for i=1:n
            D(i)=i-1;
            T(i)=1;
        end
        
        %Derivative
        for  j= 1:k
            for i= 1:n
                T(i)=T(i)*D(i);
                if D(i)>0
                    D(i)=D(i)-1;
                end
            end
        end
        
        %put t value
        for i=1:n
            T(i)=T(i)*t^D(i);
        end
        T=T';
end