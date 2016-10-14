function [ output ] = getcoeff( waypoints )
%%
%
n=length(waypoints)-1;
coeff=zeros(8*n,8*n);
B=zeros(3,8*n);
%T=0 and T=1 pos eqn
for i=1:n
    coeff(i,(i-1)*8+1:8*i)=diffcoeff(7,0,0);
    coeff(i+n,(i-1)*8+1:8*i)=diffcoeff(7,0,1);
    B(:,i)=waypoints(:,i);
    B(:,i+n)=waypoints(:,i+1);
end
%%
%1st to 6th order diff eqns of continuity
% for j=2:7
%     for i=j*n+2:(j+1)*n-1
%         %coeff(i,(i-j*n-1)*8+1:8*(i-j*n)+8)=horzcat(diffcoeff(7,j-1,1),-diffcoeff(7,j-1,0));
%         coeff(i,(mod(i-1,n))*8+1:8*(mod(i-1,n)+1)+8)=horzcat(diffcoeff(7,j-1,1),-diffcoeff(7,j-1,0));
%     end
% end
index=2*n+1;

i=1;
for k=1:3
   coeff(index,((i-1)*8)+1:i*8)=diffcoeff(7,k,0);
   index=index+1;
end
i=n;
for k=1:3
   coeff(index,((i-1)*8)+1:i*8)=diffcoeff(7,k,1);
   index=index+1;
end
   
for i=1:n-1
    for j=1:6
        coeff(index,((i-1)*8)+1:((i-1)*8+16))=horzcat(diffcoeff(7,j,1),-diffcoeff(7,j,0));
        index=index+1;
    end
end
%Initial and final conditions to be zero
% for j=2:7
%     coeff(j*n+1,1:8)=diffcoeff(7,j-1,0)/max(diffcoeff(7,j-1,0));
%     coeff((j+1)*n,8*(n-1)+1:8*n)=diffcoeff(7,j-1,1);
% end
output=pinv(coeff)*B';
end

