function [c,ceq] = nldisfn(u,A,B,x,DH,h,base,obs,cap)
c=[];
ceq=[];
nu=size(B,2);
for i=1:h
    x=A*x+B*u((i-1)*nu+1:(i)*nu);
    I = 0.1-dist_arm_3D(x(1:nu),DH(1:nu,:),base,obs,cap);
    c=[c;I];
end

end