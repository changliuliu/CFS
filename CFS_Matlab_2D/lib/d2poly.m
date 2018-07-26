function [L,S,d] = d2poly(point,poly)

d = Inf;
nside = size(poly,1);
for i=1:nside
    x1 = poly(i,1); y1 = poly(i,2);
    x2 = poly(mod(i,nside)+1,1);
    y2 = poly(mod(i,nside)+1,2);
    
    trid = [];
    trid(1) = norm([x1-x2,y1-y2]);
    trid(2) = norm([x1-point(1),y1-point(2)]);
    trid(3) = norm([x2-point(1),y2-point(2)]);
    
    Lr = [y1-y2,x2-x1];
    Sr = -x1*y2+x2*y1;
    vd = abs(Lr(1)*point(1)+Lr(2)*point(2)-Sr)/trid(1);
    
    if trid(2)^2>trid(1)^2+trid(3)^2
        vd = trid(3);
        Lr = [point(1)-x2,point(2)-y2];
        Sr = [point(1)-x2,point(2)-y2]*[x2,y2]';
    end
    if trid(3)^2>trid(1)^2+trid(2)^2
        vd = trid(2);
        Lr = [point(1)-x1,point(2)-y1];
        Sr = [point(1)-x1,point(2)-y1]*[x1,y1]';
    end
    
    if vd < d
        d = vd;
        L = Lr;
        S = Sr;
        ii = i;
    end
end

nL = norm(L);
L = L./nL;
S = S/nL;

if L*poly(mod(ii+1,nside)+1,:)'<S
    L = -L;
    S = -S;
end

if d == 0
    return
end

area = 0; polyarea = 0;
for i=1:nside
    area = area + triArea(point,poly(i,:),poly(mod(i,nside)+1,:));
end
for i=2:nside-1
    polyarea = polyarea + triArea(poly(1,:),poly(i,:),poly(mod(i,nside)+1,:));
end
if norm(polyarea-area) < 0.01
    d = -d;
end

end

