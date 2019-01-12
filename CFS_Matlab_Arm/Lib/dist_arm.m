function d = dist_arm(theta,DH,base,obs)


nstate = size(DH,1);
for i=1:nstate
DH(i,1)=theta(i);
end%theta,d,a,alpha
d = Inf;
if size(base,2)>1
    base=base';
end
line1 = base;tacc=0;
if length(line1)==3
    pos=ArmPos(base,DH);
end
for i=2:nstate
    if length(line1)==2
        line2 = line1+[DH(i,3)*cos(DH(i,1)+tacc);DH(i,3)*sin(DH(i,1)+tacc)];%;DH(i,2)];
    else
        line2 = pos(i*3+1:(i+1)*3);
    end
    [dis, points] = distLinSeg(line1,line2, obs(:,1),obs(:,2));
    if norm(dis)<0.0001
        dis = -norm(points(:,1)-line1);
    end        
    if dis < d
        d = dis;
    end
    line1 = line2;
    tacc = tacc+DH(i,1);
end
end