function d = dist_link_Heu(theta,DH,base,obs,cap,linkid)
nstate = size(DH,1);
for i=1:nstate
DH(i,1)=theta(i);
end%theta,d,a,alpha
d = Inf;
if size(base,2)>1
    base=base';
end

pos=CapPos(base,DH,cap);

for i=linkid
    [dis, points] = distLinSeg(pos{i}.p(:,1),pos{i}.p(:,2), obs(:,1),obs(:,2));
    if norm(dis)<0.0001
        dis = -norm(points(1:3)-pos{i}.p(:,2));
    end        
    if dis < d
        d = dis;
    end
end
end