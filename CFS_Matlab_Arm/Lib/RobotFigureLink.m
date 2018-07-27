load('figure/M16iB-figure.mat');
DH=robot.DH;
M={};
M{1}=eye(4);
for i=1:6
    R=[cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4));
        sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4));
        0  sin(DH(i,4)) cos(DH(i,4))];
    T=[DH(i,3)*cos(DH(i,1));DH(i,3)*sin(DH(i,1));DH(i,2)];
    M{i+1}=M{i}*[R T;zeros(1,3) 1];
end

for i=1:6
    v=link{i}.v; f=link{i}.f; c=link{i}.c; color=link{i}.color;
    for j=1:size(v,1)
        v(j,:)=v(j,:)*M{i+1}(1:3,1:3)'+M{i+1}(1:3,4)'.*1000+offset;
    end
    gcf;
    %color = [0.6,0.6,1];
    h=patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');
    alpha(h,valpha);
    if i==1 || i==3
        for k=1:4
            v=link{i}.motor{k}.v; f=link{i}.motor{k}.f; c=link{i}.motor{k}.c; color=link{i}.motor{k}.color;
            for j=1:size(v,1)
                v(j,:)=v(j,:)*M{i+1}(1:3,1:3)'+M{i+1}(1:3,4)'.*1000+offset;
            end
            h=patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');
            alpha(h,valpha);
        end
    end
    
end