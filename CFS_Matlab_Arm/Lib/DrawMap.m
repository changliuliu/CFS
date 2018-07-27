figure(1);clf;
hold on
load('map/environment.mat');
patch('Faces',envir.f,'Vertices',envir.v,'FaceVertexCData',envir.c,'FaceColor',envir.color,'EdgeColor','None');
%%
load('figure/M16iB-figure.mat');
robot=robotproperty(3);
offset=[3250, 8500,0];

%% Base
for i=1:2
    f=base{i}.f; v=base{i}.v+ones(size(base{i}.v,1),1)*offset; c=base{i}.c; color=base{i}.color;
    %color = [0.6,0.6,1];
    patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');
end

%% links
M={};
M{1}=eye(4);
DH=robot.DH;
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
    %color = [0.6,0.6,1];
    patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');
    if i==1 || i==3
        for k=1:4
            v=link{i}.motor{k}.v; f=link{i}.motor{k}.f; c=link{i}.motor{k}.c; color=link{i}.motor{k}.color;
            for j=1:size(v,1)
                v(j,:)=v(j,:)*M{i+1}(1:3,1:3)'+M{i+1}(1:3,4)'.*1000+offset;
            end
            patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');
        end
    end
    
end

% %% Payload
% i=6;
% v=payload.v;f=payload.f;c=payload.c;color=payload.color;
% for j=1:size(v,1)
%     v(j,:)=v(j,:)*M{i+1}(1:3,1:3)'+M{i+1}(1:3,4)'.*1000;
% end
% patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');

%% Human Worker
load('figure/Worker-figure.mat');
figure(1); hold on
for i=1%:2:horizon
v=Worker.v;f=Worker.f;c=Worker.c;color=Worker.color;
v=v.*9;
theta = (i-1)/horizon*pi/2;
R=[0 -1 0;1 0 0; 0 0 1]*[cos(theta) sin(theta) 0;-sin(theta) cos(theta) 0; 0 0 1]*[1  0 0; 0 0 -1;0 1 0];
for j=1:size(v,1)
    v(j,:)=v(j,:)*R';
end
workeroffset=offset+[1000, 0 ,0];
T=[-600-(i-1)*10, 1000-(i-1)*20, -min(v(:,3))]+workeroffset;
for j=1:size(v,1)
    v(j,:)=v(j,:)+T;
end
Worker.hanle=patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',[0,i/horizon,i/horizon],'EdgeColor','None');
end
%%
axis equal
axis([1000 8000,5500 15000,0,2500])
view([1,0.4,1])
lighting flat
light=camlight('right');
