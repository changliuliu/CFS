%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function updates the axis of the capsules
%
% Changliu Liu
% 2015.8.5
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [pos,M]=CapPos(base,DH,RoCap)
nlink=size(DH,1);
pos=cell(1,nlink);
M=cell(1,nlink+1); M{1}=eye(4);
for i=1:nlink
    R=[cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4));
        sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4));
        0  sin(DH(i,4)) cos(DH(i,4))];
    T=[DH(i,3)*cos(DH(i,1));DH(i,3)*sin(DH(i,1));DH(i,2)];
    M{i+1}=M{i}*[R T; zeros(1,3) 1];
    for k=1:2
        pos{i}.p(:,k)=M{i+1}(1:3,1:3)*RoCap{i}.p(:,k)+M{i+1}(1:3,4)+base;
    end
end
end