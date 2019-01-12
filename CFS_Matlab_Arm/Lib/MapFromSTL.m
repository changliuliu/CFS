addpath('/Users/changliuliu/Matlab/stlread');
prefix='/Users/changliuliu/Matlab/LongTerm/map/';
filename=strcat(prefix,'assembly line_Assem1.stl');
[v, f, n, c, stltitle]=stlread(filename);

for i=1:3
v(:,i)=v(:,i)-min(v(:,i));
end
v(:,2)=v(:,2)-100;
vv=v(:,:);
v(:,1)=vv(:,3);v(:,2)=vv(:,1);v(:,3)=vv(:,2);
envir.v=v;envir.f=f;envir.n=n;envir.c=c;envir.color=[0.5,0.5,0.5];