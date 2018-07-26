% This script specifies the planning problem

nstep = 30; % number of sampling points on the path
dim = 2; % dimention of the problem, default 2
dt = 1; % sampling time (s)
start = [0;0]; % starting point
ending = [9;0]; % ending point
margin = 0.25; % safety margin

%% Initialize the path by linear interpolation
path = [];
for i = 1:nstep
    path(:,i) = (nstep-i)/(nstep-1)*start+(i-1)/(nstep-1)*ending;
end
refpath = [];
for i=1:nstep
    refpath = [refpath;path(:,i)];
end
oripath = refpath;
%% Initialize the obstacles
nobj = 5;
obs = {};

obs{1}.v = [0;0]; % velocity of the obstacle
obs{1}.poly = [2 4.3 4 3;3 3 -1 -1]; % shape of the obstacle
obs{2}.v = [0;0];
obs{2}.poly = [6 7 7.5 6;0 1 -3 -3];
obs{3}.v = [0;0];
obs{3}.poly = [1 2 1.5 0;-2 -3 -0.1 -1];
obs{4}.v = [0;0];
obs{4}.poly = [1 6 1 5; 1 1 2 2];
obs{5}.v = [0;0];
obs{5}.poly = [3 7 7 3;-3 -3 -2.1 -2.1];

%% The cost function
% The distance metric between the original path and the new path
Q1 = eye(nstep*dim);
% The velocity
Vdiff = eye(nstep*dim)-diag(ones(1,(nstep-1)*dim),dim);
Q2 = Vdiff(1:(nstep-1)*dim,:)'*Q1(1+dim:end,1+dim:end)*Vdiff(1:(nstep-1)*dim,:);
% The accelaration
Adiff = Vdiff-diag(ones(1,(nstep-1)*dim),dim)+diag(ones(1,(nstep-2)*dim),dim*2);
Q3 = Adiff(1:(nstep-2)*dim,:)'*Adiff(1:(nstep-2)*dim,:);
% The weight
cref = [0,0,0];
cabs = [0,0,1];
% The total costj
Qref = Q1*cref(1)/nstep+Q2*cref(2)*(nstep-1)+Q3*cref(3)*(nstep-1)^4/(nstep-2);
Qabs = Q1*cabs(1)/nstep+Q2*cabs(2)*(nstep-1)+Q3*cabs(3)*(nstep-1)^4/(nstep-2);

%% The boundary constraint
Aeq = zeros(4*dim,nstep*dim+nstep);
Aeq(0*dim+1:1*dim,1:dim) = eye(dim);
Aeq(1*dim+1:2*dim,(nstep-1)*dim+1:nstep*dim) = eye(dim);
Aeq(2*dim+1:3*dim,1:2*dim) = [-eye(dim) eye(dim)];
Aeq(3*dim+1:4*dim,(nstep-2)*dim+1:nstep*dim) = [-eye(dim) eye(dim)];
beq = [path(:,1);path(:,end);path(:,2)-path(:,1);path(:,end)-path(:,end-1)];

