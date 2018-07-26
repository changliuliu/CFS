% This script implements the CFS algorithm to solve a problem specified in
% problem.m


addpath('lib')

%% Initialize the problem
problem

%% The Iteration
MAX_ITER = 100;

figure; hold on

tic
for k = 1:MAX_ITER
    % The constraints
    Lstack = []; Sstack = [];
    for i=1:nstep
        for j=1:nobj
            poly = obs{j}.poly+obs{j}.v*ones(1,4)*dt*i;
            [L,S,d] = d2poly(refpath((i-1)*dim+1:i*dim)',poly');
            Lstack = [Lstack;zeros(1,(i-1)*dim) L zeros(1,(nstep-i)*dim) zeros(1,nstep)];
            Sstack = [Sstack;S-margin];
        end
    end
    % QP
    Qe = Qref+Qabs;
    soln = quadprog(Qe,-Qref*oripath,Lstack(:,1:size(Qe,1)),Sstack,Aeq(:,1:size(Qe,1)),beq);
    
    pathnew = soln(1:dim*nstep);
    diff = norm(refpath-pathnew);
    if diff < 0.001*nstep*(dim)
        disp(['converged at step ',num2str(k)]);
        break
    end
    refpath = pathnew;
    plot(pathnew(1:dim:end),pathnew(2:dim:end))
end
%% Statistics
time = toc
disp(['final cost: ']);
cost_quadratic(pathnew,oripath,Qref,Qabs)

%% Plot final solution
plot(pathnew(1:dim:end),pathnew(2:dim:end),'k','LineWidth',2)

%% Plot obstacles
for i=1:nobj
    ob = Polyhedron('V',obs{i}.poly');
    h = ob.plot('color', [0.8 0.8 0.8], 'EdgeColor','k');
    alpha(h,0.5);
end
axis equal
grid off
box on
legend('Iter1','Iter2','Iter3','Iter4','Iter5')
axis([-0.5 9.5 -3 3])
