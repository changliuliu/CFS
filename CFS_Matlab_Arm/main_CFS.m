%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CFS on robot arm 
% In this example, the arm is FANUC M16iB
%
% Changliu Liu
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
currfold = pwd;
addpath(strcat(currfold,'/Lib'));
addpath(genpath(strcat(currfold,'/DERIVESTsuite')));

robot=robotproperty(3);
njoint=5;nstate=10;nu=5;DH=robot.DH;
base=[3250, 8500,0]'./1000;

load('data/M16_ref_2.mat')

xR=[];xR(:,1)=xref(1:nstate);
horizon=size(uref,1)/njoint;
xori=xref(nstate+1:end);
xref=xori;
obs=[[4106;8313;1072]./1000 [4106;8313;1538]./1000];

%% Cost Fn Parameters
Aaug=[];Baug=zeros(horizon*nstate,horizon*nu);Qaug=zeros(horizon*nstate);
Q=[];
Q(1:njoint,1:njoint)=[10 0 0 0 0;
    0 1 0 0 0;
    0 0 1 0 0;
    0 0 0 1 0;
    0 0 0 0 1];
Q(1:njoint,njoint+1:2*njoint)=0.1*eye(njoint);
Q(njoint+1:2*njoint,1:njoint)=0.1*eye(njoint);
Q(njoint+1:2*njoint,njoint+1:2*njoint)=[10 0 0 0 0;
    0 2 0 0 0;
    0 0 1 0 0;
    0 0 0 1 0;
    0 0 0 0 1];
for i=1:horizon
    Aaug=[Aaug;robot.A([1:njoint,7:6+njoint],[1:njoint,7:6+njoint])^i];
    Qaug((i-1)*nstate+1:i*nstate,(i-1)*nstate+1:i*nstate)=Q*0.1;
    if i==horizon
        Qaug((i-1)*nstate+1:i*nstate,(i-1)*nstate+1:i*nstate)=Q*10000;
    end
    for j=1:i
        Baug((i-1)*nstate+1:i*nstate,(j-1)*nu+1:j*nu)=robot.A([1:njoint,7:6+njoint],[1:njoint,7:6+njoint])^(i-j)*robot.B([1:njoint,7:6+njoint],1:nu);
    end
end
R=eye(horizon*nu);
for i=1:horizon
    R((i-1)*nu+1:i*nu,(i-1)*nu+1:i*nu)=[5 0 0 0 0;
        0 4 1 0 0;
        0 1 2 0 0;
        0 0 0 2 0;
        0 0 0 0 1];
end
R=R+R';

f = @(x) dist_arm_3D_Heu(x,DH(1:njoint,:),base,obs,robot.cap);
%% CFS Iteration
MAX_ITER = 10;
time_record =[];
D=0.09;
for k=1:MAX_ITER
    tic
    Lstack=[];Sstack=[];
    for j=1:1
        I=[];
        for i=1:horizon
            theta=xref(nstate*(i-1)+1:nstate*(i-1)+njoint);
            [distance,linkid]=dist_arm_3D_Heu(theta,DH(1:njoint,:),base,obs,robot.cap);
            %distance=dist_arm_3D(theta,DH(1:njoint,:),base,obs,robot.cap);
            I = [I;distance-D];
%             Diff=zeros(njoint,1);
%             for s=1:njoint                
%                 [Diff(s),~]=derivest(@(x) dist_link_Heu([theta(1:s-1);x;theta(s+1:end)],DH(1:njoint,:),base,obs,robot.cap,linkid),theta(s),'Vectorized','no','FixedStep',1e-5);
%                 %[Diff(s),~]=derivest(@(x) dist_arm_3D([theta(1:s-1);x;theta(s+1:end)],DH(1:njoint,:),base,obs,robot.cap),theta(s),'Vectorized','no');
%             end
%             Diff;
            Diff = num_jac(f,theta); Diff = Diff';
            %Hess=hessian(@(x) dist_link_Heu(x,DH(1:njoint,:),base,obs,robot.cap,linkid),theta);
            Bj=Baug((i-1)*nstate+1:i*nstate,1:horizon*nu);
            s=I(i)-Diff'*Bj(1:njoint,:)*uref;
            l=-Diff'*Bj(1:njoint,:);
            
%             [E,lambda]=eig(Hess);
%             for m=1:size(lambda,1)
%                 if lambda(m,m)<0
%                     s = s+lambda(m,m)/size(lambda,1);
%                     flag = 'yes';
%                 end
%             end
            
            Sstack=[Sstack;s];
            Lstack=[Lstack;l];
            
        end
    end

    pretime=toc
    tic;
    unew = quadprog(Baug'*Qaug*Baug+R.*10,((Aaug*xR(:,1)-xori)'*Qaug*Baug)',Lstack,Sstack);
    time=toc
    time_record = [time_record; pretime, time];
    % fun= @(u) u'* (Baug'*Qaug*Baug+R)* u+ 2*((Aaug*xR(:,1)-xori)'*Qaug*Baug)*u;%+(Aaug*xR(:,1)-xref)'*Qaug*(Aaug*xR(:,1)-xref);
    % nonlcon=@(u) nldisfn(u,robot.A([1:njoint,7:6+njoint],[1:njoint,7:6+njoint]),robot.B([1:njoint,7:6+njoint],1:nu),xori(1:nstate),robot.DH,horizon,base,obs,robot.cap);
    % options = optimoptions(@fmincon,'Display','iter','Algorithm','sqp','TolCon',1,'MaxFunEvals',30000);
    % tic;
    % unew = fmincon(fun,uref,[],[],[],[],[],[],nonlcon,options);
    % time=toc
    
    oldref=xref;
    xref=[];
    for i=2:horizon+1
        xR(:,i)=robot.A([1:njoint,7:6+njoint],[1:njoint,7:6+njoint])*xR(:,i-1)+robot.B([1:njoint,7:6+njoint],1:nu)*unew((i-2)*nu+1:(i-1)*nu);
        xref=[xref;xR(:,i)];
    end
    uref=unew;
    norm(xref-oldref)
    if norm(xref-oldref)<0.01
        disp(strcat('Converged at step',num2str(k)));
        break;
    end
    if Lstack*uref<Sstack
        disp(strcat('Local Optimal Found at step ',num2str(k)));
        break;
    end
end
%% Visualization
DrawMap; view([105,34]);
for i=1:1:horizon
    robot.DH(1:njoint,1)=xref((i-1)*nstate+1:(i-1)*nstate+njoint);
    
    % Use the following lines to draw capsules
    %     color=[i/horizon,i/horizon,i/horizon];
    %     valpha=0.2;
    %     RobotCapLink;
    
    % Use the following lines to draw robot arm
    valpha=i/horizon;
    RobotFigureLink;
    % Use the following line to enable animation plot
    pause(0.2);
end