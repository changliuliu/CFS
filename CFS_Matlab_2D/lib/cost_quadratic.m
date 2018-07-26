function [fun,grad,hess] = cost_quadratic(x,refpath,Qref,Qabs)
fun = (x-refpath)'*Qref*(x-refpath)+x'*Qabs*x;% -refpath'*Qref*refpath;
if nargout > 1
    grad = (2*Qref*(x-refpath) + 2*Qabs*x)';
end
if nargout > 2
    hess = 2*Qref + 2*Qabs;
end
end