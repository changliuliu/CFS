function grad = num_jac(f,x, eps)
y = f(x);
grad = zeros(length(y),length(x));

if nargin <3
    eps = 1e-5;
end
xp = x;

for i = 1:length(x)
    xp(i) = x(i)+eps/2;
    yhi = f(xp);
    xp(i) = x(i)-eps/2;
    ylo = f(xp);
    grad(:,i) = (yhi - ylo)/eps;
    xp(i) = x(i);
end
end