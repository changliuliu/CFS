function v = setVertice(v, TransM, scale, offset)
if nargin < 4, offset = [0,0,0]; end
if nargin < 3, scale = 1; end
v = v*TransM(1:3,1:3)' +repmat(TransM(1:3,4)'+offset,size(v,1),1).*scale;