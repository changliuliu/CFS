% Function for fast computation of the shortest distance between two line segments
%
% Algorithm implemented:
%   Vladimir J. LUMELSKY,
%   ``ON FAST COMPUTATION OF DISTANCE BETWEEN LINE SEGMENTS'',
%   Information Processing Letters 21 (1985) 55-61
%
%
% Input: ([start point of line1], [end point of line1], [start point of
% line2], [end point of line2])
%
% Output:   dist - shortest distance between the line segments (in N dimensions)
%           points (optional) - shortest points on the line segments
%
% Example:
% >> [dist,points] = distLinSeg([0 0], [1 1], [1 0], [2 0])
% >> dist = 0.7071
% >> points = [0.5 0.5; 1 0]
%
%
% 1.2.2015     - created: Ondrej Sluciak (osluciak@gmail.com)
%
function [dist,varargout] = distLinSeg(point1s,point1e,point2s,point2e)

d1  = point1e - point1s;
d2  = point2e - point2s;
d12 = point2s - point1s;

D1 = sum(d1.^2);
D2 = sum(d2.^2);

S1 = sum(d1.*d12);
S2 = sum(d2.*d12);
R = sum(d1.*d2);

den = D1*D2-R^2;    %denominator

if (D1 == 0 || D2 == 0)     % if one of the segments is a point
    if (D1 ~= 0)            % if line1 is a segment and line2 is a point
        u = 0;
        t = S1/D1;

        t = fixbound(t);
 
    elseif (D2 ~= 0)        % if line2 is a segment and line 1 is a point
        t = 0;
        u = -S2/D2;

        u = fixbound(u);
  
    else                    % both segments are points
        t = 0;          
        u = 0;
    end
elseif (den == 0)           % if lines are parallel
    t = 0;
    u = -S2/D2;
    
    uf = fixbound(u);
    
    if (uf~= u)
        t = (uf*R+S1)/D1;
        t = fixbound(t);
    
        u = uf;
    end
else                        % general case
    
    t = (S1*D2-S2*R)/den;
    
    t = fixbound(t);
    
    u = (t*R-S2)/D2;
    uf = fixbound(u);
    
    if (uf ~= u)
        t = (uf*R+S1)/D1;
        t = fixbound(t);
      
        u = uf;
    end
end

% Compute distance given parameters 't' and 'u'
dist = norm(d1*t-d2*u-d12);   %dist = sqrt(sum((d1*t-d2*u-d12).^2));

if (nargout > 1)
    varargout = {[point1s + d1*t;point2s+d2*u]};
end

end
    
function num = fixbound(num)    % if num is out of (0,1) round to {0,1}

if (num < 0)
    num = 0;
elseif (num > 1)
    num = 1;
end

end
