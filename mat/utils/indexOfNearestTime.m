function id = indexOfNearestTime(t,tLog)
%INDEXOFNEARESTTIME
%
% id = INDEXOFNEARESTTIME(t,tLog)
%
% t    - Scalar.
% tLog - Array.
%
% id   - Index in tLog.

condn = (t >= tLog(1)) & (t <= tLog(end));
msg = sprintf('%s: t not in tLog limits.\n',mfilename);
assert(condn,msg);

flag = (tLog <= t);
id = sum(flag);
end