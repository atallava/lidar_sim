function id = indexOfNearestTime(t,tLog)

condn = (t >= tLog(1)) && (t <= tLog(end));
msg = sprintf('%s: t not in tLog limits.\n',mfilename);
assert(condn,msg);

flag = (tLog <= t);
id = sum(flag);
end