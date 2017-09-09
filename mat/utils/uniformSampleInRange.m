function samples = uniformSampleInRange(a,b,n)
%UNIFORMSAMPLEINRANGE
%
% samples = UNIFORMSAMPLEINRANGE(a,b,n)
%
% a       - scalars. range min.
% b       - scalars. range max.
% n       - scalars. number of samples. defaults to 1.
%
% samples - [1,n] array.

samples = rand(1,n)*(b-a)+a;
end