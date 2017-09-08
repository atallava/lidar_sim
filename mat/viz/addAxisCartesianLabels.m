function addAxisCartesianLabels(hfig,dim,units)
%ADDAXISCARTESIANLABELS
%
% ADDAXISCARTESIANLABELS(hfig,dim,units)
%
% hfig  - figure handle.
% dim   - scalar. default = 2.
% units - string. default = ''.

figure(hfig);
switch nargin
    case 1
        dim = 2;
        units = '';
    case 2
        units = '';
    otherwise
end

if (dim == 2) && isempty(units)
    xlabel('x'); ylabel('y');
elseif (dim == 2) && ~isempty(units)
    xlabel(sprintf('x (%s)',units)); 
    ylabel(sprintf('y (%s)',units)); 
elseif (dim == 3) && isempty(units)
    xlabel('x'); ylabel('y'); zlabel('z');
elseif (dim == 3) && ~isempty(units)
    xlabel(sprintf('x (%s)',units)); 
    ylabel(sprintf('y (%s)',units)); 
    zlabel(sprintf('z (%s)',units));
end  
end