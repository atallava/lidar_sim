function [xEll,yEll,zEll] = genSurfXyzEllipse2(covMat,mu,level)
   % like genSurfXyz, but proceeds via svd
   
   if nargin < 3
        level = 3^2; % three sigma
   end
    
   [U,S,~] = svd(inv(level*covMat));
   
   % todo: delete me
   U(:,1) = -U(:,1);
   U(:,2) = -U(:,2);
   
   S_p5 = sqrt(S);
   scales = 1./diag(S_p5);
   
   [xSph,ySph,zSph] = sphere(10);
   n = size(xSph,1);
   ptsSph = [xSph(:)'; ySph(:)'; zSph(:)'];
   
   ptsEll = ptsSph;
   % first scale
   for i = 1:3
       ptsEll(i,:) = ptsEll(i,:)*scales(i);
   end
   % then rotate
   ptsEll = U*ptsEll;
   
   mu = flipVecToColumn(mu);
   ptsEll = bsxfun(@plus,ptsEll,mu);
   
   xEll = reshape(ptsEll(1,:),n,n);
   yEll = reshape(ptsEll(2,:),n,n);
   zEll = reshape(ptsEll(3,:),n,n);
end

