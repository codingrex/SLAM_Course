function [v]= t2v(t)
  theta= acos(t(1,1));
  v= [t(1,3); t(2,3); theta];
 end