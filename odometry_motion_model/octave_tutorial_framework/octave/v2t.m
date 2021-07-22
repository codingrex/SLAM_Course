function [t]= v2t(p)
  theta= p(3);
  t= [cos(theta), -sin(theta), p(1);
  sin(theta), cos(theta), p(2);
  0, 0, 1];
 end
 