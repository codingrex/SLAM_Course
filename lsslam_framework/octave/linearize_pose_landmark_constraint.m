% Compute the error of a pose-landmark constraint
% x 3x1 vector (x,y,theta) of the robot pose
% l 2x1 vector (x,y) of the landmark
% z 2x1 vector (x,y) of the measurement, the position of the landmark in
%   the coordinate frame of the robot given by the vector x
%
% Output
% e 2x1 error of the constraint
% A 2x3 Jacobian wrt x
% B 2x2 Jacobian wrt l
function [e, A, B] = linearize_pose_landmark_constraint(x, l, z)

  % TODO compute the error and the Jacobians of the error
  X= v2t(x);
  
  
  Ri= X(1:2, 1:2);
  
  e= Ri' * (l - x(1:2)) - z;
  
  theta_i = atan2(Ri(2, 1), Ri(1, 1));
  
  xi= x(1);
  yi= x(2);
  
  xl= l(1);
  yl= l(2);
  
  eij_xi= [-cos(theta_i); sin(theta_i)];
  eij_yi= [-sin(theta_i); -cos(theta_i)];
  eij_thetai= [-(xl-xi)*sin(theta_i)+(yl-yi)*cos(theta_i);
               -(xl-xi)*cos(theta_i)-(yl-yi)*sin(theta_i)];
  
  
  A= [eij_xi, eij_yi, eij_thetai];
  
  eij_xl= [cos(theta_i); -sin(theta_i)];
  eij_yl= [sin(theta_i); cos(theta_i)];
  
  B=[eij_xl, eij_yl];
  


end;
