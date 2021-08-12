% Compute the error of a pose-pose constraint
% x1 3x1 vector (x,y,theta) of the first robot pose
% x2 3x1 vector (x,y,theta) of the second robot pose
% z 3x1 vector (x,y,theta) of the measurement
%
% You may use the functions v2t() and t2v() to compute
% a Homogeneous matrix out of a (x, y, theta) vector
% for computing the error.
%
% Output
% e 3x1 error of the constraint
% A 3x3 Jacobian wrt x1
% B 3x3 Jacobian wrt x2
function [e, A, B] = linearize_pose_pose_constraint(x1, x2, z)

  % TODO compute the error and the Jacobians of the error
  
  Z= v2t(z);
  
  X1= v2t(x1);
  X2= v2t(x2);
  
  e= inv(Z) * inv(X1) * X2;
    
  e= t2v(e);
  
  R_i= X1(1:2, 1:2);
  R_ij= Z(1:2, 1:2);
  
  theta_i= atan2(R_i(2,1), R_i(1,1));
  theta_ij= atan2(R_ij(2,1), R_ij(1,1));
  
  xi= x1(1);
  yi= x1(2);
  
  xj = x2(1);
  yj = x2(2);
  
  eij_xi= [-cos(theta_i)*cos(theta_ij)+sin(theta_i)*sin(theta_ij);
            cos(theta_i)*sin(theta_ij)+sin(theta_i)*cos(theta_ij);
            0];
            
  
  eij_yi= [-sin(theta_i)*cos(theta_ij)-cos(theta_i)*sin(theta_ij);
           sin(theta_i)*sin(theta_ij)-cos(theta_i)*cos(theta_ij);
           0];
  
  eij_thetai= [-(xj - xi)*(sin(theta_i)*cos(theta_ij)+cos(theta_i)*sin(theta_ij))+(yj - yi)*(cos(theta_i)*cos(theta_ij)-sin(theta_i)*sin(theta_ij));
               (xj - xi)*(sin(theta_i)*sin(theta_ij) - cos(theta_i)*cos(theta_ij))-(yj - yi)*(cos(theta_i)*sin(theta_ij)+sin(theta_i)*cos(theta_ij));
               -1];
               
  A= [eij_xi, eij_yi, eij_thetai];
  
  
  eij_xj= [cos(theta_i)*cos(theta_ij)-sin(theta_i)*sin(theta_ij);
           -cos(theta_i)*sin(theta_ij)-sin(theta_i)*cos(theta_ij);
           0];
  eij_yj= [sin(theta_i)*cos(theta_ij)+cos(theta_i)*sin(theta_ij);
           -sin(theta_i)*sin(theta_ij)+cos(theta_i)*cos(theta_ij);
           0];
          
  eij_thetaj= [0;0;1];
  
  B= [eij_xj, eij_yj, eij_thetaj];
  
  
end;
