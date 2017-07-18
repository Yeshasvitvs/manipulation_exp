function [Phyp Rhyp] = algorithmRevInertial(data,mass1,mass2,comass1,comass2,Ic1,Ic2)

  g = [0;0;-9.8;0;0;0]; %%Gravity
  t = data(:,1); %%Time received in seconds
  t = t - t(1,1); %%Corrected to zero
  dt = diff(t); %%dt
  dt=[0;dt];

  %%Pose Values - Angular part is quaternion
  pose_1 = data(:,2:8); 
  pose_2 = data(:,9:15);

  %%Interaction Wrench - Measured in local frame
  left_wrench = data(:,16:21); %%FT at Body1
  right_wrench = data(:,22:27); %%FT at Body2

  %%Rigid Body Properties
  m1 = mass1; %%Kgs
  I_c1 = Ic1;
  com1 = comass1;

  m2 = mass2; %%Kgs
  I_c2 = Ic2; %%Inertia at CoM - taken from  SDF
  com2 = comass2;

  M_1 = spatialInertia(m1,I_c1,com1);
  M_2 = spatialInertia(m2,I_c2,com2);

  for i=1:1:size(t,1)
    
      T_A_1 = transformH(pose_1(i,:));
      T_A_2 = transformH(pose_2(i,:));
    
      T_1_2 = inv(T_A_1)*(T_A_2);
%       X_1_2 = transformX(T_1_2);
    
      %%Joint Quantities
      P_1_2 = linear(T_1_2);   
      d(i,:) = norm(P_1_2); %%Prismatic Distance
    
      %%Logic to remove NaN values
      if d(i,:) == 0
          Sp(i,:) = [0;0;0;0;0;0];
      else
          Sp(i,:) = [(P_1_2/d(i,:));0;0;0];
      end
    
% %     if d(i,:) <= 0.01
% %         d(i,:) = 0;
% %         Sp(i,:) = [0;0;0;0;0;0]; %%This is setting the axis to zero - meaning Fixed joint
% %         %%Sp(i,:) = [0;0;1;0;0;0]; %%This is setting axis to y direction
% %     else
% %         Sp(i,:) = [P_1_2/(d(i,:));0;0;0]; %%Prismatic axis - TODO: Change in the algorithm
% %     end
    
      R_1_2 = rot(T_1_2);
      axisAngleVector = vrrotmat2vec(R_1_2);
      theta(i,:) = axisAngleVector(4);
      Sr(i,:) = [0;0;0;axisAngleVector(1:3)'];
    
      %%Computing Joint Velocity
      if i == 1
          dd(i,:) = 0;
          dtheta(i,:) = 0;
      else
        
          dd(i,:) = (d(i,:)-d(i-1,:))/dt(i);
          dtheta(i,:) = (theta(i,:)-theta(i-1,:))/dt(i);    
      end
    
      %%Joint Velocity wrt inertial frame
      V_A_PJ(i,:) = transformX(T_A_1)*(Sp(i,:)')*dd(i,:);
      V_A_RJ(i,:) = transformX(T_A_1)*(Sr(i,:)')*dtheta(i,:);
    
      %%Body 1 Velocity
      P_A_1(i,:) = linear(T_A_1);
      R_A_1(i).R = rot(T_A_1);
      if i == 1
          V_A_1(i,:) = zeros(6,1);
      else
          v_A_1 = (P_A_1(i,:) - P_A_1(i-1,:))/dt(i);
          dRot_A_1 = (R_A_1(i).R - R_A_1(i-1).R)/dt(i);
          omega_A_1 = skew2vec(dRot_A_1*R_A_1(i).R);
          V_A_1(i,:) = [v_A_1 omega_A_1];
      end
    
      %%Computing Body 2 Velocity
      V_A_P2(i,:) = V_A_1(i,:) + V_A_PJ(i,:);
      V_A_R2(i,:) = V_A_1(i,:) + V_A_RJ(i,:);
    
      %%Computing Spatial Inertia
      %%TODO Check if the transformation is X or X*
      M_A_1 = transformXstar(T_A_1)*M_1*inv(transformX(T_A_1)); %%This is the same as transformX(inv(T))
      M_A_2 = transformXstar(T_A_2)*M_2*inv(transformX(T_A_1));
    
      %%Total momentum of the system
      h2P(i,:) = M_A_2 * V_A_P2(i,:)';
      h2R(i,:) = M_A_2 * V_A_R2(i,:)';
    
      h_A_P(i,:) = M_A_1 * V_A_1(i,:)' + M_A_2 * V_A_P2(i,:)';
      h_A_R(i,:) = M_A_1 * V_A_1(i,:)' + M_A_2 * V_A_R2(i,:)';
    
      %%Rate of change in momentum
      if i == 1 
          dh_A_P(i,:) = zeros(6,1);
          dh_A_R(i,:) = zeros(6,1);
      else
          dh_A_P(i,:) = (h_A_P(i,:) - h_A_P(i-1,:))/dt(i);
          dh_A_R(i,:) = (h_A_R(i,:) - h_A_R(i-1,:))/dt(i);
      end
    
      %%Computing External Wrench
      ft2_offset = 0.275; %%NOTE: This theta angle is w.r.t the first link, but it has to be gotten w.r.t the world
      F_A_1(i,:) = transformFT(T_A_1,[-0.025;0;0])*left_wrench(i,:)';
      F_A_2(i,:) = transformFT(T_A_2,[ft2_offset*cos(theta(i,:));ft2_offset*sin(theta(i,:));0])*right_wrench(i,:)';
    
      %%Gravity Forces on links
      com2_offset = com2(1);
      G_A_1(i,:) = (transformFT(T_A_1,com1)*m1*g)';
      G_A_2(i,:) = (transformFT(T_A_2,[com2_offset*cos(theta(i,:)); com2_offset*sin(theta(i,:));0])*m2*g)';
      G(i,:) = G_A_1(i,:) + G_A_2(i,:);
    
      W_A(i,:) = F_A_1(i,:) + F_A_2(i,:) + G(i,:);
      
      %%Hypothesis Computation
      P(i,:) = W_A(i,:) - dh_A_P(i,:);
      R(i,:) = W_A(i,:) - dh_A_R(i,:);
    
  end

  %%Hypothesis with 6 individual components
  Phyp6 = sum(P.*P);
  Rhyp6 = sum(R.*R);
  
  %%Norm of the hypothesis
  Phyp = norm(Phyp6);
  Rhyp = norm(Rhyp6);
  
%   pause

end