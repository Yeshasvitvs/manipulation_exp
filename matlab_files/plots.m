function plots(t, d, theta, Sp, Sr, V_A_PJ, V_A_RJ, V_A_1, V_A_P2, V_A_R2, h_A_P, h_A_R, F_A_1, F_A_2, W_A, P, R)

  %%Joint Angle radians to angles
  theta = theta*(180/pi);

  %%Joint Variables
  figure;
  subplot(2,1,1); plot(t,d); title('Prismatic Joint Variable');  xlabel('sec'); ylabel('meters');
  subplot(2,1,2); plot(t,theta); title('Revolute Joint Variable');  xlabel('sec'); ylabel('degrees');

  %%Joint Axes
  figure;
  subplot(2,1,1); plot(t,Sp); title('Prismatic Joint Axes');  xlabel('sec'); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}')
  subplot(2,1,2); plot(t,Sr); title('Revolute Joint Axes');  xlabel('sec'); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}')

  %%Joint Velocity
  figure;
  subplot(2,1,1); plot(t,V_A_PJ); title('Prismatic Joint Velocity');  xlabel('sec'); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}')
  subplot(2,1,2); plot(t,V_A_RJ); title('Revolute Joint Velocity');  xlabel('sec'); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}')

  %%Body 2 Velocity
  figure;
  subplot(3,1,1); plot(t,V_A_1); title('Body1 Velocity');  xlabel('sec'); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}')
  subplot(3,1,2); plot(t,V_A_P2); title('Prismatic Body2 Velocity');  xlabel('sec'); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}')
  subplot(3,1,3); plot(t,V_A_R2); title('Revolute Body2 Velocity');  xlabel('sec'); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}')

  %%Total Momentum
  figure;
  subplot(2,1,1); plot(t,h_A_P); title('Prismatic - Total Momentum');  xlabel('sec'); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}')
  subplot(2,1,2); plot(t,h_A_R); title('Revolute - Total Momentum');  xlabel('sec'); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}')

  %%Wrench - Inertial Frame
  figure;
  subplot(3,1,1); plot(t,F_A_1); title('FT1 Inertial Frame');  xlabel('sec'); legend('e_{ox}','e_{oy}','e_{oz}','e_x','e_y','e_z');
  subplot(3,1,2); plot(t,F_A_2); title('FT2 Inertial Frame');  xlabel('sec'); legend('e_{ox}','e_{oy}','e_{oz}','e_x','e_y','e_z');
  subplot(3,1,3); plot(t,W_A); title('Total Wrench');  xlabel('sec'); legend('e_{ox}','e_{oy}','e_{oz}','e_x','e_y','e_z');

  %%Joint Hypothesis
  figure;
  subplot(2,1,1); plot(t,P); title('Prismatic Hypothesis');  xlabel('sec');
  subplot(2,1,2); plot(t,R); title('Revolute Hypothesis');  xlabel('sec');
end