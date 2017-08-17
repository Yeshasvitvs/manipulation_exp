function T = transformH(pose)

  P = pose(1:3);
  R = quaternion2rotm(pose(4:end));
  T= [R,P';0 0 0 1];
  
end