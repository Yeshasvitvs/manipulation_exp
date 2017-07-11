function angle = quaternion2euler(q)

  R = quaternion2rotm(q);
  angle = rotm2euler(R);

end