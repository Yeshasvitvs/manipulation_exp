function q = euler2quaternion(ang)

  R = euler2rotm(ang);
  q = rotm2quaternion(R);

end