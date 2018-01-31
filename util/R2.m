function mat = R2(theta)
  mat = [cos(theta) 0 -sin(theta);   ...
              0     1          0;    ...
         sin(theta) 0 cos(theta)];
end
