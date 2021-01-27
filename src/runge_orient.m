function x = runge_orient(w,q0)


qdot = 0.5 * [    0, -w(1), -w(2), -w(3);...
              w(1),      0,  w(3), -w(2);...
              w(2), -w(3),      0,  w(1);...
              w(3),  w(2), -w(1),     0];
x = qdot * q0;

end