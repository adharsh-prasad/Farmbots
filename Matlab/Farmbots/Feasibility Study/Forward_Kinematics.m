function T = Forward_Kinematics(Link1, Link2, Link3)
    syms theta1 theta2 theta3
    T1 = tranformation_matrix([theta1 Link1]);
    T2 = tranformation_matrix([theta2 Link2]);
    T3 = tranformation_matrix([theta3 Link3]);
    T = T1 * T2 * T3;
end

function T = tranformation_matrix(Link)
    theta = Link(1);
    d = Link(2);
    alpha = Link(3);
    a = Link(4);
    T = [cos(theta) -sin(theta)*cos(alpha) -sin(theta)*sin(alpha) a*cos(theta);
         sin(theta)  cos(theta)*cos(alpha)  cos(theta)            a*sin(theta);
         0           sin(alpha)             cos(alpha)            d;
         0           0                      0                     1];
end