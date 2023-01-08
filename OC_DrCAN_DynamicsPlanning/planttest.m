clear;clc
x = [ 0 0 0 0 0 0]';
u = [ -1 1 0 0 ]'; 
while 1
    l1 = 1.8 ;
    l2 = 1.4 ;
    m1 = 2.7 ;
    m2 = 2.1 ;
    g  = 9.8 ; 
    p1 = (m1+m2)*l1*l1  ;
    p2 = m2*l2*l2       ;
    p3 = m2*l1*l2       ;
    p4 = (m1+m2)*l1     ;
    p5 = m2*l2          ;
    
    %   x(1) - q1
    %   x(2) - dq1
    %   x(3) - ddq1
    %   x(4) - q2
    %   x(5) - dq2
    %   x(6) - ddq2
    M = [
            p1+p2+2*p3*cos(x(4)),   p2+p3*cos(x(4));
            p2+p3*cos(x(4))     ,   p2
        ];
    C = [
            -p3*x(5)*sin(x(4))  ,   -p3*(x(2)+x(5))*sin(x(4));
            p3*x(2)*sin(x(4))   ,   0
        ];
%     G = [
%             p4*g*cos(x(1))+p5*g*cos(x(1)+x(4));
%             p5*g*cos(x(1)+x(4))
%         ];
    G = [0 ; 0];
    Matrix1 = -M\C;
    Matrix2 = inv(M);
    A = [
            0 1             0 0 0               0;
            0 0             1 0 0               0;
            0 Matrix1(1,1)  0 0 Matrix1(1,2)    0;
            0 0             0 0 1               0;
            0 0             0 0 0               1;
            0 Matrix1(2,1)  0 0 Matrix1(2,2)    0;
        ];
    B = [
            0               0;
            0               0;
            Matrix2(1,1) Matrix2(1,2);
            0               0;
            0               0;
            Matrix2(2,1) Matrix2(2,2);
        ];
    %   x(1) - t1
    %   x(2) - t2
    %   x(3) - td1
    %   x(4) - td2
    x = A*x + B*([u(1);u(2)]-[u(3);u(4)]-G)
end