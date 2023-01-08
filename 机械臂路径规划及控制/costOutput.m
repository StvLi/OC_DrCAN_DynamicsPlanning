function J = costOutput( m_1 , m_2 )
    [m,n]=size(m_1);
    J =zeros(m,n);
    for i=1:m
        for j=1:n
            J(i,j) = m_1(i,j)^2+m_2(i,j)^2;
        end
    end
end
