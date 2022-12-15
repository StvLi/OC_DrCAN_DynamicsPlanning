function J = costPenalty( m_1 , m_2 )
    [m,n]=size(m_1);
    J = zeros(m,n);
    for i=1:m
        for j=1:n
            if abs(m_1(i,j)) > 8 || abs(m_2(i,j))>8
                J(i,j) = 10*(m_1(i,j)^2 + m_2(i,j)^2);
            end
        end
    end
end