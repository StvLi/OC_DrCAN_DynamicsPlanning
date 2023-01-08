function [ m_1 , m_2 ] = armDynamicModel(a_1,da_1,dda_1,a_2,da_2,dda_2)
    [m,n] = size(da_1);
    m_1 = zeros (m,n);
    m_1 = dda_1
    m_2 = zeros (m,n);
    m_2 = dda_2
end
