%   Dynamics Planning Layer Fuction
%       used to calculate the itration of DP
%       created by StvLi 2022-12-10
%       based on idea from Dr.CAN
%       renew: general form that be able to handle links between two layers
%               which could contain any number of nodes
%
%   Inputs: k - recipocal number of the current layer
%           J - matrix of loss fuction  (p,n*n)
%        Path - matrix of scheme matrix (p,n*n)
%    Theta_up - matrix of upper angles  (2,n*n)
%       Vd_up - vector of upper layer   (1,n*n)
%       Vd_lw - vector of lower layer   (1,n*n)

%
%   Outputs:J - loss fuction matrix
%        Path - scheme matrix
%        
function [J,Path] = myArmDPLayer( k , J , Path , Theta_up , Vd_up , Vd_lw)
    
    %   Judge whether the angle motion has been finished
    if theta(1) > 90
        flag(1) = 0;
    end
    if theta(2) > 90
        flag(2) = 0;
    end
    
    if flag ~= [ 0 0 ]  %   motion hasn`t been finished
        a_1 = Theta_up(1,:);
        a_2 = Theta_up(2,:);
        %   Calculate present angle to go in future steps 
        a_1 = 0.5*( da_1p + da_1c )*dt + a_1;
        a_2 = 0.5*( da_2p + da_2c )*dt + a_2;
        %   Calculate angular accelerations
        dda_1 = ( da_1c - da_1p)/dt;
        dda_2 = ( da_2c - da_2p)/dt;
        %   Calcutate motors` output moments
        [ m_1 , m_2 ] = armDynamicModel(a_1,da_1,dda_1,a_2,da_2,dda_2);
        %   Calculate COST FUNCTION
            %   Output cost
            J_temp = costOutput( m_1 , m_2 );
            %   Penalty cost
            J_temp = J_temp + costPenalty( m_1 , m_2 );

        %   Renew the PLAN COST & PATH with least COST scheme
            for col = 1:length(Vd_lw)
                [J(k,col),pos(col)] = min(J_temp(:,col));   %   renew cost
                Path(k,col) = [ Path(k,col) ; pos(row)];    %   renew path
            end
        
    else
        %   No need to renew parameters & direct Output
        
    end
    Path = [];
end