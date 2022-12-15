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
%       Vd_up - matrix of upper layer   (2,n*n)
%       Vd_lw - matrix of lower layer   (2,n*n)

%
%   Outputs:J - loss fuction matrix
%        Path - scheme matrix
%        
function [J,Path] = myArmDPLayer( k , J , Path , Theta_up , ...
    Vd_up , Vd_lw , dt )
    
    %   Judge whether the angle motion has been finished
    flag = [1 1];
    if Theta_up(1) > 90
        flag(1) = 0;
    end
    if Theta_up(2) > 90
        flag(2) = 0;
    end

    if flag(1) ~= 0 && flag(2) ~= 0  %   motion hasn`t been finished
        %   Matrix Arrangement 
        %       Theta_up → a_1 a_2
        [~,a_1] = meshgrid( Vd_lw(1,:) , Theta_up(1,:) );
        [~,a_2] = meshgrid( Vd_lw(2,:) , Theta_up(2,:) );
        %       Vd_up → da_1p da_2p   Vd_lw → da_1c da_2c
        [da_1c,da_1p] = meshgrid( Vd_lw(1,:) , Vd_up(1,:)' );
        [da_2c,da_2p] = meshgrid( Vd_lw(2,:) , Vd_up(2,:)' );

        %   Calculate present angle to go in future steps 
        a_1 = 0.5*( da_1p + da_1c )*dt + a_1;
        a_2 = 0.5*( da_2p + da_2c )*dt + a_2;

        %   Calculate angular accelerations
        dda_1 = ( da_1c - da_1p)/dt;
        dda_2 = ( da_2c - da_2p)/dt;
        
        %   Calcutate motors` output moments
        [ m_1 , m_2 ] = armDynamicModel(a_1,da_1c,dda_1,a_2,da_2c,dda_2);
        
        %   Calculate COST FUNCTION
        J_temp = costOutput( m_1 , m_2 );           %   Output cost
        J_temp = J_temp + costPenalty( m_1 , m_2 ); %   Penalty cost

        %   Renew the PLAN COST & PATH with least COST scheme
            for col = 1:length(Vd_lw)
                [J(k,col),pos(col)] = min(J_temp(:,col));   %   renew cost
                Path(k,col) = pos(col);    %   renew path
            end
        
    else
        %   No need to renew parameters & direct Output

    end
end
