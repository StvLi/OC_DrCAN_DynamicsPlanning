%   Dynamics Planning Layer Fuction
%       used to calculate the itration of DP
%       created by StvLi 2022-12-10
%       based on idea from Dr.CAN
%       renew: general form that be able to handle links between two layers
%               which could contain any number of nodes
%
%   Inputs: k - recipocal number of the current layer
%           J - loss fuction matrix
%           I - input matrix
%           h_delta - height difference between two layers
%           Vd_up   - vector of upper layer
%           Vd_dn   - vector of lower layer
%           u_min   - minimum of input value
%           u_max   - maximum of input value
%
%   Outputs:J - loss fuction matrix
%           I - input matrix
function [ J , I ] = my3DoFArmDPLayer( k , J , I , ...
    h_delta , Vd_up , Vd_lw , u_min , u_max )
    Vd_up1 = Vd_up(1,:) ;
    Vd_up2 = Vd_up(2,:) ;
    Vd_up3 = Vd_up(3,:) ;
    Vd_lw1 = Vd_lw(1,:) ;
    Vd_lw2 = Vd_lw(2,:) ;
    Vd_lw3 = Vd_lw(3,:) ;

    [vd_x,vd_y]     = meshgrid(Vd_up1,Vd_lw1);            %   Prepare the matrix
    v_avg           = 0.5*(vd_x+vd_y);                  %   Calculate average time
    T_delta         = (h_delta)./v_avg;                 %   Calculate travel time, this is the cost
    inp             = (vd_x - vd_y)./T_delta;           %   Calculate acceleration
    J_temp          = T_delta;                          %   Assign delta T to cost to go
    [acc_x,acc_y]   = find(inp<u_min|inp>u_max);        %   FIne Acc is over the limit
    Ind_lin_acc     = sub2ind(size(inp),acc_x,acc_y);   %   Fine linear index
    J_temp(Ind_lin_acc) = inf;                          %   Let certain elements to infinity
    %%%%    Very Important Step！    %%%%
    [~,m] = size(Vd_up1);
    [~,n] = size(Vd_lw1);
    for row = 1:n
        for col =1:m
            J_temp(row,col) = J_temp(row,col) + J(k-1,col);
        end
    end
    for row = 1:length(Vd_lw1)
        [J(k,row),pos(row)] = min(J_temp(row,:));  %   Save to cost to go matrix
        I(k,row)  = inp(row,pos(row));              %   Save to acceleration matrix
    end    
end