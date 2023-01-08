%   create state array
clc; clear all; close all;

%   Define IC
h_init = 0;    %   h: height
v_init = 0;    %   v: velocity
%   Final state
h_final = 10;
v_final = 0;
%   Boundary condition
h_min = 0;
h_max = 10; N_h = 500;
v_min = 0;
v_max = 3; N_v = 300;
%   Create state array
Hd = h_min:(h_max-h_min)/N_h:h_max;
Vd = v_min:(v_max-v_min)/N_v:v_max;
%   Input constraint, input is the system acceleration
u_min = -3; u_max = 2;
%   Define cost to go matrix
J_costtogo = zeros(N_h+1,N_v+1);
%   Define input acceleration matrix
Input_acc = zeros(N_h+1,N_v+1);

%%%% From 10m to 8m %%%%
[ J_costtogo , Input_acc ] = DynaPlanLayer( 2 , J_costtogo , Input_acc , ...
     (h_max-h_min)/(N_h) , [0] , Vd , u_min , u_max );
%%%% From 8m to 2m %%%%
for k = 3:1:N_h
    [ J_costtogo , Input_acc ] = DynaPlanLayer( k , J_costtogo , Input_acc , ...
         (h_max-h_min)/(N_h) , Vd , Vd , u_min , u_max );
end
%%%% From 2m to 0m %%%%
[ J_costtogo , Input_acc ] = DynaPlanLayer( N_h+1 , J_costtogo , Input_acc , ...
     (h_max-h_min)/(N_h) , Vd , [0] , u_min , u_max );

%%%%%%%%%%%%%% Plot %%%%%%%%%%%%%%%%%%%%
%   作者：DR_CAN https://www.bilibili.com/read/cv20060597?spm_id_from=333.999.0.0 出处：bilibili
h_plot_init = 0;                % Initial height
v_plot_init = 0;                % Initial velocity
acc_plot = zeros(length(Hd),1); % Define acc plot array
h_plot = zeros(length(Hd),1);   % Define height plot array
v_plot = zeros(length(Hd),1);   % Define velocity plot array
h_plot (1) = h_plot_init;       % First value
v_plot (1) = v_plot_init;       % First value

for k = 1 : 1 : N_h
    [min_h,h_plot_index] = min(abs(h_plot(k) - Hd));    % Table look up
    [min_v,v_plot_index] = min(abs(v_plot(k) - Vd));    % Table look up
    acc_index = sub2ind(size(Input_acc), N_h+2-h_plot_index, v_plot_index); % Find control input acceleration
    acc_plot (k) = Input_acc(acc_index);                                    % Save acceleration to the matrix
    v_plot (k + 1) = sqrt((2 * (h_max - h_min)/N_h * acc_plot(k))+ v_plot (k)^2);   % Calculate speed and height
    h_plot (k + 1) = h_plot(k) + (h_max - h_min)/N_h;
end

% Plot
subplot(2,1,1);
plot(v_plot,h_plot,'--^'),grid on;
ylabel('h(m)');
xlabel('v(m/s)');
subplot(2,1,2);
plot(acc_plot,h_plot,'^'),grid on;
ylabel('h(m)');
xlabel('a(m/s^2)'); 