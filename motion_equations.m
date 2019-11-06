function [ddy1, ddy2] = motion_equations(t, y1, y2, u1, u2)
%motion_equation_y1

global a b c d N yc m g

% a = params(1);
% b = params(2);
% c = params(3);
% d = params(4);
% N = params(5);
% yc= params(6);
% m = params(7);
% g = params(8);

ddy1 = get_force_u11(u1, y1, a, b, N) - ...
       get_force_m12(y1, y2, yc, c, d, N) - m*g; 


ddy2 = get_force_u22(u2, y2, a, b, N) + ...
       get_force_m12(y1, y2, yc, c, d, N) - m*g;
   
end

