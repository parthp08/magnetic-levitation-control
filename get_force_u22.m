function force_u22 = get_force_u22(u2, y2, a, b, N)
%get_force_u22

force_u22 = u2/(a*(-y2 + b)^N);

end
