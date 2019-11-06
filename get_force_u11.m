function force_u11 = get_force_u11(u1, y1, a, b, N)
%get_force_u11

force_u11 = u1/(a*(y1 + b)^N);

end

