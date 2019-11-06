function force_m12 = get_force_m12(y1, y2, yc, c, d, N)
%get_force_m12

force_m12 = c/((yc + y2 - y1 + d)^N);

end