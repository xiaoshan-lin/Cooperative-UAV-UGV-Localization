function x_out = wrapx(x_in)
x_out = x_in;
x_out(3,:) = wrapToPi(x_in(3,:));
x_out(6,:) = wrapToPi(x_in(6,:));
end