# this code is to visualize the effect of varying the constants a,b,c in  the 3rd order curvature equation
a = 1;
b = 1;
c = 1;
k0 = 0;

col_list = ['-r', '-g', '-b', '-k', '-c', '-m'];

s = 1:5; # arc length 
K = k0 + a.*s + b.*(s.^2) + c.*(s.*s.*s);

plot(s,K,'-r');

# vary constant -a 
min_a = 1;
max_a = 6;
delta_a = 1;

figure();
hold on;
grid on;
for a_iter = min_a:delta_a:max_a
    K = k0 + a_iter.*s + b.*(s.^2) + c.*(s.*s.*s);
    plot(s,K,col_list(a_iter))

end

