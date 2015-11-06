u = [3 4 4.5 5];
v = [0.6 1.67 2.26 3];

x = 0:0.1:3;
f = 0.84*x+2.543;
plot(v, u, 'o', x, f)
xlabel('Car speed (m/s)');
ylabel('Engine value (-)');
% f = fit(v', u', 'poly1')