%% From measurements of Left_Clothoid_2 (7 dec 2015)

delta = [0.0 0.025 0.09 0.15 0.23 0.31 0.37];
u = [0.0 0.1 0.2 0.3 0.4 0.5 0.6];

x = 0:0.01:0.6;
% f = 1.416*x + 0.07264;
f = 1.4630*x + 0.059;
plot(delta, u, 'o', x, f)
axis([0 0.6 00 0.6])
xlabel('Steering angle (rad)');
ylabel('Servo value (-)');

% f = fit(delta', u', 'poly1')

%% From measurements of Right_Clothoid (7 dec 2015)

delta = [-0.04 -0.1 -0.16 -0.24 -0.31 -0.36];
u = [-0.1 -0.2 -0.3 -0.4 -0.5 -0.6];

x = 0:-0.01:-0.6;
% f = 1.51*x -0.04546;
f = 1.4630*x - 0.059;
hold on
plot(delta, u, 'o', x, f)
axis([-0.6 0.6 -0.6 0.6])
xlabel('Steering angle (rad)');
ylabel('Servo value (-)');
f = fit(delta', u', 'poly1')

%%
(1.51+1.416)/2
(0.07264+0.04546)/2
