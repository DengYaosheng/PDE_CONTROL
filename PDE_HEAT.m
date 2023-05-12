% Heat equation with boundary control
% u_t = alpha * u_xx
% u(x,0) = u0(x), u(0,t) = uL(t), u(L,t) = uR(t)

% Parameters
alpha = 1;
L = 1;
T = 1;
dx = 0.01;
dt = 0.0001;

% Grid
x = 0:dx:L;
t = 0:dt:T;
N = length(x);
M = length(t);

% Initial condition
u = sin(pi*x);

% Boundary conditions
uL = zeros(1,M);  % Modified: Initialize uL with the correct size
uR = zeros(1,M);  % Modified: Initialize uR with the correct size

% Control input
ub = zeros(1,M);

% Finite difference scheme
for n = 1:M-1
    % Boundary condition
    u(1,n+1) = uL(n+1);
    u(N,n+1) = uR(n+1);
    
    % Control input
    u(fix(N/2),n+1) = ub(n);
    
    % Interior points
    for i = 2:N-1
        u(i,n+1) = u(i,n) + alpha*dt/(dx^2)*(u(i+1,n) - 2*u(i,n) + u(i-1,n));
    end
end

% Plot results
[X,T] = meshgrid(x,t);
surf(X,T,transpose(u))
xlabel('x')
ylabel('t')
zlabel('u')
