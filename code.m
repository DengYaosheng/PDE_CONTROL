% 多智能体系统的编队控制（模型预测控制）

% 定义系统参数
N = 6; % 智能体数量
targetPositions = [0, 1, 2, 3, 4, 5]; % 目标位置，直线编队

% 定义控制器参数
horizon = 10; % 预测时域
Q = 1; % 状态权重
R = 1; % 控制输入权重

% 初始化智能体的位置和速度
x = zeros(N, 1); % 位置
v = zeros(N, 1); % 速度

% 模拟多智能体系统的运动
tspan = 0:0.1:10; % 时间范围
[t, y] = ode45(@dynamics, tspan, [x; v]);

% 绘制结果
figure;
for i = 1:N
    plot(t, y(:, i), 'LineWidth', 1.5);
    hold on;
end
xlabel('时间');
ylabel('位置');
legend('Agent 1', 'Agent 2', 'Agent 3', 'Agent 4', 'Agent 5', 'Agent 6');
title('多智能体系统的直线编队');

% 动力学函数
function dydt = dynamics(t, y)
    x = y(1:N); % 位置
    v = y(N+1:end); % 速度
    
    % 计算每个智能体的速度控制量
    u = controlLaw(x, v);
    
    % 更新速度和位置
    dxdt = v;
    dvdt = u;
    
    dydt = [dxdt; dvdt];
end

% 控制器设计
function u = controlLaw(x, v)
    % 状态和控制输入的维度
    nx = N; % 状态维度
    nu = N-1; % 控制输入维度
    
    % 构建模型预测控制问题
    model = buildModel(nx, nu);
    problem = buildProblem(model, x, v);
    
    % 求解模型预测控制问题
    solverOptions = struct('Solver', 'quadprog');
    [uOpt, ~, ~, ~] = mpcsolve(problem, solverOptions);
    
    % 获取最优控制输入
    u = uOpt(:, 1);
end

% 构建模型预测控制问题
function model = buildModel(nx, nu)
    model.A = eye(nx);
    model.B = [-eye(nu-1), zeros(nu-1, 1); zeros(1, nu)];
    model.Q = Q * eye(nx);
    model.R = R * eye(nu);
end

% 构建模型预测控制问题
function problem = buildProblem(model, x0, v0)
    nx = size(model.A, 1);
    nu = size(model.B, 2);
    
    % 构建约束矩阵
    A = [model.A, model.B; zeros(nu, nx), eye(nu)];
    b = [targetPositions - x0; zeros(nu, 1)];
    lb = [-Inf(nx, 1); -Inf(nu, 1)];
    ub = [Inf(nx, 1); Inf(nu, 1)];
    
    % 构建目标函数
    Q = blkdiag(kron(eye(N), model.Q), zeros(nu));
    R = kron(eye(N-1), model.R);
    f = -Q * [x0; v0; zeros(nu, 1)] - R * targetPositions';
    
    % 构建模型预测控制问题
    problem.H = 2 * (A' * Q * A + R);
    problem.f = f;
    problem.Aineq = -eye(nx+nu);
    problem.bineq = [lb; -ub];
    problem.Aeq = [];
    problem.beq = [];
    problem.lb = [];
    problem.ub = [];
end
