function sim = cpp_sim_verify()
% Run a configuration with the control
% inputs suggested by the C++ planner
% and see if the results match.

chain_3 = [
    new_link('HT1', 'rotate', rotY(pi/2));
    new_link('HT1');
    new_link('HT1');
    ];

chain_3(1).damping = 1.0;
chain_3(2).damping = 1.0;
chain_3(3).damping = 1.0;

N = size(chain_3, 1);

result_json = parse_json(fileread('results.txt'));

% The control steps tend to be pretty long timewise, so 
% split them into equal parts for the simulator steps.
step_splits = 3;
dt = zeros(size(result_json{1}.control,2)*step_splits, 1);
T = zeros(N, size(result_json{1}.control,2)*step_splits);

for i = 1:(size(result_json{1}.control,2))
    expanded_steps = ((i-1)*step_splits + 1):((i-1)*step_splits+1)+step_splits-1;
    dt(expanded_steps) = repmat(result_json{1}.control{i}.dt/3,step_splits,1);
    T_vec = cell2mat(result_json{1}.control{i}.control)';
    T(:,expanded_steps) = repmat(T_vec, 1, step_splits);
end


t_sim = sum(dt);
num_s = length(dt);
q0 = zeros(N,1);
qd0 = zeros(N,1);

sim = new_sim('steps', num_s, 'sim_time', t_sim, 'chain', chain_3, ...
    'torques', T, 'q0', q0, 'qd0', qd0,'dt', dt);

end