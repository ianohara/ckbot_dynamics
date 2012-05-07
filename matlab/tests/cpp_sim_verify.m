function sim = cpp_sim_verify(varargin)
% Run a configuration with the control
% inputs suggested by the C++ planner
% and see if the results match.

result_file = 'results/results_upright_600s.txt';
step_splits = 3;
overlay = 0;

% Note: All options are 'key', value options. 
if (mod(nargin, 2) ~= 0)
    error('Number of arguments must be even (''key'', ''value'' pairs)');
end

for i = 1:2:nargin-1
   opt = varargin{i};
   val = varargin{i+1};
   switch opt
       case 'file'
           result_file = val;
       case 'splits'
           step_splits = val;
       case 'overlay'
           overlay = val;
       otherwise
           error('Unknown option: %s', opt);
   end
end


fprintf('Reading control from ''%s''.', result_file);
result_json = parse_json(fileread(result_file));

json = result_json{1};

N = size(json.chain, 2);
chain(N) = new_link('HT1');
chain = chain';

for i=1:N
    jc = json.chain{i};
    f_j_a = cell2mat([jc.f_jt_axis{1}; jc.f_jt_axis{2}; jc.f_jt_axis{3}]);
    r_ip1 = cell2mat([jc.r_ip1{1}; jc.r_ip1{2}; jc.r_ip1{3}]);
    r_im1 = cell2mat([jc.r_im1{1}; jc.r_im1{2}; jc.r_im1{3}]);
    I_cm = [cell2mat(jc.I_cm{1}); cell2mat(jc.I_cm{2}); cell2mat(jc.I_cm{3})];
    R_jts = [cell2mat(jc.R_jts{1}); cell2mat(jc.R_jts{2}); cell2mat(jc.R_jts{3})];
    m = jc.mass;
    init_rotation = [cell2mat(jc.init_rotation{1}); cell2mat(jc.init_rotation{2}); cell2mat(jc.init_rotation{3})];
    
    chain(i) = new_link('HT1');
    chain(i).damping = jc.damping;
    chain(i).forward_joint_axis = f_j_a;
    chain(i).r_ip1 = r_ip1;
    chain(i).r_im1 = r_im1;
    chain(i).I_cm = I_cm;
    chain(i).R_jts = R_jts;
    chain(i).init_rotation = init_rotation;
    chain(i).m = m;
end

% The control steps tend to be pretty long timewise, so 
% split them into equal parts for the simulator steps.
dt = zeros(size(json.control,2)*step_splits, 1);
T = zeros(N, size(json.control,2)*step_splits);

for i = 1:(size(json.control,2))
    expanded_steps = ((i-1)*step_splits + 1):((i-1)*step_splits+1)+step_splits-1;
    dt(expanded_steps) = repmat(json.control{i}.dt/step_splits,step_splits,1);
    T_vec = cell2mat(json.control{i}.control)';
    T(:,expanded_steps) = repmat(T_vec, 1, step_splits);
end


t_sim = sum(dt);
num_s = length(dt);
q0 = zeros(N,1);
qd0 = zeros(N,1);

sim = new_sim('steps', num_s, 'sim_time', t_sim, 'chain', chain, ...
    'torques', T, 'q0', q0, 'qd0', qd0,'dt', dt, 'integrator', @rk4step);

tic;
sim = run_sim(sim, num_s);
toc;

draw_sim(sim,varargin{:})

if (overlay)
   cpp_len = length(json.control);
   q_cpp = zeros(N,cpp_len);
   qd_cpp = zeros(N, cpp_len);
   t_cpp = zeros(1, cpp_len);
   for i = 1:cpp_len
       end_time = json.control{i}.end_time;
       t_cpp(i) = end_time;
       s_mat = cell2mat(json.control{i}.end_state)'; % Column.
       q_cpp(:,i) = s_mat(1:N);
       qd_cpp(:,i) = s_mat(N+1:2*N);
   end
   
   legend_strs = {};
   for i=1:N
       legend_strs{end+1} = sprintf('Link %d', i);
   end
   
   o_fig = figure();
   subplot(211);
   grid on;
   hold on;
   title('Time History of C++ Planned System','FontSize', 14);
   xlabel('Time [s]','FontSize', 14);
   ylabel('Joint Angle [rad]','FontSize',14);
   plot(t_cpp,q_cpp,'LineWidth',2);
   legend(legend_strs);
   
   
   subplot(212);
   grid on;
   hold on;
   xlabel('Time [s]', 'FontSize', 14);
   ylabel('Joint Angular Velocity [rad/s]', 'FontSize', 14);
   plot(t_cpp, qd_cpp,'LineWidth',2);
   legend(legend_strs);
   
end

end