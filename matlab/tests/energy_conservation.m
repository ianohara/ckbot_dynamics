function sim = energy_conservation(varargin)
% For any simulation configuration where the
% damping for all modules is 0.0 and there are
% no input torques, the total system energy
% should be constant.
g = 9.81;
N = 1;
chain(N) = new_link('HT1');
chain = chain';
chain(1) = new_link('HT1', 'rotate', rotX(pi/2));

for i=2:N-1
   chain(i) = new_link('HT1');
end

for i=1:N
    chain(i).damping = 0.0;
end

t_sim = 2; % Time length to simulate
dt = 0.001;
num_s = t_sim/dt;

q0 = zeros(N,1);
q0(1) = 0;
qd0 = zeros(N,1);
%qd0(1) = 1;

T = zeros(N, num_s);

sim = new_sim('steps', num_s, 'sim_time', t_sim, 'chain', chain, ...
    'torques', T, 'q0', q0, 'qd0', qd0, 'dt', dt, 'integrator', @rk4Step);

tic;
sim = run_sim(sim, num_s);
toc;

% Calculate the pe, ke, and total energy at each timestep
energies = zeros(num_s, 3); % total, ke, pe columns.  one row per timestep.

for i=1:sim.s
   ch = propogate_angles_and_rates(sim.chain, sim.q(:,i), sim.qd(:,i));
   vels = forward_kinematics(ch);
   cms = cm_vectors(ch);
   if (length(ch) > 1)
        r_cm_sys = (sum(cms')./length(ch))';
   else
       r_cm_sys = cms;
   end
   mass_sys = length(ch)*ch(1).m;
   ke = 0.0;
   pe = 0.0; %mass_sys*g*r_cm_sys(3);
   tot = 0.0;
   for j=1:length(ch)
       omeg = vels(:,3,j);
       v_cm = cm_vel(ch,j);
       %v_cm = vels(:,2,j);
       ke = ke + (1/2)*omeg'*ch(j).I_cm*omeg + (1/2)*ch(j).m*(v_cm'*v_cm);
       pe = pe + ch(j).m*g*cms(3,j);  % Only z portion in pe
   end
   energies(i,:) = [(ke+pe)'; ke'; pe'];
end

f = figure();
subplot(211);
grid on;
hold on;
title(sprintf('Total energy, PE, and KE of system over time (%d modules)', length(chain)), 'FontSize', 14)
xlabel('Time [s]', 'LineWidth', 14);
ylabel('Energy [J]', 'LineWidth', 14);
plot(sim.t, energies(:,1), 'b-', 'LineWidth', 2);
plot(sim.t, energies(:,2), 'ro');
plot(sim.t, energies(:,3), 'go');
legend('Total', 'Ke', 'Pe');

subplot(212);
grid on;
hold on;
xlabel('Time [s]', 'LineWidth', 14);
ylabel('Difference in Energy [J]', 'LineWidth', 14);
plot(sim.t, energies(1,1)*ones(num_s,1) - energies(:,1), 'o');

end