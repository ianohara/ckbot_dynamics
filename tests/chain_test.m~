function success = chain_test()
% Create some simple chains, plot them, and run any tests
% we can to check their validity
%
% ARGUMENTS
%  None
%
% RETURNS
%  Success - 1 if all tests pass, 0 if not

success = 0;

sets = {};
sets.draw_vels = 1;
sets.draw_geom_vecs = 1;

chain = [
    new_link('HT1', 'q',pi/2,  'qd',0);
    new_link('HT2', 'q',-pi/4, 'qd',0);
    new_link('HT1', 'q', pi/12,'qd',.1);
    new_link('HT2', 'q',pi/6,  'qd',.1);
    new_link('HT1', 'q', pi/10,'qd',.1);
    new_link('HT1', 'q', pi/4, 'qd',.1);
    new_link('HT2', 'q', pi/3, 'qd',.1);
    
];

close all;
figure;
hold on;
grid on;
axis equal;
xlabel('X [m]','FontSize',14);
ylabel('Y [m]','FontSize',14);
zlabel('Z [m]','FontSize',14);
view(45,45);


draw_chain(chain);


if (sets.draw_vels)
    draw_lin_vels(chain);
end
if (sets.draw_geom_vecs)
    draw_geom_vecs(chain);
end
end