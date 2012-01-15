function chain = chain_test()
% Create some simple chains, plot them, and run any tests
% we can to check their validity
%
% ARGUMENTS
%  None
%
% RETURNS
%  chain - returns the last chain made (or a chain made...)
sets = {};
sets.draw_geom = 1;
sets.draw_vels = 1;
sets.draw_geom_vecs = 1;

chain1 = [
    new_link('HT_tail_cap', 'q',0,  'qd',0, 'rotate', rotY(pi/2));
    new_link('HT1', 'q',-pi/4, 'qd',0);
    new_link('HT1', 'q', pi/12,'qd',.4);
    new_link('HT2', 'q',pi/6,  'qd',.1);
    new_link('HT1', 'q', pi/10,'qd',0.5);
    new_link('HT1', 'q', pi/4, 'qd',.1);
    new_link('HT_head_cap', 'q', pi/3, 'qd',.1);
    
];

% chain2 = [
%     new_link('HT1', 'q',pi/2,  'qd',0);
%     new_link('HT2', 'q',-pi/4, 'qd',0);
%     new_link('HT1', 'q', pi/12,'qd',.1);
%     new_link('HT2', 'q',pi/6,  'qd',.1);
%     new_link('HT1', 'q', pi/10,'qd',.1);
%     new_link('HT1', 'q', pi/4, 'qd',.1);
%     new_link('HT2', 'q', pi/3, 'qd',.1);
%     
% ];
% 
% chain3 = [
%     new_link('HT1', 'q',pi/2,  'qd',0);
%     new_link('HT2', 'q',-pi/4, 'qd',0);
%     new_link('HT1', 'q', pi/12,'qd',.1);
%     new_link('HT2', 'q',pi/6,  'qd',.1);
%     new_link('HT1', 'q', pi/10,'qd',.1);
%     new_link('HT1', 'q', pi/4, 'qd',.1);
%     new_link('HT2', 'q', pi/3, 'qd',.1);
%     
% ];
% 
% chain4 = [
%     new_link('HT_tail_cap', 'q',pi/2,  'qd',0);
%     new_link('HT2', 'q',-pi/4, 'qd',0);
%     new_link('HT1', 'q', pi/12,'qd',.1);
%     new_link('HT2', 'q',pi/6,  'qd',.1);
%     new_link('HT1', 'q', pi/10,'qd',.1);
%     new_link('HT1', 'q', pi/4, 'qd',.1);
%     new_link('HT_head_cap', 'q', pi/3, 'qd',.1);
    

close all;
figure;
hold on;
grid on;
axis equal;
xlabel('X [m]','FontSize',14);
ylabel('Y [m]','FontSize',14);
zlabel('Z [m]','FontSize',14);
view(45,45);
if (sets.draw_geom)
    draw_chain(chain1);
end
if (sets.draw_vels)
    draw_lin_vels(chain1);
end
if (sets.draw_geom_vecs)
    draw_geom_vecs(chain1);
end

% figure;
% hold on;
% grid on;
% axis equal;
% xlabel('X [m]','FontSize',14);
% ylabel('Y [m]','FontSize',14);
% zlabel('Z [m]','FontSize',14);
% view(45,45);
% if (sets.draw_geom)
%     draw_chain(chain2);
% end
% if (sets.draw_vels)
%     draw_lin_vels(chain2);
% end
% if (sets.draw_geom_vecs)
%     draw_geom_vecs(chain2);
% end
% 
% figure
% hold on;
% grid on;
% axis equal;
% xlabel('X [m]','FontSize',14);
% ylabel('Y [m]','FontSize',14);
% zlabel('Z [m]','FontSize',14);
% view(45,45);
% if (sets.draw_geom)
%     draw_chain(chain3);
% end
% if (sets.draw_vels)
%     draw_lin_vels(chain3);
% end
% if (sets.draw_geom_vecs)
%     draw_geom_vecs(chain3);
% end
% 
% figure;
% hold on;
% grid on;
% axis equal;
% xlabel('X [m]','FontSize',14);
% ylabel('Y [m]','FontSize',14);
% zlabel('Z [m]','FontSize',14);
% view(45,45);
% if (sets.draw_geom)
%     draw_chain(chain4);
% end
% 
% if (sets.draw_vels)
%     draw_lin_vels(chain4);
% end
% if (sets.draw_geom_vecs)
%     draw_geom_vecs(chain4);
% end

chain = chain1;
end