function cms = cm_vectors(chain)
% For a certain chain state, calculate r_cm from the
% origin and return them in an array that has size
% size(cms) = [3, len(chain)];

cms = zeros(3, length(chain));

if (isfield(chain(1), 'init_rotation'))
    R = chain(1).init_rotation;
else
    R = eye(3);
end

r_base = [0; 0; 0]; % Base of first link starts at origin

for i = 1:length(chain)
    R = R*rotZ(chain(i).q)*chain(i).R_jts; % from i to inertial
    r_cm = r_base + R*(-chain(i).r_im1);
    r_base = r_cm + R*(chain(i).r_ip1); % Base of next
    cms(:,i) = r_cm;
end
end