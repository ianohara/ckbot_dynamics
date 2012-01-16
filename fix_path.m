% Add all of the directories in the current folder to the matlab path
% variable

ls = dir();

for i = 1:length(ls)
   if (~ls(i).isdir) 
       continue;
   end
   % Ignore '.' '..' and any hidden files
   if (strcmp(ls(i).name(1), '.')) 
       continue;
   end
   addpath(strcat(pwd,'/',ls(i).name));
end

clear all