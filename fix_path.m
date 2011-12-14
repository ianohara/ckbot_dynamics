% Add all of the directories in the current folder to the matlab path
% variable

ls = dir();

for i = 1:length(ls)
   if (~ls(i).isdir) 
       continue;
   end
   if (strcmp(ls(i).name, '.') || strcmp(ls(i).name, '..'))
       continue;
   end
   addpath(ls(i).name);
end

clear all