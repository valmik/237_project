disp('Initializing 237 Project')
cur = fileparts(mfilename('fullpath'));

if isempty(cur)
    cur = pwd;
end
disp('adding third-party packages');
addpath(genpath(fullfile(cur,'third_party','ipopt')));
% addpath(genpath(fullfile(cur,'third_party','Matlab-NestedMap')));

disp('Initialized')