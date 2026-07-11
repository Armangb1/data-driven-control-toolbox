function devPath()
%DEVPATH Add toolbox source folders to the MATLAB path for local development.
%   Run this once at the start of a development session:
%       run('tools/devPath.m')
%
%   Do NOT add toolbox/+ddc itself to the path -- only its parent
%   (toolbox/) must be on the path so that the ddc.* package namespace
%   resolves correctly.  toolbox/lib must also be on the path so that
%   Simulink can find the ddc_*_lib.slx library files by name.

    here = fileparts(mfilename('fullpath'));
    root = fullfile(here, '..');
    addpath(fullfile(root, 'toolbox'));
    addpath(fullfile(root, 'toolbox', 'lib'));
    fprintf('Data-Driven Control Toolbox dev path set (root: %s)\n', root);
end
