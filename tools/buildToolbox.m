function buildToolbox()
%BUILDTOOLBOX Package the toolbox as a .mltbx file via Toolbox Packager API.
%   run('tools/buildToolbox.m')
%
%   Produces a .mltbx file in the repository root suitable for distribution
%   via File Exchange, GitHub Releases, or manual install.

    here = fileparts(mfilename('fullpath'));
    root = fullfile(here, '..');
    toolboxDir = fullfile(root, 'toolbox');
    outputMltbx = fullfile(root, 'Data-Driven-Control-Toolbox.mltbx');
    identifier = 'com-datadrivencontrol-toolbox';

    opts = matlab.addons.toolbox.ToolboxOptions(toolboxDir, identifier, ...
        'ToolboxName',        'Data-Driven Control Toolbox', ...
        'ToolboxVersion',     '0.1.0', ...
        'Summary',            'Data-driven and adaptive control algorithms (DeePC, MFAC, UFC, SPSA, STR, VRFT) with a Simulink block library.', ...
        'AuthorName',         'DataDrivenControlToolbox contributors', ...
        'ToolboxMatlabPath',  {toolboxDir, fullfile(toolboxDir, 'lib')}, ...
        'OutputFile',         outputMltbx);

    fprintf('Packaging Data-Driven Control Toolbox v0.1.0 ...\n');
    matlab.addons.toolbox.packageToolbox(opts);
    fprintf('Done: %s\n', outputMltbx);
end
