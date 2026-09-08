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
    version = readVersion(fullfile(root, 'CHANGELOG.md'));
    [author, ~] = readAuthor(fullfile(root, 'CITATION.cff'));

    % Build the standalone VRFTDesignApp installer and stage it into the
    % toolbox so the .mltbx also ships it.
    mlappinstall = packageApp();
    appsDir = fullfile(toolboxDir, 'apps');
    if ~exist(appsDir, 'dir'); mkdir(appsDir); end
    copyfile(mlappinstall, fullfile(appsDir, [getAppName(mlappinstall), '.mlappinstall']), 'f');

    opts = matlab.addons.toolbox.ToolboxOptions(toolboxDir, identifier, ...
        'ToolboxName',        'Data-Driven Control Toolbox', ...
        'ToolboxVersion',     version, ...
        'Summary',            'Data-driven and adaptive control algorithms (DeePC, MFAC, UFC, SPSA, STR, VRFT) with a Simulink block library and a VRFT design app.', ...
        'AuthorName',         author, ...
        'ToolboxMatlabPath',  {toolboxDir, fullfile(toolboxDir, 'lib')}, ...
        'OutputFile',         outputMltbx);

    fprintf('Packaging Data-Driven Control Toolbox v%s ...\n', version);
    matlab.addons.toolbox.packageToolbox(opts);
    fprintf('Done: %s\n', outputMltbx);
end

function name = getAppName(mlappinstallFull)
    [~, name] = fileparts(mlappinstallFull);
end

function version = readVersion(changelogFile)
    txt = fileread(changelogFile);
    tokens = regexp(txt, '^\s*##\s+\[v?([0-9]+\.[0-9]+\.[0-9]+[^\]]*)\]', 'lineanchors', 'tokens');
    if ~isempty(tokens)
        version = strtrim(tokens{1}{1});
    else
        version = '0.1.0';
    end
end

function [author, email] = readAuthor(citationFile)
    txt = fileread(citationFile);
    authorTokens = regexp(txt, 'given-names:\s*"([^"]+)"\s+family-names:\s*"([^"]+)"', 'tokens', 'once');
    if isempty(authorTokens)
        authorTokens = regexp(txt, 'family-names:\s*"([^"]+)"\s+given-names:\s*"([^"]+)"', 'tokens', 'once');
    end
    if isempty(authorTokens)
        author = 'DataDrivenControlToolbox contributors';
    else
        author = [authorTokens{2}, ' ', authorTokens{1}];
    end
    emailTokens = regexp(txt, 'email:\s*"([^"]+)"', 'tokens', 'once');
    if isempty(emailTokens)
        email = '';
    else
        email = emailTokens{1};
    end
end
