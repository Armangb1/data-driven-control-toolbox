function mlappinstallFile = packageApp()
%PACKAGEAPP Package VRFTDesignApp as a standalone .mlappinstall file.
%   run('tools/packageApp.m')
%
%   Produces build/VRFTDesignApp.mlappinstall in the repository root.
%   The app is self-contained: its launcher and the full ddc.vrft package
%   (with +ddc/+vrft namespace preserved) are bundled so the app runs
%   standalone from the MATLAB Apps gallery without a separately installed
%   toolbox.
%
%   Build metadata (author, version, summary) is derived from CITATION.cff
%   and CHANGELOG.md so the app stays in sync with the toolbox release.

    here = fileparts(mfilename('fullpath'));
    root = fullfile(here, '..');
    toolboxDir = fullfile(root, 'toolbox');
    vrftDir = fullfile(toolboxDir, '+ddc', '+vrft');

    buildDir = fullfile(root, 'build');
    if ~exist(buildDir, 'dir'); mkdir(buildDir); end

    version = readVersion(fullfile(root, 'CHANGELOG.md'));
    [author, email] = readAuthor(fullfile(root, 'CITATION.cff'));

    appName = 'VRFTDesignApp';
    mlappinstallFile = fullfile(buildDir, [appName, '.mlappinstall']);

    % --- Stage sources preserving the +ddc/+vrft package structure --------
    % The Apps packaging service copies resource files relative to each
    % file's containing folder, so placing the package under a staging root
    % and adding the +ddc/+vrft files as resources keeps the namespace
    % intact inside the packaged app.
    stage = fullfile(tempdir, 'ddc_mlappinstall_stage');
    if exist(stage, 'dir'); rmdir(stage, 's'); end
    pkgDir = fullfile(stage, '+ddc', '+vrft');
    mkdir(pkgDir);
    srcFiles = dir(fullfile(vrftDir, '*.m'));
    mainFile = fullfile(vrftDir, 'vrftDesignAppLauncher.m');
    for i = 1:numel(srcFiles)
        copyfile(fullfile(vrftDir, srcFiles(i).name), pkgDir);
    end
    copyfile(mainFile, fullfile(stage, 'vrftDesignAppLauncher.m'));

    % --- Package via the Apps packaging service (no GUI) ------------------
    import com.mathworks.toolbox.apps.services.AppsPackagingService;
    closeProjectSafe(appName); % release any project open in a previous session
    prjFile = fullfile(buildDir, [appName, '.prj']);
    if exist(prjFile, 'file'); delete(prjFile); end
    key = AppsPackagingService.createAppsProject(char(buildDir), appName);
    cleanup = onCleanup(@() closeProjectSafe(key));

    AppsPackagingService.setAuthorName(char(key), author);
    AppsPackagingService.setEmail(char(key), email);
    AppsPackagingService.setCompany(char(key), 'DataDrivenControlToolbox');
    AppsPackagingService.setSummary(char(key), ...
        'Interactive Virtual Reference Feedback Tuning (VRFT) controller design.');
    AppsPackagingService.setDescription(char(key), ...
        'Tabbed UIFigure app wrapping ddc.vrft.vrftDesign for interactive, offline VRFT controller design. Self-contained and independent of the toolbox install.');
    AppsPackagingService.setVersion(char(key), version);
    AppsPackagingService.setOutputFolderAsFile(char(key), java.io.File(buildDir));
    AppsPackagingService.addMainFile(char(key), fullfile(stage, 'vrftDesignAppLauncher.m'));

    stagedFiles = dir(fullfile(pkgDir, '*.m'));
    resourcePaths = arrayfun(@(s) fullfile(pkgDir, s.name), stagedFiles, 'UniformOutput', false);
    javaPaths = javaArray('java.lang.String', numel(resourcePaths));
    for i = 1:numel(resourcePaths)
        javaPaths(i) = java.lang.String(resourcePaths{i});
    end
    AppsPackagingService.addResourceFile(char(key), javaPaths);

    pause(1); % let the packaging backend register the added files
    AppsPackagingService.packageProject(char(key));
    pause(2); % packaging runs asynchronously; allow it to finish writing

    if ~exist(mlappinstallFile, 'file')
        error('ddc:packageApp:NoOutput', ...
            'AppsPackagingService reported success but %s was not created.', mlappinstallFile);
    end

    % --- Cleanup ----------------------------------------------------------
    clear cleanup;
    closeProjectSafe(key);
    if exist(stage, 'dir'); rmdir(stage, 's'); end

    fprintf('Packaged app: %s\n', mlappinstallFile);
end

function closeProjectSafe(key)
    import com.mathworks.toolbox.apps.services.AppsPackagingService;
    try
        AppsPackagingService.closeProject(char(key), false);
    catch
        % ignore if already closed
    end
end

function version = readVersion(changelogFile)
    txt = fileread(changelogFile);
    tokens = regexp(txt, '^\s*##\s+\[v?([0-9]+\.[0-9]+\.[0-9]+[^\]]*)\]', 'lineanchors', 'tokens');
    if ~isempty(tokens)
        version = strtrim(tokens{1}{1});
        return;
    end
    version = '0.1.0';
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
