function buildLibraries()
%BUILDLIBRARIES Regenerate the single Data-Driven Control Toolbox Simulink library.
%   Produces ddc_lib.slx with all blocks organized into category subsystems
%   (Common Utilities, DeePC, MFAC, UFC, SPSA, STR) so they appear as a
%   single browsable node in the Simulink Library Browser.
%
%   Usage:
%       run('tools/buildLibraries.m')

    here = fileparts(mfilename('fullpath'));
    toolboxPath = fullfile(here, '..', 'toolbox');
    libPath = fullfile(toolboxPath, 'lib');
    addpath(toolboxPath);

    if ~exist(libPath, 'dir')
        mkdir(libPath);
    end

    libName = 'ddc_lib';
    libFile = fullfile(libPath, [libName '.slx']);
    if bdIsLoaded(libName)
        close_system(libName, 0);
    end
    if exist(libFile, 'file')
        delete(libFile);
    end
    new_system(libName, 'Library');
    open_system(libName);

    % --- Category: Common Utilities ---
    buildSubsystem(libName, 'Common Utilities', { ...
        struct('Name','Data Buffer',              'System','ddc.common.DataBuffer'), ...
        struct('Name','RLS Estimator',            'System','ddc.common.RLSEstimator'), ...
        struct('Name','Excitation Signal\nGenerator','System','ddc.common.ExcitationSignalGenerator'), ...
    });

    % --- Category: DeePC ---
    buildSubsystem(libName, 'DeePC', { ...
        struct('Name','DeePC Controller',          'System','ddc.deepc.DeePCController'), ...
    });

    % --- Category: MFAC ---
    buildSubsystem(libName, 'MFAC', { ...
        struct('Name','MFAC Controller',           'System','ddc.mfac.MFACController'), ...
    });

    % --- Category: UFC ---
    ufcBlocks = { ...
        struct('Name','Candidate Controller Bank',   'System','ddc.ufc.CandidateControllerBank'), ...
        struct('Name','Unfalsified Switching\nController','System','ddc.ufc.UnfalsifiedSwitchingController'), ...
        struct('Name','Multimodel Switching\nController', 'System','ddc.ufc.MultimodelSwitchingController'), ...
    };
    buildSubsystem(libName, 'Unfalsified Switching', ufcBlocks);
    buildUfcMaskedSubsystem(libName);

    % --- Category: SPSA ---
    buildSubsystem(libName, 'SPSA', { ...
        struct('Name','SPSA Optimizer',            'System','ddc.spsa.SPSAOptimizer'), ...
    });

    % --- Category: STR ---
    buildSubsystem(libName, 'STR Baselines', { ...
        struct('Name','Direct STR Controller',     'System','ddc.str.DirectSTRController'), ...
        struct('Name','Indirect STR Controller',   'System','ddc.str.IndirectSTRController'), ...
    });

    Simulink.BlockDiagram.arrangeSystem(libName);
    save_system(libName, libFile);
    close_system(libName, 0);

    fprintf('Built %s\n', libFile);
end

% ------------------------------------------------------------------
function buildSubsystem(libName, catName, blocks)
    catPath = [libName '/' catName];
    add_block('built-in/SubSystem', catPath, 'Position', [30 30 260 60]);
    set_param(catPath, 'BackgroundColor', 'lightBlue');
    Simulink.SubSystem.deleteContents(catPath);

    y = 30;
    for i = 1:numel(blocks)
        blk = blocks{i};
        fullBlk = [catPath '/' blk.Name];
        add_block('simulink/User-Defined Functions/MATLAB System', fullBlk, ...
            'Position', [30 y 220 y+50]);
        set_param(fullBlk, 'System', blk.System);
        y = y + 80;
    end
end

% ------------------------------------------------------------------
function buildUfcMaskedSubsystem(libName)
    ssName = [libName '/' sprintf('Unfalsified Switching\n(P-Bank, ready-to-use)')];
    add_block('built-in/SubSystem', ssName, 'Position', [30 30 260 130]);
    set_param(ssName, 'BackgroundColor', 'green');
    open_system(ssName);
    Simulink.SubSystem.deleteContents(ssName);

    add_block('simulink/Sources/In1', [ssName '/r'],  'Position', [30 30 60 50]);
    add_block('simulink/Sources/In1', [ssName '/y'],  'Position', [30 100 60 120]);

    bankBlk = [ssName '/Candidate Bank'];
    add_block('simulink/User-Defined Functions/MATLAB System', bankBlk, ...
        'Position', [120 20 260 70]);
    set_param(bankBlk, 'System', 'ddc.ufc.CandidateControllerBank');
    set_param(bankBlk, 'Gains', 'Gains');

    swBlk = [ssName '/Switching Logic'];
    add_block('simulink/User-Defined Functions/MATLAB System', swBlk, ...
        'Position', [320 20 460 90]);
    set_param(swBlk, 'System', 'ddc.ufc.UnfalsifiedSwitchingController');
    set_param(swBlk, 'Gains', 'Gains');

    add_block('simulink/Sinks/Out1', [ssName '/uSelected'],  'Position', [520 20 550 40]);
    add_block('simulink/Sinks/Out1', [ssName '/activeIndex'],'Position', [520 50 550 70]);
    add_block('simulink/Sinks/Out1', [ssName '/costs'],      'Position', [520 80 550 100]);

    add_line(ssName, 'r/1', 'Candidate Bank/1');
    add_line(ssName, 'y/1', 'Candidate Bank/2');
    add_line(ssName, 'Candidate Bank/1', 'Switching Logic/1');
    add_line(ssName, 'Switching Logic/1', 'uSelected/1');
    add_line(ssName, 'Switching Logic/2', 'activeIndex/1');
    add_line(ssName, 'Switching Logic/3', 'costs/1');

    mask = Simulink.Mask.create(ssName);
    mask.Type = 'Unfalsified Switching (P-Bank)';
    mask.Description = sprintf(['Ready-to-use unfalsified adaptive switching control ' ...
        'block: a bank of proportional candidate controllers plus the cost-based ' ...
        'switching logic, pre-wired. Set Gains to the vector of candidate ' ...
        'proportional gains.']);
    p = mask.addParameter('Type', 'edit', 'Prompt', 'Candidate gains', ...
        'Name', 'Gains', 'Value', '[0.5; 1; 2]');
    p.Evaluate = 'on'; %#ok<NASGU>

    close_system(ssName);
end
