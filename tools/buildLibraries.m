function buildLibraries()
%BUILDLIBRARIES Regenerate all Data-Driven Control Toolbox Simulink libraries.
%   Since .slx files are binary, this script is the source of truth for
%   how each library is constructed; re-run it any time a block needs to
%   be added/changed rather than hand-editing the .slx directly (though
%   hand-editing in the Simulink editor and re-saving is also fine -- the
%   two are meant to stay consistent, this script is primarily for
%   reproducibility and CI regeneration).
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

    buildCommonLib(libPath);
    buildDeepcLib(libPath);
    buildMfacLib(libPath);
    buildStrLib(libPath);
    buildUfcLib(libPath);
    buildSpsaLib(libPath);

    fprintf('All Simulink libraries built in %s\n', libPath);
    fprintf(['Note: there is no single aggregator .slx -- each ddc_*_lib is ' ...
        'registered as its own node in the Simulink Library Browser, grouped ' ...
        'under "Data-Driven Control Toolbox" by toolbox/lib/slblocks.m (cross-' ...
        'library block links are not reliably preserved by Simulink, so a ' ...
        'true nested aggregator library was intentionally avoided).\n']);
end

% ------------------------------------------------------------------
function lib = newLibrary(name, libPath)
    fullName = fullfile(libPath, name);
    if bdIsLoaded(name)
        close_system(name, 0);
    end
    if exist([fullName '.slx'], 'file')
        delete([fullName '.slx']);
    end
    new_system(name, 'Library');
    open_system(name);
    lib = name; % use the model name (char) as the handle throughout
end

function addSystemBlock(lib, blockName, className, position, varargin)
    blk = add_block('simulink/User-Defined Functions/MATLAB System', ...
        [lib '/' blockName], 'Position', position);
    set_param(blk, 'System', className);
    for i = 1:2:numel(varargin)
        set_param(blk, varargin{i}, varargin{i+1});
    end
end

function saveAndClose(lib, libPath)
    Simulink.BlockDiagram.arrangeSystem(lib);
    save_system(lib, fullfile(libPath, lib));
    close_system(lib, 0);
end

% ------------------------------------------------------------------
function buildCommonLib(libPath)
    name = 'ddc_common_lib';
    lib = newLibrary(name, libPath);

    addSystemBlock(lib, 'Data Buffer', 'ddc.common.DataBuffer', [30 30 160 80]);
    addSystemBlock(lib, 'RLS Estimator', 'ddc.common.RLSEstimator', [30 120 160 170]);
    addSystemBlock(lib, sprintf('Excitation Signal\nGenerator'), 'ddc.common.ExcitationSignalGenerator', [30 210 160 260]);

    saveAndClose(lib, libPath);
end

% ------------------------------------------------------------------
function buildDeepcLib(libPath)
    name = 'ddc_deepc_lib';
    lib = newLibrary(name, libPath);

    addSystemBlock(lib, 'DeePC Controller', 'ddc.deepc.DeePCController', [30 30 200 90]);

    saveAndClose(lib, libPath);
end

% ------------------------------------------------------------------
function buildMfacLib(libPath)
    name = 'ddc_mfac_lib';
    lib = newLibrary(name, libPath);

    addSystemBlock(lib, 'MFAC Controller', 'ddc.mfac.MFACController', [30 30 190 90]);

    saveAndClose(lib, libPath);
end

% ------------------------------------------------------------------
function buildStrLib(libPath)
    name = 'ddc_str_lib';
    lib = newLibrary(name, libPath);

    addSystemBlock(lib, 'Direct STR Controller', 'ddc.str.DirectSTRController', [30 30 210 90]);
    addSystemBlock(lib, 'Indirect STR Controller', 'ddc.str.IndirectSTRController', [30 120 210 180]);

    saveAndClose(lib, libPath);
end

% ------------------------------------------------------------------
function buildSpsaLib(libPath)
    name = 'ddc_spsa_lib';
    lib = newLibrary(name, libPath);

    addSystemBlock(lib, 'SPSA Optimizer', 'ddc.spsa.SPSAOptimizer', [30 30 190 90]);

    saveAndClose(lib, libPath);
end

% ------------------------------------------------------------------
function buildUfcLib(libPath)
    name = 'ddc_ufc_lib';
    lib = newLibrary(name, libPath);

    addSystemBlock(lib, 'Candidate Controller Bank', 'ddc.ufc.CandidateControllerBank', [30 30 220 90]);
    addSystemBlock(lib, sprintf('Unfalsified Switching\nController'), 'ddc.ufc.UnfalsifiedSwitchingController', [30 130 220 190]);
    addSystemBlock(lib, sprintf('Multimodel Switching\nController'), 'ddc.ufc.MultimodelSwitchingController', [30 230 220 290]);

    buildUfcMaskedSubsystem(lib);

    saveAndClose(lib, libPath);
end

function buildUfcMaskedSubsystem(lib)
    % A ready-to-use masked subsystem wiring CandidateControllerBank ->
    % UnfalsifiedSwitchingController together, exposing a single "Gains"
    % mask parameter shared by both underlying blocks.
    ssName = [lib '/' sprintf('Unfalsified Switching\n(P-Bank, ready-to-use)')];
    add_block('built-in/Subsystem', ssName, 'Position', [300 30 480 190]);
    open_system(ssName);

    % Clear default contents.
    Simulink.SubSystem.deleteContents(ssName);

    add_block('simulink/Sources/In1', [ssName '/r'], 'Position', [30 30 60 50]);
    add_block('simulink/Sources/In1', [ssName '/y'], 'Position', [30 100 60 120]);

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

    add_block('simulink/Sinks/Out1', [ssName '/uSelected'], 'Position', [520 20 550 40]);
    add_block('simulink/Sinks/Out1', [ssName '/activeIndex'], 'Position', [520 50 550 70]);
    add_block('simulink/Sinks/Out1', [ssName '/costs'], 'Position', [520 80 550 100]);

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
