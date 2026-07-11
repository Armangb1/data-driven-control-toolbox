function blkStruct = slblocks
%SLBLOCKS Register the Data-Driven Control Toolbox in the Simulink Library Browser.
%   Each algorithm category is its own library file (ddc_*_lib.slx); they
%   are grouped here as children of a single "Data-Driven Control
%   Toolbox" node in the Library Browser (multiple Browser entries
%   sharing the same top-level Name are grouped by Simulink into one
%   tree node with each library as a child).

    blkStruct.Name = 'Data-Driven Control Toolbox';
    blkStruct.OpenFcn = 'ddc_common_lib';
    blkStruct.MaskInitialization = '';

    entries = { ...
        'ddc_common_lib', 'Common Utilities'; ...
        'ddc_deepc_lib',  'DeePC'; ...
        'ddc_mfac_lib',   'MFAC'; ...
        'ddc_ufc_lib',    'Unfalsified Switching'; ...
        'ddc_spsa_lib',   'SPSA'; ...
        'ddc_str_lib',    'STR Baselines' ...
    };

    Browser = repmat(struct('Library', '', 'Name', '', 'IsFlat', 0), size(entries, 1), 1);
    for i = 1:size(entries, 1)
        Browser(i).Library = entries{i, 1};
        Browser(i).Name = ['Data-Driven Control Toolbox/' entries{i, 2}];
        Browser(i).IsFlat = 0;
    end

    blkStruct.Browser = Browser;
end
