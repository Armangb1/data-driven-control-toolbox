function blkStruct = slblocks
%SLBLOCKS Register the Data-Driven Control Toolbox in the Simulink Library Browser.
%   Registers a single library (ddc_lib) containing all blocks organized
%   into category subsystems. The Simulink Library Browser shows it as one
%   top-level "Data-Driven Control Toolbox" entry; double-clicking opens
%   the subsystem folders (Common Utilities, DeePC, MFAC, etc.).

    blkStruct.Name     = 'Data-Driven Control Toolbox';
    blkStruct.OpenFcn  = 'ddc_lib';
    blkStruct.MaskInitialization = '';

    Browser.Library  = 'ddc_lib';
    Browser.Name     = 'Data-Driven Control Toolbox';
    Browser.IsFlat   = 0;   % 0 = show subsystem folders when double-clicked

    blkStruct.Browser = Browser;
end
