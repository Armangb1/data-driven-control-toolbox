function result = runVrftDesign(u, y, Md, basis, prefilter)
%RUNVRFTDESIGN Error-safe wrapper around ddc.vrft.vrftDesign for app use.
%   RESULT = ddc.vrft.runVrftDesign(U, Y, MD, BASIS, PREFILTER) calls
%   ddc.vrft.vrftDesign(U, Y, MD, BASIS, 'Prefilter', PREFILTER) (or with
%   the default prefilter when PREFILTER is empty) and converts any error
%   it raises into a structured, non-throwing result so that
%   ddc.vrft.VRFTDesignApp can surface failures as a status message /
%   dialog instead of letting a raw MATLAB error propagate to the command
%   window.
%
%   U, Y      - open-loop data vectors, as required by ddc.vrft.vrftDesign.
%   MD        - discrete-time LTI reference model.
%   BASIS     - cell array of basis filters.
%   PREFILTER - discrete-time LTI prefilter, or [] to use the
%               ddc.vrft.vrftDesign default (ddc.vrft.prefilterDesign(Md)).
%
%   RESULT is a scalar struct with fields:
%     Success      - true/false.
%     Theta        - nBasis-by-1 parameter vector (only if Success).
%     Info         - diagnostics struct from ddc.vrft.vrftDesign (only if
%                    Success).
%     ErrorId      - MException identifier (only if ~Success).
%     ErrorMessage - human-readable message (only if ~Success).
%
%   See also ddc.vrft.vrftDesign, ddc.vrft.VRFTDesignApp.

    arguments
        u double
        y double
        Md
        basis cell
        prefilter = []
    end

    result.Success = false;
    try
        if isempty(prefilter)
            [theta, info] = ddc.vrft.vrftDesign(u, y, Md, basis);
        else
            [theta, info] = ddc.vrft.vrftDesign(u, y, Md, basis, 'Prefilter', prefilter);
        end
        result.Success = true;
        result.Theta = theta;
        result.Info = info;
    catch ME
        result.ErrorId = ME.identifier;
        result.ErrorMessage = ME.message;
    end
end
