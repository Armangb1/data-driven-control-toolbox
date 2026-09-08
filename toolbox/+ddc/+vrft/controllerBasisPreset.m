function basis = controllerBasisPreset(presetName, Ts)
%CONTROLLERBASISPRESET Named controller-basis presets for VRFT design.
%   BASIS = ddc.vrft.controllerBasisPreset(PRESETNAME, TS) returns a cell
%   array of discrete-time tf objects defining a fixed controller
%   structure, suitable for direct use as the BASIS input of
%   ddc.vrft.vrftDesign. Backs the preset dropdown in the Controller
%   Structure panel of ddc.vrft.VRFTDesignApp.
%
%   PRESETNAME - one of "PID", "Integrator", "Gain" (case-insensitive).
%   TS         - sample time in seconds (> 0), used to build the
%                discrete integral/derivative terms.
%
%   Presets (z is the discrete-time shift operator at sample time TS):
%     "PID"         - {1, Ts/(z-1), (z-1)/(Ts*z)}   (proportional,
%                     forward-Euler integral, backward-Euler derivative)
%     "Integrator"  - {Ts/(z-1)}
%     "Gain"        - {1}
%
%   See also ddc.vrft.parseBasisExpression, ddc.vrft.vrftDesign.

    arguments
        presetName (1,1) string
        Ts (1,1) double {mustBePositive}
    end

    z = tf('z', Ts);
    switch lower(presetName)
        case "pid"
            basis = {tf(1, 1, Ts), Ts / (z - 1), (z - 1) / (Ts * z)};
        case "integrator"
            basis = {Ts / (z - 1)};
        case "gain"
            basis = {tf(1, 1, Ts)};
        otherwise
            error('ddc:vrft:controllerBasisPreset:UnknownPreset', ...
                'Unknown preset "%s"; expected "PID", "Integrator", or "Gain".', presetName);
    end
end
