% +VRFT  Virtual Reference Feedback Tuning (offline, MATLAB-only)
%
% Functions.
%   vrftDesign             - Batch least-squares VRFT controller design.
%   prefilterDesign        - Classical VRFT prefilter recommendation for a given Md.
%   importTimeSeriesData   - Load/validate an open-loop (u, y) data record.
%   referenceModelFromSpec - Build a canonical 2nd-order discrete Md from specs.
%   controllerBasisPreset  - Named controller-basis presets (PID, Integrator, Gain).
%   parseBasisExpression   - Parse a discrete tf expression in z and Ts.
%   assembleController     - Build C(z, theta) = sum_i theta_i * basis_i(z).
%   runVrftDesign          - Error-safe wrapper around vrftDesign for app use.
%   vrftAppTheme           - Centralized color palette for VRFTDesignApp.
%
% Apps.
%   VRFTDesignApp - Interactive uifigure app wrapping vrftDesign end-to-end.
%
% See also ddc.common, ddc.str.
