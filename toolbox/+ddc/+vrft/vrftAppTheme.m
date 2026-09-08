function theme = vrftAppTheme()
%VRFTAPPTHEME Centralized color palette for ddc.vrft.VRFTDesignApp.
%   THEME = ddc.vrft.vrftAppTheme() returns a struct of brand and chart
%   colors used by ddc.vrft.VRFTDesignApp, so colors are defined once
%   instead of being scattered as literal RGB triples across callbacks.
%
%   Fields:
%     primary     - accent color for titles/highlights.
%     success     - color for "operation succeeded" status text.
%     error       - color for "operation failed" status text.
%     plotColors  - 5-by-3 RGB matrix of categorical series colors used
%                   across the app's charts (raw data, Md step response,
%                   and the four VRFT diagnostic signals).
%
%   Note: the R2025a uifigure Theme API (fliplightness, ThemeChangedFcn)
%   is intentionally not used here because it requires MATLAB R2025a or
%   newer; this struct covers only the version-independent parts of
%   MATLAB theming (colororder-style categorical palette, centralized
%   brand colors).
%
%   See also ddc.vrft.VRFTDesignApp.

    theme.primary = [0 0.447 0.741];
    theme.success = [0.22 0.56 0.24];
    theme.error = [0.83 0.18 0.18];
    theme.plotColors = [ ...
        0    0.447 0.741; ...
        0.85 0.33  0.10;  ...
        0.93 0.69  0.13;  ...
        0.49 0.18  0.56;  ...
        0.47 0.67  0.19];
end
