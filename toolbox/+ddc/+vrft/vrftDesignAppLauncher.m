function fig = vrftDesignAppLauncher
%VRFTDESIGNAPPLAUNCHER Launch the interactive VRFT Design app.
%   FIG = ddc.vrft.vrftDesignAppLauncher() constructs the interactive VRFT
%   Design app and returns its figure. This is the entry point registered as
%   the main file when the app is packaged into an MLAPPINSTALL file.
%
%   Example:
%       fig = ddc.vrft.vrftDesignAppLauncher();

app = ddc.vrft.VRFTDesignApp();
fig = app.getFigure();
end
