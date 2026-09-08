function Md = referenceModelFromSpec(spec)
%REFERENCEMODELFROMSPEC Build a canonical second-order discrete Md from specs.
%   MD = ddc.vrft.referenceModelFromSpec(SPEC) builds a discrete-time,
%   unity-DC-gain, second-order reference model
%
%       Mc(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
%
%   from a desired settling time and damping ratio, then discretizes it
%   with a zero-order-hold at the requested sample time. This is the
%   computational core of the "design interactively" mode of the
%   Reference Model panel in ddc.vrft.VRFTDesignApp.
%
%   SPEC is a scalar struct with fields:
%     SettlingTime      - desired 2% settling time in seconds (> 0).
%     DampingRatio      - desired damping ratio zeta, 0 < zeta <= 1.
%     Ts                - sample time in seconds (> 0).
%     SettlingTolerance - (optional) settling-time criterion, one of
%                         0.02 or 0.05 (default 0.02).
%
%   MD is a discrete-time tf object with Md.Ts == SPEC.Ts.
%
%   The natural frequency is derived from the standard approximation
%   ts ~= -log(tol)/(zeta*wn), which reduces to the familiar ts ~= 4/(zeta*wn)
%   for the 2% criterion and ts ~= 3/(zeta*wn) for the 5% criterion.
%
%   See also ddc.vrft.VRFTDesignApp, ddc.vrft.vrftDesign.

    arguments
        spec (1,1) struct
    end

    requiredFields = ["SettlingTime", "DampingRatio", "Ts"];
    for f = requiredFields
        if ~isfield(spec, f)
            error('ddc:vrft:referenceModelFromSpec:MissingField', ...
                'spec must have a field "%s".', f);
        end
    end

    ts = spec.SettlingTime;
    zeta = spec.DampingRatio;
    Ts = spec.Ts;
    tol = 0.02;
    if isfield(spec, 'SettlingTolerance') && ~isempty(spec.SettlingTolerance)
        tol = spec.SettlingTolerance;
    end

    if ~(isscalar(ts) && isnumeric(ts) && ts > 0)
        error('ddc:vrft:referenceModelFromSpec:InvalidSettlingTime', ...
            'SettlingTime must be a positive scalar.');
    end
    if ~(isscalar(zeta) && isnumeric(zeta) && zeta > 0 && zeta <= 1)
        error('ddc:vrft:referenceModelFromSpec:InvalidDampingRatio', ...
            'DampingRatio must be a scalar in (0, 1].');
    end
    if ~(isscalar(Ts) && isnumeric(Ts) && Ts > 0)
        error('ddc:vrft:referenceModelFromSpec:InvalidTs', ...
            'Ts must be a positive scalar.');
    end
    if ~(isscalar(tol) && isnumeric(tol) && tol > 0 && tol < 1)
        error('ddc:vrft:referenceModelFromSpec:InvalidTolerance', ...
            'SettlingTolerance must be a scalar in (0, 1).');
    end

    wn = -log(tol) / (zeta * ts);

    Mc = tf(wn^2, [1, 2*zeta*wn, wn^2]);
    Md = c2d(Mc, Ts, 'zoh');
end
