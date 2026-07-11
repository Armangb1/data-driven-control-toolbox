function L = prefilterDesign(Md)
%PREFILTERDESIGN VRFT prefilter recommendation for a reference model.
%   L = ddc.vrft.prefilterDesign(MD) returns the classical VRFT prefilter
%   L(z) = MD(z) * (1 - MD(z)), as recommended by Campi, Lecchini &
%   Savaresi, "Virtual reference feedback tuning: a direct method for the
%   design of feedback controllers", Automatica 2002, for open-loop data
%   collected under (approximately) white measurement noise. This choice
%   approximately minimizes the asymptotic variance of the VRFT estimate
%   by spectrally shaping the virtual tracking error to match MD.
%
%   MD must be a discrete-time LTI model (tf/zpk/ss) with a defined
%   sample time.
%
%   See also ddc.vrft.vrftDesign.

    if ~isa(Md, 'DynamicSystem') && ~isa(Md, 'lti')
        error('ddc:vrft:prefilterDesign:InvalidModel', ...
            'Md must be a discrete-time LTI model (tf/zpk/ss).');
    end
    if Md.Ts == 0
        error('ddc:vrft:prefilterDesign:ContinuousModel', ...
            'Md must be discrete-time (Ts > 0).');
    end

    L = Md * (1 - Md);
end
