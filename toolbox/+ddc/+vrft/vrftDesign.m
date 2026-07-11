function [theta, info] = vrftDesign(u, y, Md, basis, varargin)
%VRFTDESIGN Virtual Reference Feedback Tuning (offline, batch controller design).
%   THETA = ddc.vrft.vrftDesign(U, Y, MD, BASIS) computes, in a single
%   batch least-squares step and without any plant model, the parameter
%   vector THETA of a fixed-structure linear controller
%
%       C(z, theta) = sum_i theta_i * BASIS{i}(z)
%
%   from one open-loop input/output data record (U, Y) collected from the
%   plant and a desired closed-loop reference model MD(z), following
%   Campi, Lecchini & Savaresi, "Virtual reference feedback tuning: a
%   direct method for the design of feedback controllers", Automatica
%   2002.
%
%   Algorithm:
%     1. Virtual reference:  r_v = Md^-1 * y     (applied via lsim; see
%        the relative-degree note below for how a strictly-proper Md,
%        the normal case, is handled)
%     2. Virtual error:      e_v = r_v - y
%     3. Prefilter:          e_f = L * e_v,  u_f = L * u  (see ddc.vrft.prefilterDesign;
%        L is applied to BOTH signals since the least-squares criterion
%        is ||L*(u - C(theta)*e_v)||^2 = ||L*u - C(theta)*(L*e_v)||^2,
%        using commutativity of LTI filtering)
%     4. Regressors:         phi_i = BASIS{i} * e_f,  i = 1..nBasis
%     5. Least squares:      theta = argmin || u_f - Phi*theta ||^2
%
%   Inputs:
%     U, Y   - 1-by-T (or T-by-1) input/output data vectors from an
%              open-loop plant experiment, sampled at MD's sample time.
%     MD     - discrete-time LTI reference model (tf/zpk/ss) for the
%              desired closed-loop behavior from r to y. MD may be
%              strictly proper (relative degree d > 0, the normal case
%              for a physical reference model): since VRFT computes r_v
%              OFFLINE from the full data record, the relative degree is
%              handled by inverting the biproper part of Md and
%              time-advancing Y by d samples (equivalent to inverting a
%              noncausal operator using future data, valid only because
%              this is a batch, not real-time, computation). Y and U are
%              truncated by d samples accordingly before the least
%              squares fit. Md must still be minimum-phase (stable
%              inverse) and must not use the LTI 'InputDelay' property
%              (encode any pure delay directly via the Md polynomial
%              degrees instead, i.e. relative degree = deg(den)-deg(num)).
%     BASIS  - cell array of discrete-time LTI filters {beta_1,...,beta_n}
%              defining the controller structure (e.g. for a discrete
%              PID: {1, Ts/(z-1), (z-1)/(Ts*z)}).
%
%   Name-value options:
%     'Prefilter' - LTI filter L(z) (default: ddc.vrft.prefilterDesign(Md))
%
%   Outputs:
%     THETA - nBasis-by-1 controller parameter vector.
%     INFO  - struct with fields Phi, rv, ev, ef (intermediate signals)
%             and L (the prefilter actually used), for diagnostics.
%
%   See also ddc.vrft.prefilterDesign.

    p = inputParser;
    p.addParameter('Prefilter', []);
    p.parse(varargin{:});
    L = p.Results.Prefilter;
    if isempty(L)
        L = ddc.vrft.prefilterDesign(Md);
    end

    u = u(:);
    y = y(:);
    if numel(u) ~= numel(y)
        error('ddc:vrft:vrftDesign:LengthMismatch', ...
            'u and y must have the same number of samples.');
    end
    Ts = Md.Ts;

    % Relative degree d of Md = deg(den) - deg(num); make Md biproper by
    % multiplying its numerator by z^d (a pure time-advance), then invert
    % that biproper system causally. Applying the resulting causal
    % inverse to Y ADVANCED by d samples (Y(1+d:end)) is mathematically
    % equivalent to applying the true (noncausal) Md^-1 to Y -- valid
    % here because the whole record is available offline.
    [num, den] = tfdata(Md, 'v'); % num is zero-padded to length(den) by tfdata
    firstNonzero = find(num ~= 0, 1);
    if isempty(firstNonzero)
        error('ddc:vrft:vrftDesign:ZeroModel', 'Md must not be identically zero.');
    end
    d = firstNonzero - 1; % relative degree = number of leading zeros in num
    numBiproper = [num(firstNonzero:end), zeros(1, d)];
    MdBiproper = tf(numBiproper, den, Ts);
    try
        MdInv = inv(MdBiproper); %#ok<MINV> % requires Md minimum-phase
    catch ME
        error('ddc:vrft:vrftDesign:NoncausalInverse', ...
            'Md could not be causally inverted (%s); Md must be minimum-phase.', ...
            ME.message);
    end

    T = numel(y);
    yAdvanced = y(1+d:end);       % y(k+d), k = 1..T-d, used only to invert Md's biproper part
    yAligned  = y(1:T-d);         % y(k),   k = 1..T-d, aligned with rv for the virtual error
    uTrunc = u(1:T-d);
    Tv = T - d;
    tv = (0:Tv-1).' * Ts;

    rv = lsim(MdInv, yAdvanced, tv);
    ev = rv - yAligned;
    ef = lsim(L, ev, tv);
    uf = lsim(L, uTrunc, tv);

    nBasis = numel(basis);
    Phi = zeros(Tv, nBasis);
    for i = 1:nBasis
        bi = basis{i};
        if isnumeric(bi)
            bi = tf(bi, 1, Ts); % allow plain scalar gains as basis entries
        end
        Phi(:, i) = lsim(bi, ef, tv);
    end

    theta = Phi \ uf;

    info.Phi = Phi;
    info.rv = rv;
    info.ev = ev;
    info.ef = ef;
    info.uf = uf;
    info.L = L;
    info.relativeDegree = d;
end
