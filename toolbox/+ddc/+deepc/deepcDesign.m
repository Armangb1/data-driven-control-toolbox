function design = deepcDesign(uData, yData, Tini, N, varargin)
%DEEPCDESIGN Offline data-matrix construction for DeePC.
%   DESIGN = ddc.deepc.deepcDesign(UDATA, YDATA, TINI, N) builds the
%   Hankel data matrices (Up, Uf, Yp, Yf) required by DeePC from an
%   offline, open-loop, persistently-exciting input/output data set.
%
%   UDATA - m-by-T input data (or 1-by-T / T-by-1 vector for SISO).
%   YDATA - p-by-T output data (or vector for SISO).
%   TINI  - length of the initial-condition (past) trajectory window.
%   N     - prediction horizon (future) length.
%
%   Name-value options:
%   'AssumedOrder' - upper bound on the system order n, used only to
%                    validate persistency of excitation (default: 0,
%                    i.e. only checks order Tini+N).
%
%   DESIGN is a struct with fields: Up, Uf, Yp, Yf, m, p, Tini, N, T,
%   isPE (persistency-of-excitation flag for the required order).
%
%   See also ddc.deepc.DeePCController, ddc.common.HankelBuilder,
%   ddc.common.checkPersistencyExcitation.

    p_opts = inputParser;
    p_opts.addParameter('AssumedOrder', 0, @(x) isscalar(x) && x >= 0);
    p_opts.parse(varargin{:});
    n = p_opts.Results.AssumedOrder;

    if isvector(uData); uData = uData(:).'; end
    if isvector(yData); yData = yData(:).'; end

    [m, Tu] = size(uData);
    [p, Ty] = size(yData);
    if Tu ~= Ty
        error('ddc:deepc:deepcDesign:LengthMismatch', ...
            'uData and yData must have the same number of time samples.');
    end
    T = Tu;
    L = Tini + N;
    if T < L
        error('ddc:deepc:deepcDesign:NotEnoughData', ...
            'Need at least Tini+N = %d samples, got %d.', L, T);
    end

    order = L + n;
    [isPE, r, rRequired] = ddc.common.checkPersistencyExcitation(uData, order, m);
    if ~isPE
        warning('ddc:deepc:deepcDesign:NotPersistentlyExciting', ...
            ['Input data is NOT persistently exciting of the required order ' ...
             '(rank %d, required %d). DeePC predictions may be unreliable.'], ...
            r, rRequired);
    end

    Hu = ddc.common.HankelBuilder(uData, L);
    Hy = ddc.common.HankelBuilder(yData, L);

    design.Up = Hu(1:m*Tini, :);
    design.Uf = Hu(m*Tini+1:end, :);
    design.Yp = Hy(1:p*Tini, :);
    design.Yf = Hy(p*Tini+1:end, :);
    design.m = m;
    design.p = p;
    design.Tini = Tini;
    design.N = N;
    design.T = T;
    design.isPE = isPE;
end
