function H = HankelBuilder(data, L)
%HANKELBUILDER Build a (block) Hankel matrix from a data sequence.
%   H = ddc.common.HankelBuilder(DATA, L) builds a Hankel matrix with L
%   block rows from the signal DATA.
%
%   DATA can be:
%     - a 1-by-T or T-by-1 vector (scalar signal), or
%     - an m-by-T matrix, where m is the signal dimension and T is the
%       number of time samples (columns = time).
%
%   The resulting Hankel matrix has size (L*m)-by-(T-L+1):
%
%       H = [ d(1)   d(2)   ...  d(T-L+1)  ]
%           [ d(2)   d(3)   ...  d(T-L+2)  ]
%           [  :        :             :    ]
%           [ d(L)   d(L+1) ...  d(T)      ]
%
%   where each d(k) is an m-by-1 column (block row of H is m rows).
%
%   This is the standard construction used by Willems' Fundamental Lemma
%   / DeePC to build data matrices (e.g. splitting H into "past" and
%   "future" blocks Up, Uf, Yp, Yf).
%
%   Example:
%       u = randn(1,50);
%       Hu = ddc.common.HankelBuilder(u, 10);
%
%   See also ddc.common.checkPersistencyExcitation, ddc.deepc.deepcDesign.

    if isvector(data)
        data = data(:).'; % row vector, m = 1
    end
    [m, T] = size(data);

    if ~(isscalar(L) && L == floor(L) && L >= 1)
        error('ddc:common:HankelBuilder:InvalidL', ...
            'L must be a positive integer.');
    end
    if L > T
        error('ddc:common:HankelBuilder:LTooLarge', ...
            'L (%d) must not exceed the number of data samples T (%d).', L, T);
    end

    nCols = T - L + 1;
    H = zeros(L*m, nCols);
    for k = 1:L
        H((k-1)*m+1:k*m, :) = data(:, k:k+nCols-1);
    end
end
