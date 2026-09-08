function C = assembleController(theta, basis)
%ASSEMBLECONTROLLER Build C(z, theta) = sum_i theta_i * basis_i(z).
%   C = ddc.vrft.assembleController(THETA, BASIS) forms the final
%   controller transfer function from the VRFT parameter vector THETA
%   and the same BASIS cell array passed to ddc.vrft.vrftDesign. Used by
%   the Results panel of ddc.vrft.VRFTDesignApp to display and export the
%   designed controller.
%
%   THETA - nBasis-by-1 (or 1-by-nBasis) numeric vector.
%   BASIS - cell array of nBasis discrete-time LTI filters or numeric
%           scalars (a plain scalar entry is treated as a static gain,
%           matching ddc.vrft.vrftDesign's own convention).
%
%   C is a discrete-time tf object.
%
%   See also ddc.vrft.vrftDesign, ddc.vrft.controllerBasisPreset.

    arguments
        theta double
        basis cell
    end

    if ~isempty(theta) && ~isvector(theta)
        error('ddc:vrft:assembleController:InvalidTheta', ...
            'theta must be a numeric vector.');
    end

    theta = theta(:);
    if numel(theta) ~= numel(basis)
        error('ddc:vrft:assembleController:SizeMismatch', ...
            'theta has %d element(s) but basis has %d entr(y/ies); sizes must match.', ...
            numel(theta), numel(basis));
    end
    if isempty(basis)
        error('ddc:vrft:assembleController:EmptyBasis', ...
            'basis must contain at least one entry.');
    end

    Ts = [];
    for i = 1:numel(basis)
        bi = basis{i};
        if ~isnumeric(bi)
            Ts = bi.Ts;
            break;
        end
    end
    if isempty(Ts)
        error('ddc:vrft:assembleController:NoSampleTime', ...
            'basis must contain at least one LTI entry to determine the sample time (all-scalar bases are ambiguous).');
    end

    C = tf(0, 1, Ts);
    for i = 1:numel(basis)
        bi = basis{i};
        if isnumeric(bi)
            bi = tf(bi, 1, Ts);
        end
        C = C + theta(i) * bi;
    end
end
