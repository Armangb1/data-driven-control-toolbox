classdef DeePCController < matlab.System
    %DEEPCCONTROLLER Data-Enabled Predictive Control (DeePC).
    %   Implements the regularized DeePC receding-horizon law of
    %   Coulson, Lygeros & Dorfler, "Data-Enabled Predictive Control:
    %   In the Shallows of the DeePC", ECC 2019. At each step, solves
    %
    %     min_{g,sigma}  ||Yf*g - rFuture||_Q^2 + ||Uf*g||_R^2
    %                     + lambdaG*||g||^2 + lambdaY*||sigma||^2
    %     s.t.  Up*g = uIni
    %           Yp*g = yIni + sigma
    %
    %   and applies the first InputDimension entries of Uf*g.
    %
    %   Offline data (DataU, DataY) must be an open-loop,
    %   persistently-exciting input/output record collected from the
    %   plant; see ddc.deepc.deepcDesign for the offline data-matrix
    %   construction and PE check performed internally at setup.
    %
    %   The input and output dimensions are derived from the offline data
    %   (InputDimension = size(DataU,1), OutputDimension = size(DataY,1))
    %   and are read-only; they cannot be set independently.
    %
    %   Usable as a plain MATLAB object or as a Simulink "MATLAB System"
    %   block. Requires Optimization Toolbox (quadprog).
    %
    %   Example (MATLAB):
    %       ctrl = ddc.deepc.DeePCController('DataU', uData, 'DataY', yData, ...
    %           'Tini', 4, 'N', 10);
    %       uApply = ctrl.step(yCurr, rFuture);
    %
    %   See also ddc.deepc.deepcDesign, ddc.common.HankelBuilder.

    properties (Nontunable)
        DataU (:,:) double = []   % m-by-T offline input data
        DataY (:,:) double = []   % p-by-T offline output data
        Tini  (1,1) double {mustBePositive, mustBeInteger} = 4
        N     (1,1) double {mustBePositive, mustBeInteger} = 10
        AssumedOrder (1,1) double {mustBeNonnegative} = 0
    end

    properties (Dependent)
        InputDimension   % m = size(DataU,1), derived (read-only)
        OutputDimension  % p = size(DataY,1), derived (read-only)
    end

    properties
        Q (1,1) double {mustBeNonnegative} = 1      % output tracking weight
        R (1,1) double {mustBeNonnegative} = 0.01   % input effort weight
        LambdaG (1,1) double {mustBeNonnegative} = 1    % g-regularization
        LambdaY (1,1) double {mustBeNonnegative} = 1e4  % slack regularization
    end

    properties (Access = private)
        Up_
        Uf_
        Yp_
        Yf_
        YfT_      % Yf.'   (cached for the linear cost term)
        YtY_      % Yf.'*Yf (cached for H)
        UtU_      % Uf.'*Uf (cached for H)
        Aeq_      % invariant equality-constraint matrix [Up 0; Yp -I]
        LastU_    % last successfully applied input (fallback)
        UBuf_     % m-by-Tini buffer of past applied inputs (oldest->newest)
        YBuf_     % p-by-Tini buffer of past measured outputs (oldest->newest)
        QuadprogOpts_
    end

    methods
        function obj = DeePCController(varargin)
            setProperties(obj, nargin, varargin{:});
        end

        function m = get.InputDimension(obj)
            m = size(obj.DataU, 1);
        end

        function p = get.OutputDimension(obj)
            p = size(obj.DataY, 1);
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            if isempty(obj.DataU) || isempty(obj.DataY)
                error('ddc:deepc:DeePCController:NoData', ...
                    'DataU and DataY must be provided before use.');
            end
            design = ddc.deepc.deepcDesign(obj.DataU, obj.DataY, obj.Tini, obj.N, ...
                'AssumedOrder', obj.AssumedOrder);
            obj.Up_ = design.Up;
            obj.Uf_ = design.Uf;
            obj.Yp_ = design.Yp;
            obj.Yf_ = design.Yf;
            obj.YfT_ = design.Yf.';
            obj.YtY_ = design.Yf.' * design.Yf;
            obj.UtU_ = design.Uf.' * design.Uf;
            obj.Aeq_ = [design.Up, zeros(size(design.Up, 1), size(design.Yp, 1)); ...
                        design.Yp, -eye(size(design.Yp, 1))];
            obj.LastU_ = zeros(size(obj.DataU, 1), 1);
            obj.UBuf_ = zeros(obj.InputDimension, obj.Tini);
            obj.YBuf_ = zeros(obj.OutputDimension, obj.Tini);
            obj.QuadprogOpts_ = optimoptions('quadprog', 'Display', 'none','MaxIterations',5);
        end

        function resetImpl(obj)
            obj.LastU_ = zeros(size(obj.DataU, 1), 1);
            obj.UBuf_ = zeros(obj.InputDimension, obj.Tini);
            obj.YBuf_ = zeros(obj.OutputDimension, obj.Tini);
        end

        %NOTE: This buffer-based formulation assumes the input actually
        %          applied to the plant equals the controller's own computed
        %          uApply (no external saturation, override, or switching
        %          between the controller output and the plant). If that
        %          assumption does not hold, feed uPrev externally instead.
        function [uApply, yPred] = stepImpl(obj, yCurr, rFuture)
            m = obj.InputDimension;
            p = obj.OutputDimension;
            Nh = obj.N;

            if numel(yCurr) ~= p
                error('ddc:deepc:DeePCController:BadYCurr', ...
                    'yCurr must have length %d (= OutputDimension), got %d.', ...
                    p, numel(yCurr));
            end

            obj.UBuf_ = [obj.UBuf_(:, 2:end), obj.LastU_];
            obj.YBuf_ = [obj.YBuf_(:, 2:end), yCurr(:)];

            u_ini = obj.UBuf_(:);
            y_ini = obj.YBuf_(:);

            if numel(rFuture) ~= p * Nh
                error('ddc:deepc:DeePCController:BadRFuture', ...
                    'rFuture must have length %d (= OutputDimension*N), got %d.', ...
                    p * Nh, numel(rFuture));
            end

            r_future = rFuture(:);

            Uf = obj.Uf_; Yf = obj.Yf_;
            nG = size(obj.Up_, 2);
            nSigma = size(obj.Yp_, 1);

            % H must be rebuilt each step: Q, R, LambdaG, LambdaY are
            % tunable and may change between calls. Cost block a is
            % Q*(Yf.'*Yf), b is R*(Uf.'*Uf); build from cached products
            % instead of constructing the full identity-weight matrices.
            H = zeros(nG + nSigma);
            H(1:nG, 1:nG) = 2 * (obj.Q * obj.YtY_ + obj.R * obj.UtU_ + obj.LambdaG * eye(nG));
            H(nG+1:end, nG+1:end) = 2 * obj.LambdaY * eye(nSigma);
            H = (H + H.')/2; % enforce symmetry

            f = zeros(nG + nSigma, 1);
            f(1:nG) = -2 * obj.Q * (obj.YfT_ * r_future);

            beq = [u_ini; y_ini];

            x = quadprog(H, f, [], [], obj.Aeq_, beq, [], [], [], obj.QuadprogOpts_);
            if isempty(x)
                % Infeasible/solver failure fallback: hold previous input.
                uApply = obj.LastU_;
                yPred = Yf * zeros(nG, 1);
            else
                g = x(1:nG);
                uPred = Uf * g;
                yPred = Yf * g;
                uApply = uPred(1:m);
            end
            obj.LastU_ = uApply;
        end

        function [sz1, sz2] = getOutputSizeImpl(obj)
            sz1 = [obj.InputDimension, 1];
            sz2 = [obj.OutputDimension * obj.N, 1]; % predicted future output trajectory
        end

        function [dt1, dt2] = getOutputDataTypeImpl(~)
            dt1 = 'double';
            dt2 = 'double';
        end

        function [cp1, cp2] = isOutputComplexImpl(~)
            cp1 = false; cp2 = false;
        end

        function [fz1, fz2] = isOutputFixedSizeImpl(~)
            fz1 = true; fz2 = true;
        end

        function num = getNumInputsImpl(~)
            num = 2;
        end

        function num = getNumOutputsImpl(~)
            num = 2;
        end
    end
end
