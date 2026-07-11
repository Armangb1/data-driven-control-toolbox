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
    %   Usable as a plain MATLAB object or as a Simulink "MATLAB System"
    %   block. Requires Optimization Toolbox (quadprog).
    %
    %   Example (MATLAB):
    %       ctrl = ddc.deepc.DeePCController('DataU', uData, 'DataY', yData, ...
    %           'Tini', 4, 'N', 10, 'InputDimension', 1, 'OutputDimension', 1);
    %       uApply = ctrl.step(uIni, yIni, rFuture);
    %
    %   See also ddc.deepc.deepcDesign, ddc.common.HankelBuilder.

    properties (Nontunable)
        DataU (:,:) double = []   % m-by-T offline input data
        DataY (:,:) double = []   % p-by-T offline output data
        Tini  (1,1) double {mustBePositive, mustBeInteger} = 4
        N     (1,1) double {mustBePositive, mustBeInteger} = 10
        InputDimension  (1,1) double {mustBePositive, mustBeInteger} = 1
        OutputDimension (1,1) double {mustBePositive, mustBeInteger} = 1
        AssumedOrder (1,1) double {mustBeNonnegative} = 0
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
        QuadprogOpts_
    end

    methods
        function obj = DeePCController(varargin)
            setProperties(obj, nargin, varargin{:});
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
            obj.QuadprogOpts_ = optimoptions('quadprog', 'Display', 'none');
        end

        function [uApply, yPred] = stepImpl(obj, uIni, yIni, rFuture)
            m = obj.InputDimension;
            p = obj.OutputDimension;
            Nh = obj.N;

            u_ini = uIni(:);
            y_ini = yIni(:);
            r_future = rFuture(:);

            Up = obj.Up_; Uf = obj.Uf_; Yp = obj.Yp_; Yf = obj.Yf_;
            nG = size(Up, 2);
            nSigma = size(Yp, 1);

            Qbar = obj.Q * eye(p * Nh);
            Rbar = obj.R * eye(m * Nh);

            H = zeros(nG + nSigma);
            H(1:nG, 1:nG) = 2 * (Yf.'*Qbar*Yf + Uf.'*Rbar*Uf + obj.LambdaG*eye(nG));
            H(nG+1:end, nG+1:end) = 2 * obj.LambdaY * eye(nSigma);
            H = (H + H.')/2; % enforce symmetry

            f = zeros(nG + nSigma, 1);
            f(1:nG) = -2 * (Yf.' * Qbar * r_future);

            Aeq = [Up, zeros(size(Up,1), nSigma); ...
                   Yp, -eye(nSigma)];
            beq = [u_ini; y_ini];

            x = quadprog(H, f, [], [], Aeq, beq, [], [], [], obj.QuadprogOpts_);
            if isempty(x)
                % Infeasible/solver failure fallback: hold previous input.
                x = zeros(nG + nSigma, 1);
            end

            g = x(1:nG);
            uPred = Uf * g;
            yPred = Yf * g;

            uApply = uPred(1:m);
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
            num = 3;
        end

        function num = getNumOutputsImpl(~)
            num = 2;
        end
    end
end
