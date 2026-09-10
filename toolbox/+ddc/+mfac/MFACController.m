classdef MFACController < matlab.System
    %MFACCONTROLLER Model-Free Adaptive Control (CFDL / PFDL / FFDL).
    %   Generalized MFAC supporting Compact-Form (CFDL), Partial-Form
    %   (PFDL), and Full-Form (FFDL) Dynamic Linearization.
    %
    %   CFDL and PFDL are special cases of FFDL:
    %       CFDL  <=>  Ly=0, Lu=1   (scalar PPD, scalar rho)
    %       PFDL  <=>  Ly=0, Lu=L   (vector PPD of length L)
    %       FFDL  <=>  general Ly, Lu
    %
    %   Pseudo-order vector:
    %       H(t)  = [y(t),...,y(t-Ly+1), u(t),...,u(t-Lu+1)]'  in R^(Ly+Lu)
    %       dH(t) = H(t) - H(t-1)
    %
    %   PPD/PG vector estimate (projection algorithm with reset):
    %       phi(t) = phi(t-1) + eta*dH(t-1)/(mu + ||dH(t-1)||^2) *
    %               (dy(t) - phi(t-1)'*dH(t-1))
    %       reset phi(t) -> phi(0) if:
    %           ||phi(t)|| <= epsilon,  ||dH(t-1)|| <= epsilon, or
    %           sign(phi(Ly+1)(t)) ~= sign(phi(Ly+1)(0))
    %
    %   Control law:
    %       u(t) = u(t-1)
    %            + rho(Ly+1)*phi(Ly+1)(t)*(yd(t+1)-y(t)) / (lambda+phi(Ly+1)(t)^2)
    %            - phi(Ly+1)(t) * [ sum_{i=1}^{Ly} rho(i)*phi(i)*dy(t-i+1)
    %                             + sum_{i=Ly+2}^{Ly+Lu} rho(i)*phi(i)*du(t+Ly-i+1) ]
    %                             / (lambda + phi(Ly+1)(t)^2)
    %
    %   Second output is a scalar = phi(Ly+1), the "current-input PPD"
    %   (the one most relevant for tuning / gain scheduling).
    %
    %   Usable as a plain MATLAB object or as a Simulink "MATLAB System"
    %   block.
    %
    %   Examples:
    %       % CFDL (default, equivalent to old MFACController)
    %       ctrl = ddc.mfac.MFACController('PhiInit', 1);
    %       u = ctrl.step(y, rNext);
    %
    %       % PFDL with pseudo-order L=3
    %       ctrl = ddc.mfac.MFACController('Ly', 0, 'Lu', 3, ...
    %           'PhiInit', [0.01 0.01 0.01], 'Rho', [0.3 0.3 0.3]);
    %
    %       % FFDL with Ly=2, Lu=2
    %       ctrl = ddc.mfac.MFACController('Ly', 2, 'Lu', 2, ...
    %           'PhiInit', 0.01*ones(4,1), 'Rho', 0.3*ones(4,1));
    %
    %   See also ddc.str.DirectSTRController, ddc.common.RLSEstimator.

    properties (Nontunable)
        Ly  (1,1) double {mustBeInteger, mustBeNonnegative} = 0  % output pseudo-order
        Lu  (1,1) double {mustBeInteger, mustBePositive}    = 1  % input pseudo-order
        PhiInit = 1                                               % scalar or (Ly+Lu)-vector, initial PPD/PG
    end

    properties
        Eta     (1,1) double {mustBeReal}          = 1       % PPD step size
        Mu      (1,1) double {mustBePositive}      = 1       % PPD weighting
        Rho     = 1                                           % scalar or (Ly+Lu)-vector, control step size
        Lambda  (1,1) double {mustBePositive}      = 1       % control weighting
        Epsilon (1,1) double {mustBePositive}      = 1e-5    % reset threshold
    end

    properties (Access = private)
        Phi_          % (Ly+Lu)x1 vector
        PhiInit_      % expanded (Ly+Lu)x1 initial vector
        Rho_          % expanded (Ly+Lu)x1 vector
        YBuf_         % shift buffer: [y(t), y(t-1), ..., y(t-Ly)]  length Ly+1
        UBuf_         % shift buffer: [u(t), u(t-1), ..., u(t-Lu)]  length Lu+1
        PrevU_        % previous u(t-1)
        HPrev_        % previous H(t-1) vector for computing dH
    end

    methods
        function obj = MFACController(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            n = obj.Ly + obj.Lu;
            if obj.Ly == 0 && obj.Lu == 0
                error('ddc:mfac:MFACController:InvalidOrders', ...
                    'Ly and Lu cannot both be zero.');
            end

            % Validate & expand PhiInit
            if isscalar(obj.PhiInit)
                obj.PhiInit_ = obj.PhiInit * ones(n, 1);
            elseif isvector(obj.PhiInit) && numel(obj.PhiInit) == n
                obj.PhiInit_ = obj.PhiInit(:);
            else
                error('ddc:mfac:MFACController:BadPhiInit', ...
                    'PhiInit must be a scalar or a vector of length Ly+Lu (%d).', n);
            end

            % Validate & expand Rho
            if isscalar(obj.Rho)
                obj.Rho_ = obj.Rho * ones(n, 1);
            elseif isvector(obj.Rho) && numel(obj.Rho) == n
                obj.Rho_ = obj.Rho(:);
            else
                error('ddc:mfac:MFACController:BadRho', ...
                    'Rho must be a scalar or a vector of length Ly+Lu (%d).', n);
            end

            obj.Phi_   = obj.PhiInit_;
            obj.YBuf_  = zeros(max(obj.Ly, 1) + 1, 1);
            obj.UBuf_  = zeros(obj.Lu + 1, 1);
            obj.PrevU_ = 0;
            obj.HPrev_ = zeros(n, 1);
        end

        function resetImpl(obj)
            obj.Phi_   = obj.PhiInit_;
            obj.YBuf_  = zeros(max(obj.Ly, 1) + 1, 1);
            obj.UBuf_  = zeros(obj.Lu + 1, 1);
            obj.PrevU_ = 0;
            obj.HPrev_ = zeros(size(obj.PhiInit_));
        end

        function [u, phiOut] = stepImpl(obj, y, rNext)
            H = zeros(obj.Ly + obj.Lu, 1);
            if obj.Ly > 0
                H(1:obj.Ly) = obj.YBuf_(1:obj.Ly);           % [y(t-1), ..., y(t-Ly)]
            end
            H(obj.Ly+1:end) = obj.UBuf_(1:obj.Lu);            % [u(t-1), ..., u(t-Lu)]

            dH = H - obj.HPrev_;                              % dH(t-1)

            % Shift current y into YBuf_ so YBuf_(1) = y(t)
            obj.YBuf_(2:end) = obj.YBuf_(1:end-1);
            obj.YBuf_(1)     = y;

            dy = obj.YBuf_(1) - obj.YBuf_(2);                 % dy(t)

            % --- PPD/PG vector update (projection algorithm) ---
            dHnorm2 = dH' * dH;
            phi = obj.Phi_ + (obj.Eta / (obj.Mu + dHnorm2)) * dH * (dy - obj.Phi_' * dH);

            % --- Reset conditions ---
            phiNorm = sqrt(phi' * phi);
            if phiNorm <= obj.Epsilon || dHnorm2 <= obj.Epsilon^2 || ...
                    sign(phi(obj.Ly+1)) ~= sign(obj.PhiInit_(obj.Ly+1))
                phi = obj.PhiInit_;
            end

            % --- Control law ---
            phi1 = phi(obj.Ly+1);          % "current-input" PPD
            denom = obj.Lambda + phi1^2;

            % Tracking term:  rho(Ly+1)*phi(Ly+1)*(yd - y(t)) / denom
            du = obj.Rho_(obj.Ly+1) * phi1 * (rNext - obj.YBuf_(1)) / denom;

            % Sum over output deltas:  i = 1..Ly  ->  dy(t-i+1)
            for i = 1:obj.Ly
                dy_i = obj.YBuf_(i) - obj.YBuf_(i+1);
                du = du - phi1 * obj.Rho_(i) * phi(i) * dy_i / denom;
            end

            % Sum over input deltas:  i = Ly+2 .. Ly+Lu  ->  du(t+Ly-i+1)
            for i = (obj.Ly+2):(obj.Ly+obj.Lu)
                j = i - obj.Ly - 1;
                du_i = obj.UBuf_(j) - obj.UBuf_(j+1);
                du = du - phi1 * obj.Rho_(i) * phi(i) * du_i / denom;
            end

            u = obj.PrevU_ + du;

            % --- Shift input buffer: make room at index 1 ---
            obj.UBuf_(2:end) = obj.UBuf_(1:end-1);
            obj.UBuf_(1)     = u;

            % --- Update state ---
            obj.Phi_   = phi;
            obj.PrevU_ = u;
            obj.HPrev_ = H;

            % Second output: scalar "current-input PPD"
            phiOut = phi1;
        end

        function [sz1, sz2] = getOutputSizeImpl(~)
            sz1 = [1 1];
            sz2 = [1 1];
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
