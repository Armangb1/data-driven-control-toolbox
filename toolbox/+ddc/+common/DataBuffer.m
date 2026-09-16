classdef DataBuffer < matlab.System
    %DATABUFFER Sliding-window FIFO buffer for online data-driven control.
    %   Buffers the last WindowLength samples of one or more signals
    %   (e.g. plant input/output pairs) for use by online algorithms such
    %   as DeePC (recent input/output trajectory) or RLS-based STR/MFAC
    %   (regressor history). Usable as a plain MATLAB object or as a
    %   Simulink "MATLAB System" block.
    %
    %   Example (MATLAB):
    %       buf = ddc.common.DataBuffer('WindowLength', 20, 'SignalWidth', 1);
    %       for k = 1:100
    %           [full, data] = buf.step(randn);
    %       end
    %
    %   See also ddc.common.HankelBuilder, ddc.deepc.DeePCController.

    properties (Nontunable)
        % WindowLength Buffer window length (number of samples stored)
        WindowLength (1,1) double {mustBePositive, mustBeInteger} = 10

        % SignalWidth Number of signals to buffer
        SignalWidth (1,1) double {mustBePositive, mustBeInteger} = 1
    end

    properties (Access = private)
        Buffer      % SignalWidth-by-WindowLength circular storage
        Count       % number of valid samples currently stored
    end

    methods
        function obj = DataBuffer(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.Buffer = zeros(obj.SignalWidth, obj.WindowLength);
            obj.Count = 0;
        end

        function resetImpl(obj)
            obj.Buffer(:) = 0;
            obj.Count = 0;
        end

        function [isFull, data] = stepImpl(obj, sample)
            sample = sample(:);
            obj.Buffer = [obj.Buffer(:, 2:end), sample];
            obj.Count = min(obj.Count + 1, obj.WindowLength);
            isFull = (obj.Count >= obj.WindowLength);
            data = obj.Buffer;
        end

        function [sz1, sz2] = getOutputSizeImpl(obj)
            sz1 = [1 1];
            sz2 = [obj.SignalWidth, obj.WindowLength];
        end

        function [dt1, dt2] = getOutputDataTypeImpl(~)
            dt1 = 'logical';
            dt2 = 'double';
        end

        function [cp1, cp2] = isOutputComplexImpl(~)
            cp1 = false;
            cp2 = false;
        end

        function [fz1, fz2] = isOutputFixedSizeImpl(~)
            fz1 = true;
            fz2 = true;
        end

        function num = getNumInputsImpl(~)
            num = 1;
        end

        function num = getNumOutputsImpl(~)
            num = 2;
        end
    end

    methods (Static, Access = protected)
        function header = getHeaderImpl
            header = matlab.system.display.Header(mfilename('class'), ...
                'Title', 'Data Buffer', ...
                'Text', [ ...
                'Sliding-window FIFO buffer for online data-driven control.' ...
                newline ...
                'Retains the last WindowLength samples of one or more ' ...
                'signals for online algorithms such as DeePC or RLS-based ' ...
                'STR/MFAC estimators.']);
        end

        function groups = getPropertyGroupsImpl
            groups = matlab.system.display.Section(...
                'Title', 'Data Buffer', ...
                'PropertyList', {'WindowLength', 'SignalWidth'});
        end
    end
end
