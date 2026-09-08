function data = importTimeSeriesData(source)
%IMPORTTIMESERIESDATA Load and validate an open-loop (u, y) data record.
%   DATA = ddc.vrft.importTimeSeriesData(SOURCE) loads an input/output
%   pair from one of three sources and returns it as column vectors,
%   validated for shape and length consistency. This is the data-layer
%   helper behind the Data Input panel of ddc.vrft.VRFTDesignApp; it
%   contains no UI code so it can be unit-tested directly.
%
%   SOURCE is a scalar struct with a required field Type:
%
%     Type = "workspace"
%       U, Y - numeric vectors already extracted from the workspace by
%              the caller (the App reads base-workspace variables via
%              evalin, then hands the arrays to this function; this
%              function itself never touches the workspace).
%
%     Type = "mat"
%       FilePath    - path to a .mat file.
%       UVariable   - name of the variable holding u inside the file.
%       YVariable   - name of the variable holding y inside the file.
%
%     Type = "csv"
%       FilePath  - path to a CSV file with (at least) two columns.
%       UColumn   - (optional) 1-based column index for u (default 1).
%       YColumn   - (optional) 1-based column index for y (default 2).
%
%   Output DATA is a struct with fields:
%     U          - T-by-1 double column vector.
%     Y          - T-by-1 double column vector.
%     NumSamples - T, the common sample count.
%
%   See also ddc.vrft.VRFTDesignApp, ddc.vrft.vrftDesign.

    arguments
        source (1,1) struct
    end

    if ~isfield(source, 'Type')
        error('ddc:vrft:importTimeSeriesData:MissingType', ...
            'source must have a ''Type'' field: "workspace", "mat", or "csv".');
    end
    type = string(source.Type);

    switch type
        case "workspace"
            [u, y] = importFromWorkspaceArrays(source);
        case "mat"
            [u, y] = importFromMatFile(source);
        case "csv"
            [u, y] = importFromCsvFile(source);
        otherwise
            error('ddc:vrft:importTimeSeriesData:UnknownType', ...
                'Unknown source.Type "%s"; expected "workspace", "mat", or "csv".', type);
    end

    [u, y] = validateAndAlignPair(u, y);

    data.U = u;
    data.Y = y;
    data.NumSamples = numel(u);
end

function [u, y] = importFromWorkspaceArrays(source)
    if ~isfield(source, 'U') || ~isfield(source, 'Y')
        error('ddc:vrft:importTimeSeriesData:MissingWorkspaceFields', ...
            'source.Type = "workspace" requires numeric fields U and Y.');
    end
    u = source.U;
    y = source.Y;
end

function [u, y] = importFromMatFile(source)
    if ~isfield(source, 'FilePath') || ~isfield(source, 'UVariable') || ~isfield(source, 'YVariable')
        error('ddc:vrft:importTimeSeriesData:MissingMatFields', ...
            'source.Type = "mat" requires FilePath, UVariable, and YVariable.');
    end
    if ~isfile(source.FilePath)
        error('ddc:vrft:importTimeSeriesData:FileNotFound', ...
            'File not found: %s', source.FilePath);
    end
    contents = load(source.FilePath, source.UVariable, source.YVariable);
    if ~isfield(contents, source.UVariable) || ~isfield(contents, source.YVariable)
        error('ddc:vrft:importTimeSeriesData:VariableNotFound', ...
            'Variables "%s" and/or "%s" not found in %s.', ...
            source.UVariable, source.YVariable, source.FilePath);
    end
    u = contents.(source.UVariable);
    y = contents.(source.YVariable);
end

function [u, y] = importFromCsvFile(source)
    if ~isfield(source, 'FilePath')
        error('ddc:vrft:importTimeSeriesData:MissingCsvFields', ...
            'source.Type = "csv" requires FilePath.');
    end
    if ~isfile(source.FilePath)
        error('ddc:vrft:importTimeSeriesData:FileNotFound', ...
            'File not found: %s', source.FilePath);
    end
    uCol = 1;
    yCol = 2;
    if isfield(source, 'UColumn') && ~isempty(source.UColumn)
        uCol = source.UColumn;
    end
    if isfield(source, 'YColumn') && ~isempty(source.YColumn)
        yCol = source.YColumn;
    end
    raw = readmatrix(source.FilePath);
    if size(raw, 2) < max(uCol, yCol)
        error('ddc:vrft:importTimeSeriesData:TooFewColumns', ...
            '%s has %d column(s); need at least column %d.', ...
            source.FilePath, size(raw, 2), max(uCol, yCol));
    end
    u = raw(:, uCol);
    y = raw(:, yCol);
end

function [u, y] = validateAndAlignPair(u, y)
    if ~isnumeric(u) || ~isnumeric(y) || ~isvector(u) || ~isvector(y)
        error('ddc:vrft:importTimeSeriesData:NotVectors', ...
            'u and y must be numeric vectors.');
    end
    u = double(u(:));
    y = double(y(:));
    if numel(u) ~= numel(y)
        error('ddc:vrft:importTimeSeriesData:LengthMismatch', ...
            'u (%d samples) and y (%d samples) must have the same length.', ...
            numel(u), numel(y));
    end
    if numel(u) < 2
        error('ddc:vrft:importTimeSeriesData:TooFewSamples', ...
            'u and y must contain at least 2 samples (got %d).', numel(u));
    end
end
