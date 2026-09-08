classdef VRFTDesignApp < handle
    %VRFTDESIGNAPP Interactive Virtual Reference Feedback Tuning design tool.
    %   APP = ddc.vrft.VRFTDesignApp() opens a tabbed uifigure app that wraps
    %   ddc.vrft.vrftDesign for interactive, offline VRFT controller design:
    %
    %     1. Data       - import open-loop (u, y) data and a sample time Ts.
    %     2. Reference  - design or import the closed-loop reference model Md.
    %     3. Basis      - pick a controller-structure preset or type a
    %                     custom discrete basis.
    %     4. Prefilter  - use the classical VRFT prefilter or a custom L(z).
    %     5. Run        - call ddc.vrft.vrftDesign and surface any error as
    %                     a status message / dialog.
    %     6. Results    - inspect theta, C(z,theta), and the VRFT
    %                     diagnostic signals (rv, ev, ef, uf); export.
    %
    %   This class contains ONLY UI construction and callback wiring. All
    %   computation is delegated to small, independently testable
    %   functions in ddc.vrft (importTimeSeriesData, referenceModelFromSpec,
    %   controllerBasisPreset, parseBasisExpression, assembleController,
    %   runVrftDesign), so the algorithmic behavior can be unit-tested
    %   without driving the UI (see tests/tVRFTDesignAppHelpers.m).
    %
    %   Example:
    %       app = ddc.vrft.VRFTDesignApp();
    %
    %   See also ddc.vrft.vrftDesign, ddc.vrft.prefilterDesign.

    properties (Access = private)
        Figure matlab.ui.Figure

        % --- Data tab state ---
        Ts double = 0.1
        DataU double = []
        DataY double = []

        % --- Reference model tab state ---
        Md = []

        % --- Basis tab state ---
        Basis cell = {}

        % --- Prefilter tab state ---
        Prefilter = []

        % --- Results tab state ---
        LastTheta double = []
        LastInfo struct = struct()
        LastController = []

        % --- Theming ---
        Theme struct
    end

    properties (Access = private)
        % Data tab widgets
        DataSourceDropdown
        DataWorkspacePanel
        DataMatPanel
        DataCsvPanel
        UVarField
        YVarField
        MatFileField
        MatUVarField
        MatYVarField
        CsvFileField
        TsField
        DataStatusLabel
        DataAxesU
        DataAxesY

        % Reference model tab widgets
        MdModeDropdown
        MdDesignPanel
        MdImportPanel
        SettlingTimeField
        DampingField
        MdWorkspaceVarField
        MdFileField
        MdStatusLabel
        MdAxes

        % Basis tab widgets
        BasisPresetDropdown
        BasisTable
        BasisStatusLabel

        % Prefilter tab widgets
        PrefilterModeDropdown
        PrefilterExpressionField
        PrefilterStatusLabel

        % Run tab widgets
        RunSummaryArea
        RunButton
        RunStatusLabel

        % Results tab widgets
        ThetaTable
        ControllerArea
        ResultsAxes
        ExportButton
    end

    methods
        function app = VRFTDesignApp()
            %VRFTDESIGNAPP Construct and show the VRFT design app.
            app.Theme = ddc.vrft.vrftAppTheme();
            app.buildUi();
        end

        function delete(app)
            if isvalid(app.Figure)
                delete(app.Figure);
            end
        end

        function fig = getFigure(app)
            %GETFIGURE Return the app's uifigure handle.
            fig = app.Figure;
        end
    end

    methods (Access = private)
        %% ------------------------------------------------------------------
        %  UI construction
        %  ------------------------------------------------------------------
        function buildUi(app)
            app.Figure = uifigure('Name', 'VRFT Design Tool', ...
                'Position', [100 100 1000 680], ...
                'CloseRequestFcn', @(src, ~) delete(app));

            rootGrid = uigridlayout(app.Figure, [2 1]);
            rootGrid.RowHeight = {48, '1x'};
            rootGrid.Padding = [0 0 0 0];
            rootGrid.RowSpacing = 0;

            header = uipanel(rootGrid);
            header.Layout.Row = 1;
            headerGrid = uigridlayout(header, [1 1]);
            uilabel(headerGrid, 'Text', 'Virtual Reference Feedback Tuning (VRFT) Design Tool', ...
                'FontSize', 16, 'FontWeight', 'bold', 'FontColor', app.Theme.primary);

            tabGroup = uitabgroup(rootGrid);
            tabGroup.Layout.Row = 2;

            app.buildDataTab(tabGroup);
            app.buildReferenceModelTab(tabGroup);
            app.buildBasisTab(tabGroup);
            app.buildPrefilterTab(tabGroup);
            app.buildRunTab(tabGroup);
            app.buildResultsTab(tabGroup);
        end

        function buildDataTab(app, tabGroup)
            tab = uitab(tabGroup, 'Title', '1. Data');
            grid = uigridlayout(tab, [1 2]);
            grid.ColumnWidth = {300, '1x'};

            sidePanel = uipanel(grid, 'Title', 'Import Open-Loop Data');
            sidePanel.Layout.Column = 1;
            sideGrid = uigridlayout(sidePanel, [4 1]);
            sideGrid.RowHeight = {'fit', 'fit', 'fit', '1x'};

            uilabel(sideGrid, 'Text', 'Source:');
            app.DataSourceDropdown = uidropdown(sideGrid, ...
                'Items', {'Workspace', 'MAT file', 'CSV file'}, ...
                'ValueChangedFcn', @(src, ~) app.onDataSourceChanged());

            sourceStack = uigridlayout(sideGrid, [1 1]);
            sourceStack.Padding = [0 0 0 0];

            app.DataWorkspacePanel = app.buildWorkspaceSourcePanel(sourceStack);
            app.DataMatPanel = app.buildMatSourcePanel(sourceStack);
            app.DataCsvPanel = app.buildCsvSourcePanel(sourceStack);
            app.DataMatPanel.Visible = 'off';
            app.DataCsvPanel.Visible = 'off';

            bottomGrid = uigridlayout(sideGrid, [3 1]);
            bottomGrid.RowHeight = {'fit', 'fit', 'fit'};
            uilabel(bottomGrid, 'Text', 'Sample time Ts (s):');
            app.TsField = uieditfield(bottomGrid, 'numeric', 'Value', app.Ts, 'Limits', [0 Inf], ...
                'ValueChangedFcn', @(src, ~) app.onTsChanged(src.Value));
            uibutton(bottomGrid, 'Text', 'Load Data', ...
                'ButtonPushedFcn', @(~, ~) app.onLoadDataPressed());

            app.DataStatusLabel = uilabel(sideGrid, 'Text', '', 'FontColor', app.Theme.error);

            plotPanel = uipanel(grid, 'Title', 'Raw Data');
            plotPanel.Layout.Column = 2;
            plotGrid = uigridlayout(plotPanel, [2 1]);
            app.DataAxesU = uiaxes(plotGrid);
            app.DataAxesU.Title.String = 'Input u';
            app.DataAxesY = uiaxes(plotGrid);
            app.DataAxesY.Title.String = 'Output y';
        end

        function panel = buildWorkspaceSourcePanel(app, parent)
            panel = uipanel(parent, 'Title', 'Workspace Variables');
            g = uigridlayout(panel, [2 2]);
            g.ColumnWidth = {'fit', '1x'};
            uilabel(g, 'Text', 'u variable:');
            app.UVarField = uieditfield(g, 'text', 'Value', 'u');
            uilabel(g, 'Text', 'y variable:');
            app.YVarField = uieditfield(g, 'text', 'Value', 'y');
        end

        function panel = buildMatSourcePanel(app, parent)
            panel = uipanel(parent, 'Title', 'MAT File');
            g = uigridlayout(panel, [4 2]);
            g.ColumnWidth = {'fit', '1x'};
            uilabel(g, 'Text', 'File:');
            fileRow = uigridlayout(g, [1 2]);
            fileRow.ColumnWidth = {'1x', 'fit'};
            fileRow.Padding = [0 0 0 0];
            app.MatFileField = uieditfield(fileRow, 'text');
            uibutton(fileRow, 'Text', 'Browse...', ...
                'ButtonPushedFcn', @(~, ~) app.onBrowseFile(app.MatFileField, {'*.mat'}));
            uilabel(g, 'Text', 'u variable:');
            app.MatUVarField = uieditfield(g, 'text', 'Value', 'u');
            uilabel(g, 'Text', 'y variable:');
            app.MatYVarField = uieditfield(g, 'text', 'Value', 'y');
        end

        function panel = buildCsvSourcePanel(app, parent)
            panel = uipanel(parent, 'Title', 'CSV File (2 columns: u, y)');
            g = uigridlayout(panel, [2 2]);
            g.ColumnWidth = {'fit', '1x'};
            uilabel(g, 'Text', 'File:');
            fileRow = uigridlayout(g, [1 2]);
            fileRow.ColumnWidth = {'1x', 'fit'};
            fileRow.Padding = [0 0 0 0];
            app.CsvFileField = uieditfield(fileRow, 'text');
            uibutton(fileRow, 'Text', 'Browse...', ...
                'ButtonPushedFcn', @(~, ~) app.onBrowseFile(app.CsvFileField, {'*.csv'}));
        end

        function buildReferenceModelTab(app, tabGroup)
            tab = uitab(tabGroup, 'Title', '2. Reference Model');
            grid = uigridlayout(tab, [1 2]);
            grid.ColumnWidth = {300, '1x'};

            sidePanel = uipanel(grid, 'Title', 'Reference Model Md');
            sidePanel.Layout.Column = 1;
            sideGrid = uigridlayout(sidePanel, [4 1]);
            sideGrid.RowHeight = {'fit', 'fit', 'fit', '1x'};

            uilabel(sideGrid, 'Text', 'Mode:');
            app.MdModeDropdown = uidropdown(sideGrid, ...
                'Items', {'Design (2nd-order)', 'Import from workspace/.mat'}, ...
                'ValueChangedFcn', @(src, ~) app.onMdModeChanged());

            modeStack = uigridlayout(sideGrid, [1 1]);
            modeStack.Padding = [0 0 0 0];
            app.MdDesignPanel = app.buildMdDesignPanel(modeStack);
            app.MdImportPanel = app.buildMdImportPanel(modeStack);
            app.MdImportPanel.Visible = 'off';

            uibutton(sideGrid, 'Text', 'Build / Import Md', ...
                'ButtonPushedFcn', @(~, ~) app.onBuildMdPressed());

            app.MdStatusLabel = uilabel(sideGrid, 'Text', '', 'FontColor', app.Theme.error);

            plotPanel = uipanel(grid, 'Title', 'Md Step Response');
            plotPanel.Layout.Column = 2;
            plotGrid = uigridlayout(plotPanel, [1 1]);
            app.MdAxes = uiaxes(plotGrid);
        end

        function panel = buildMdDesignPanel(app, parent)
            panel = uipanel(parent, 'Title', 'Design from Specs');
            g = uigridlayout(panel, [2 2]);
            g.ColumnWidth = {'fit', '1x'};
            uilabel(g, 'Text', 'Settling time (s):');
            app.SettlingTimeField = uieditfield(g, 'numeric', 'Value', 2, 'Limits', [0 Inf]);
            uilabel(g, 'Text', 'Damping ratio:');
            app.DampingField = uieditfield(g, 'numeric', 'Value', 0.8, 'Limits', [0 1]);
        end

        function panel = buildMdImportPanel(app, parent)
            panel = uipanel(parent, 'Title', 'Import Existing Md');
            g = uigridlayout(panel, [2 2]);
            g.ColumnWidth = {'fit', '1x'};
            uilabel(g, 'Text', 'Workspace var:');
            app.MdWorkspaceVarField = uieditfield(g, 'text', 'Value', 'Md');
            uilabel(g, 'Text', 'or .mat file:');
            fileRow = uigridlayout(g, [1 2]);
            fileRow.ColumnWidth = {'1x', 'fit'};
            fileRow.Padding = [0 0 0 0];
            app.MdFileField = uieditfield(fileRow, 'text');
            uibutton(fileRow, 'Text', 'Browse...', ...
                'ButtonPushedFcn', @(~, ~) app.onBrowseFile(app.MdFileField, {'*.mat'}));
        end

        function buildBasisTab(app, tabGroup)
            tab = uitab(tabGroup, 'Title', '3. Controller Structure');
            grid = uigridlayout(tab, [3 1]);
            grid.RowHeight = {'fit', '1x', 'fit'};

            topGrid = uigridlayout(grid, [1 3]);
            topGrid.ColumnWidth = {'fit', 'fit', 'fit'};
            uilabel(topGrid, 'Text', 'Preset:');
            app.BasisPresetDropdown = uidropdown(topGrid, ...
                'Items', {'PID', 'Integrator', 'Gain', 'Custom'}, ...
                'ValueChangedFcn', @(src, ~) app.onBasisPresetChanged(src.Value));
            uibutton(topGrid, 'Text', 'Apply Basis', ...
                'ButtonPushedFcn', @(~, ~) app.onApplyBasisPressed());

            app.BasisTable = uitable(grid, ...
                'ColumnName', {'Index', 'Basis Expression (function of z, Ts)'}, ...
                'ColumnEditable', [false true], ...
                'Data', table((1:3)', {'1'; 'Ts/(z-1)'; '(z-1)/(Ts*z)'}, ...
                    'VariableNames', {'Index', 'Expression'}));
            app.BasisTable.Enable = 'off';

            app.BasisStatusLabel = uilabel(grid, 'Text', '', 'FontColor', app.Theme.error);
        end

        function buildPrefilterTab(app, tabGroup)
            tab = uitab(tabGroup, 'Title', '4. Prefilter');
            grid = uigridlayout(tab, [3 1]);
            grid.RowHeight = {'fit', 'fit', 'fit'};

            topGrid = uigridlayout(grid, [1 2]);
            topGrid.ColumnWidth = {'fit', 'fit'};
            uilabel(topGrid, 'Text', 'Mode:');
            app.PrefilterModeDropdown = uidropdown(topGrid, ...
                'Items', {'Default (prefilterDesign(Md))', 'Custom L(z)'}, ...
                'ValueChangedFcn', @(src, ~) app.onPrefilterModeChanged(src.Value));

            exprGrid = uigridlayout(grid, [1 2]);
            exprGrid.ColumnWidth = {'fit', '1x'};
            uilabel(exprGrid, 'Text', 'L(z) expression:');
            app.PrefilterExpressionField = uieditfield(exprGrid, 'text', ...
                'Value', 'Ts/(z-1)', 'Enable', 'off');

            app.PrefilterStatusLabel = uilabel(grid, 'Text', '', 'FontColor', app.Theme.error);
        end

        function buildRunTab(app, tabGroup)
            tab = uitab(tabGroup, 'Title', '5. Run');
            grid = uigridlayout(tab, [3 1]);
            grid.RowHeight = {'1x', 'fit', 'fit'};

            app.RunSummaryArea = uitextarea(grid, 'Editable', 'off');

            app.RunButton = uibutton(grid, 'Text', 'Design Controller', ...
                'ButtonPushedFcn', @(~, ~) app.onDesignControllerPressed());

            app.RunStatusLabel = uilabel(grid, 'Text', '');
        end

        function buildResultsTab(app, tabGroup)
            tab = uitab(tabGroup, 'Title', '6. Results');
            grid = uigridlayout(tab, [2 2]);
            grid.RowHeight = {'fit', '1x'};
            grid.ColumnWidth = {320, '1x'};

            leftGrid = uigridlayout(grid, [3 1]);
            leftGrid.Layout.Row = [1 2];
            leftGrid.Layout.Column = 1;
            leftGrid.RowHeight = {'fit', 'fit', 'fit'};

            app.ThetaTable = uitable(leftGrid, ...
                'ColumnName', {'Basis Index', 'Coefficient'});

            app.ControllerArea = uitextarea(leftGrid, 'Editable', 'off');

            app.ExportButton = uibutton(leftGrid, 'Text', 'Export...', ...
                'ButtonPushedFcn', @(~, ~) app.onExportPressed(), 'Enable', 'off');

            plotPanel = uipanel(grid, 'Title', 'VRFT Diagnostic Signals');
            plotPanel.Layout.Row = [1 2];
            plotPanel.Layout.Column = 2;
            plotGrid = uigridlayout(plotPanel, [2 2]);
            app.ResultsAxes = struct( ...
                'RvY', uiaxes(plotGrid), ...
                'Ev', uiaxes(plotGrid), ...
                'Ef', uiaxes(plotGrid), ...
                'Uf', uiaxes(plotGrid));
            app.ResultsAxes.RvY.Title.String = 'Virtual Reference r_v vs y';
            app.ResultsAxes.Ev.Title.String = 'Virtual Error e_v';
            app.ResultsAxes.Ef.Title.String = 'Filtered Error e_f';
            app.ResultsAxes.Uf.Title.String = 'Filtered Input u_f';
        end

        %% ------------------------------------------------------------------
        %  Data tab callbacks
        %  ------------------------------------------------------------------
        function onDataSourceChanged(app)
            source = app.DataSourceDropdown.Value;
            app.DataWorkspacePanel.Visible = 'off';
            app.DataMatPanel.Visible = 'off';
            app.DataCsvPanel.Visible = 'off';
            switch source
                case 'Workspace'
                    app.DataWorkspacePanel.Visible = 'on';
                case 'MAT file'
                    app.DataMatPanel.Visible = 'on';
                case 'CSV file'
                    app.DataCsvPanel.Visible = 'on';
            end
        end

        function onTsChanged(app, value)
            app.Ts = value;
        end

        function onLoadDataPressed(app)
            app.DataStatusLabel.Text = '';
            try
                source = app.buildDataSourceStruct();
                data = ddc.vrft.importTimeSeriesData(source);
            catch ME
                app.DataStatusLabel.Text = ME.message;
                return;
            end

            app.DataU = data.U;
            app.DataY = data.Y;
            t = (0:data.NumSamples - 1)' * app.Ts;

            plot(app.DataAxesU, t, app.DataU, 'Color', app.Theme.plotColors(1, :), 'LineWidth', 1.2);
            app.DataAxesU.Title.String = 'Input u';
            app.DataAxesU.XLabel.String = 'Time (s)';
            grid(app.DataAxesU, 'on');

            plot(app.DataAxesY, t, app.DataY, 'Color', app.Theme.plotColors(2, :), 'LineWidth', 1.2);
            app.DataAxesY.Title.String = 'Output y';
            app.DataAxesY.XLabel.String = 'Time (s)';
            grid(app.DataAxesY, 'on');

            app.DataStatusLabel.FontColor = app.Theme.success;
            app.DataStatusLabel.Text = sprintf('Loaded %d samples.', data.NumSamples);
            app.updateRunSummary();
        end

        function source = buildDataSourceStruct(app)
            switch app.DataSourceDropdown.Value
                case 'Workspace'
                    source.Type = "workspace";
                    source.U = app.readWorkspaceVariable(app.UVarField.Value);
                    source.Y = app.readWorkspaceVariable(app.YVarField.Value);
                case 'MAT file'
                    source.Type = "mat";
                    source.FilePath = app.MatFileField.Value;
                    source.UVariable = app.MatUVarField.Value;
                    source.YVariable = app.MatYVarField.Value;
                case 'CSV file'
                    source.Type = "csv";
                    source.FilePath = app.CsvFileField.Value;
            end
        end

        %% ------------------------------------------------------------------
        %  Reference model tab callbacks
        %  ------------------------------------------------------------------
        function onMdModeChanged(app)
            isDesign = strcmp(app.MdModeDropdown.Value, 'Design (2nd-order)');
            app.MdDesignPanel.Visible = matlab.lang.OnOffSwitchState(isDesign);
            app.MdImportPanel.Visible = matlab.lang.OnOffSwitchState(~isDesign);
        end

        function onBuildMdPressed(app)
            app.MdStatusLabel.Text = '';
            try
                if strcmp(app.MdModeDropdown.Value, 'Design (2nd-order)')
                    spec.SettlingTime = app.SettlingTimeField.Value;
                    spec.DampingRatio = app.DampingField.Value;
                    spec.Ts = app.Ts;
                    mdModel = ddc.vrft.referenceModelFromSpec(spec);
                else
                    mdModel = app.importMdFromImportPanel();
                end
            catch ME
                app.MdStatusLabel.Text = ME.message;
                return;
            end

            app.Md = mdModel;
            [y, t] = step(mdModel);
            plot(app.MdAxes, t, y, 'Color', app.Theme.plotColors(3, :), 'LineWidth', 1.2);
            app.MdAxes.Title.String = 'Md Step Response';
            app.MdAxes.XLabel.String = 'Time (s)';
            app.MdAxes.YLabel.String = 'y';
            grid(app.MdAxes, 'on');

            app.MdStatusLabel.FontColor = app.Theme.success;
            app.MdStatusLabel.Text = 'Md set.';
            app.updateRunSummary();
        end

        function mdModel = importMdFromImportPanel(app)
            filePath = strtrim(app.MdFileField.Value);
            if ~isempty(filePath)
                if ~isfile(filePath)
                    error('ddc:vrft:VRFTDesignApp:MdFileNotFound', ...
                        'File not found: %s', filePath);
                end
                varName = strtrim(app.MdWorkspaceVarField.Value);
                contents = load(filePath);
                fields = fieldnames(contents);
                if isempty(fields)
                    error('ddc:vrft:VRFTDesignApp:MdFileEmpty', ...
                        'MAT file %s contains no variables.', filePath);
                end
                if isfield(contents, varName)
                    mdModel = contents.(varName);
                else
                    mdModel = contents.(fields{1});
                end
            else
                mdModel = app.readWorkspaceVariable(app.MdWorkspaceVarField.Value);
            end

            if ~(isa(mdModel, 'DynamicSystem') || isa(mdModel, 'lti'))
                error('ddc:vrft:VRFTDesignApp:InvalidMd', ...
                    'Imported Md is not an LTI model (tf/zpk/ss).');
            end
            if mdModel.Ts == 0
                error('ddc:vrft:VRFTDesignApp:ContinuousMd', ...
                    'Imported Md must be discrete-time (Ts > 0).');
            end
        end

        %% ------------------------------------------------------------------
        %  Basis tab callbacks
        %  ------------------------------------------------------------------
        function onBasisPresetChanged(app, presetName)
            isCustom = strcmp(presetName, 'Custom');
            app.BasisTable.Enable = matlab.lang.OnOffSwitchState(isCustom);
            app.BasisTable.ColumnEditable = [false, isCustom];
        end

        function onApplyBasisPressed(app)
            app.BasisStatusLabel.Text = '';
            try
                presetName = app.BasisPresetDropdown.Value;
                if strcmp(presetName, 'Custom')
                    expressions = string(app.BasisTable.Data.Expression);
                    basis = cell(numel(expressions), 1);
                    for i = 1:numel(expressions)
                        basis{i} = ddc.vrft.parseBasisExpression(expressions(i), app.Ts);
                    end
                else
                    basis = ddc.vrft.controllerBasisPreset(presetName, app.Ts);
                end
            catch ME
                app.BasisStatusLabel.Text = ME.message;
                return;
            end

            app.Basis = basis;
            app.BasisStatusLabel.FontColor = app.Theme.success;
            app.BasisStatusLabel.Text = sprintf('Basis set: %d entries.', numel(basis));
            app.updateRunSummary();
        end

        %% ------------------------------------------------------------------
        %  Prefilter tab callbacks
        %  ------------------------------------------------------------------
        function onPrefilterModeChanged(app, modeValue)
            isCustom = strcmp(modeValue, 'Custom L(z)');
            app.PrefilterExpressionField.Enable = matlab.lang.OnOffSwitchState(isCustom);
            app.PrefilterStatusLabel.Text = '';
            if ~isCustom
                app.Prefilter = [];
                app.updateRunSummary();
            end
        end

        %% ------------------------------------------------------------------
        %  Run tab callbacks
        %  ------------------------------------------------------------------
        function updateRunSummary(app)
            lines = {};
            if isempty(app.DataU)
                lines{end+1} = 'Data: not loaded';
            else
                lines{end+1} = sprintf('Data: %d samples, Ts = %.4g s', numel(app.DataU), app.Ts);
            end
            if isempty(app.Md)
                lines{end+1} = 'Reference model Md: not set';
            else
                lines{end+1} = 'Reference model Md: set';
            end
            if isempty(app.Basis)
                lines{end+1} = 'Controller basis: not set';
            else
                lines{end+1} = sprintf('Controller basis: %d entries', numel(app.Basis));
            end
            if strcmp(app.PrefilterModeDropdown.Value, 'Custom L(z)')
                lines{end+1} = sprintf('Prefilter: custom, L(z) = %s', app.PrefilterExpressionField.Value);
            else
                lines{end+1} = 'Prefilter: default (ddc.vrft.prefilterDesign(Md))';
            end
            app.RunSummaryArea.Value = lines;

            ready = ~isempty(app.DataU) && ~isempty(app.Md) && ~isempty(app.Basis);
            app.RunButton.Enable = matlab.lang.OnOffSwitchState(ready);
        end

        function onDesignControllerPressed(app)
            app.RunStatusLabel.FontColor = [0 0 0];
            app.RunStatusLabel.Text = 'Designing...';
            drawnow;

            prefilter = [];
            if strcmp(app.PrefilterModeDropdown.Value, 'Custom L(z)')
                try
                    prefilter = ddc.vrft.parseBasisExpression(app.PrefilterExpressionField.Value, app.Ts);
                catch ME
                    app.RunStatusLabel.FontColor = app.Theme.error;
                    app.RunStatusLabel.Text = ME.message;
                    return;
                end
            end

            result = ddc.vrft.runVrftDesign(app.DataU, app.DataY, app.Md, app.Basis, prefilter);

            if ~result.Success
                app.RunStatusLabel.FontColor = app.Theme.error;
                app.RunStatusLabel.Text = result.ErrorMessage;
                uialert(app.Figure, result.ErrorMessage, 'VRFT Design Failed');
                return;
            end

            app.LastTheta = result.Theta;
            app.LastInfo = result.Info;
            app.LastController = ddc.vrft.assembleController(result.Theta, app.Basis);

            app.RunStatusLabel.FontColor = app.Theme.success;
            app.RunStatusLabel.Text = 'Controller designed successfully. See Results tab.';
            app.populateResultsTab();
        end

        %% ------------------------------------------------------------------
        %  Results tab
        %  ------------------------------------------------------------------
        function populateResultsTab(app)
            nBasis = numel(app.LastTheta);
            app.ThetaTable.Data = table((1:nBasis)', app.LastTheta, ...
                'VariableNames', {'BasisIndex', 'Coefficient'});

            app.ControllerArea.Value = app.formatControllerText(app.LastController);

            info = app.LastInfo;
            tv = (0:numel(info.rv) - 1)' * app.Ts;

            axRvY = app.ResultsAxes.RvY;
            plot(axRvY, tv, info.rv, 'Color', app.Theme.plotColors(1, :), 'LineWidth', 1.2);
            hold(axRvY, 'on');
            plot(axRvY, tv, app.DataY(1:numel(info.rv)), '--', 'Color', app.Theme.plotColors(2, :), 'LineWidth', 1.2);
            hold(axRvY, 'off');
            legend(axRvY, {'r_v', 'y'}, 'Location', 'best');
            grid(axRvY, 'on');

            plot(app.ResultsAxes.Ev, tv, info.ev, 'Color', app.Theme.plotColors(3, :), 'LineWidth', 1.2);
            grid(app.ResultsAxes.Ev, 'on');

            plot(app.ResultsAxes.Ef, tv, info.ef, 'Color', app.Theme.plotColors(4, :), 'LineWidth', 1.2);
            grid(app.ResultsAxes.Ef, 'on');

            plot(app.ResultsAxes.Uf, tv, info.uf, 'Color', app.Theme.plotColors(5, :), 'LineWidth', 1.2);
            grid(app.ResultsAxes.Uf, 'on');

            app.ExportButton.Enable = 'on';
        end

        function onExportPressed(app)
            choice = uiconfirm(app.Figure, ...
                'Export results to the base workspace or to a .mat file?', ...
                'Export VRFT Results', ...
                'Options', {'Workspace', 'MAT file', 'Cancel'}, ...
                'DefaultOption', 'Workspace', 'CancelOption', 'Cancel');

            switch choice
                case 'Workspace'
                    assignin('base', 'theta', app.LastTheta);
                    assignin('base', 'C', app.LastController);
                    assignin('base', 'info', app.LastInfo);
                    uialert(app.Figure, 'Exported theta, C, and info to the base workspace.', ...
                        'Export Complete', 'Icon', 'success');
                case 'MAT file'
                    [fileName, pathName] = uiputfile('*.mat', 'Save VRFT Results');
                    if isequal(fileName, 0)
                        return;
                    end
                    theta = app.LastTheta;
                    C = app.LastController;
                    info = app.LastInfo;
                    save(fullfile(pathName, fileName), 'theta', 'C', 'info');
                    uialert(app.Figure, sprintf('Saved to %s', fullfile(pathName, fileName)), ...
                        'Export Complete', 'Icon', 'success');
            end
        end
    end

    methods (Access = private, Static)
        function value = readWorkspaceVariable(varName)
            varName = strtrim(varName);
            if isempty(varName)
                error('ddc:vrft:VRFTDesignApp:EmptyVariableName', ...
                    'Variable name must not be empty.');
            end
            if ~evalin('base', sprintf('exist(''%s'', ''var'')', varName))
                error('ddc:vrft:VRFTDesignApp:VariableNotFound', ...
                    'Variable "%s" not found in the base workspace.', varName);
            end
            value = evalin('base', varName);
        end

        function onBrowseFile(targetField, extensions)
            [fileName, pathName] = uigetfile(extensions);
            if isequal(fileName, 0)
                return;
            end
            targetField.Value = fullfile(pathName, fileName);
        end

        function text = formatControllerText(C)
            [num, den] = tfdata(C, 'v');
            text = {sprintf('C(z, theta):'), '', ...
                sprintf('Numerator:   %s', mat2str(round(num, 6))), ...
                sprintf('Denominator: %s', mat2str(round(den, 6))), ...
                sprintf('Ts = %.4g s', C.Ts)};
        end
    end
end
