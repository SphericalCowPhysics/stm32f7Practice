namespace F7SkeletonNS {
    partial class MainForm {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose( bool disposing ) {
            if (disposing && (components != null)) {
                components.Dispose();
            }
            base.Dispose( disposing );
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent() {
            this.components = new System.ComponentModel.Container();
            System.Windows.Forms.DataVisualization.Charting.ChartArea chartArea1 = new System.Windows.Forms.DataVisualization.Charting.ChartArea();
            System.Windows.Forms.DataVisualization.Charting.CustomLabel customLabel1 = new System.Windows.Forms.DataVisualization.Charting.CustomLabel();
            System.Windows.Forms.DataVisualization.Charting.Legend legend1 = new System.Windows.Forms.DataVisualization.Charting.Legend();
            this.openFileDialog1 = new System.Windows.Forms.OpenFileDialog();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.saveFileDialog1 = new System.Windows.Forms.SaveFileDialog();
            this.quit = new System.Windows.Forms.Button();
            this.guiRev = new System.Windows.Forms.Label();
            this.comboBox_CommPort = new System.Windows.Forms.ComboBox();
            this.label1 = new System.Windows.Forms.Label();
            this.checkBox_Connect = new System.Windows.Forms.CheckBox();
            this.nvParamsPage = new System.Windows.Forms.TabPage();
            this.fwProg = new System.Windows.Forms.Label();
            this.fromFile = new System.Windows.Forms.Button();
            this.toFile = new System.Windows.Forms.Button();
            this.toFlash = new System.Windows.Forms.Button();
            this.parameters = new System.Windows.Forms.DataGridView();
            this.Parameter = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.pName = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.pValue = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.newValue = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.regsPage = new System.Windows.Forms.TabPage();
            this.chart1 = new System.Windows.Forms.DataVisualization.Charting.Chart();
            this.unCheckAll = new System.Windows.Forms.Button();
            this.statusList = new System.Windows.Forms.DataGridView();
            this.selected = new System.Windows.Forms.DataGridViewCheckBoxColumn();
            this.name = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.value = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.newVal = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.plot = new System.Windows.Forms.CheckBox();
            this.read = new System.Windows.Forms.Button();
            this.period = new System.Windows.Forms.NumericUpDown();
            this.maxY = new System.Windows.Forms.NumericUpDown();
            this.label36 = new System.Windows.Forms.Label();
            this.minY = new System.Windows.Forms.NumericUpDown();
            this.label71 = new System.Windows.Forms.Label();
            this.label67 = new System.Windows.Forms.Label();
            this.nSamples = new System.Windows.Forms.NumericUpDown();
            this.widX = new System.Windows.Forms.NumericUpDown();
            this.log = new System.Windows.Forms.CheckBox();
            this.label80 = new System.Windows.Forms.Label();
            this.autoScale = new System.Windows.Forms.CheckBox();
            this.tabControl_ctrl = new System.Windows.Forms.TabControl();
            this.nvParamsPage.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.parameters)).BeginInit();
            this.regsPage.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.chart1)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.statusList)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.period)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.maxY)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.minY)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.nSamples)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.widX)).BeginInit();
            this.tabControl_ctrl.SuspendLayout();
            this.SuspendLayout();
            // 
            // openFileDialog1
            // 
            this.openFileDialog1.DefaultExt = "csv";
            this.openFileDialog1.Filter = "csv files|*.csv|all files|*.*";
            // 
            // timer1
            // 
            this.timer1.Interval = 500;
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // saveFileDialog1
            // 
            this.saveFileDialog1.DefaultExt = "csv";
            this.saveFileDialog1.Filter = "csv files|*.csv|all files|*.*";
            // 
            // quit
            // 
            this.quit.Location = new System.Drawing.Point(1238, 727);
            this.quit.Margin = new System.Windows.Forms.Padding(4);
            this.quit.Name = "quit";
            this.quit.Size = new System.Drawing.Size(100, 28);
            this.quit.TabIndex = 112;
            this.quit.Text = "Quit";
            this.quit.UseVisualStyleBackColor = true;
            this.quit.Click += new System.EventHandler(this.quit_Click);
            // 
            // guiRev
            // 
            this.guiRev.AutoSize = true;
            this.guiRev.Location = new System.Drawing.Point(18, 736);
            this.guiRev.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.guiRev.Name = "guiRev";
            this.guiRev.Size = new System.Drawing.Size(51, 16);
            this.guiRev.TabIndex = 113;
            this.guiRev.Text = "Rev XX";
            // 
            // comboBox_CommPort
            // 
            this.comboBox_CommPort.FormattingEnabled = true;
            this.comboBox_CommPort.Location = new System.Drawing.Point(228, 733);
            this.comboBox_CommPort.Margin = new System.Windows.Forms.Padding(4);
            this.comboBox_CommPort.Name = "comboBox_CommPort";
            this.comboBox_CommPort.Size = new System.Drawing.Size(160, 24);
            this.comboBox_CommPort.TabIndex = 118;
            this.comboBox_CommPort.SelectedIndexChanged += new System.EventHandler(this.comboBox_CommPort_SelectedIndexChanged);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(152, 736);
            this.label1.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(62, 16);
            this.label1.TabIndex = 119;
            this.label1.Text = "Com Port";
            // 
            // checkBox_Connect
            // 
            this.checkBox_Connect.AutoSize = true;
            this.checkBox_Connect.Location = new System.Drawing.Point(396, 735);
            this.checkBox_Connect.Margin = new System.Windows.Forms.Padding(4);
            this.checkBox_Connect.Name = "checkBox_Connect";
            this.checkBox_Connect.Size = new System.Drawing.Size(78, 20);
            this.checkBox_Connect.TabIndex = 120;
            this.checkBox_Connect.Text = "Connect";
            this.checkBox_Connect.UseVisualStyleBackColor = true;
            this.checkBox_Connect.CheckedChanged += new System.EventHandler(this.checkBox_Connect_CheckedChanged);
            // 
            // nvParamsPage
            // 
            this.nvParamsPage.Controls.Add(this.fwProg);
            this.nvParamsPage.Controls.Add(this.fromFile);
            this.nvParamsPage.Controls.Add(this.toFile);
            this.nvParamsPage.Controls.Add(this.toFlash);
            this.nvParamsPage.Controls.Add(this.parameters);
            this.nvParamsPage.Location = new System.Drawing.Point(4, 25);
            this.nvParamsPage.Margin = new System.Windows.Forms.Padding(4);
            this.nvParamsPage.Name = "nvParamsPage";
            this.nvParamsPage.Padding = new System.Windows.Forms.Padding(4);
            this.nvParamsPage.Size = new System.Drawing.Size(1318, 576);
            this.nvParamsPage.TabIndex = 1;
            this.nvParamsPage.Text = "NvParams";
            this.nvParamsPage.UseVisualStyleBackColor = true;
            // 
            // fwProg
            // 
            this.fwProg.AutoSize = true;
            this.fwProg.Location = new System.Drawing.Point(580, 7);
            this.fwProg.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.fwProg.Name = "fwProg";
            this.fwProg.Size = new System.Drawing.Size(0, 16);
            this.fwProg.TabIndex = 111;
            // 
            // fromFile
            // 
            this.fromFile.Location = new System.Drawing.Point(242, 684);
            this.fromFile.Margin = new System.Windows.Forms.Padding(4);
            this.fromFile.Name = "fromFile";
            this.fromFile.Size = new System.Drawing.Size(110, 28);
            this.fromFile.TabIndex = 102;
            this.fromFile.Text = "Load from File";
            this.fromFile.UseVisualStyleBackColor = true;
            this.fromFile.Click += new System.EventHandler(this.fromFile_Click);
            // 
            // toFile
            // 
            this.toFile.Location = new System.Drawing.Point(126, 684);
            this.toFile.Margin = new System.Windows.Forms.Padding(4);
            this.toFile.Name = "toFile";
            this.toFile.Size = new System.Drawing.Size(110, 28);
            this.toFile.TabIndex = 101;
            this.toFile.Text = "Save to File";
            this.toFile.UseVisualStyleBackColor = true;
            this.toFile.Click += new System.EventHandler(this.toFile_Click);
            // 
            // toFlash
            // 
            this.toFlash.Location = new System.Drawing.Point(8, 684);
            this.toFlash.Margin = new System.Windows.Forms.Padding(4);
            this.toFlash.Name = "toFlash";
            this.toFlash.Size = new System.Drawing.Size(110, 28);
            this.toFlash.TabIndex = 100;
            this.toFlash.Text = "Save to Flash";
            this.toFlash.UseVisualStyleBackColor = true;
            this.toFlash.Click += new System.EventHandler(this.toFlash_Click);
            // 
            // parameters
            // 
            this.parameters.AllowUserToAddRows = false;
            this.parameters.AllowUserToDeleteRows = false;
            this.parameters.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize;
            this.parameters.Columns.AddRange(new System.Windows.Forms.DataGridViewColumn[] {
            this.Parameter,
            this.pName,
            this.pValue,
            this.newValue});
            this.parameters.Location = new System.Drawing.Point(8, 7);
            this.parameters.Margin = new System.Windows.Forms.Padding(4);
            this.parameters.Name = "parameters";
            this.parameters.RowHeadersWidth = 51;
            this.parameters.Size = new System.Drawing.Size(564, 561);
            this.parameters.TabIndex = 99;
            this.parameters.CellValueChanged += new System.Windows.Forms.DataGridViewCellEventHandler(this.parameters_CellValueChanged);
            // 
            // Parameter
            // 
            this.Parameter.HeaderText = "Parameter";
            this.Parameter.MinimumWidth = 6;
            this.Parameter.Name = "Parameter";
            this.Parameter.ReadOnly = true;
            this.Parameter.Width = 30;
            // 
            // pName
            // 
            this.pName.HeaderText = "Name";
            this.pName.MinimumWidth = 6;
            this.pName.Name = "pName";
            this.pName.ReadOnly = true;
            this.pName.Width = 150;
            // 
            // pValue
            // 
            this.pValue.HeaderText = "Value";
            this.pValue.MinimumWidth = 6;
            this.pValue.Name = "pValue";
            this.pValue.ReadOnly = true;
            this.pValue.Width = 90;
            // 
            // newValue
            // 
            this.newValue.HeaderText = "New Value";
            this.newValue.MinimumWidth = 6;
            this.newValue.Name = "newValue";
            this.newValue.Width = 90;
            // 
            // regsPage
            // 
            this.regsPage.Controls.Add(this.chart1);
            this.regsPage.Controls.Add(this.unCheckAll);
            this.regsPage.Controls.Add(this.statusList);
            this.regsPage.Controls.Add(this.plot);
            this.regsPage.Controls.Add(this.read);
            this.regsPage.Controls.Add(this.period);
            this.regsPage.Controls.Add(this.maxY);
            this.regsPage.Controls.Add(this.label36);
            this.regsPage.Controls.Add(this.minY);
            this.regsPage.Controls.Add(this.label71);
            this.regsPage.Controls.Add(this.label67);
            this.regsPage.Controls.Add(this.nSamples);
            this.regsPage.Controls.Add(this.widX);
            this.regsPage.Controls.Add(this.log);
            this.regsPage.Controls.Add(this.label80);
            this.regsPage.Controls.Add(this.autoScale);
            this.regsPage.Location = new System.Drawing.Point(4, 25);
            this.regsPage.Margin = new System.Windows.Forms.Padding(4);
            this.regsPage.Name = "regsPage";
            this.regsPage.Padding = new System.Windows.Forms.Padding(4);
            this.regsPage.Size = new System.Drawing.Size(1318, 675);
            this.regsPage.TabIndex = 0;
            this.regsPage.Text = "Regs";
            this.regsPage.UseVisualStyleBackColor = true;
            // 
            // chart1
            // 
            customLabel1.Text = "customxaxis";
            chartArea1.AxisX.CustomLabels.Add(customLabel1);
            chartArea1.AxisX.Maximum = 500D;
            chartArea1.AxisX.Minimum = 0D;
            chartArea1.AxisX.Title = "Time";
            chartArea1.AxisY.Title = "arbritrary-labels";
            chartArea1.Name = "ChartArea1";
            this.chart1.ChartAreas.Add(chartArea1);
            legend1.Docking = System.Windows.Forms.DataVisualization.Charting.Docking.Bottom;
            legend1.Name = "Legend1";
            this.chart1.Legends.Add(legend1);
            this.chart1.Location = new System.Drawing.Point(552, 45);
            this.chart1.Margin = new System.Windows.Forms.Padding(4);
            this.chart1.Name = "chart1";
            this.chart1.Size = new System.Drawing.Size(754, 622);
            this.chart1.TabIndex = 103;
            this.chart1.Text = "chart1";
            // 
            // unCheckAll
            // 
            this.unCheckAll.Location = new System.Drawing.Point(8, 10);
            this.unCheckAll.Margin = new System.Windows.Forms.Padding(4);
            this.unCheckAll.Name = "unCheckAll";
            this.unCheckAll.Size = new System.Drawing.Size(38, 28);
            this.unCheckAll.TabIndex = 102;
            this.unCheckAll.Text = "un";
            this.unCheckAll.UseVisualStyleBackColor = true;
            this.unCheckAll.Click += new System.EventHandler(this.unCheckAll_Click);
            // 
            // statusList
            // 
            this.statusList.AllowUserToAddRows = false;
            this.statusList.AllowUserToDeleteRows = false;
            this.statusList.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize;
            this.statusList.Columns.AddRange(new System.Windows.Forms.DataGridViewColumn[] {
            this.selected,
            this.name,
            this.value,
            this.newVal});
            this.statusList.Enabled = false;
            this.statusList.Location = new System.Drawing.Point(8, 45);
            this.statusList.Margin = new System.Windows.Forms.Padding(4);
            this.statusList.Name = "statusList";
            this.statusList.RowHeadersWidth = 51;
            this.statusList.Size = new System.Drawing.Size(536, 622);
            this.statusList.TabIndex = 88;
            this.statusList.CellValueChanged += new System.Windows.Forms.DataGridViewCellEventHandler(this.statusList_CellValueChanged);
            // 
            // selected
            // 
            this.selected.HeaderText = "Sel";
            this.selected.MinimumWidth = 6;
            this.selected.Name = "selected";
            this.selected.Width = 40;
            // 
            // name
            // 
            this.name.HeaderText = "Name";
            this.name.MinimumWidth = 6;
            this.name.Name = "name";
            this.name.ReadOnly = true;
            this.name.Width = 150;
            // 
            // value
            // 
            this.value.HeaderText = "Value";
            this.value.MinimumWidth = 6;
            this.value.Name = "value";
            this.value.ReadOnly = true;
            this.value.Width = 75;
            // 
            // newVal
            // 
            this.newVal.HeaderText = "New Value";
            this.newVal.MinimumWidth = 6;
            this.newVal.Name = "newVal";
            this.newVal.Width = 75;
            // 
            // plot
            // 
            this.plot.AutoSize = true;
            this.plot.Location = new System.Drawing.Point(270, 15);
            this.plot.Margin = new System.Windows.Forms.Padding(4);
            this.plot.Name = "plot";
            this.plot.Size = new System.Drawing.Size(52, 20);
            this.plot.TabIndex = 89;
            this.plot.Text = "Plot";
            this.plot.UseVisualStyleBackColor = true;
            this.plot.CheckedChanged += new System.EventHandler(this.plot_CheckedChanged);
            // 
            // read
            // 
            this.read.Location = new System.Drawing.Point(54, 10);
            this.read.Margin = new System.Windows.Forms.Padding(4);
            this.read.Name = "read";
            this.read.Size = new System.Drawing.Size(54, 28);
            this.read.TabIndex = 90;
            this.read.Text = "Read";
            this.read.UseVisualStyleBackColor = true;
            this.read.Click += new System.EventHandler(this.read_Click);
            // 
            // period
            // 
            this.period.Location = new System.Drawing.Point(338, 13);
            this.period.Margin = new System.Windows.Forms.Padding(4);
            this.period.Maximum = new decimal(new int[] {
            100000,
            0,
            0,
            0});
            this.period.Minimum = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.period.Name = "period";
            this.period.Size = new System.Drawing.Size(74, 22);
            this.period.TabIndex = 91;
            this.period.ThousandsSeparator = true;
            this.period.Value = new decimal(new int[] {
            100,
            0,
            0,
            0});
            this.period.ValueChanged += new System.EventHandler(this.period_ValueChanged);
            // 
            // maxY
            // 
            this.maxY.Enabled = false;
            this.maxY.Location = new System.Drawing.Point(622, 13);
            this.maxY.Margin = new System.Windows.Forms.Padding(4);
            this.maxY.Maximum = new decimal(new int[] {
            65535,
            0,
            0,
            0});
            this.maxY.Name = "maxY";
            this.maxY.Size = new System.Drawing.Size(78, 22);
            this.maxY.TabIndex = 92;
            this.maxY.Value = new decimal(new int[] {
            10000,
            0,
            0,
            0});
            this.maxY.ValueChanged += new System.EventHandler(this.maxY_ValueChanged);
            // 
            // label36
            // 
            this.label36.AutoSize = true;
            this.label36.Location = new System.Drawing.Point(570, 16);
            this.label36.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label36.Name = "label36";
            this.label36.Size = new System.Drawing.Size(41, 16);
            this.label36.TabIndex = 93;
            this.label36.Text = "MaxY";
            // 
            // minY
            // 
            this.minY.Enabled = false;
            this.minY.Location = new System.Drawing.Point(758, 13);
            this.minY.Margin = new System.Windows.Forms.Padding(4);
            this.minY.Maximum = new decimal(new int[] {
            65535,
            0,
            0,
            0});
            this.minY.Minimum = new decimal(new int[] {
            32768,
            0,
            0,
            -2147483648});
            this.minY.Name = "minY";
            this.minY.Size = new System.Drawing.Size(78, 22);
            this.minY.TabIndex = 94;
            this.minY.ValueChanged += new System.EventHandler(this.minY_ValueChanged);
            // 
            // label71
            // 
            this.label71.AutoSize = true;
            this.label71.Location = new System.Drawing.Point(418, 16);
            this.label71.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label71.Name = "label71";
            this.label71.Size = new System.Drawing.Size(25, 16);
            this.label71.TabIndex = 101;
            this.label71.Text = "ms";
            // 
            // label67
            // 
            this.label67.AutoSize = true;
            this.label67.Location = new System.Drawing.Point(708, 16);
            this.label67.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label67.Name = "label67";
            this.label67.Size = new System.Drawing.Size(37, 16);
            this.label67.TabIndex = 95;
            this.label67.Text = "MinY";
            // 
            // nSamples
            // 
            this.nSamples.Location = new System.Drawing.Point(116, 13);
            this.nSamples.Margin = new System.Windows.Forms.Padding(4);
            this.nSamples.Maximum = new decimal(new int[] {
            10000,
            0,
            0,
            0});
            this.nSamples.Minimum = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.nSamples.Name = "nSamples";
            this.nSamples.Size = new System.Drawing.Size(80, 22);
            this.nSamples.TabIndex = 100;
            this.nSamples.Value = new decimal(new int[] {
            1,
            0,
            0,
            0});
            // 
            // widX
            // 
            this.widX.Location = new System.Drawing.Point(928, 13);
            this.widX.Margin = new System.Windows.Forms.Padding(4);
            this.widX.Maximum = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            this.widX.Name = "widX";
            this.widX.Size = new System.Drawing.Size(78, 22);
            this.widX.TabIndex = 96;
            this.widX.Value = new decimal(new int[] {
            500,
            0,
            0,
            0});
            this.widX.ValueChanged += new System.EventHandler(this.widX_ValueChanged);
            // 
            // log
            // 
            this.log.AutoSize = true;
            this.log.Location = new System.Drawing.Point(204, 15);
            this.log.Margin = new System.Windows.Forms.Padding(4);
            this.log.Name = "log";
            this.log.Size = new System.Drawing.Size(52, 20);
            this.log.TabIndex = 99;
            this.log.Text = "Log";
            this.log.UseVisualStyleBackColor = true;
            this.log.CheckedChanged += new System.EventHandler(this.log_CheckedChanged);
            // 
            // label80
            // 
            this.label80.AutoSize = true;
            this.label80.Location = new System.Drawing.Point(864, 16);
            this.label80.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label80.Name = "label80";
            this.label80.Size = new System.Drawing.Size(49, 16);
            this.label80.TabIndex = 97;
            this.label80.Text = "WidthX";
            // 
            // autoScale
            // 
            this.autoScale.AutoSize = true;
            this.autoScale.Checked = true;
            this.autoScale.CheckState = System.Windows.Forms.CheckState.Checked;
            this.autoScale.Location = new System.Drawing.Point(458, 15);
            this.autoScale.Margin = new System.Windows.Forms.Padding(4);
            this.autoScale.Name = "autoScale";
            this.autoScale.Size = new System.Drawing.Size(94, 20);
            this.autoScale.TabIndex = 98;
            this.autoScale.Text = "Auto Scale";
            this.autoScale.UseVisualStyleBackColor = true;
            this.autoScale.CheckedChanged += new System.EventHandler(this.autoScale_CheckedChanged);
            // 
            // tabControl_ctrl
            // 
            this.tabControl_ctrl.Controls.Add(this.regsPage);
            this.tabControl_ctrl.Controls.Add(this.nvParamsPage);
            this.tabControl_ctrl.Location = new System.Drawing.Point(16, 15);
            this.tabControl_ctrl.Margin = new System.Windows.Forms.Padding(4);
            this.tabControl_ctrl.Name = "tabControl_ctrl";
            this.tabControl_ctrl.SelectedIndex = 0;
            this.tabControl_ctrl.Size = new System.Drawing.Size(1326, 704);
            this.tabControl_ctrl.TabIndex = 117;
            this.tabControl_ctrl.SelectedIndexChanged += new System.EventHandler(this.tabControl1_SelectedIndexChanged);
            // 
            // MainForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 16F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1355, 768);
            this.Controls.Add(this.checkBox_Connect);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.comboBox_CommPort);
            this.Controls.Add(this.tabControl_ctrl);
            this.Controls.Add(this.quit);
            this.Controls.Add(this.guiRev);
            this.Margin = new System.Windows.Forms.Padding(4);
            this.Name = "MainForm";
            this.Text = "F7SkeletonGUI";
            this.Shown += new System.EventHandler(this.MainForm_Shown);
            this.nvParamsPage.ResumeLayout(false);
            this.nvParamsPage.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.parameters)).EndInit();
            this.regsPage.ResumeLayout(false);
            this.regsPage.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.chart1)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.statusList)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.period)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.maxY)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.minY)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.nSamples)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.widX)).EndInit();
            this.tabControl_ctrl.ResumeLayout(false);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.OpenFileDialog openFileDialog1;
        private System.Windows.Forms.Timer timer1;
        private System.Windows.Forms.SaveFileDialog saveFileDialog1;
        private System.Windows.Forms.Button quit;
        private System.Windows.Forms.Label guiRev;
        private System.Windows.Forms.ComboBox comboBox_CommPort;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.CheckBox checkBox_Connect;
        private System.Windows.Forms.TabPage nvParamsPage;
        private System.Windows.Forms.Label fwProg;
        private System.Windows.Forms.Button fromFile;
        private System.Windows.Forms.Button toFile;
        private System.Windows.Forms.Button toFlash;
        private System.Windows.Forms.DataGridView parameters;
        private System.Windows.Forms.DataGridViewTextBoxColumn Parameter;
        private System.Windows.Forms.DataGridViewTextBoxColumn pName;
        private System.Windows.Forms.DataGridViewTextBoxColumn pValue;
        private System.Windows.Forms.DataGridViewTextBoxColumn newValue;
        private System.Windows.Forms.TabPage regsPage;
        private System.Windows.Forms.DataVisualization.Charting.Chart chart1;
        private System.Windows.Forms.Button unCheckAll;
        private System.Windows.Forms.DataGridView statusList;
        private System.Windows.Forms.DataGridViewCheckBoxColumn selected;
        private System.Windows.Forms.DataGridViewTextBoxColumn name;
        private System.Windows.Forms.DataGridViewTextBoxColumn value;
        private System.Windows.Forms.DataGridViewTextBoxColumn newVal;
        private System.Windows.Forms.CheckBox plot;
        private System.Windows.Forms.Button read;
        private System.Windows.Forms.NumericUpDown period;
        private System.Windows.Forms.NumericUpDown maxY;
        private System.Windows.Forms.Label label36;
        private System.Windows.Forms.NumericUpDown minY;
        private System.Windows.Forms.Label label71;
        private System.Windows.Forms.Label label67;
        private System.Windows.Forms.NumericUpDown nSamples;
        private System.Windows.Forms.NumericUpDown widX;
        private System.Windows.Forms.CheckBox log;
        private System.Windows.Forms.Label label80;
        private System.Windows.Forms.CheckBox autoScale;
        private System.Windows.Forms.TabControl tabControl_ctrl;
    }
}

