using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace motor_controller_uart
{
    public partial class Form1 : Form
    {
        bool IsConnected = false;

        public Form1()
        {
            InitializeComponent();
        }

        private void timer1_Tick(object sender, EventArgs e) => SendUartMsg();
        private void button2_Click(object sender, EventArgs e) => SetMessage();
        private void button4_Click(object sender, EventArgs e) => SetPeriod();
        private void button1_Click(object sender, EventArgs e) => RefreshPorts();
        private void button3_Click(object sender, EventArgs e)
        {
            if (IsConnected)
                Disconnect();
            else
                Connect();

            IsConnected = !IsConnected;
        }

        void SendUartMsg()
        {

        }

        void Connect()
        {
            buttonConnect.Text = "Disconnect";
            comboBoxPort.Enabled = false;
            buttonPort.Enabled = false;

            progressBar.Style = ProgressBarStyle.Marquee;
        }

        void Disconnect()
        {
            buttonConnect.Text = "Connect";
            comboBoxPort.Enabled = true;
            buttonPort.Enabled = true;

            progressBar.Style = ProgressBarStyle.Blocks;
        }

        void RefreshPorts()
        {

        }

        private void numericUpDownPeriod_ValueChanged(object sender, EventArgs e) => buttonPeriod.Enabled = true;
        private void textBoxMsg_TextChanged(object sender, EventArgs e) => buttonMsg.Enabled = true;

        void SetMessage()
        {
            buttonMsg.Enabled = false;
        }

        void SetPeriod()
        {
            buttonPeriod.Enabled = false;
        }

        
    }
}
