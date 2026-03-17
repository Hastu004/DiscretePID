using System;
using System.Threading;
using System.Windows.Forms;
using System.IO.Ports;
using System.IO;

namespace PID_GUI
{
    public partial class Form1 : Form
    {
        private Thread tareaLeerSerial;
        private bool seguir = true;
        private string msgDesdeArduino = "";
        private double dist = 0;
        private double SetPoint = 45;
        private int contador = 0;
        private int tiempoMuestreo = 2;
        
        public Form1()
        {
            InitializeComponent();
            WindowState = FormWindowState.Maximized;

            chart1.ChartAreas[0].AxisY.Maximum = 200-90;
            chart1.ChartAreas[0].AxisY.Minimum = 70-90;

            chart1.ChartAreas[0].AxisY.Interval = 10;
            chart1.ChartAreas[0].AxisX.Interval = 50;

            chart1.ChartAreas[0].AxisX.Maximum = 200;
            chart1.ChartAreas[0].AxisX.Minimum = 0;
            textBoxPWM.Text = "1000";
            textBoxSetpoint.Text = SetPoint.ToString();
        }
        
        private void Form1_Load(object sender, EventArgs e)
        {
            llenarDropdownSerial();
            tareaLeerSerial = new Thread(new ThreadStart(leerSerial));
        
        }
        private void leerSerial()
        {
            while (seguir == true)
            {
                try
                {
                    Invoke((MethodInvoker)delegate {actualizarChart();});
                    Thread.Sleep(tiempoMuestreo);
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message);
                }  
            }
        }

        private void actualizarChart()
        {
            try
            {
                msgDesdeArduino = serialPort1.ReadLine().ToString();

                if (double.TryParse(msgDesdeArduino, out dist))
                {
                    dist = dist / 100;
                    labelDistancia.Text = dist.ToString();
                    labelSetPoint.Text = SetPoint.ToString();
                    labelTiempo.Text = contador.ToString();
                    chart1.Series[0].Points.AddXY(contador, dist);
                    chart1.Series[1].Points.AddXY(contador, SetPoint);

                    StreamWriter sw = new StreamWriter("D:/Google Drive/Projects/PID_GUI/test.txt", true);

                    sw.WriteLine((contador*tiempoMuestreo*10).ToString()+" "+textBoxPWM.Text.ToString() + " " + dist);

                    sw.Close();

                if (chart1.Series[0].Points.Count > 200)
                    {
                        chart1.Series[0].Points.RemoveAt(0);
                        chart1.Series[1].Points.RemoveAt(0);
                        chart1.ChartAreas[0].AxisX.Maximum = contador;
                        chart1.ChartAreas[0].AxisX.Minimum = contador - chart1.Series[0].Points.Count;
                    }
                    contador++;
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }
        private void buttonPuertoSerial_Click(object sender, EventArgs e)
        {
            if (comboBoxPuertoSerial.Text!="")
            {
                try
                {
                    seguir = true;
                    cambiarColor();
                    serialPort1.PortName = comboBoxPuertoSerial.SelectedItem.ToString();
                    serialPort1.BaudRate = 9600;
                    serialPort1.Open();
                    buttonCerrarPuertoSerial.Enabled = true;
                    buttonPuertoSerial.Enabled = false;
                    buttonActualizar.Enabled = false;
                    tareaLeerSerial = new Thread(new ThreadStart(leerSerial)); 
                    tareaLeerSerial.IsBackground = true;
                    tareaLeerSerial.Start();
                    serialPort1.Write("a");               }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message);
                }
            }
            else
            {
                MessageBox.Show("Seleccione un puerto serial!");
            }
        }
        private void buttonCerrarPuertoSerial_Click(object sender, EventArgs e)
        {
                if (serialPort1.IsOpen)
                {
                    serialPort1.Write(1000.ToString());
                }
                
                textBoxPWM.Text = "1000";
                seguir = false;
                contador = 0;
                chart1.ChartAreas[0].AxisX.Maximum = 200;
                chart1.ChartAreas[0].AxisX.Minimum = contador;
                chart1.Series[0].Points.Clear();
                chart1.Series[1].Points.Clear();
                chart1.Update();
                Thread.Sleep(tiempoMuestreo);
                tareaLeerSerial.Abort();
                serialPort1.Close();
                buttonPuertoSerial.Enabled = true;
                buttonCerrarPuertoSerial.Enabled = false;
                buttonActualizar.Enabled = true;
                

        }
        private void buttonCambiar_Click(object sender, EventArgs e)
        {
            if(double.Parse(textBoxSetpoint.Text.ToString()) > 90)
            {
                MessageBox.Show("El setpoint no puede ser mayor a 90");
                textBoxSetpoint.Text = SetPoint.ToString();
                return;
            }
            if (Double.Parse(textBoxSetpoint.Text.ToString()) < 0)
            {
                MessageBox.Show("El setpoint no puede ser menor que 0");
                textBoxSetpoint.Text = SetPoint.ToString();
                return;
            }

            double.TryParse(textBoxSetpoint.Text, out SetPoint);
            serialPort1.Write(textBoxSetpoint.Text.ToString());

        }

        private void buttonActualizar_Click(object sender, EventArgs e)
        {
            comboBoxPuertoSerial.Items.Clear();
            llenarDropdownSerial();
        }
        private void llenarDropdownSerial()
        {
            string[] v = SerialPort.GetPortNames();
            Array.Sort(v);
            foreach (string s in v)
            {
                comboBoxPuertoSerial.Items.Add(s);
            }
        }

        private void buttonCambiarPWM_Click(object sender, EventArgs e)
        {
            serialPort1.Write(textBoxPWM.ToString());
        }
    }
}

