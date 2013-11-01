using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CmdLine.Net.Control
{
    class FlightParameters
    {
        private static FlightParameters singleton = null;

        public static FlightParameters get()
        {
            if (singleton == null)
                singleton = new FlightParameters();
            return singleton;
        }

        private FlightParameters() { }

        private float _Thrust = 0.0f;
        private float _PID_P = 0.0f;
        private float _PID_I = 0.0f;
        private float _PID_D = 0.0f;

        // Thrust
        private void sendThrust()
        {

        }

        public float Thrust
        {
            get { return _Thrust; }
            set { _Thrust = value; sendThrust();  }
        }

        // PID.P
        private void sendPID()
        {

        }

        public float PID_P
        {
            get { return _PID_P; }
            set { _PID_P = value; sendPID(); }
        }

        // PID.I
        public float PID_I
        {
            get { return _PID_I; }
            set { _PID_I = value; sendPID(); }
        }

        // PID.D
        public float PID_D
        {
            get { return _PID_D; }
            set { _PID_D = value; sendPID(); }
        }

        // Envoi les paramètres à l'hélico
        public void sendParameters()
        {
            sendThrust();
            sendPID();
        }
    }
}
