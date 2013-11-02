using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;
using System.IO.Ports;

namespace SimplestCmd
{
    enum FlightVariable
    {
        None,
        Thrust,
        P,
        I,
        D
    }

    /*******************************************************************/
    class Program
    {
        static SerialCommunication com;

        delegate void HandleKeyMethod(ConsoleKey pKey);

        class HandledKey
        {
            public HandledKey(ConsoleKey pKey, HandleKeyMethod pMethod, string pName)
            {
                Method = pMethod;
                Key = pKey;
                Name = pName;
            }

            public HandleKeyMethod Method;
            public ConsoleKey Key;
            public string Name;
        }

        /*******************************************************************/
        static List<HandledKey> _handledKeys = new List<HandledKey>();

        /*******************************************************************/
        static byte idNoop = 0x00;
        static byte idPing = 0x01;
        static byte idThrust = 0x02;
        static byte idPID = 0x05;
        static byte idServo = 0x03;
        static byte idSetEngineState = 0x04;
        static byte idDeadStop = 0x08;

        static byte idPrintFloat = 0x06;
        static byte idPrintString = 0x07;
        static byte idPlot = 0x09;

        /*******************************************************************/
        const byte plotAccelerationId = 0x01;
        const byte plotAttitudeId = 0x02;

        /*******************************************************************/
        static PlotRecorder plotAcceleration;
        static PlotRecorder plotAttitude;

        /*******************************************************************/
        static float _servoAngle = 0.0f;
        static float _flightThrust = 0.0f;
        static float _flightP = 0.0f;
        static float _flightI = 0.0f;
        static float _flightD = 0.0f;

        /*******************************************************************/
        static FlightVariable _flightVariable = FlightVariable.None;
        static bool _changingCom = false;

        /*******************************************************************/
        static void setComPort(int port)
        {
            if (_changingCom == false)
                return;

            bool reconnect = false;
            if (com.isStarted == true)
            {
                com.close();
                reconnect = true;
            }
            com.ComPort = port;
            _changingCom = false;
            
            if (reconnect)
                com.open();

            Console.WriteLine(String.Format("Port : {0}", com.ComPort));
        }

        /*******************************************************************/
        static FlightVariable flightVariable
        {
            get { return _flightVariable; }
            set 
            {
                _flightVariable = value;
                Console.WriteLine();
                Console.WriteLine(">>> " + Enum.GetName(typeof(FlightVariable), _flightVariable));
            }
        }

        /*******************************************************************/
        static void sendThrust()
        {
            sendCmd(idThrust, rawFloat(_flightThrust));
        }

        static float flightThrust
        {
            get { return _flightThrust; }
            set 
            {
                _flightThrust = value;
                sendThrust();
                Console.WriteLine(String.Format("{0}", value)); 
            }
        }

        /*******************************************************************/
        static void sendPIDValues()
        {
            byte[] P = rawFloat(_flightP);
            byte[] I = rawFloat(_flightI);
            byte[] D = rawFloat(_flightD);

            byte[] args = new byte[P.Length + I.Length + D.Length];
            Array.Copy(P, 0, args, 0, P.Length);
            Array.Copy(I, 0, args, P.Length, I.Length);
            Array.Copy(D, 0, args, P.Length + I.Length, D.Length);

            sendCmd(idPID, args);
        }
        
        static float flightP
        {
            get { return _flightP; }
            set 
            { 
                _flightP = value;
                sendPIDValues();
                Console.WriteLine(String.Format("{0}", value)); 
            }
        }

        static float flightI
        {
            get { return _flightI; }
            set 
            { 
                _flightI = value;
                sendPIDValues();
                Console.WriteLine(String.Format("{0}", value)); 
            }
        }

        static float flightD
        {
            get { return _flightD; }
            set 
            { 
                _flightD = value;
                sendPIDValues();
                Console.WriteLine(String.Format("{0}", value)); 
            }
        }

        /*******************************************************************/
        static void sendServoAngle()
        {
            sendCmd(idServo, rawFloat(_servoAngle));
        }

        static float servoAngle
        {
            get { return _servoAngle; }
            set
            {
                _servoAngle = value;
                sendServoAngle();
                Console.WriteLine(String.Format("{0}", value));
            }
        }

        /*******************************************************************/
        static void saveParams()
        {
            Console.WriteLine();
            Console.WriteLine(">>> Enregistrement des paramètres de vol");
            showParams();

            StreamWriter stream = new StreamWriter("params.txt");
            stream.WriteLine(_flightThrust);
            stream.WriteLine(_flightP);
            stream.WriteLine(_flightI);
            stream.WriteLine(_flightD);
            stream.WriteLine(_servoAngle);
            stream.Flush();
            stream.Close();
        }

        static void loadParams()
        {
            Console.WriteLine();
            Console.WriteLine(">>> Chargement des paramètres de vol");

            float _flightThrustBak = _flightThrust;
            float _flightPBak = _flightP;
            float _flightIBak = _flightI;
            float _flightDBak = _flightD;
            float _servoAngleBak = _servoAngle;

            StreamReader stream = new StreamReader("params.txt");
            _flightThrust = Convert.ToSingle(stream.ReadLine());
            _flightP = Convert.ToSingle(stream.ReadLine());
            _flightI = Convert.ToSingle(stream.ReadLine());
            _flightD = Convert.ToSingle(stream.ReadLine());
            _servoAngle = Convert.ToSingle(stream.ReadLine());
            stream.Close();

            showParams();
            Console.WriteLine();
            if (secondChance(8) == false)
            {
                _flightThrust = _flightThrustBak;
                _flightP = _flightPBak;
                _flightI = _flightIBak;
                _flightD = _flightDBak;
                _servoAngle = _servoAngleBak;
                return;
            }

            sendAllParams();
        }

        /*******************************************************************/
        static void showParams(bool pShowTitle = false)
        {
            if (pShowTitle)
            {
                Console.WriteLine();
                Console.WriteLine(">>> Paramètres de vol");
            }

            Console.WriteLine(String.Format("Poussée : {0}", _flightThrust));
            Console.WriteLine(String.Format("PID : {0}, {1}, {2}", _flightP, _flightI, _flightD));
            Console.WriteLine(String.Format("Angle du servomoteur : {0}", _servoAngle));
        }

        /*******************************************************************/
        static void sendAllParams()
        {
            sendThrust();
            sendPIDValues();
            sendServoAngle();
        }

        /*******************************************************************/
        static void sendCmd(byte id)
        {
            sendCmd(id, new byte[0]);
        }

        static void sendCmd(byte id, byte pParam)
        {
            byte[] lArgs = new byte[1];
            lArgs[0] = pParam;
            sendCmd(id, lArgs);
        }

        static void sendCmd(byte id, byte[] values)
        {
            connect();

            byte[] cmd = new byte[1 + values.Length];
            cmd[0] = id;
            values.CopyTo(cmd, 1);
            com.enqueueMsg(cmd);
        }

        /*******************************************************************/
        static byte[] rawFloat(float pValue)
        {
            return BitConverter.GetBytes(pValue);
        }

        /*******************************************************************/
        static bool secondChance(int duration)
        {
            int prevTime = -1;
            DateTime from = DateTime.Now;
            while (true)
            {
                TimeSpan delta = DateTime.Now - from;

                if (delta.Seconds != prevTime)
                {
                    prevTime = delta.Seconds;
                    Console.WriteLine(String.Format("{0} s...", duration-prevTime));
                    Console.Beep(5000, 500);
                }

                if (delta.Seconds >= duration)
                {
                    Console.WriteLine("Go !");
                    Console.Beep(6000, 1000);
                    return true;
                }

                if (Console.KeyAvailable)
                {
                    Console.WriteLine("Annulé !");
                    Console.Beep(800, 300);
                    Console.ReadKey(true);
                    return false;
                }
            }
        }

        /*******************************************************************/
        static void start()
        {
            Console.WriteLine();
            Console.WriteLine(">>> Démarrage des moteurs.");
            if (secondChance(5) == false)
                return;
            Console.WriteLine("Allumage.");
            sendCmd(idSetEngineState, 0xAA);
        }

        static void deadStop()
        {
            connect();
            for (int i = 0; i < 16; ++i)
            {
                sendCmd(idDeadStop);
                System.Threading.Thread.Sleep(16);
            }
            Console.WriteLine();
            Console.WriteLine(">>> Arrêt d'urgence.");
            Console.WriteLine(">>> Plus aucune commande ne sera traitée.");
        }

        static void stop()
        {
            connect();
            for(int i = 0; i < 3; ++i)
                sendCmd(idSetEngineState, 0x00);
            Console.WriteLine();
            Console.WriteLine(">>> Arrêt des moteurs.");
        }

        /*******************************************************************/
        static void add(FlightVariable pVar, float pValue)
        {
            switch (pVar)
            {
                case FlightVariable.Thrust: flightThrust += pValue; break;
                case FlightVariable.P: flightP += pValue; break;
                case FlightVariable.I: flightI += pValue; break;
                case FlightVariable.D: flightD += pValue; break;
            }
        }

        /*******************************************************************/
        static float default_var_step(FlightVariable pVar)
        {
            switch (pVar)
            {
                case FlightVariable.Thrust: return 0.1f;
                case FlightVariable.P: return 0.1f;
                case FlightVariable.I: return 0.1f;
                case FlightVariable.D: return 0.1f;
                default: return 0.0f;
            }
        }

        /*******************************************************************/
        static void increase(FlightVariable pVar)
        {
            add(pVar, default_var_step(pVar));
        }

        static void decrease(FlightVariable pVar)
        {
            add(pVar, -default_var_step(pVar));
        }

        /*******************************************************************/
        static void connect()
        {
            if (com.isStarted == false)
                com.open();
        }

        /*******************************************************************/
        static void handleKey(ConsoleKey key)
        {
            // Gestion des commandes entrées par l'utilisateur

            foreach (HandledKey lHandler in _handledKeys)
            {
                if (lHandler.Key == key)
                {
                    lHandler.Method(key);
                    break;
                }
            }
        }

        /*******************************************************************/
        static void handlePlot(byte[] args)
        {
            byte id = args[0];

            float T = BitConverter.ToSingle(args, 1);
            List<float> lValues = new List<float>();
            for (int i = 5; i < args.Length; i += 4)
            {
                float lValue = BitConverter.ToSingle(args, i);
                lValues.Add(lValue);
            }

            switch (id)
            {
                case plotAccelerationId: plotAcceleration.addPoint(T, lValues); break;
                case plotAttitudeId: plotAttitude.addPoint(T, lValues); break;
            }
        }

        /*******************************************************************/
        static void handlePrintFloat(byte[] args)
        {
            float lValue = BitConverter.ToSingle(args, 0);
            Console.WriteLine(String.Format("{0}", lValue));
        }

        /*******************************************************************/
        static void handlePrintString(byte[] args)
        {
            String lText = System.Text.Encoding.ASCII.GetString(args);
            Console.WriteLine(lText);
        }

        /*******************************************************************/
        static void handleMsg(byte[] msg)
        {
            // Gestion des messages reçus par l'hélico
            if (msg.Length <= 0)
                return;

            byte lId = msg[0];
            byte[] lParams = new byte[msg.Length-1];
            Array.Copy(msg, 1, lParams, 0, lParams.Length-1);

            if (lId == idPlot)
                handlePlot(lParams);
            else if (lId == idPrintFloat)
                handlePrintFloat(lParams);
            else if (lId == idPrintString)
                handlePrintString(lParams);
        }

        /*******************************************************************/
        static void showComParams(bool pTitle = true)
        {
            if (pTitle)
            {
                Console.WriteLine(">>> Paramètres COM");
                Console.WriteLine();
            }

            Console.WriteLine(String.Format("Port : {0}", com.ComPort));
            Console.WriteLine(String.Format("Baudrate : {0}", com.ComBaudrate));
            Console.WriteLine(String.Format("Parity : {0}", Enum.GetName(typeof(Parity), com.ComParity)));
            Console.WriteLine(String.Format("Data Bits : {0}", com.ComDataBits));
            Console.WriteLine(String.Format("Stop Bits : {0}", Enum.GetName(typeof(StopBits), com.ComStopBits)));
            Console.WriteLine(String.Format("Handshake : {0}", Enum.GetName(typeof(Handshake), com.ComHandshake)));
        }

        /*******************************************************************/

        static void RegisterCOMChanger(int i)
        {
            string lCOMString = "Sélectionne le port COM n°{0}.";
            _handledKeys.Add(new HandledKey(
                ConsoleKey.D0 + i,
                delegate(ConsoleKey pKey) { setComPort(i); },
                string.Format(lCOMString, i)));
        }

        static void RegisterHandledKeys()
        {
            // Active la sélection du port COM
            _handledKeys.Add(new HandledKey(
                ConsoleKey.C,
                delegate(ConsoleKey pKey) { _changingCom = !_changingCom; },
                "Active la sélection du port COM."));

            // Touches permettant de changer de port COM
            for (int i = 0; i <= 5; ++i)
                RegisterCOMChanger(i);

            // Affiche la liste des paramètres
            _handledKeys.Add(new HandledKey(
                ConsoleKey.I,
                delegate(ConsoleKey pKey) { showParams(true); },
                "Affiche la liste des paramètres."));

            // Connexion
            _handledKeys.Add(new HandledKey(
                ConsoleKey.F12,
                delegate(ConsoleKey pKey)
                {
                    Console.WriteLine();
                    Console.WriteLine(">>> Connexion.");
                    connect();
                },
                "Connexion à l'hélicoptère."));

            // Démarre les moteurs
            _handledKeys.Add(new HandledKey(
                ConsoleKey.S,
                delegate(ConsoleKey pKey) { start(); },
                "Démarre les moteurs."));

            // Arrêt d'urgence
            _handledKeys.Add(new HandledKey(
                ConsoleKey.Spacebar,
                delegate(ConsoleKey pKey) { deadStop(); },
                "Arrêt d'urgence."));

            // Arrêt normal
            HandleKeyMethod lNormalStopDel = delegate(ConsoleKey pKey) { };
            _handledKeys.Add(new HandledKey(
                ConsoleKey.Escape,
                lNormalStopDel,
                "Arrêt des moteurs."));
            _handledKeys.Add(new HandledKey(
                ConsoleKey.Q,
                lNormalStopDel,
                "Arrêt des moteurs."));

            // Paramètres de vol
            _handledKeys.Add(new HandledKey(
                ConsoleKey.F1,
                delegate(ConsoleKey pKey) {flightVariable = FlightVariable.Thrust;},
                "Réglage de la poussée."));

            _handledKeys.Add(new HandledKey(
                ConsoleKey.F2,
                delegate(ConsoleKey pKey) { flightVariable = FlightVariable.P; },
                "Réglage du paramètre P (proportionnel)."));

            _handledKeys.Add(new HandledKey(
                ConsoleKey.F3,
                delegate(ConsoleKey pKey) { flightVariable = FlightVariable.I; },
                "Réglage du paramètre I (intégrateur)."));

            _handledKeys.Add(new HandledKey(
                ConsoleKey.F4,
                delegate(ConsoleKey pKey) { flightVariable = FlightVariable.D; },
                "Réglage du paramètre D (dérivée)."));

            // Modifie le paramètre de vol sélectionné
            _handledKeys.Add(new HandledKey(
                ConsoleKey.Add,
                delegate(ConsoleKey pKey) { increase(_flightVariable); },
                "Augmente le paramètre de vol sélectionné."));

            _handledKeys.Add(new HandledKey(
                ConsoleKey.Subtract,
                delegate(ConsoleKey pKey) { decrease(_flightVariable); },
                "Diminue le paramètre de vol sélectionné."));

            // Chargement / enregistrement des paramètres
            _handledKeys.Add(new HandledKey(
                ConsoleKey.F8,
                delegate(ConsoleKey pKey) { saveParams(); },
                "Enregistre les paramètres."));

            _handledKeys.Add(new HandledKey(
                ConsoleKey.F9,
                delegate(ConsoleKey pKey) { loadParams(); },
                "Charge les paramètres."));
        }

        /*******************************************************************/
        static void ShowHandledKeys()
        {
            Console.WriteLine(">>> Liste des commandes");
            Console.WriteLine();

            foreach (HandledKey lHandler in _handledKeys)
            {
                Console.WriteLine(string.Format("{0} : {1}", lHandler.Key, lHandler.Name));
            }

            Console.WriteLine();
        }

        /*******************************************************************/
        static void Main(string[] args)
        {
            RegisterHandledKeys();
            ShowHandledKeys();

            com = new SerialCommunication();
            plotAcceleration = new PlotRecorder("plot_acceleration.csv");
            plotAttitude = new PlotRecorder("plot_attitude.csv");

            // Affichage des paramètres du port com
            showComParams();
            Console.WriteLine();

            // Boucle principale
            while (true)
            {
                try
                {
                    // Gestion des entrées utilisateur
                    if (Console.KeyAvailable)
                    {
                        ConsoleKeyInfo cki = Console.ReadKey(true);
                        handleKey(cki.Key);
                    }

                    // Réception des messages
                    byte[] msg;
                    while (com.dequeueMsg(out msg) == true)
                        handleMsg(msg);
                }
                catch (Exception e)
                {
                    Console.WriteLine(e.Message);
                }

                // Pause
                System.Threading.Thread.Sleep(1);
            }
        }
    }
}
