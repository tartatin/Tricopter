using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.IO.Ports;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;

namespace CmdLine.Net.Control
{
    class SerialCommunication
    {


        static SerialCommunication singleton = null;

        private SerialPort serialPort;

        // thread related machins
        private bool stopRequest;
        private Thread mainThread;

        // liste des messages reçus, fifo, thread safe
        private ConcurrentQueue<string> queuedMsg;

        // valeurs générales
        private int       comPort      = 4;
        private int       comBaudrate  = 115200;
        private Parity    comParity    = Parity.Odd;
        private int       comDataBits  = 8;
        private StopBits  comStopBits  = StopBits.One;
        private Handshake comHandshake = Handshake.None;

        private int redundancyLvl = 2;

        // valeurs à réinitialiser à chaque ouverture
        private string lastChars;
        private bool   startFound;
        private bool   msgReceived;
        private bool   isEscapedChar;
        private string msgBuf;
        private int    dropCount;

        // constantes
        private const char escapeCharValue = (char) 0x11;
        private const char startCharValue  = (char) 0x21;
        private const char stopCharValue   = (char) 0x31;

        private int RedundancyCount
        {
            get { return redundancyLvl * 2 - 1; }
            set 
            {
                if ((value + 1) % 2 != 0)
                    throw new Exception("Redundancy count must be odd.");
                redundancyLvl = (value + 1) / 2;
            }
        }

        public int ComPort
        {
            get { return comPort; }
            set { if (mainThread.IsAlive == false) comPort = value; }
        }

        public int ComBaudrate
        {
            get { return comBaudrate; }
            set { if (mainThread.IsAlive == false) comBaudrate = value; }
        }

        public Parity ComParity
        {
            get { return comParity; }
            set { if (mainThread.IsAlive == false) comParity = value; }
        }

        public int ComDataBits
        {
            get { return comDataBits; }
            set { if (mainThread.IsAlive == false) comDataBits = value; }
        }

        public StopBits ComStopBits
        {
            get { return comStopBits; }
            set { if (mainThread.IsAlive == false) comStopBits = value; }
        }

        public Handshake ComHandshake
        {
            get { return comHandshake; }
            set { if (mainThread.IsAlive == false) comHandshake = value; }
        }

        public static SerialCommunication get()
        {
            if (singleton == null)
                singleton = new SerialCommunication();
            return singleton;
        }

        private SerialCommunication()
        {
        }

        ~SerialCommunication()
        {
            close();
        }

        private string get_left(string buffer, int count)
        {
            return buffer.Substring(0, count);
        }

        private string extract_left(ref string buffer, int count)
        {
            string result = buffer.Substring(0, count);
            buffer = buffer.Substring(count, buffer.Length - count);
            return result;
        }

        private string get_right(string buffer, int count)
        {
            return buffer.Substring(buffer.Length-count, count);
        }

        private string extract_right(ref string buffer, int count)
        {
            string result = buffer.Substring(buffer.Length - count, count);
            buffer = buffer.Substring(0, buffer.Length - count);
            return result;
        }

        private void handleMessage(string buffer)
        {
            queuedMsg.Enqueue(buffer);
        }

        private void handleReceived(ref string buffer)
        {
            while (buffer.Length > 0)
            {
                // On lit de quoi avoir 3 caractères
                int toRead = Math.Min(RedundancyCount, buffer.Length) - lastChars.Length;
                lastChars += extract_left(ref buffer, toRead);

                if (lastChars.Length < RedundancyCount)
                    continue;

                // On regarde la cohérence des caractères reçus
                byte char_buf = 0x00;
                if (startFound)
                {
                    bool found = false;
                    for (int i = 0; i < RedundancyCount; ++i)
                    {
                        int sameCount = 0;

                        for (int j = i + 1; j < RedundancyCount; ++j)
                        {
                            if (lastChars[i] == lastChars[j])
                                ++sameCount;
                        }

                        if (sameCount >= redundancyLvl)
                        {
                            char_buf = (byte)lastChars[i];
                            found = true;
                            break;
                        }
                    }

                    lastChars = "";

                    if (found == false)
                    {
                        // Les données reçues ne sont pas consistantes, donc un morcea udes données estr mauvais, donc le message est foutu
                        startFound = false;
                        msgReceived = false;
                        isEscapedChar = false;
                        msgBuf = "";
                        ++dropCount;
                    }
                }
                else
                {
                    bool areAllSame = true;
                    for (int i = 1; i < RedundancyCount; ++i)
                    {
                        if (lastChars[0] != lastChars[i])
                        {
                            areAllSame = false;
                            break;
                        }
                    }

                    if (areAllSame)
                    {
                        char_buf = (byte)lastChars[0];
                        lastChars = "";
                    }
                    else
                    {
                        extract_right(ref lastChars, RedundancyCount - 1);
                        continue;
                    }
                }

                // Deux cas : soit on attend le début du message, soit on est dedans
                if (startFound)
                {
                    // on lit les données petit à petit
                    if (isEscapedChar)
                    {
                        // quel que le soit le caractère suivant, on l'insère comme un caractère de donnée
                        msgBuf += char_buf;
                        isEscapedChar = false;
                    }
                    else
                    {
                        if (char_buf == escapeCharValue)
                        {
                            // il s'agit d'un caractère d'échappement
                            isEscapedChar = true;
                        }
                        else if (char_buf == stopCharValue)
                        {
                            // caractère de fin
                            msgReceived = true;
                            startFound = false;
                        }
                        else
                        {
                            msgBuf += char_buf;
                        }
                    }
                }
                else
                {
                    if (char_buf == startCharValue)
                    {
                        startFound = true;
                        msgReceived = false;
                        isEscapedChar = false;
                        msgBuf = "";
                    }
                }

                // A ce stade il est possible qu'un message complet ait été obtenu, si c'est le cas on doit cesser de chercher le message,
                // quitter, et attendre que celui-ci soit traité dans la boucle principale de l'application.
                if (msgReceived)
                {
                    handleMessage(msgBuf);
                    msgBuf = "";
                }
            }
        }

        private void localThreadLoop()
        {
            // Buffer stockant l'ensemble des données non encore traitées
            string lBuffer = "";
            lastChars      = "";
            startFound     = false;
            msgReceived    = false;
            isEscapedChar  = false;
            msgBuf         = "";
            dropCount      = 0;

            // Ouverture du port série
            serialPort = new SerialPort();
            serialPort.Encoding  = Encoding.ASCII;
            serialPort.PortName  = String.Format("COM{0}", comPort);
            serialPort.BaudRate  = comBaudrate;
            serialPort.Parity    = comParity;
            serialPort.StopBits  = comStopBits;
            serialPort.DataBits  = comDataBits;
            serialPort.Handshake = comHandshake;

            serialPort.Open();

            // Réception des données et traitement
            while (stopRequest == false)
            {
                // Lecture et ajout au buffer général
                string newData = serialPort.ReadExisting();
                lBuffer += newData;

                // Traitement
                handleReceived(ref lBuffer);
            }

            // Fin
            serialPort.Close();
        }

        private static void threadLoop()
        {
            SerialCommunication.get().localThreadLoop();
        }

        public void open()
        {
            close();
            stopRequest = false;
            mainThread = new Thread(threadLoop);
            mainThread.Start();
        }

        public void close()
        {
            if (mainThread == null)
                return;

            stopRequest = true;
            if (mainThread.ThreadState != ThreadState.Unstarted)
                mainThread.Join();
        }

        public bool dequeueMsg(out string msg)
        {
            return queuedMsg.TryDequeue(out msg);
        }
    }
}
