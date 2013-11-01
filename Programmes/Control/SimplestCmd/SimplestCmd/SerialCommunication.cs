using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.IO.Ports;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;

namespace SimplestCmd
{
    class SerialCommunication
    {
        private SerialPort serialPort;

        // thread related machins
        private bool stopRequest;
        private Thread mainThread;

        // liste des messages reçus, fifo, thread safe
        private ConcurrentQueue<byte[]> receivedMsg;
        private ConcurrentQueue<byte[]> sentMsg;

        // valeurs générales
        private int comPort = 4;
        private int comBaudrate = 38400;
        private Parity comParity = Parity.None;
        private int comDataBits = 8;
        private StopBits comStopBits = StopBits.One;
        private Handshake comHandshake = Handshake.None;

        private int redundancyLvl = 2;

        // valeurs à réinitialiser à chaque ouverture
        private byte[] lastChars;
        private bool startFound;
        private bool msgReceived;
        private bool isEscapedChar;
        private byte[] msgBuf;
        private int dropCount;

        // constantes
        private const byte escapeCharValue = (byte)0x11;
        private const byte startCharValue = (byte)0x21;
        private const byte stopCharValue = (byte)0x31;

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
            set { if (mainThread == null) comPort = value; }
        }

        public int ComBaudrate
        {
            get { return comBaudrate; }
            set { if (mainThread == null) comBaudrate = value; }
        }

        public Parity ComParity
        {
            get { return comParity; }
            set { if (mainThread == null) comParity = value; }
        }

        public int ComDataBits
        {
            get { return comDataBits; }
            set { if (mainThread == null) comDataBits = value; }
        }

        public StopBits ComStopBits
        {
            get { return comStopBits; }
            set { if (mainThread == null) comStopBits = value; }
        }

        public Handshake ComHandshake
        {
            get { return comHandshake; }
            set { if (mainThread == null) comHandshake = value; }
        }

        public SerialCommunication()
        {
            receivedMsg = new ConcurrentQueue<byte[]>();
            sentMsg = new ConcurrentQueue<byte[]>();
        }

        ~SerialCommunication()
        {
            close();
        }

        private byte[] get_left(byte[] buffer, int count)
        {
            count = Math.Min(count, buffer.Length);
            byte[] lResult = new byte[count];
            Array.Copy(buffer, 0, lResult, 0, count);
            return lResult;
        }

        private byte[] extract_left(ref byte[] buffer, int count)
        {
            count = Math.Min(count, buffer.Length);
            byte[] lResult = get_left(buffer, count);
            buffer = get_right(buffer, buffer.Length - count);
            return lResult;
        }

        private byte[] get_right(byte[] buffer, int count)
        {
            count = Math.Min(count, buffer.Length);
            byte[] lResult = new byte[count];
            Array.Copy(buffer, buffer.Length-count, lResult, 0, count);
            return lResult;
        }

        private byte[] extract_right(ref byte[] buffer, int count)
        {
            count = Math.Min(count, buffer.Length);
            byte[] lResult = get_right(buffer, count);
            buffer = get_left(buffer, buffer.Length - count);
            return lResult;
        }

        private void add_back(ref byte[] from, byte[] data)
        {
            byte[] lNew = new byte[from.Length + data.Length];
            from.CopyTo(lNew, 0);
            data.CopyTo(lNew, from.Length);
            from = lNew;
        }

        private void add_back(ref byte[] from, byte data)
        {
            byte[] lNew = new byte[from.Length + 1];
            from.CopyTo(lNew, 0);
            lNew[from.Length] = data;
            from = lNew;
        }

        private byte[] toByte(string str)
        {
            byte[] result = new byte[str.Length];
            for (int i = 0; i < str.Length; ++i)
                result[i] = (byte)str[i];
            return result;
        }

        private void handleMessage(byte[] buffer)
        {
            receivedMsg.Enqueue(buffer);
        }

        private void handleReceived(ref byte[] buffer)
        {
            while (buffer.Length > 0)
            {
                // On lit de quoi avoir 3 caractères
                int toRead = Math.Min(buffer.Length, RedundancyCount - lastChars.Length);
                add_back(ref lastChars, extract_left(ref buffer, toRead));

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

                    lastChars = new byte[0];

                    if (found == false)
                    {
                        // Les données reçues ne sont pas consistantes, donc un morcea udes données estr mauvais, donc le message est foutu
                        startFound = false;
                        msgReceived = false;
                        isEscapedChar = false;
                        msgBuf = new byte[0];
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
                        lastChars = new byte[0];
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
                        add_back(ref msgBuf, char_buf);
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
                            add_back(ref msgBuf, char_buf);
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
                        msgBuf = new byte[0];
                    }
                }

                // A ce stade il est possible qu'un message complet ait été obtenu, si c'est le cas on doit cesser de chercher le message,
                // quitter, et attendre que celui-ci soit traité dans la boucle principale de l'application.
                if (msgReceived)
                {
                    handleMessage(msgBuf);
                    msgBuf = new byte[0];
                }
            }
        }

        private void sendByte(byte c)
        {
            string s = "";

            if ((c == startCharValue) || (c == stopCharValue) || (c == escapeCharValue))
            {
                for (int i = 0; i < RedundancyCount; ++i)
                    s += escapeCharValue;
            }

            for (int i = 0; i < RedundancyCount; ++i)
                s += c;

            serialPort.Write(s);
        }

        private void sendMessage(byte[] msg)
        {
            sendByte(startCharValue);
            for (int i = 0; i < msg.Length; ++i)
            {
                sendByte(msg[i]);
            }
            sendByte(stopCharValue);
        }

        private void sendAllMessages()
        {
            byte[] msg;
            while (sentMsg.TryDequeue(out msg) == true)
            {
                sendMessage(msg);
            }
        }

        private void clearMsgList(ref ConcurrentQueue<byte[]> pList)
        {
            byte[] not_used;
            while (pList.Count > 0)
                pList.TryDequeue(out not_used);
        }

        private void localThreadLoop()
        {
            // Buffer stockant l'ensemble des données non encore traitées
            byte[] lBuffer = new byte[0];
            lastChars = new byte[0];
            startFound = false;
            msgReceived = false;
            isEscapedChar = false;
            msgBuf = new byte[0];
            dropCount = 0;

            // Ouverture du port série
            serialPort = new SerialPort();
            serialPort.Encoding = Encoding.ASCII;
            serialPort.PortName = String.Format("COM{0}", comPort);
            serialPort.BaudRate = comBaudrate;
            serialPort.Parity = comParity;
            serialPort.StopBits = comStopBits;
            serialPort.DataBits = comDataBits;
            serialPort.Handshake = comHandshake;

            serialPort.Open();

            // Réception des données et traitement
            while (stopRequest == false)
            {
                try
                {
                    // Lecture et ajout au buffer général
                    string newData = serialPort.ReadExisting();
                    add_back(ref lBuffer, toByte(newData));

                    // Traitement des données reçues
                    handleReceived(ref lBuffer);

                    // Envoi des commandes
                    sendAllMessages();
                }
                catch (Exception e)
                {
                    clearMsgList(ref sentMsg);
                    clearMsgList(ref receivedMsg);

                    lastChars = new byte[0];
                    startFound = false;
                    msgReceived = false;
                    isEscapedChar = false;
                    msgBuf = new byte[0];

                    Console.WriteLine(e.Message);
                }
            }

            // Fin
            serialPort.Close();
        }

        private static void threadLoop(object com)
        {
            try
            {
                (com as SerialCommunication).localThreadLoop();
            }
            catch (Exception e)
            {
                Console.WriteLine(e.Message);
            }
        }

        public bool isStarted
        {
            get
            {
                if (mainThread == null)
                    return false;
                else
                    return true;
            }
        }
        
        public void open()
        {
            close();
            stopRequest = false;
            mainThread = new Thread(new ParameterizedThreadStart(threadLoop));
            mainThread.Start(this);
        }

        public void close()
        {
            if (mainThread == null)
                return;

            stopRequest = true;
            if (mainThread.ThreadState != ThreadState.Unstarted)
                mainThread.Join();

            mainThread = null;
        }

        public bool dequeueMsg(out byte[] msg)
        {
            return receivedMsg.TryDequeue(out msg);
        }

        public void enqueueMsg(byte[] msg)
        {
            sentMsg.Enqueue(msg);
        }
    }
}
