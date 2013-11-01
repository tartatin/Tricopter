using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using CmdLine.Net.States;
using CmdLine.Net.Control;

namespace CmdLine.Net.Console
{
    class StateManager
    {
        private int _CurrentConsoleBlockTop;
        private int _LastBlockEndLine;
        private StateBase _RootState;

        public StateManager()
        {
            _RootState = new RootState();
            _CurrentConsoleBlockTop = 0;
            _LastBlockEndLine = 0;
        }

        private int writeFullLine(string pValue)
        {
            int lLineIndex = System.Console.CursorTop;
            System.Console.SetCursorPosition(0, System.Console.CursorTop);
            
            // On écrit le texte sur autant de lignes que nécessaire
            while (pValue.Length > 0)
            {
                string lValue = pValue.Substring( 0, Math.Min(pValue.Length, System.Console.WindowWidth) );
                pValue = pValue.Substring( lValue.Length, pValue.Length - lValue.Length );

                System.Console.Write(lValue);
            }

            // On complète la ligne actuelle d'espaces, pour effacer le buffer
            for (int i = pValue.Length; i < System.Console.WindowWidth; i++)
                System.Console.Write(" ");

            return lLineIndex;
        }

        private void clearScreenFrom(int pLineIdx)
        {
            for (int i = pLineIdx; i < _LastBlockEndLine; ++i)
            {
                System.Console.SetCursorPosition(0, i);
                for (int j = 0; j < System.Console.WindowWidth; j++)
                    System.Console.Write(" ");
            }
        }

        private void setConsoleCursorPosition(int pLeftOrigin, int pTopOrigin)
        {
            pTopOrigin += pLeftOrigin / System.Console.WindowWidth;
            pLeftOrigin = pLeftOrigin % System.Console.WindowWidth;

            if (pTopOrigin < 0)
                pTopOrigin = 0;

            System.Console.SetCursorPosition(pLeftOrigin, pTopOrigin);
        }

        private void updateOutput(string pStillLine = "", string pInputLine = "", string pRemarkLine = "")
        {
            setConsoleCursorPosition(0, _CurrentConsoleBlockTop);

            if (pStillLine != "")
                writeFullLine("> " + pStillLine);

            _CurrentConsoleBlockTop = writeFullLine("");

            System.Console.ResetColor();
            System.Console.BackgroundColor = ConsoleColor.DarkBlue;
            System.Console.ForegroundColor = ConsoleColor.White;
            writeFullLine("[" + _RootState.getStatePath() + "]");

            System.Console.ResetColor();
            System.Console.ForegroundColor = ConsoleColor.Yellow;
            writeFullLine("(" + _RootState.getStateTreeInfos() + ")");
            
            System.Console.ForegroundColor = ConsoleColor.White;
            int lInputLineIdx = writeFullLine("> " + pInputLine);

            System.Console.ResetColor();
            int lLastLine = writeFullLine("# " + pRemarkLine);

            clearScreenFrom(lLastLine+1);
            _LastBlockEndLine = lLastLine;

            setConsoleCursorPosition(2 + pInputLine.Length, lInputLineIdx);
        }

        public void handleCommands()
        {
            /**
             *  Les 3 dernières lignes de la console contiennent :
             *      - le chemin de l'état actuel,
             *      - la commande en cours d'écriture,
             *      - une ligne affichant le retour de l'état en cours, vis-à-vis de la commande en cours d'écriture
             */

            System.Console.WriteLine("Enter your commands.");

            _CurrentConsoleBlockTop = System.Console.CursorTop;
            updateOutput("", "", "");

            System.ConsoleKeyInfo cki = new System.ConsoleKeyInfo();

            // Boucle de traitement infinie
            string lLineBuffer = "";
            bool lLineDone = false;
            bool lDone = false;

            while (lDone == false)
            {
                if (System.Console.KeyAvailable == true)
                {
                    // On récupère le caractère sans l'afficher
                    cki = System.Console.ReadKey(true);

                    // On traite les caractères spéciaux, notamment la fin de ligne
                    switch (cki.Key)
                    {
                        case ConsoleKey.Enter:
                            lLineDone = true;
                            break;

                        case ConsoleKey.Backspace:
                            if (lLineBuffer.Length > 0)
                                lLineBuffer = lLineBuffer.Substring(0, lLineBuffer.Length - 1);
                            break;

                        default:
                            lLineBuffer += cki.KeyChar;
                            break;
                    }

                    string lResultText = "";
                    string lRemarkText = "";

                    if ((_RootState.getStateTreeActionType() == StateBase.ActionType.Continuous) || (lLineDone == true))
                    {
                        // On traite la ligne
                        CmdLineStruct lCmdLine = new CmdLineStruct();
                        lCmdLine.Line = lLineBuffer;

                        if (lLineBuffer.Length > 0)
                        {
                            CmdLineResult lCmdResult = _RootState.handleCommand(lCmdLine);

                            lResultText = lCmdResult.ResultText;
                            lRemarkText = lCmdResult.RemarkText;

                            if (lCmdResult.CmdAccepted)
                            {
                                if (lCmdResult.ResetLine)
                                    lLineDone = true;
                            }

                            if (lCmdResult.StateEnded)
                                lDone = true;
                        }

                        //
                        if (lLineDone)
                        {
                            lLineBuffer = "";
                            lLineDone = false;
                        }
                    }

                    // On met à jour les deux dernières lignes
                    updateOutput(lResultText, lLineBuffer, lRemarkText);
                }
                else
                {
                    //if (MessageManager.get().treatMessages() == true)
                    {

                    }
                    System.Threading.Thread.Sleep(1);
                }
            }
        }
    }
}
