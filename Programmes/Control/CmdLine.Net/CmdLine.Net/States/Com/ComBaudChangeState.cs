using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Text.RegularExpressions;
using CmdLine.Net.Control;

namespace CmdLine.Net.States.Com
{
    class ComBaudChangeState : StateBase
    {
        public override ActionType getStateActionType()
        {
            return ActionType.Once;
        }

        public override string getStateName()
        {
            return "Com baud rate";
        }

        public override string getStateInfos()
        {
            return getBaudrate();
        }

        private string getBaudrate()
        {
            return String.Format("Baud rate : {0}", SerialCommunication.get().ComBaudrate);
        }

        public override CmdLineResult handleCommand(CmdLineStruct pCmd)
        {
            // Gestion héritée des commandes
            CmdLineResult lCmdLineResult = base.handleCommand(pCmd);
            if (lCmdLineResult.CmdAccepted)
                return lCmdLineResult;

            // Sortie
            if (pCmd.Line == "q")
            {
                return new CmdLineResult(true, "", getBaudrate(), true, true);
            }

            // Regex
            Regex regex = new Regex(@"^(\d+)+$");
            Match match = regex.Match(pCmd.Line);

            // Commande incorrecte
            if (match.Groups[1].Success == false)
                return new CmdLineResult(true, "Bad command", "", false, false);

            // De quelle commande s'agit-il ?
            int lValue;

            if (match.Groups[1].Value.Length > 0)
            {
                lValue = (int)Convert.ToInt32(match.Groups[1].Value);
                SerialCommunication.get().ComBaudrate = lValue;
            }

            return new CmdLineResult(true, "", "", true, false);
        }
    }
}
