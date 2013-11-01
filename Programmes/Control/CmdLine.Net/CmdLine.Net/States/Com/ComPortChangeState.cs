using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Text.RegularExpressions;

namespace CmdLine.Net.States.Com
{
    class ComPortChangeState: StateBase
    {
        public override ActionType getStateActionType()
        {
            return ActionType.Once;
        }

        public override string getStateName()
        {
            return "Com Port";
        }

        public override string getStateInfos()
        {
            return getPort();
        }

        private string getPort()
        {
            return "Com port: COMxx";
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
                return new CmdLineResult(true, "", getPort(), true, true);
            }

            // Regex
            Regex regex = new Regex(@"^(COM)?(\d+)$");
            Match match = regex.Match(pCmd.Line);

            // Commande incorrecte
            if ((match.Groups[1].Success == false) && (match.Groups[2].Success == false))
                return new CmdLineResult(true, "Bad command", "", false, false);

            // De quelle commande s'agit-il ?
            int lValue;

            if (match.Groups[2].Value.Length > 0)
                lValue = (int) Convert.ToInt32(match.Groups[2].Value);

            return new CmdLineResult(true, "", "", true, false);
        }
    }
}
