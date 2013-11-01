using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CmdLine.Net.States.Com
{
    class ComState: StateBase
    {
        public override ActionType getStateActionType()
        {
            return ActionType.Once;
        }

        public override string getStateName()
        {
            return "Communication";
        }

        public override string getStateInfos()
        {
            return "115200 / 57600 / 38400";
        }

        private string getCOMState()
        {
            return "";
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
                return new CmdLineResult(true, "", getCOMState(), true, true);
            }

            // Accès aux P, I, D de manière individuelle
            StateBase lNewState = null;
            if (pCmd.Line == "port")
                lNewState = new ComPortChangeState();
            else if (pCmd.Line == "baud")
                lNewState = new ComBaudChangeState();

            if (lNewState != null)
                setSubState(lNewState);

            return new CmdLineResult(true, "", "", true, false);
        }
    }
}
