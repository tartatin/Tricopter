using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using CmdLine.Net.States.Com;
using CmdLine.Net.States.Flight;
using CmdLine.Net.Control;

namespace CmdLine.Net.States
{
    class RootState: StateBase
    {
        public override string getStateName()
        {
            return "Root";
        }

        public override string getStateInfos()
        {
            return "Disconnected";
        }

        public override ActionType getStateActionType()
        {
            return ActionType.Once;
        }

        public override CmdLineResult handleCommand(CmdLineStruct pCmd)
        {
            // Gestion héritée des commandes
            CmdLineResult lCmdLineResult = base.handleCommand(pCmd);
            if (lCmdLineResult.CmdAccepted)
                return lCmdLineResult;

            // Accès aux états de base
            if (pCmd.Line == "q")
            {
                return new CmdLineResult(true, "", "Exiting", true, true);
            }

            // Changement d'états
            StateBase lNewSate = null;
            if ( (pCmd.Line == "thrust") || (pCmd.Line == "t") )
                lNewSate = new ThrustChangingState();
            else if (pCmd.Line == "pid")
                lNewSate = new PIDChangingState();
            else if (pCmd.Line == "com")
                lNewSate = new ComState();
            else if (pCmd.Line == "open")
            {
                lNewSate = new InFlightState();
            }

            if (lNewSate != null)
            {
                setSubState(lNewSate);
                return new CmdLineResult(true, "", "", true, false);
            }

            // Fin
            return new CmdLineResult(false, "Unknown command", "", false, false);
        }
    }
}
