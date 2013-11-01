using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using CmdLine.Net.Control;

namespace CmdLine.Net.States.Flight
{
    class InFlightState : StateBase
    {
        public override ActionType getStateActionType()
        {
            return ActionType.Once;
        }

        public override string getStateName()
        {
            return "In Flight";
        }

        public override string getStateInfos()
        {
            return "";
        }

        public InFlightState()
        {
            SerialCommunication.get().open();
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
                SerialCommunication.get().close();
                return new CmdLineResult(true, "", "", true, true);
            }

            // Accès aux P, I, D de manière individuelle
            StateBase lNewState = null;

            if (lNewState != null)
                setSubState(lNewState);

            return new CmdLineResult(true, "", "", true, false);
        }
    }
}
