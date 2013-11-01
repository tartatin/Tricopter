using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using CmdLine.Net.Control;

namespace CmdLine.Net.States
{
    class PIDChangingState : StateBase
    {
        public override ActionType getStateActionType()
        {
            return ActionType.Continuous;
        }

        public override string getStateName()
        {
            return "PID";
        }

        public override string getStateInfos()
        {
            return getPIDValues();
        }

        private string getPIDValues()
        {
            return String.Format(@"P: {0}, I: {1}, D: {2}", FlightParameters.get().PID_P, FlightParameters.get().PID_I, FlightParameters.get().PID_D);
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

                return new CmdLineResult(true, "", getPIDValues(), true, true);
            }

            // Accès aux P, I, D de manière individuelle
            StateBase lNewState = null;
            if (pCmd.Line == "p")
                lNewState = new PChangingState();
            else if (pCmd.Line == "i")
                lNewState = new IChangingState();
            else if (pCmd.Line == "d")
                lNewState = new DChangingState();

            if (lNewState != null)
                setSubState(lNewState);

            return new CmdLineResult(true, "", "", true, false);
        }
    }
}
