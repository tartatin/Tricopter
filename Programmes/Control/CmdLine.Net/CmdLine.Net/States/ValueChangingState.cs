using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;

namespace CmdLine.Net.States
{
    abstract class ValueChangingState: StateBase
    {
        protected ActionType CurrentActionType;

        public ValueChangingState()
        {
            CurrentActionType = getDefaultActionType();
        }

        public override ActionType getStateActionType()
        {
            return CurrentActionType;
        }

        public override string getStateInfos()
        {
            if (CurrentActionType == ActionType.Once)
                return "Once";
            else
                return "Continuous";
        }

        public override CmdLineResult handleCommand(CmdLineStruct pCmd)
        {
            // Gestion héritée des commandes
            CmdLineResult lCmdLineResult = base.handleCommand(pCmd);
            if (lCmdLineResult.CmdAccepted)
                return lCmdLineResult;

            /**
             *  Formats acceptés :
             *      q
             *      s
             *      v
             *      +/-
             *      +/- 1
             *      +/- 1.
             *      +/- 1.2
             *      1
             *      1.
             *      1.2
             */

            // Sortie de l'état
            string lResult;

            if (pCmd.Line == "q")
            {
                lResult = String.Format("{0} : {1}", getStateName(), getCurrentValue());
                return new CmdLineResult(true, "", lResult, true, true);
            }

            // Récupération de la valeur actuelle
            else if (pCmd.Line == "g")
            {
                lResult = String.Format("{0} : {1}", getStateName(), getCurrentValue());
                return new CmdLineResult(true, lResult, "", true, false);
            }

            // Changement d'action type
            else if (pCmd.Line == "s")
            {
                CurrentActionType = ActionType.Continuous;
                return new CmdLineResult(true, "", "", true, false);
            }
            else if (pCmd.Line == "v")
            {
                CurrentActionType = ActionType.Once;
                return new CmdLineResult(true, "", "", true, false);
            }

            // Regex
            Regex regex = new Regex(@"^([\+\-]?)\s*(\d*\,?\d*)$");
            Match match = regex.Match(pCmd.Line);

            // Commande incorrecte
            if ((match.Groups[1].Success == false) && (match.Groups[2].Success == false))
                return new CmdLineResult(true, "Bad command", "", false, false);

            // De quelle commande s'agit-il ?
            float lValue;

            if (match.Groups[2].Value.Length > 0)
                lValue = (float)Convert.ToDouble(match.Groups[2].Value);
            else
                lValue = getStepValue();

            if (match.Groups[1].Value.Length > 0)
            {
                if (match.Groups[1].Value == "-")
                    lValue = -lValue;
            }

            // Application de la commande
            increateValue(lValue);
            
            // Affichage du résultat
            lResult = String.Format("{0} : {1}", getStateName(), getCurrentValue());

            lCmdLineResult = new CmdLineResult(true, lResult, "", true, false);

            return lCmdLineResult;
        }

        protected abstract ActionType getDefaultActionType();
        protected abstract void increateValue(float pValue);
        protected abstract float getStepValue();
        protected abstract float getCurrentValue();
    }
}
