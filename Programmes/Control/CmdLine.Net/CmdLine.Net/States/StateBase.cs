using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CmdLine.Net
{
    abstract class StateBase
    {
        public enum ActionType
        {
            Once,
            Continuous
        }

        private StateBase _SubState;

        public StateBase()
        {
            // Pas de sous état par défaut
            _SubState = null;
        }

        public abstract ActionType getStateActionType();
        public abstract string getStateName();
        public abstract string getStateInfos();

        public ActionType getStateTreeActionType()
        {
            if (_SubState != null)
                return _SubState.getStateTreeActionType();
            else
                return getStateActionType();
        }

        public string getStateTreeInfos()
        {
            if (_SubState != null)
                return _SubState.getStateTreeInfos();
            else
                return getStateInfos();
        }

        public string getStatePath()
        {
            string lResult = getStateName();
            if (_SubState != null)
                lResult += @"\" + _SubState.getStatePath();
            return lResult;
        }

        public void setSubState(StateBase pState)
        {
            _SubState = pState;
        }

        public virtual CmdLineResult handleCommand(CmdLineStruct pCmd)
        {
            // Passage de la commande au sous-état, s'il en existe un
            if (_SubState != null)
            {
                // Récupération du résultat du sous état
                CmdLineResult lSubStateResult = _SubState.handleCommand(pCmd);

                // Désactivation du sous état s'il a terminé sa tâche
                if (lSubStateResult.StateEnded == true)
                    _SubState = null;
                
                // Sortie
                lSubStateResult.StateEnded = false;
                lSubStateResult.CmdAccepted = true;
                return lSubStateResult;
            }

            return new CmdLineResult(false, "", "", false, false);
        }
    }
}
