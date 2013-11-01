using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using CmdLine.Net.Control;

namespace CmdLine.Net.States
{
    class IChangingState : ValueChangingState
    {
        // Cosntructeurs
        public IChangingState()
        {

        }

        // Implémentations
        protected override ActionType getDefaultActionType()
        {
            return ActionType.Continuous;
        }

        public override string getStateName()
        {
            return "PID.I";
        }

        protected override void increateValue(float pValue)
        {
            FlightParameters.get().PID_I += pValue;
        }

        protected override float getStepValue()
        {
            return 0.1f;
        }

        protected override float getCurrentValue()
        {
            return FlightParameters.get().PID_I;
        }
    }
}
