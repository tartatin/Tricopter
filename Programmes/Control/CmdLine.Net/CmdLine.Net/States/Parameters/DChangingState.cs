using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using CmdLine.Net.Control;

namespace CmdLine.Net.States
{
    class DChangingState : ValueChangingState
    {
        // Cosntructeurs
        public DChangingState()
        {

        }

        // Implémentations
        protected override ActionType getDefaultActionType()
        {
            return ActionType.Continuous;
        }

        public override string getStateName()
        {
            return "PID.D";
        }

        protected override void increateValue(float pValue)
        {
            FlightParameters.get().PID_D += pValue;
        }

        protected override float getStepValue()
        {
            return 0.1f;
        }

        protected override float getCurrentValue()
        {
            return FlightParameters.get().PID_D;
        }
    }
}
