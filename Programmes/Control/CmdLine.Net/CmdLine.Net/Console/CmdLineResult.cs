using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CmdLine.Net
{
    class CmdLineResult
    {
        public bool CmdAccepted;
        public string RemarkText;
        public string ResultText;
        public bool ResetLine;
        public bool StateEnded;

        public CmdLineResult(bool pCmdAccepted = true, string pRemarkText = "", string pResultText = "", bool pResetLine = false, bool pStateEnded = true)
        {
            CmdAccepted = pCmdAccepted;
            RemarkText = pRemarkText;
            ResultText = pResultText;
            ResetLine = pResetLine;
            StateEnded = pStateEnded;
        }
    }
}
