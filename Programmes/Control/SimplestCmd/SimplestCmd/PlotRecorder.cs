using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;

namespace SimplestCmd
{
    class PlotRecorder
    {
        static string _sessionName = "";

        StreamWriter stream;

        static string getSessionName()
        {
            if (_sessionName == "")
            {
                _sessionName = String.Format("{0:yyMMdd_HHmmssff}", DateTime.Now);
            }
            return _sessionName;
        }

        public PlotRecorder(string pFilename)
        {
            string lFilename = getSessionName() + "_" + System.IO.Path.GetFileName(pFilename);
            string lDirName = System.IO.Path.GetDirectoryName(pFilename);
            lFilename = lDirName + lFilename;
            stream = new StreamWriter(lFilename, true);
        }

        public void addPoint(float X, List<int> Values)
        {
            String line = String.Format("{0}", X);
            for (int i = 0; i < Values.Count; ++i)
                line += String.Format(";{0}", Values[i]);
            stream.WriteLine(line);
            stream.Flush();
        }

        public void addPoint(float X, List<float> Values)
        {
            String line = String.Format("{0}", X);
            for (int i = 0; i < Values.Count; ++i)
                line += String.Format(";{0}", Values[i]);
            stream.WriteLine(line);
            stream.Flush();
        }

        public void addPoint(float X, float Y)
        {
            String line = String.Format("{0};{1}", X, Y);
            stream.WriteLine(line);
            stream.Flush();
        }

        public void addPoint(float X, float Y, float Z)
        {
            String line = String.Format("{0};{1};{2}", X, Y, Z);
            stream.WriteLine(line);
            stream.Flush();
        }

        public void addPoint(float X, float Y, float Z, float W)
        {
            String line = String.Format("{0};{1};{2};{3}", X, Y, Z, W);
            stream.WriteLine(line);
            stream.Flush();
        }
    }
}
