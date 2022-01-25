using Microsoft.VisualStudio.TestTools.UnitTesting;
using OxyPlot;
using RosTests.ControlFunctionality;
using System;
using System.Collections.Generic;
using System.IO;

namespace RosTests
{
    [TestClass]
    public class PIDController
    {
        [TestInitialize]
        public void setup()
        {
            ProcessLogic.setUpProcesses();
        }

        /// <summary>
        /// Iterates over increasing P-values and determines the smoothness of a path,
        /// which is then plotted as a graph
        /// </summary>
        [TestMethod]
        public void changeCoefficientP()
        {
            var slopes = new List<DataPoint>();

            double p = 1;

            for(int i = 0; i < 1; i++)
            {
                writeValuesPID(p, 0.75, 0.05);
                ProcessLogic.launchProcessMonitor();
                slopes.Add(new DataPoint(p, findSmoothnessValue()));

                p += 1;
            }

            var graphInfo = new GraphInformation();
            graphInfo.title = "Relationship Between P-Coefficient and Delta Speed Between Even Time Intervals";
            graphInfo.xLabel = "P-Value";
            graphInfo.yLabel = "Change of Speed (m/s)";
            graphInfo.exportedWidth = 640;
            graphInfo.exportedHeight = 480;
            graphInfo.dataPoints = slopes;

            GraphCreation.createLineGraphFromCSV(graphInfo, GraphCreation.getGraphOutputFolder() + "/p-coefficient.pdf");
        }

        /// <summary>
        /// Iterates over increasing I-values and determines the smoothness of a path,
        /// which is then plotted as a graph
        /// </summary>
        [TestMethod]
        public void changeCoefficientI()
        {
            var slopes = new List<DataPoint>();

            double i_value = 0.75;

            for (int i = 0; i < 5; i++)
            {
                writeValuesPID(1.0, i_value, 0.05);
                ProcessLogic.launchProcessMonitor();
                slopes.Add(new DataPoint(i_value, findSmoothnessValue()));

                i_value += 1;
            }

            var graphInfo = new GraphInformation();
            graphInfo.title = "Relationship Between I-Coefficient and Delta Speed Between Even Time Intervals";
            graphInfo.xLabel = "I-Value";
            graphInfo.yLabel = "Change of Speed (m/s)";
            graphInfo.exportedWidth = 640;
            graphInfo.exportedHeight = 480;
            graphInfo.dataPoints = slopes;

            GraphCreation.createLineGraphFromCSV(graphInfo, GraphCreation.getGraphOutputFolder() + "/i-coefficient.pdf");
        }

        /// <summary>
        /// Iterates over increasing D-values and determines the smoothness of a path,
        /// which is then plotted as a graph
        /// </summary>
        [TestMethod]
        public void changeCoefficientD()
        {
            var slopes = new List<DataPoint>();

            double d_value = 0.05;

            for (int i = 0; i < 4; i++)
            {
                writeValuesPID(1.0, 0.75, d_value);
                ProcessLogic.launchProcessMonitor();
                slopes.Add(new DataPoint(d_value, findSmoothnessValue()));

                d_value += 1;
            }

            var graphInfo = new GraphInformation();
            graphInfo.title = "Relationship Between D-Coefficient and Delta Speed Between Even Time Intervals";
            graphInfo.xLabel = "D-Value";
            graphInfo.yLabel = "Change of Speed (m/s)";
            graphInfo.exportedWidth = 640;
            graphInfo.exportedHeight = 480;
            graphInfo.dataPoints = slopes;

            GraphCreation.createLineGraphFromCSV(graphInfo, GraphCreation.getGraphOutputFolder() + "/d-coefficient.pdf");
        }

        /// <summary>
        /// Writes the given PID coefficients to a file that the cruise_control node reads to set the PID coefficients
        /// during a path that the car navigates
        /// </summary>
        /// <param name="p">Value of kP</param>
        /// <param name="i">Value of kI</param>
        /// <param name="d">Value of kD</param>
        public static void writeValuesPID(double p, double i, double d)
        {
            string pidFilePath = UserDefinedControls.getProjectDirectory() + "/src/cruise_control/scripts/pid_values.txt";

            string fileContents = string.Empty;
            fileContents += p.ToString() + "\n";
            fileContents += i.ToString() + "\n";
            fileContents += d.ToString() + "\n";

            File.WriteAllText(pidFilePath, fileContents);
        }

        /// <summary>
        /// Determines how smooth a ride is by determining average change of speed between set intervals of time
        /// </summary>
        /// <returns>Numeric value for the calculated smoothness</returns>
        public static double findSmoothnessValue()
        {
            string csvFileLocation = UserDefinedControls.getProjectDirectory() + "/src/cruise_control/scripts/speed_output.txt";
            var extractedPoints = GraphCreation.readCSVFile(csvFileLocation);

            if (extractedPoints.Count == 0)
            {
                return 0.0;
            }

            double totalChangeInSpeed = 0;

            for(int i = 0; i < extractedPoints.Count - 1; i++)
            {
                totalChangeInSpeed += Math.Abs((extractedPoints[i + 1].Y - extractedPoints[i].Y));
            }

            return totalChangeInSpeed / (extractedPoints.Count);
        }
    }
}
