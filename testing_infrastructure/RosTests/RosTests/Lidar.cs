using Microsoft.VisualStudio.TestTools.UnitTesting;
using OxyPlot;
using RosTests.ControlFunctionality;
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace RosTests
{
    [TestClass]
    public class Lidar
    {
        [TestInitialize]
        public void setup()
        {
            ProcessLogic.setUpProcesses();
        }

        [TestMethod]
        public void changeOnlyEpsilon()
        {
            var slopes = new List<DataPoint>();

            double epsilon = 0.025;
            const uint minNumberPoints = 25;

            while(epsilon <= 1.0)
            {
                writeValuesClusterParam(minNumberPoints, epsilon);
                ProcessLogic.launchProcessMonitor();
                slopes.Add(new DataPoint(epsilon, findNumClusters()));

                epsilon += 0.025;
            }

            var graphInfo = new GraphInformation();
            graphInfo.title = "Number of Clusters Detected at Once vs Change of Epsilon";
            graphInfo.xLabel = "Epsilon";
            graphInfo.yLabel = "Max Number Clusters Detected";
            graphInfo.exportedWidth = 640;
            graphInfo.exportedHeight = 480;
            graphInfo.dataPoints = slopes;

            GraphCreation.createLineGraphFromCSV(graphInfo, GraphCreation.getGraphOutputFolder() + "/epsilon-cluster.pdf");
        }

        [TestMethod]
        public void changeMinNumPoints()
        {
            var slopes = new List<DataPoint>();
            uint minNumPoints = 5;

            while(minNumPoints < 100)
            {
                uint totalNumberClusters = 0;
                uint numberIterations = 0;
                double epsilon = 0.05;

                while (epsilon <= 1.1)
                {
                    writeValuesClusterParam(minNumPoints, epsilon);
                    ProcessLogic.launchProcessMonitor();

                    totalNumberClusters += findNumClusters();
                    numberIterations += 1;

                    epsilon += 0.20;
                }

                double averageClusters = (double)totalNumberClusters / numberIterations;
                slopes.Add(new DataPoint(minNumPoints, averageClusters));

                minNumPoints += 5;
            }

            var graphInfo = new GraphInformation();
            graphInfo.title = "Number of Clusters Detected at Once vs Change of Min Num Points";
            graphInfo.xLabel = "Min Number of Points";
            graphInfo.yLabel = "Average Numbers of Clusters Detected";
            graphInfo.exportedWidth = 640;
            graphInfo.exportedHeight = 480;
            graphInfo.dataPoints = slopes;

            GraphCreation.createLineGraphFromCSV(graphInfo, GraphCreation.getGraphOutputFolder() + "/min-num_points-epsilon_changer.pdf");
        }

        public static void writeValuesClusterParam(uint minNumPoints, double epsilon)
        {
            string pidFilePath = UserDefinedControls.getProjectDirectory() + "/src/cruise_control/scripts/cluster_values.txt";

            string fileContents = string.Empty;
            fileContents += minNumPoints.ToString() + "\n";
            fileContents += epsilon.ToString();

            File.WriteAllText(pidFilePath, fileContents);
        }

        public static uint findNumClusters()
        {
            string cluster_output_location = UserDefinedControls.getProjectDirectory() + "/src/cruise_control/scripts/cluster_output.txt";
            string[] lines = File.ReadAllLines(cluster_output_location);
            
            uint maxNumClusters = 1; // Not being a part of any cluster is calculated as a cluster; therefore always at least one "cluster"
            foreach(var line in lines)
            {
                uint currentNumClusters = uint.Parse(line);
                if(currentNumClusters > maxNumClusters)
                {
                    maxNumClusters = currentNumClusters;
                }
            }

            // Realistically though, the "not a part of any cluster" cluster should not be considered a cluster
            return maxNumClusters - 1;
        }
    }
}
