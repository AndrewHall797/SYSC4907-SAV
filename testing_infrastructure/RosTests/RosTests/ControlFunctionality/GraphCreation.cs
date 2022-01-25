using System;
using System.Collections.Generic;
using System.IO;
using System.Text.RegularExpressions;
using OxyPlot;
using OxyPlot.Axes;
using OxyPlot.Series;

namespace RosTests.ControlFunctionality
{
    /// <summary>
    /// Information needed to generate a line graph as a PDF
    /// </summary>
    public class GraphInformation
    {
        public string title;
        public string xLabel;
        public string yLabel;
        public uint exportedWidth;
        public uint exportedHeight;
        public List<DataPoint> dataPoints;
    }

    public class GraphCreation
    {
        /// <summary>
        /// Creates a line graph from the graph information, that is written as a PDF to the specified location
        /// </summary>
        /// <param name="graphInfo">Information controlling how to render the graph</param>
        /// <param name="outputLocation">Location to write graph PDF to</param>
        public static void createLineGraphFromCSV(GraphInformation graphInfo, string outputLocation)
        {
            var model = new OxyPlot.PlotModel();
            model.Title = graphInfo.title;

            var xAxis = new LinearAxis();
            xAxis.Position = AxisPosition.Bottom;
            xAxis.Title = graphInfo.xLabel;
            xAxis.FontSize = 22;

            var yAxis = new LinearAxis();
            yAxis.Position = AxisPosition.Left;
            yAxis.Title = graphInfo.yLabel;
            yAxis.FontSize = 22;

            model.Axes.Add(xAxis);
            model.Axes.Add(yAxis);

            var lineSeries = new LineSeries();
            lineSeries.Color = OxyColors.SkyBlue;
            lineSeries.MarkerType = MarkerType.Circle;
            lineSeries.MarkerSize = 6;
            lineSeries.MarkerStroke = OxyColors.White;
            lineSeries.MarkerFill = OxyColors.SkyBlue;
            lineSeries.MarkerStrokeThickness = 1.5;

            foreach(var point in graphInfo.dataPoints)
            {
                lineSeries.Points.Add(point);
            }

            model.Series.Add(lineSeries);
            using (var stream = File.Create(outputLocation))
            {
                var pdfExporter = new PdfExporter { Width = graphInfo.exportedWidth, Height = graphInfo.exportedHeight };
                pdfExporter.Export(model, stream);
            }
        }

        /// <summary>
        /// Get the location of the folder that contains test graphs
        /// </summary>
        /// <returns>Test graph folder location</returns>
        public static string getGraphOutputFolder()
        {
            return Path.Combine(Directory.GetParent(Directory.GetCurrentDirectory()).Parent.Parent.Parent.FullName, "GraphOutput");
        }

        /// <summary>
        /// Converts a CSV file in the format of:
        /// 
        /// pointX, pointY
        /// pointX, pointY
        /// ...
        /// 
        /// To a list of data point to be processed and/or plotted
        /// </summary>
        /// <param name="fileLocation"></param>
        /// <returns></returns>
        public static List<DataPoint> readCSVFile(string fileLocation)
        {

            var extractedPoints = new List<DataPoint>();
            string[] fileLines = File.ReadAllLines(fileLocation);

            foreach(string line in fileLines)
            {
                // Could switch to a regex if more robust parsing is needed. As of time of writing this is adequate
                string[] values = line.Split(",");
                extractedPoints.Add(new DataPoint(double.Parse(values[0]), double.Parse(values[1])));
            }

            return extractedPoints;
        }
    }
}
