using System;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Threading;

namespace RosTests.ControlFunctionality
{
    public class ProcessLogic
    {
        private static bool exitedTestThreadSuccessfully = false;
        private static Process airSimProcess = new Process();
        private static Process rosProcess = new Process();

        /// <summary>
        /// Configures the appropriate processes to launch AirSim and the ROS project.
        /// This function does not launch the specified programs
        /// </summary>
        public static void setUpProcesses()
        {
            if (!File.Exists(UserDefinedControls.getAirSimNHDirectory() + "/AirSimNH.exe"))
            {
                throw new Exception("Could not find AirSimNH executable at: " + UserDefinedControls.getAirSimNHDirectory());
            }

            if (!Directory.Exists(UserDefinedControls.getProjectDirectory()))
            {
                throw new Exception("Could not find ROS Project at: " + UserDefinedControls.getProjectDirectory());
            }

            // *** Setup launching AirSim ***

            airSimProcess.StartInfo.FileName = "cmd.exe";

            string airSimLaunchArgs = string.Empty;
            airSimLaunchArgs += " /c "; // This is required so value of string is seen as a command
            airSimLaunchArgs += " cd " + UserDefinedControls.getAirSimNHDirectory();
            airSimLaunchArgs += " && AirSimNH";

            if (!UserDefinedControls.getAirSimFullScreen())
            {
                airSimLaunchArgs += " -ResX = " + UserDefinedControls.getAirSimWindowWidth().ToString();
                airSimLaunchArgs += " -ResY = " + UserDefinedControls.getAirSimWindowHeight().ToString();
                airSimLaunchArgs += " -windowed";
            }

            airSimProcess.StartInfo.Arguments = airSimLaunchArgs;

            // *** Setup launching ROS Project ***

            rosProcess.StartInfo.FileName = "cmd.exe";

            string rosLaunchArgs = string.Empty;
            rosLaunchArgs += " /c ";
            rosLaunchArgs += " cd " + UserDefinedControls.getProjectDirectory();
            rosLaunchArgs += " && .\\devel\\setup.bat";
            rosLaunchArgs += " && roslaunch central_control full_system.launch";

            rosProcess.StartInfo.Arguments = rosLaunchArgs;
        }

        /// <summary>
        /// Launches the monitor that oversees execution of AirSim and ROS
        /// </summary>
        public static bool launchProcessMonitor()
        {
            exitedTestThreadSuccessfully = false;

            // Launch AirSim and ROS in a different thread so that this thread remains active.
            // Allows this thread to check if the current test logic being executed by ROS
            // is longer than the test timeout
            Thread testThread = new Thread(new ThreadStart(launchProcesses));
            testThread.Start();

            for (int i = 0; i < UserDefinedControls.getTimeOutTestSeconds(); i++)
            {
                if (!testThread.IsAlive)
                {
                    break;
                }

                Thread.Sleep(1000);
            }

            // Terminate ROS and AirSim in that order, to minimize chance that ROS breaks from having AirSim close before it does.
            // If test finished before timeout, these processes won't exist and the following loops will have no effect

            foreach (var process in Process.GetProcesses().Where(p => p.ProcessName.Contains("roslaunch") || p.ProcessName.Contains("rosmaster") || p.ProcessName.Contains("rosout")))
            {
                process.Kill();
            }

            foreach (var process in Process.GetProcesses().Where(p => p.ProcessName.Contains("AirSim")))
            {
                process.Kill();
            }

            return exitedTestThreadSuccessfully;
        }

        /// <summary>
        /// Launches the AirSim and ROS processes
        /// </summary>
        private static void launchProcesses()
        {
            airSimProcess.Start();

            // Wait a bit to let AirSim launch, so that it is ready before ROS finishes intiailizing itself
            Thread.Sleep(UserDefinedControls.getAirSimPauseMilliseconds());

            rosProcess.Start();
            rosProcess.WaitForExit();

            exitedTestThreadSuccessfully = true;
        }
    }
}
