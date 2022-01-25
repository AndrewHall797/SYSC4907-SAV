using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RosTests.ControlFunctionality
{
    public class UserDefinedControls
    {
        public static string getProjectDirectory() { return projectDirectory; }
        public static string getAirSimNHDirectory() { return airSimNHDirectory; }
        public static int getTimeOutTestSeconds() { return timeoutTestSeconds; }
        public static int getAirSimPauseMilliseconds() { return airSimPauseMilliseconds; }
        public static int getAirSimWindowWidth() { return airSimWindowWidth; }
        public static int getAirSimWindowHeight() { return airSimWindowHeight; }
        public static bool getAirSimFullScreen() { return airSimFullScreen; }

        // ***** These variables nay need to be modified to match correct location on your computer ******

        private static string projectDirectory = findProjectDirectory();
        private static string airSimNHDirectory = findAirSimNHDirectory();

        // ****** End of User specific location variables ******

        // ****** Can modify these variables values to whatever is desired ********

        private static int timeoutTestSeconds = 270; // Current running test will be aborted if longer than this timeout
        private static int airSimPauseMilliseconds = 2000; // Wait this amount of time after launching AirSim before launching ROS
        private static int airSimWindowWidth = 640;
        private static int airSimWindowHeight = 480;
        private static bool airSimFullScreen = false; // If true, then width and height variables don't matter

        // ******* End of user defined optional value variables *******

        /// <summary>
        /// Gets the ROS project directory. Assumes that PyCharm is being used, in which case the folder will be
        /// in the user's account folder
        /// </summary>
        /// <returns>Location of the ROS Project</returns>
        private static string findProjectDirectory()
        {
            return Environment.GetFolderPath(Environment.SpecialFolder.UserProfile) + "/PycharmProjects/SYSC4907-SAV";
        }

        /// <summary>
        /// Gets the location of the AirSim neighbourhood environment executable. Assumed to be in the
        /// User's document folder
        /// </summary>
        /// <returns>Location of AirSim Neighbourhood binary</returns>
        private static string findAirSimNHDirectory()
        {
            return Environment.GetFolderPath(Environment.SpecialFolder.UserProfile) + "/Documents/AirSimNH/WindowsNoEditor";
        }

    }
}
