#----------------------------------------------------------------------------------
#-- Company: GRAMS
#-- Designer: Tommy Rossignol
#--
#-- Create Date: 2025-03-20
#-- Description:
#--     module to set path and variable to use for the scripts
#--
#-- Dependencies:
#-- Revision:
#-- Revision 1.0 - File Created
#-- Additional Comments:
#----------------------------------------------------------------------------------
import os, sys
from enum import IntEnum

# custom modules
from modules.fgColors import fgColors
import modules.systemHelper as systemHelper
import modules.sshClientHelper as sshClientHelper
#from modules.zynqHelper import *

try:
    moduleName = os.path.basename(__file__)
except NameError:
    moduleName = "moduleNameNotFound.py"

systemHelper.sectionPrint("Setting project path variables")

# -----------------------------------------------
# --- get PROJECT_PATH based on setup
# -----------------------------------------------
# NOTE: possible options:
#       1- from PROJECT_PATH environment variable
#       3- from the hardcoded path (not recommended)
if os.environ.get("PROJECT_PATH") is not None:
    # user specified the PROJECT_PATH from shell
    PROJECT_PATH = os.environ['PROJECT_PATH']
    print(f"{fgColors.green}[{moduleName}] Using project path defined by os.environ['PROJECT_PATH']: {PROJECT_PATH}{fgColors.endc}")

else:
    try:
        modulesPath = os.path.dirname(os.path.abspath(__file__))
        pythonPath = os.path.dirname(modulesPath)
    except NameError:
        modulesPath = os.path.abspath("./")
        pythonPath = os.path.dirname(modulesPath)
    if "hostApps" in pythonPath:
        projectPath = pythonPath.split("hostApps")[0]
        if projectPath[-1] == "/":
            projectPath = projectPath[:-1]
        if os.path.isdir(projectPath):
            PROJECT_PATH = projectPath
            print(f"{fgColors.green}[{moduleName}] Using project path from current path: {PROJECT_PATH}{fgColors.endc}")

if not "PROJECT_PATH" in locals():
    # fall back, should not end here
    PROJECT_PATH = os.path.abspath("../../")
    print(f"{fgColors.yellow}[{moduleName}] Using relative path from current path: {PROJECT_PATH}{fgColors.endc}")


# -----------------------------------------------
# --- get HOST_APPS_PATH based on setup
# -----------------------------------------------
if os.path.isfile("/etc/debian_version"):
    HOST_APPS_PATH = f"{PROJECT_PATH}/hostApps/cpp/debianBasedOS"
elif os.path.isfile("/etc/redhat-release"):
    HOST_APPS_PATH = f"{PROJECT_PATH}/hostApps/cpp/redHatBasedOS"
else:
    print(f"{fgColors.red}[{moduleName}] ERROR: Unsupported OS. Please ask for support with your OS info.{fgColors.endc}")
    sys.exit()


# -----------------------------------------------
# --- get USER_DATA_DIR path based on setup
# -----------------------------------------------
HOME = os.environ.get("HOME", "~") # home of user
defaultUserDataDir = os.path.join(HOME, "PDCv2-data")
if os.environ.get("USER_DATA_DIR") is not None:
    # user specified the USER_DATA_DIR from shell
    USER_DATA_DIR = os.environ['USER_DATA_DIR']
    print(f"{fgColors.green}[{moduleName}] Using user data directory defined by os.environ['USER_DATA_DIR']: {USER_DATA_DIR}{fgColors.endc}")

elif os.path.isdir(defaultUserDataDir):
    # user created default directory (normally created in setup.sh)
    USER_DATA_DIR = defaultUserDataDir
    print(f"{fgColors.green}[{moduleName}] Using default user data directory: {USER_DATA_DIR}{fgColors.endc}")

else:
    # user want to hard code its data directory
    print(f"{fgColors.yellow}[{moduleName}] To use a custom user data directory, set environment variable 'USER_DATA_DIR' from your shell,{fgColors.endc}")
    print(f"{fgColors.yellow}[{moduleName}] or create the default directory at {defaultUserDataDir}{fgColors.endc}.")
    USER_DATA_DIR = "/mnt/zynq/PDCv2/user-data"
    print(f"{fgColors.green}[{moduleName}] Using manually defined user data directory: {USER_DATA_DIR}{fgColors.endc}")

# -----------------------------------------------
# --- get HDF5 input path based on setup
# -----------------------------------------------
# HDF5 directory to read from
# NOTE: possible options:
#       1- from HDF5_DATA_DIR environment variable
#       2- from default directory (defaultHdf5Dir)
#       3- from the hardcoded path (not recommended)

defaultHdf5Dir = os.path.join(defaultUserDataDir, "HDF5")
if os.environ.get("HDF5_DATA_DIR") is not None:
    # user specified the HDF5_DATA_DIR from shell
    HDF5_DATA_DIR = os.environ['HDF5_DATA_DIR']
    print(f"{fgColors.green}[{moduleName}] Using HDF5 input directory defined by os.environ['HDF5_DATA_DIR']: {HDF5_DATA_DIR}{fgColors.endc}")

elif os.path.isdir(defaultHdf5Dir):
    # user created default hdf5 directory (normally created in setup.sh)
    HDF5_DATA_DIR = defaultHdf5Dir
    print(f"{fgColors.green}[{moduleName}] Using default HDF5 input directory: {HDF5_DATA_DIR}{fgColors.endc}")

else:
    # user want to hard code its HDF5 directory
    print(f"{fgColors.yellow}[{moduleName}] To use a custom directory for HDF5, set environment variable 'HDF5_DATA_DIR' from your shell,{fgColors.endc}")
    print(f"{fgColors.yellow}[{moduleName}] or create the default directory at {defaultHdf5Dir}{fgColors.endc}.")
    HDF5_DATA_DIR = "/mnt/zynq/PDCv2/user-data/HDF5"
    print(f"{fgColors.green}[{moduleName}] Using manually defined HDF5 input directory: {HDF5_DATA_DIR}{fgColors.endc}")










