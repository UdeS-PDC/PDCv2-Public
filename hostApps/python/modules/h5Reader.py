#----------------------------------------------------------------------------------
#-- Company: GRAMS
#-- Designer: Tommy Rossignol
#--
#-- Create Date: 2024-05-21
#-- Description:
#--     import Controller data from a HDF5 file
#--
#-- Dependencies:
#-- Revision:
#-- Revision 1.0 - File Created
#-- Additional Comments:
#----------------------------------------------------------------------------------
import os, sys
from pathlib import Path
import time
from time import sleep as timeSleep
import pandas as pd
import h5py
import numpy as np
import math

from enum import IntEnum

# -----------------------------------------------
# --- Global settings
# -----------------------------------------------
parsedH5 = []

def dbgPrint(message, enabled=False):
    if enabled: print(message)


# -----------------------------------------
# --- Classes to hold the ZPP information
# -----------------------------------------
class PDC_ZPP_ITEM(IntEnum):
    AVG = 0
    BIN = 1
    LAST = 2
    MAX = 3
    MIN = 4
    NUL = 5
    PRD = 6
    TOT = 7
    NONE = 8

PDC_ZPP_NAME = [
    "AVG",
    "BIN",
    "LAST",
    "MAX",
    "MIN",
    "NUL",
    "PRD",
    "TOT",
    "NONE"]

class PDC_ZPP:
    def __init__(self):
        self.empty = True
        self.AVG = -1.0
        self.BIN = -1.0
        self.LAST = -1.0
        self.MAX = -1.0
        self.MIN = -1.0
        self.NUL = -1.0
        self.PRD = -1.0
        self.TOT = -1.0

        # calculated values
        self.TCR = -1
        self.UCR = -1
        self.CCR = -1

    def print(self):
        """
        print all members of the class
        """
        for item in self.__dict__:
            print(f"{item}={self.__dict__[item]}")


    def setItem(self, item: PDC_ZPP_ITEM, value):
        if value is None:
            return

        try:
            val = int(value[0])
        except OverflowError:
            return
        except ValueError:
            return

        if item == PDC_ZPP_ITEM.AVG:
            self.empty = False
            self.AVG = val
        elif item == PDC_ZPP_ITEM.BIN:
            self.empty = False
            self.BIN = val
        elif item == PDC_ZPP_ITEM.LAST:
            self.empty = False
            self.LAST = val
        elif item == PDC_ZPP_ITEM.MAX:
            self.empty = False
            self.MAX = val
        elif item == PDC_ZPP_ITEM.MIN:
            self.empty = False
            self.MIN = val
        elif item == PDC_ZPP_ITEM.NUL:
            self.empty = False
            self.NUL = val
        elif item == PDC_ZPP_ITEM.PRD:
            self.empty = False
            self.PRD = val
        elif item == PDC_ZPP_ITEM.TOT:
            self.empty = False
            self.TOT = val

    def isEmpty(self):
        return self.empty

    def process(self):
        #self.print()
        if self.TOT != -1 and self.BIN != -1 and self.PRD != -1:
            # calculation
            self.TCR = (self.TOT/self.BIN)/(10e-9*self.PRD)
        else:
            # unable to calculate
            self.TCR = -1
        if self.TCR != -1 and self.NUL > 0:
            self.UCR = -1*math.log(self.NUL/self.BIN)/(10.0e-9*self.PRD)
            if self.UCR > self.TCR:
                self.UCR = -1
        else:
            self.UCR = -1
        if self.TCR > 0 and self.UCR > 0:
            self.CCR = (self.TCR-self.UCR)/self.TCR
        else:
            self.CCR = -1

# -----------------------------------------------
# --- Class to parse HDF5 database
# -----------------------------------------------
class h5Reader:
    def __init__(self,
                 deleteAfter : bool = False,
                 hfRelPath : str = "",
                 hfAbsPath : str = "",
                 hfFile : str = "",
                 sysClkPrd : float = 10.0e-9,
                 dsumPrd : int = -1
                ):
        """
        init of the h5reader class
        """
        # settings for the measurements
        self.h5 = None
        self.deleteAfter = deleteAfter
        self.scripts_path = os.path.dirname(__file__)
        self.parsedH5 = []
        if not hfRelPath == "":
            # relative path
            self.hdf5_path = os.path.join(self.scripts_path, hfRelPath, "")
        elif not hfAbsPath == "":
            # aqbsolute path
            self.hdf5_path = hfAbsPath
        else:
            # default path
            self.hdf5_path = os.path.join(self.scripts_path, "HDF5", "DEVEL", "")

        # HDF5 members
        self.HDF_TRANSMIT = None
        self.HDF_CTL = None

        # PDC settings
        self.sysClkPrd = sysClkPrd
        if dsumPrd > 0:
            # value is specified
            self.dsumPrd = self.sysClkPrd*dsumPrd
        else:
            # default value, dsumPrd is based on system clock
            self.dsumPrd = self.sysClkPrd

        # using getLastH5 takes time, keep it at the end of the constructor
        if hfFile == "":
            # automatically find a file
            self.hfFile = self.getLastH5()
        else:
            # specified file
            self.hfFile = os.path.join(self.hdf5_path, hfFile)

    def __del__(self):
        """
        destructor of the class
        """
        self.h5Close()

    def print(self):
        """
        print all members of the class
        """
        for item in self.__dict__:
            print(f"{item}={self.__dict__[item]}")

    def getPathList(self, path, newFirst=True):
        """
        function to list the files in a path
        path: name of the path to look for files
        newFirst: True = New files first, Fale = Old files first
        """
        #return sorted(Path(path).iterdir(), reverse=reverseOrder, key=os.path.getmtime)
        if newFirst:
            reverse=''
        else:
            reverse='r'
        with os.popen(f"ls -1t{reverse} {path}/*.h5 2> /dev/null") as cmdFile:
            return cmdFile.read().split()

    def getLastH5(self):
        """
        function to get the most recent H5 file
        """
        pathList=self.getPathList(path=self.hdf5_path)
        for pathName in pathList:
            # look only for data files from dma
            fileName=os.path.basename(pathName);
            if (not ".h5" in fileName):
                continue

            if ("CTL_XXX" in fileName):
                # file is not ready
                continue

            # Reduce list to 5 files to save memory when not deleting after reading
            if len(self.parsedH5) > 5 and not self.deleteAfter:
                self.parsedH5 = self.parsedH5[-5:]

            # for each file in order, check if file is not empty
            if (os.path.getsize(pathName) > 0):
                # check if file has already been written to
                if (pathName not in self.parsedH5):
                    # save filename to prevent reading it multiple times
                    #print(fileName)
                    self.parsedH5.append(pathName)
                    return pathName
        # no new file to parse
        return ""

    def newFileReady(self):
        """
        returns true when the name of the file is set
        """
        if (os.path.basename(self.hfFile) == ""):
            return False
        else:
            return True
        
    def waitForNewFile(self, timeout_sec=45):
        start_time = time.time()
        self.hfFile = self.getLastH5()

        while self.hfFile == "" and time.time() - start_time < timeout_sec :
            timeSleep(0.001)
            self.hfFile = self.getLastH5()

        return self.hfFile


    def h5Open(self):
        """
        open the HDF5 file
        """
        if self.h5 == None:
            dbgPrint(f"Opening file {self.hfFile}")
            try:
                self.h5 = h5py.File(self.hfFile, 'r+')
            except BlockingIOError as ex:
                """
                [Errno 11] Unable to open file
                (unable to lock file, errno = 11,
                error message = 'Resource temporarily
                unavailable')
                """
                dbgPrint(f"File busy")
                timeSleep(0.001)
                try:
                    self.h5 = h5py.File(self.hfFile, 'r+')
                except BlockingIOError as ex:
                    dbgPrint(f"File still busy")
                    self.h5 = None # make sure it is still set to None
        return self.h5 != None

    def h5Close(self):
        """
        close the HDF5 file
        """
        if self.h5:
            dbgPrint(f"Closing file {self.hfFile}")
            self.h5.close()
            self.h5 = None
        if self.deleteAfter and os.path.isfile(self.hfFile):
            dbgPrint(f"Deleting file {self.hfFile}")
            os.remove(self.hfFile)
            self.hfFile = ""

    def h5GetCtl(self):
        """
        Function to get 'transmit' and 'CTL' keys only once
        if not previously set
        """
        if not self.HDF_TRANSMIT:
            if type(self.h5) != type(None):
                self.HDF_TRANSMIT = self.h5.get('TRANSMIT/')
        if self.HDF_TRANSMIT and not self.HDF_CTL:
            for key in self.HDF_TRANSMIT.keys():
                if "CTL" in key:
                    dbgPrint(f"Reading from {key}")
                    self.HDF_CTL = self.HDF_TRANSMIT.get(key)
        return self.HDF_CTL != None

    def getPdcDsum(self, iPdc):
        """
        extract the digital sum data from the HDF5 database
        """
        self.h5GetCtl()
        if self.HDF_CTL == None:
            # no Controller in the file
            print(f"ERROR: no Controller found")
            return [None, None]
        # default value if execution fails
        PDC_DSUM = None
        try:
            PDC_DSUM = self.HDF_CTL.get(f"PDC/PDC_{iPdc:02d}/PDC_DATA/DGTL_SUM/DGTL_SUM")
        except ValueError as ex:
            dbgPrint("getDsumError: retrying")
            timeSleep(0.001)
            try:
                PDC_DSUM = self.HDF_CTL.get(f"PDC/PDC_{iPdc:02d}/PDC_DATA/DGTL_SUM/DGTL_SUM")
            except ValueError as ex:
                dbgPrint("getDsumError: aborting")
            
        if PDC_DSUM == None:
            #print(f"ERROR: no PDC DGTL_SUM found for PDC {iPdc}")
            return [None, None]
        dbgPrint(f"  Found DGTL_SUM for PDC {iPdc}")

        data = np.array(PDC_DSUM[:], dtype=np.uintc)
        time = np.arange(0, len(data), 1)*self.dsumPrd
        return [time, data]

    # Function to get data from HDF5 file
    def getPdcZPP(self, iPdc, zppSingle: PDC_ZPP_ITEM=None, zppList: list=None)-> PDC_ZPP:
        zppParam = []
        if (zppSingle is None) and (zppList is None):
            print(f"ERROR: zppSingle or zppList must be specified")
            return None
        elif (zppSingle is not None) and (zppList is not None):
            print(f"ERROR: zppSingle and zppList are mutually exclusive")
            return None
        elif zppSingle is not None:
            zppParam = [zppSingle]
        elif zppList is not None:
            zppParam = zppList

        self.h5GetCtl()
        if self.HDF_CTL == None:
            # not Controller in the file
            print(f"ERROR: no Controller found")
            return None

        # init an empty object for the results
        ZPP = PDC_ZPP()

        # loop for each requested parameter
        for param in zppParam:
            paramName=PDC_ZPP_NAME[param]
            #print(f"param={paramName}")
            ZPP.setItem(param, self.HDF_CTL.get(f"PDC/PDC_{iPdc:02d}/PDC_DATA/ZPP/{paramName}"))

        #ZPP = self.HDF_CTL.get(f"PDC/PDC_{iPdc:02d}/PDC_DATA/ZPP/{param}")
        if ZPP.isEmpty():
            #print(f"ERROR: no PDC ZPP {param} found for PDC {iPdc}")
            return None
        dbgPrint(f"  Found ZPP for PDC {iPdc}")
        return ZPP
















