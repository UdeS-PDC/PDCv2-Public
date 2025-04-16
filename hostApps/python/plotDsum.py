#----------------------------------------------------------------------------------
#-- Company: GRAMS
#-- Designer: Tommy Rossignol
#--
#-- Create Date: 2024-05-21
#-- Description:
#--     Importing PDC digital sum (Dsum) from HDF5 file (from Controller)
#--     This script do not configure the PDCs nor the Controller.
#--     It only display the results from the digital sum.
#--     For data to be read user must do the following:
#--         1 - NFS must be configured on the Zynq board.
#--         2 - 'dataReader' app must run on the Zynq board.
#--         3 - an hex to HDF5 app must run on the NFS server (e.g. dma2h5, hexRead)
#--         4 - a script must be run to configure the Controller and PDCs for acquisition
#--
#-- Dependencies:
#-- Revision:
#-- Revision 1.0 - File Created
#-- Revision 2.0 - Updated for sharing on pdcv2-public
#-- Additional Comments:
#--     NOTE: Since this app is in Python, the execution speed is limited.
#--           Once an HDF5 file is read, it is deleted (deleteAfter parameter of h5Reader).
#--           Be carefull not to generate too much data too quickly.
#--           If the data generation speed is higher than the delete speed,
#--           you may fill your server disk, or get errors.
#--     NOTE: Possible options to specify the input HDF5 directory (HDF5_DATA_DIR):
#--           1- from HDF5_DATA_DIR environment variable
#--           2- from default directory (defaultHdf5Dir)
#--           3- from the hardcoded path (not recommended)
#--           HDF5_DATA_DIR variable is setup in module zynqHelper.py
#--     NOTE: Depending on the script generating your data, you may want to change
#--           the following user settings:
#--                SYS_CLK_PRD
#--                DSUM_SAMPLE_NCLK
#----------------------------------------------------------------------------------
import os, sys
from pathlib import Path
import time, datetime
import pandas as pd
import h5py
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.widgets as widgets
import matplotlib.patches as patches

from enum import IntEnum

# to add buttons to menu bar
from matplotlib.backend_tools import ToolBase, ToolToggleBase
#from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import warnings
with warnings.catch_warnings():
    warnings.simplefilter("ignore")
    plt.rcParams['toolbar'] = 'toolmanager'

# to save data
from itertools import zip_longest

# importing all files from tkinter
import tkinter
#from tkinter import *
#from tkinter import ttk

# import only asksaveasfilename from filedialog
from tkinter import Tk
from tkinter.filedialog import asksaveasfilename

import matplotlib # for get_backend()
import matplotlib.backends as backends

import threading

# custom modules
from modules.fgColors import fgColors
from modules.zynqEnvHelper import PROJECT_PATH, HOST_APPS_PATH, USER_DATA_DIR, HDF5_DATA_DIR
from modules.h5Reader import *

try:
    scriptName = os.path.basename(__file__)
    scriptAbsPath = os.path.dirname(os.path.abspath(__file__))
except NameError:
    scriptName = "fileNameNotFound.py"
    scriptAbsPath = os.path.abspath("./")


# -----------------------------------------------
# --- Global settings
# -----------------------------------------------
parsedH5 = []

def dbgPrint(message, enabled=False):
    if enabled: print(message)

# default path to open the menu to save data as a CSV
saveCsvInitialDir = os.path.join(USER_DATA_DIR, os.path.splitext(scriptName)[0])

# -----------------------------------------------
# --- User settings - SYS_CLK_PRD
# -----------------------------------------------
# NOTE: Since the acquisition is triggered from another
#       script, this apps is not aware of the system clock period.
#       Using default value of 10 ns (100 MHz)
# system clock frequency of the Controller/PDCs
if os.environ.get("SYS_CLK_PRD") is not None:
    # using environment variable
    SYS_CLK_PRD = os.environ['SYS_CLK_PRD']
else:
    # default setting
    SYS_CLK_PRD=10.0e-9


# -----------------------------------------------
# --- User settings - DSUM_SAMPLE_NCLK
# -----------------------------------------------
# NOTE: This app is not designed to automatically get the sampling period
#       of the digital sum. It is hardcoded here for now.
#       DSUM_SAMPLE_NCLK =  1 -> one digital clock sample each SYS_CLK_PRD
#       DSUM_SAMPLE_NCLK = 10 -> one digital clock sample each 10 SYS_CLK_PRD
# number of clock cycles for each digital sum sample
if os.environ.get("DSUM_SAMPLE_NCLK") is not None:
    DSUM_SAMPLE_NCLK = os.environ['DSUM_SAMPLE_NCLK']
else:
    # default setting
    DSUM_SAMPLE_NCLK=1


# -----------------------------------------------
# --- User settings - HDF5_DATA_DIR
# -----------------------------------------------
# see line:
# from modules.zynqHelper import HDF5_DATA_DIR # to specify the HDF5 input path

# -----------------------------------------------
# --- Global vars
# -----------------------------------------------
# default state of autofit
DEFAULT_AUTOFIT = True

# default state of integrate
DEFAULT_INTEGRATE = True

# default state of TIMEX
DEFAULT_TIMEX = True

# default state of LOGY
DEFAULT_LOGY = False

# number of supported PDCs
N_PDC_MAX = 8

# database of the data
dp = None

# menu to save as data
root = Tk()
root.withdraw()  # Removes TK root window
# Prompt window to front
root.overrideredirect(True)
root.geometry('0x0+0+0')
root.lift()
root.focus_set()
root.focus_force()
root.attributes("-topmost", True)

# path to icons for custom buttons
iconPath = os.path.join(scriptAbsPath, 'icons')

# -----------------------------------------------
# --- Class for the data
# -----------------------------------------------
class dsumPlotter:
    def __init__(self, figName, nPdcMax, autofit=True, integrate=True, timex=True, logy=False):
        """
        create empty object with no data, but figure properly formatted
        """
        self.figName = figName
        self.nPdcMax = nPdcMax
        self.autofit = autofit
        self.integrate = integrate
        self.timex = timex
        self.logy = logy
        self.label = "DSUM"
        self.clearData()
        self.initAxs()
        self.initLine()
        self.updateAllData()

    def clearData(self):
        """
        reset all data to empty and ready to start
        """
        self.nEvents = [0]*self.nPdcMax
        self.DSUM_time = [0]*self.nPdcMax
        self.DSUM_data = [0]*self.nPdcMax
        self.DSUM_INTEG_time = [None]*self.nPdcMax
        self.DSUM_INTEG_data = [None]*self.nPdcMax
        self.DSUM_PLOT_time = [0]*self.nPdcMax
        self.DSUM_PLOT_data = [0]*self.nPdcMax
        self.T0 = -1
        self.TNOW = -1

    def initAxs(self):
        """
        create properly formatted axes
        """
        self.axs = [None]*self.nPdcMax
        plt.close('all')
        plt.ion()

        # settings of the figure (number of subplots)
        if self.nPdcMax == 4:
            # single 2x2 head
            NROWS = 2; NCOLS = 2
        elif self.nPdcMax == 8:
            # two 2x2 heads
            NROWS = 2; NCOLS = 4
        elif self.nPdcMax == 32:
            # half of 8x8 head
            NROWS = 2; NCOLS = 4
        else:
            # TBD find a new algorithm
            NROWS = 1; NCOLS = self.nPdcMax

        self.fig, self.axes = plt.subplots(nrows=NROWS,ncols=NCOLS,
                                           figsize=(16, 9), constrained_layout=True,
                                           num=self.figName)
        self.fig.get_layout_engine().set(w_pad=0.1, h_pad=0.1, hspace=0.05, wspace=0.05)
        for iPdc in range(self.nPdcMax):
            self.axs[iPdc] = self._getPdcAx(iPdc=iPdc, axs=self.axes)

    def _getPdcAx(self, iPdc, axs=None):
        if np.shape(axs) == ():
            axs = self.axs
        if np.shape(axs) == (2,2):
            # 1 head of 2x2
            if iPdc == 0:
                return axs[0, 0]
            elif iPdc == 1:
                return axs[1, 0]
            elif iPdc == 2:
                return axs[1, 1]
            elif iPdc == 3:
                return axs[0, 1]

        elif np.shape(axs) == (2,4):
            # 2 heads of 2x2
            if iPdc == 0:
                return axs[0, 0]
            elif iPdc == 1:
                return axs[1, 0]
            elif iPdc == 2:
                return axs[1, 1]
            elif iPdc == 3:
                return axs[0, 1]
            elif iPdc == 4:
                return axs[0, 2]
            elif iPdc == 5:
                return axs[1, 2]
            elif iPdc == 6:
                return axs[1, 3]
            elif iPdc == 7:
                return axs[0, 3]

        elif np.shape(axs) == (4,8):
            raise Exception(f"8x8 head board is not yet supported")

        # should not reach here
        raise Exception(f"specified iPdc {iPdc} is out of range {self.nPdcMax}")



    def initLine(self):
        """
        create a line for each PDC
        """
        self.line = [None]*self.nPdcMax
        for iPdc in range(self.nPdcMax):
            self.line[iPdc] = (self.axs[iPdc].plot(self.DSUM_PLOT_time[iPdc],
                                                   self.DSUM_PLOT_data[iPdc], '-'))[0]
        #if self.nPdcMax == 8:
        #    sep = plt.Line2D([0.5,0.5],[0.02, 0.98], color="black")
        #    self.fig.add_artist(sep)


    def getAllPdcData(self, db):
        """
        read from a H5 data base object (see settings class)
        """
        # init empty data
        self.DSUM_time = [0]*self.nPdcMax
        self.DSUM_data = [0]*self.nPdcMax

        # open hdf5 file
        db.h5Open()

        # get content
        for iPdc in range(self.nPdcMax):
            [self.DSUM_time[iPdc], self.DSUM_data[iPdc]] = db.getPdcDsum(iPdc=iPdc)

        # close hdf5 file
        db.h5Close()

        # process the data only once the HDF5 file is closed
        for iPdc in range(self.nPdcMax):
            if (not np.shape(self.DSUM_data[iPdc]) == ()):
                # data is not empty
                if ((type(self.DSUM_INTEG_data[iPdc]) == np.ndarray) and
                    (type(self.DSUM_data[iPdc]) == np.ndarray)):
                    # DSUM_INTEG_data and DSUM_data are both set to a np.ndarray
                    lenInteg = len(self.DSUM_INTEG_data[iPdc])
                    lenNewData = len(self.DSUM_data[iPdc])
                    if lenNewData == lenInteg:
                        # same size data
                        self.DSUM_INTEG_data[iPdc] += self.DSUM_data[iPdc]
                    #elif lenNewData < lenInteg:
                    #    # new data has less data than integration
                    #    self.DSUM_INTEG_data[iPdc][0:lenNewData-1] += self.DSUM_data[iPdc][0:lenNewData-1]
                    #elif lenNewData > lenInteg:
                    #    # new data has more data than integration
                    #    tmp = self.DSUM_INTEG_data[iPdc]
                    #    self.DSUM_INTEG_data[iPdc] = self.DSUM_data[iPdc]
                    #    self.DSUM_INTEG_data[iPdc][0:lenInteg-1] += tmp[0:lenInteg-1]
                else:
                    # init DSUM_INTEG_data to DSUM_data
                    self.DSUM_INTEG_data[iPdc] = self.DSUM_data[iPdc]
                # keep timestamp of first event
                if self.T0 == -1:
                    self.T0 = datetime.datetime.now()
                # keep timestamp of the last event
                self.TNOW = datetime.datetime.now()
                # number of events changed
                self.nEvents[iPdc] += 1

    def compareXdata(self, iPdc):
        shapeNew = np.shape(self.DSUM_time[iPdc])
        if (not shapeNew == ()):
            # time data is not empty
            shapePlot = np.shape(self.DSUM_PLOT_time[iPdc])
            if shapePlot == ():
                # init plot data
                return True
            elif shapeNew != shapePlot:
                # different shapes, do not update data
                return False
            elif ((shapeNew == shapePlot) and
                  ((self.DSUM_PLOT_time[iPdc] != self.DSUM_time[iPdc]).any())):
                # same shape, different data, must update data
                return True
            else:
                # do not update data
                return False

    def updateAllData(self):
        """
        update plots
        """
        # select data to display (integration or not)
        if self.integrate:
            self.DSUM_PLOT_data = self.DSUM_INTEG_data
        else:
            self.DSUM_PLOT_data = self.DSUM_data

        for iPdc in range(self.nPdcMax):
            if self.timex == False:
                # use data sample as X axis
                if hasattr(self.DSUM_data[iPdc], "__len__"):
                    self.DSUM_PLOT_time[iPdc] = np.arange(0, len(self.DSUM_data[iPdc]))

            # check if x axis must me updated
            setXlim=False
            if self.compareXdata(iPdc=iPdc):
                # save last time and update plot x only if values are differents
                if self.timex:
                    self.DSUM_PLOT_time[iPdc] = self.DSUM_time[iPdc]
                self.line[iPdc].set_xdata(self.DSUM_PLOT_time[iPdc])
                setXlim = True

            # update y axis
            setYlim=False
            if (not np.shape(self.DSUM_PLOT_data[iPdc]) == ()):
                self.line[iPdc].set_ydata(self.DSUM_PLOT_data[iPdc])
                setYlim=True

            # set the y axis scale
            yscale = self.axs[iPdc].get_yscale()
            if self.logy and yscale != 'symlog':
                #self.axs[iPdc].set_yscale('log')
                self.axs[iPdc].set_yscale('symlog')
                setYlim=True
            elif self.logy == False and yscale != 'linear':
                self.axs[iPdc].set_yscale('linear')
                setYlim=True

            # adjust axes limits
            if self.autofit:
                set_lim(ax=self.axs[iPdc],
                        time=self.DSUM_PLOT_time[iPdc],
                        data=self.DSUM_PLOT_data[iPdc],
                        setXlim=setXlim, setYlim=setYlim)


            # update legends with number of events
            eventLabel=""
            if self.integrate:
                eventLabel=f" - {self.nEvents[iPdc]} event"
                if self.nEvents[iPdc] > 1:
                    eventLabel+="s"
            self.line[iPdc].set_label(f"{self.label}{iPdc}{eventLabel}")
            self.axs[iPdc].legend()


        # once everything is done
        #self.fig.tight_layout(pad=1.2, h_pad=1.2, w_pad=1.2, rect=(0.02, 0.02, 0.98, 0.98))
        #self.fig.subplots_adjust(wspace=0.25)
        ##self.fig.subplots_adjust(left=0.05, bottom=0.05, right=0.95, top=0.95,
        ##                         wspace=0.25)

    def clearPlot(self):
        for iPdc in range(self.nPdcMax):
            self.line[iPdc].set_xdata([self.DSUM_PLOT_time[iPdc]])
            self.line[iPdc].set_ydata([self.DSUM_PLOT_data[iPdc]])


    def checkExit(self):
        """
        check figure by name if it still exists
        """
        if not plt.fignum_exists(self.figName):
            print("\nFigure closed: exit program")
            sys.exit()

    def pausePlot(self, pauseTime=0.001):
        """
        let user interact with a plot while waiting for new data
        """
        #plt.pause(pauseTime)  # steal the focus
        self.fig.canvas.draw_idle()
        self.fig.canvas.start_event_loop(pauseTime)

# -----------------------------------------------
# --- Classes to add buttons in toolbar
# -----------------------------------------------
class AutosizePlots(ToolBase):
    """Reset to original view replaced to custom function"""
    #default_keymap = 'h' # keyboard shortcut
    description = 'Autosize plots'
    image = 'home'

    def trigger(self, *args, **kwargs):
        # called each time the button is clicked
        global dp
        autosizeAll(dp=dp)

def autosizeAll(dp):
    if dp:
        for iPdc in range(dp.nPdcMax):
            x = dp.line[iPdc].get_xdata()
            y = dp.line[iPdc].get_ydata()
            ax = dp.axs[iPdc]
            set_lim(ax, x, y, setXlim=True, setYlim=True)


class EnableAutofitPlots(ToolToggleBase):
    """Toggle between integration and direct data"""
    default_keymap = 'z' # keyboard shortcut
    description = 'Auto Fit Plot Zoom'
    image = os.path.join(iconPath, 'AUTOFIT.ico')
    global DEFAULT_AUTOFIT
    default_toggled = DEFAULT_AUTOFIT

    def enable(self, *args):
        # called each time the button is enabled
        global dp
        if dp:
            dp.autofit = True
            dp.updateAllData()

    def disable(self, *args):
        # called each time the button is disabled
        global dp
        if dp:
            dp.autofit = False
            dp.updateAllData()


class Integrate(ToolToggleBase):
    """Toggle between integration and direct data"""
    default_keymap = 'i' # keyboard shortcut
    description = 'Integrate digital sum'
    image = os.path.join(iconPath, 'SIGMA.ico')
    global DEFAULT_INTEGRATE
    default_toggled = DEFAULT_INTEGRATE

    def enable(self, *args):
        # called each time the button is enabled
        global dp
        if dp:
            dp.integrate = True
            dp.updateAllData()

    def disable(self, *args):
        # called each time the button is disabled
        global dp
        if dp:
            dp.integrate = False
            dp.updateAllData()

class TimeX(ToolToggleBase):
    """Toggle between Time or data index on X axis"""
    #default_keymap = 't' # keyboard shortcut
    image = os.path.join(iconPath, 'TIMEX.ico')
    description = 'Toggle between Time or data index on X axis'
    global DEFAULT_TIMEX
    default_toggled = DEFAULT_TIMEX

    def enable(self, *args):
        # called each time the button is enabled
        global dp
        if dp:
            dp.timex = True
            dp.updateAllData()

    def disable(self, *args):
        # called each time the button is disabled
        global dp
        if dp:
            dp.timex = False
            dp.updateAllData()


class LogY(ToolToggleBase):
    """Toggle between liny and logy"""
    #default_keymap = 'l' # keyboard shortcut
    image = os.path.join(iconPath, 'LOGY.ico')
    description = 'Toggle between linear and logarithmic y scale'
    global DEFAULT_LOGY
    default_toggled = DEFAULT_LOGY

    def enable(self, *args):
        # called each time the button is enabled
        global dp
        if dp:
            dp.logy = True
            dp.updateAllData()

    def disable(self, *args):
        # called each time the button is disabled
        global dp
        if dp:
            dp.logy = False
            dp.updateAllData()


class ClearPlotData(ToolBase):
    """Reset integration of data"""
    #default_keymap = 'h' # keyboard shortcut
    description = 'Clear digital sum integration'
    image = os.path.join(iconPath, 'CLEAR.ico')

    def trigger(self, *args, **kwargs):
        # called each time the button is clicked
        global dp
        if dp:
            dp.clearData()
            dp.clearPlot()
            dp.updateAllData()
            autosizeAll(dp=dp)


#root = Tk()
class DownloadData(ToolBase):
    """Download digital data"""
    #default_keymap = 'd' # keyboard shortcut
    description = 'Download digital sum data'
    image = os.path.join(iconPath, 'DOWNLOAD.ico')

    def trigger(self, *args, **kwargs):
        # called each time the button is clicked
        global dp

        if dp:
            # init empty DataFrame
            df = pd.DataFrame()

            # for each PDC, check if plot data is available
            for iPdc in range(dp.nPdcMax):
                if dp.nEvents[iPdc] > 0:
                    df.insert(loc=len(df.columns), column=f"time{iPdc}", value=dp.DSUM_PLOT_time[iPdc])
                    df.insert(loc=len(df.columns), column=f"data{iPdc}", value=dp.DSUM_PLOT_data[iPdc])

            # if there are data to export
            if df.size > 0:
                # ask user where to save the data
                userFile = save()

                if not userFile == "":
                    df.to_csv(userFile, sep=';', index=False, float_format="%.3E")
            else:
                print(f"{fgColors.bYellow}WARNING: No data to save.{fgColors.endc}")


# function to call when user press
# the save button, a filedialog will
# open and ask to save file
def save():
    global dp
    if not dp:
        print(f"{fgColors.red}ERROR: database not ready. No data exported.{fgColors.endc}")
        return ""

    # format default file name based on acquisition type
    dateStr=datetime.datetime.now().strftime("%Y%m%d_%Hh%Mm%S")
    integStr=""
    if dp.integrate:
        maxEvents = max(dp.nEvents)
        integStr+=f"_INTEG_{maxEvents}event"
        if maxEvents > 1:
            integStr+="s"
        if dp.T0 != -1 and dp.TNOW != -1:
            integTime=dp.TNOW-dp.T0
            #integStr+=f"_{integTime.strftime("%d_%Hh%Mm%S")}"
            if integTime.days > 0:
                integStr+=f"_{integTime.days:02d}d"
            integStr+=f"_{time.strftime('%Hh%Mm%S', time.gmtime(integTime.seconds))}"
            # TBD NOTE validate code NOTE TBD
    # initial directory
    global saveCsvInitialDir
    initialdir = Path(saveCsvInitialDir)
    initialdir.mkdir(parents=True, exist_ok=True)

    # ask user to select a file to save data
    files = [('CSV Files', '*.csv*')]
    userFile = str(asksaveasfilename(filetypes=files, defaultextension=files,
                                     initialdir=initialdir,
                                     confirmoverwrite=True,
                                     initialfile=f"{dateStr}_DSUM{integStr}.csv"))
    if not userFile or userFile == '()':
        # user clicked cancel or clicked ok on empty file name
        print(f"{fgColors.red}ERROR: No file specified. No data exported.{fgColors.endc}")
        return ""
    userPath = os.path.dirname(str(userFile))
    if not os.path.isdir(userPath):
        print(f"{fgColors.red}ERROR: Specified path '{userPath}' is not valid. No data exported.{fgColors.endc}")
        return ""
    print(f"{fgColors.bBlue}Exporting data to {userFile}{fgColors.endc}")
    return userFile


# radio button
# https://matplotlib.org/stable/api/backend_tools_api.html
# class matplotlib.backend_tools.ToolToggleBase(*args, **kwargs)[source]
#radio_group = None
    #Attribute to group 'radio' like tools (mutually exclusive).
    #str that identifies the group or None if not belonging to a group.


# -----------------------------------------------
# --- General purpose functions
# -----------------------------------------------
def set_lim(ax, time, data, setXlim=True, setYlim=True):
    if "log" in ax.get_yscale():
        log = True
    else:
        log = False

    if setXlim:
        xMin = min(time)
        xMax = max(time)
        if xMin == 0 and xMax == 0:
            # default empty limits
            ax.set_xlim(-0.055, 0.055)
        elif xMin != xMax:
            # auto limits
            ax.set_xlim(min(time), max(time))
    if setYlim:
        try:
            if log:
                yMin = min(data)/2.0
                yMax = max(data)*2.0
            else:
                yMin = 0.8*min(data)
                yMax = 1.2*max(data)
        except:
            yMin = 0
            yMax = 0
        if (yMin == 0 and yMax == 0):
            # default empty limits
            if log:
                ax.set_ylim(0.1, 1)
            else:
                ax.set_ylim(-0.055, 0.055)
        elif (yMin != yMax):
            # auto limits
            ax.set_ylim(yMin, yMax)


def customizeMenu(fig):
    # Add the custom tools to the toolmanager
    fig.canvas.manager.toolmanager.add_tool('home_custom', AutosizePlots)
    fig.canvas.manager.toolmanager.add_tool('TIMEX', TimeX)
    fig.canvas.manager.toolmanager.add_tool('LOGY', LogY)
    fig.canvas.manager.toolmanager.add_tool('enableautofit', EnableAutofitPlots)
    fig.canvas.manager.toolmanager.add_tool('integrate', Integrate)
    fig.canvas.manager.toolmanager.add_tool('download', DownloadData)
    fig.canvas.manager.toolmanager.add_tool('clear', ClearPlotData)

    # Remove the 'home' button
    fig.canvas.manager.toolmanager.remove_tool('home')

    # To add the new 'home' tool to the toolbar at specific location inside
    # the navigation group
    fig.canvas.manager.toolbar.add_tool('home_custom', 'navigation', 0)
    fig.canvas.manager.toolbar.add_tool('TIMEX', 'display', 0)
    fig.canvas.manager.toolbar.add_tool('LOGY', 'display', 1)
    fig.canvas.manager.toolbar.add_tool('enableautofit', 'display', 2)
    fig.canvas.manager.toolbar.add_tool('integrate', 'display', 3)
    fig.canvas.manager.toolbar.add_tool('download', 'save', 0)
    fig.canvas.manager.toolbar.add_tool('clear', 'clear', 0)

# -----------------------------------------------
# --- Settings for the analysis
# -----------------------------------------------

def parse_files(dp: dsumPlotter):
    db = h5Reader(deleteAfter=True, # Needs to be true to avoid reading same files in a loop
                  hfAbsPath=HDF5_DATA_DIR,
                  sysClkPrd=SYS_CLK_PRD,
                  dsumPrd=DSUM_SAMPLE_NCLK,
                  hfFile="")
    while True:
        db.waitForNewFile(timeout_sec=0.5)
        # Update data if new file has been parsed
        if db.newFileReady():
            dp.getAllPdcData(db)
            dp.updateAllData()
            db.h5Close()

        dp.checkExit()


# Switch to disable running parsing in a separate thread from gui, like previous versions
run_thread = True
try:
    dp = dsumPlotter(figName="DSUM PLOTTER",
                     nPdcMax=N_PDC_MAX,
                     autofit=DEFAULT_AUTOFIT,
                     integrate=DEFAULT_INTEGRATE)
    customizeMenu(dp.fig)

    # Start acquisition thread
    if run_thread:
        thread_parse = threading.Thread(target=parse_files, args=[dp])
        thread_parse.start()

    while 1:
        # loop until user Ctrl+C
        if not run_thread:
            db = h5Reader(deleteAfter=True,
                          #hfRelPath="HDF5",
                          hfAbsPath=HDF5_DATA_DIR,
                          sysClkPrd=SYS_CLK_PRD,
                          dsumPrd=DSUM_SAMPLE_NCLK,
                          hfFile="")


            # nothing new to plot
            if not db.newFileReady():
                #checkExit(figure_name)
                #plotPause(fig, 0.01)
                dp.checkExit()
                dp.pausePlot(pauseTime=0.01)
                continue

            # -----------------------------------------------
            # --- Open HDF5 file to get Controller Data
            # -----------------------------------------------
            # get all PDC data for a given event in db
            dp.getAllPdcData(db=db)

            # -----------------------------------------------
            # --- Plot content
            # -----------------------------------------------
            dp.updateAllData()

        # check if figure still exist
        dp.checkExit()

        # wait between each graph
        dp.pausePlot(pauseTime=0.001)


except KeyboardInterrupt:
    print("\nKeyboard Interrupt: exit program")
    sys.exit()

