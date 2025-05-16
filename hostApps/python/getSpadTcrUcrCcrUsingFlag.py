#----------------------------------------------------------------------------------
#-- Company: GRAMS
#-- Designer: Tommy Rossignol
#--
#-- Create Date: 2024-08-02
#-- Description:
#--     Using python ssh libraries to send remote commands to the ZCU102
#--     This script starts by enabling all the 2D CMOS SPADs.
#--     It then gets the total count rate (TCR) and get an average per SPAD.
#--     After that, it enables each SPAD, one at the time to get its TCR.
#--     The results are displayed per SPAD index one the first subplot.
#--     The second subplot shows the TCR sorted in ascending order.
#--
#-- Dependencies:
#-- Revision:
#-- Revision 1.0 - File Created
#-- Revision 2.0 - Updated for sharing on pdcv2-public
#-- Additional Comments:
#--     For more details about the Zero Photon Probability method (ZPP) :
#--     Frederic Vachon et al. 2021 Meas. Sci. Technol. 32 025105
#--     Measuring count rates free from correlated noise in digital silicon photomultipliers
#--     NOTE: it is recommended to run a single PDC at the time for better results.
#--           Since the ZPP optimal period to use is based on the SPAD TCR,
#--           measuring 1 SPAD on each PDC can lead to an error on the optimal period to use.
#--     NOTE: When NUL (number of empty bins) is set to 0, ZPP algorithm can't get UCR and CCR.
#--
#----------------------------------------------------------------------------------
import sys, os
import numpy as np
import random
import time
import datetime
from itertools import chain
import statistics
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import warnings
import threading

# custom modules
from modules.fgColors import fgColors
from modules.zynqEnvHelper import PROJECT_PATH, HOST_APPS_PATH, USER_DATA_DIR, HDF5_DATA_DIR
import modules.sshClientHelper as sshClientHelper
import modules.systemHelper as systemHelper
from modules.zynqCtlPdcRoutines import initCtlPdcFromClient, packetBank
from modules.zynqDataTransfer import zynqDataTransfer
from modules.systemHelper import sectionPrint
from modules.pdcHelper import *
#from modules.zynqHelper import *
from modules.h5Reader import *
from modules.h5Reader import PDC_ZPP_ITEM as pzi

try:
    scriptName = os.path.basename(__file__)
except NameError:
    scriptName = "fileNameNotFound.py"

# get the total execution time of the test
test_start_time = time.time()

# -----------------------------------------------
# --- Global vars
# -----------------------------------------------
# database of the data
zp = None

# time to wait for each setting
measTime = 0.1  # second

nZppCycles = 5000 # number of zpp cycles to get data

# -----------------------------------------------
# --- open a connection with the ZCU102 board
# -----------------------------------------------
# open a client based on its name in the ssh config file
sectionPrint("open a connection with the ZCU102 board")
client = sshClientHelper.sshClientFromCfg(hostCfgName="zcudev")

# -----------------------------------------------
# --- prepare Zynq platform
# -----------------------------------------------
sectionPrint("prepare Zynq platform")
zynq = zynqDataTransfer(sshClientZynq=client)
zynq.init()

# -----------------------------------------------
# --- prepare controller for acquisition
# -----------------------------------------------
# NOTE: a single PDC at the time gives better results
# NOTE: select here the PDC to use:
#       pdcEn=0x1 -> PDC0
#       pdcEn=0x2 -> PDC1
#       pdcEn=0x4 -> PDC2
#       pdcEn=0x8 -> PDC3
#       pdcEn=0xF -> PDC0, PDC1, PDC2, PDC3
icp = initCtlPdcFromClient(client=client, sysClkPrd=10e-9, pdcEn=0x1)

# -----------------------------------------------
# --- set system clock period
# -----------------------------------------------
icp.setSysClkPrd()

# -----------------------------------------------
# --- reset of the controller
# -----------------------------------------------
icp.resetCtl()

# -----------------------------------------------
# --- configure controller packet
# -----------------------------------------------
# NOTE always set SCSA register first to store other configuration registers in HDF5
# configure CFG_STATUS_A
    # 0x8000 = PDC_CFG
    # 0x4000 = CTL_CFG
    # 0x2000 = PDC_STATUS
    # 0x1000 = PDC_STATUS_ALL
    # 0x0007 = ALL CTL_STATUS
SCSA = 0x0000
# configure CTL_DATA_A
SCDA = 0x0000
# configure PDC_DATA_A
    # 0x0100 = DSUM
    # 0x00F7 = ZPP
SPDA = 0x00F7
icp.setCtlPacket(bank=packetBank.BANKA, SCS=SCSA, SCD=SCDA, SPD=SPDA)

# -----------------------------------------------
# --- set delay of CFG_DATA pins
# -----------------------------------------------
icp.setDelay(signal="CFG_DATA", delay=300)

# -----------------------------------------------
# --- check for power good
# -----------------------------------------------
icp.checkPowerGood()

# -----------------------------------------------
# --- enable CFG_RTN_EN
# -----------------------------------------------
icp.setCfgRtnEn()

# -----------------------------------------------
# --- prepare PDC for configuration
# -----------------------------------------------
icp.preparePDC()


# --------------------------
# --- configure the PDCs ---
# --------------------------
sectionPrint("configure the PDCs")
PDC_SETTING = pdc_setting()
client.runPrint("ctlCmd -c MODE_CFG")  # set PDCs to configuration mode

# === PIXL REGISTER ===
print("\n=== PIXL REGISTER ===")
# active quenching of the front-end
ACTIVE_QC_EN = 0; # 0=disabled/passive, 1=enabled/active
# trigger using QC front-end (FE) or digital only (DGTL)
TRG_DGTL_FEN = 0; # 0=FE, 1=DGTL
# enable flag output of the pixel
FLAG_EN = 1; # 1=enabled, 0=disabled
# EDGE_LVLN and DIS_MEM on synchronizer
EDGE_LVLN = 0
DIS_MEM = 0
PIXL = ((DIS_MEM<<13) + (EDGE_LVLN<<12) + (FLAG_EN<<8) + (TRG_DGTL_FEN<<4) + (ACTIVE_QC_EN<<1))
client.runPrint(f"pdcCfg -a PIXL -r {PIXL} -g")  # configure pixel register
PDC_SETTING.PIXL = PIXL

# === TIME REGISTER ===
print("\n=== TIME REGISTER ===")
# NOTE: use a shorter hold-off to get a better estimate of the afterpulse (CCR)
HOLD_TIME = 15.0
RECH_TIME = 4.0
FLAG_TIME = 2.0
client.runPrint(f"pdcTime --hold {HOLD_TIME} --rech {RECH_TIME} --flag {FLAG_TIME} -g")
PDC_SETTING.TIME = client.runReturnSplitInt('pdcTime -g')


# === ANLG REGISTER ===
print("\n=== ANLG REGISTER ===")
ANLG = 0x0000; # disabled
#ANLG = 0x001F; # full amplitude (~30 µA)
client.runPrint(f"pdcCfg -a ANLG -r {ANLG} -g")  # set analog monitor
PDC_SETTING.ANLG = ANLG

# === XXXX REGISTER ===
# skipping registers STHH to DTXC

# === OUTD REGISTER ===
print("\n=== OUTD REGISTER ===")
#DATA_FUNC = OUT_MUX.FLAG
#DATA_FUNC = OUT_MUX.TRG
#DATA_FUNC = OUT_MUX.PIX_QC
DATA_FUNC = OUT_MUX.VSS
#DATA_FUNC = OUT_MUX.VDD
OUTD = (DATA_FUNC & 0x1F) + ((DATA_FUNC & 0x1F)<<6)
client.runPrint(f"pdcCfg -a OUTD -r 0x{OUTD:04x} -g")
PDC_SETTING.OUTD = OUTD

# === OUTF REGISTER ===
print("\n=== OUTF REGISTER ===")
FLAG_FUNC = OUT_MUX.FLAG
#FLAG_FUNC = OUT_MUX.TRG
#FLAG_FUNC = OUT_MUX.VSS
#FLAG_FUNC = OUT_MUX.VDD
OUTF = (FLAG_FUNC & 0x1F) + ((FLAG_FUNC & 0x1F)<<6)
client.runPrint(f"pdcCfg -a OUTF -r 0x{OUTF:04x} -g")
PDC_SETTING.OUTF = OUTF

# === TRGC REGISTER ===
print("\n=== TRGC REGISTER ===")
TRGC = 0x0000
client.runPrint(f"pdcCfg -a TRGC -r {TRGC} -g")  # disable trigger command
PDC_SETTING.TRGC = TRGC

# === DISABLE ALL THE PIXELS ===
print("\n=== DISABLE ALL THE PIXELS ===")
    # NOTE pdcPix returns the PDC to acquisition mode NOTE
client.runPrint("pdcPix --dis")

# === VALIDATE CONFIGURATIONS ===
print("\n=== VALIDATE CONFIGURATIONS ===")
icp.validPdcCfg()

# === OUTC REGISTER ===
print("\n=== OUTC REGISTER ===")
    # disable configuration output last once configuration was validated
#FLAG_CFG_FUNC = OUT_MUX.CLK_CS     # default function
FLAG_CFG_FUNC = OUT_MUX.VSS        # disabled
#DATA_CFG_FUNC = OUT_MUX.CFG_VALID  # default function
DATA_CFG_FUNC = OUT_MUX.VSS        # disabled
OUTC = (DATA_CFG_FUNC & 0x1F) + ((FLAG_CFG_FUNC & 0x1F)<<6)
client.runPrint(f"pdcCfg -a OUTC -r 0x{OUTC:04x} -g")
PDC_SETTING.OUTC = OUTC

# print the settings of all the PDCs
print("\n=== PDC SETTINGS ===")
PDC_SETTING.print()

# ---------------------------------------
# --- configure Controller ZPP module ---
# ---------------------------------------
sectionPrint("configure Controller ZPP module")
class ZppModuleSetMethod(IntEnum):
    app=0,
    registers=1

def setZppModule(lclient, sysClkPrdSec, onTimeSec, offTimeSec):
    CLK_PRD=sysClkPrdSec

    method = ZppModuleSetMethod.app
    if method == ZppModuleSetMethod.app:
        lclient.runPrint(f"set-ctl-zpp-prd {onTimeSec} {offTimeSec}")
    elif method == ZppModuleSetMethod.registers:
        print("  Configure ZPP Timer High Period")
        ZPP_HIGH_PRD=onTimeSec
        ZPP_HIGH_REG=int(ZPP_HIGH_PRD/CLK_PRD)
        lclient.runPrint(f"ctlCfg -a ZPH0 -r 0x{ZPP_HIGH_REG&0xFFFF:04x} -g")
        lclient.runPrint(f"ctlCfg -a ZPH1 -r 0x{(ZPP_HIGH_REG>>16)&0xFFFF:04x} -g")

        print("  Configure ZPP Timer Low Period")
        ZPP_LOW_PRD=offTimeSec
        ZPP_LOW_REG=int(ZPP_LOW_PRD/CLK_PRD)
        lclient.runPrint(f"ctlCfg -a ZPL0 -r 0x{ZPP_LOW_REG&0xFFFF:04x} -g")
        lclient.runPrint(f"ctlCfg -a ZPL1 -r 0x{(ZPP_LOW_REG>>16)&0xFFFF|0x8000:04x} -g")  # |0x8000 to enable ZPP

# set ZPP module to 1 sec to get average countrate per PDC to start
setZppModule(client,
             sysClkPrdSec=icp.sysClkPrd,
             onTimeSec=measTime,
             offTimeSec=icp.sysClkPrd)

# ---------------------------------------
# --- Notify user of manual steps
# ---------------------------------------
try:
    print(f"{fgColors.bYellow}Apply HV here{fgColors.endc}")
    input("Press [enter] key to continue")
except KeyboardInterrupt:
    print("\nKeyboard Interrupt: exit program")
    sys.exit()

# ------------------------------------------------
# --- Prepare Controller FSM for the acquisition
# ------------------------------------------------
client.run(f"ctlCfg -a FSMM -r 0x0101 -g"); # triggered by a COMMAND

# --------------------------------------------------
# --- Class to generate the display of the results
# --------------------------------------------------
class zppPlotter:
    def __init__(self, figName, nPdcMax, nSpad, doSavePlot=False, dataPath="default"):
        """
        create an empty object with no data, but with figure properly formatted
        """
        self.figName = figName
        self.nPdcMax = nPdcMax
        self.nSpad = nSpad

        # if PDc is enabled and gives valid data
        self.pdcValid = [False]*self.nPdcMax
        self.pdcValidPrev = [False]*self.nPdcMax # detect when pdcValid change to update legend

        # index of tested pixel
        self.current_pixel_index = -1
        self.done_test_all_pixels = False
        self.run = True

        # data
        self.zpp = [[PDC_ZPP()]*self.nSpad for iPdc in range(self.nPdcMax)]

        # data to extract from ZPP module
        self.zppList = [pzi.AVG, pzi.BIN, pzi.NUL, pzi.PRD, pzi.TOT]

        # plot constants
        self.axTCR = 0
        self.axUCR = 1
        self.axCCR = 2

        self.doSavePlot=doSavePlot
        self.plotIdx = 0
        self.fig = None
        self.dateStrPlot = datetime.datetime.now().strftime("%Y%m%d_%Hh%Mm%S")

        # path to save CSV data
        if dataPath != "default" and os.path.isDir(dataPath):
            self.dataPath = dataPath
        else:
            # default path
            #self.dataPath = os.path.join('.', 'TCR')
            self.dataPath = os.path.join(USER_DATA_DIR, 'TCR')
        # add script name to path
        self.dataPath = Path(os.path.join(self.dataPath, os.path.splitext(scriptName)[0]))
        # path to save plot
        self.plotPath = Path(os.path.join(self.dataPath, 'FIG', self.dateStrPlot))

        # init plot
        self.initPlot()

    def getTCR(self, iPdc, getSpad=False):
        """
        get total count rate from zpp object
        getSpad: when set to True, returns both spad index and TCR.
        """
        TCR = [spad.TCR for spad in self.zpp[iPdc]]
        if getSpad:
            SPAD = range(len(TCR))
            return SPAD, TCR
        else:
            return TCR

    def getUCR(self, iPdc, getSpad=False):
        """
        get uncorrelated count rate from zpp object
        This is usually considered as dark count rate (DCR).
        getSpad: when set to True, returns both spad index and UCR.
        """
        UCR = [spad.UCR for spad in self.zpp[iPdc]]
        if getSpad:
            SPAD = range(len(UCR))
            return SPAD, UCR
        else:
            return UCR

    def getCCR(self, iPdc, getSpad=False):
        """
        get uncorrelated count rate from zpp object
        If only one SPAD is enabled, this is afterpulse probability (AP).
        getSpad: when set to True, returns both spad index and CCR.
        """
        CCR = [100*spad.CCR for spad in self.zpp[iPdc]]
        if getSpad:
            SPAD = range(len(CCR))
            return SPAD, CCR
        else:
            return CCR

    def getZPPparam(self, iPdc, item, getSpad=False):
        """
        get param from zpp object
        getSpad: when set to True, returns both spad index and CCR.
        """
        PARAM = [getattr(spad, item, -2) for spad in self.zpp[iPdc]]
        if getSpad:
            SPAD = range(len(PARAM))
            return SPAD, PARAM
        else:
            return PARAM

    def flattenAllPDcs(self, func):
        """
        get data from 'func' function for all PDCs
        """
        data = []
        for iPdc in range(self.nPdcMax):
            data += func(iPdc, getSpad=False)
        return data

    def initPlot(self):
        """
        create properly formatted plot
        """
        plt.close('all')
        plt.ion()
        self.fig, self.axes = plt.subplots(nrows=1, ncols=3,
                                           figsize=(16,9), constrained_layout=True,
                                           num=self.figName)
        self.fig.get_layout_engine().set(w_pad=0.1, h_pad=0.1, hspace=0.05, wspace=0.05)

        # colors for plots
        self.colors = plt.rcParams['axes.prop_cycle'].by_key()['color']

        # show empty data for all axes
        self.lineTCR = [None]*self.nPdcMax
        self.lineUCR = [None]*self.nPdcMax
        self.lineCCR = [None]*self.nPdcMax
        self.lineTCRlabel = [""]*self.nPdcMax
        self.lineUCRlabel = [""]*self.nPdcMax
        self.lineCCRlabel = [""]*self.nPdcMax
        for iPdc in range(self.nPdcMax):
            self.lineTCRlabel[iPdc] = f"PDC{iPdc} TCR"
            self.lineTCR[iPdc] = self.axes.flat[self.axTCR].plot(*self.getTCR(iPdc, getSpad=True), 'o',
                                                                 markerfacecolor='none',
                                                                 markeredgecolor=self.colors[iPdc],
                                                                 markeredgewidth=1.5,
                                                                 label=self.lineTCRlabel[iPdc])[0]
            self.lineUCRlabel[iPdc] = f"PDC{iPdc} UCR"
            self.lineUCR[iPdc] = self.axes.flat[self.axUCR].plot(*self.getUCR(iPdc, getSpad=True), 'o',
                                                                 markerfacecolor='none',
                                                                 markeredgecolor=self.colors[iPdc],
                                                                 markeredgewidth=1.5,
                                                                 label=self.lineUCRlabel[iPdc])[0]
            self.lineCCRlabel[iPdc] = f"PDC{iPdc} CCR"
            self.lineCCR[iPdc] = self.axes.flat[self.axCCR].plot(*self.getCCR(iPdc, getSpad=True), 'o',
                                                                 markerfacecolor='none',
                                                                 markeredgecolor=self.colors[iPdc],
                                                                 markeredgewidth=1.5,
                                                                 label=self.lineCCRlabel[iPdc])[0]

        # set titles
        self.axes.flat[self.axTCR].title.set_text("TCR as a function of the SPAD index")
        self.axes.flat[self.axUCR].title.set_text("UCR as a function of the SPAD index")
        self.axes.flat[self.axCCR].title.set_text("CCR as a function of the SPAD index")

        # show legends
        self.updateLegend(updatePrev=False)

        # log y axis
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.axes.flat[self.axTCR].set_yscale('log')
            self.axes.flat[self.axUCR].set_yscale('log')
            #self.axes.flat[self.axCCR].set_yscale('log')

        # scatter ticks
        for ax in self.axes:
            ax.set_xticks(np.arange(0, 65, 8))
            ax.xaxis.set_minor_locator(mticker.MultipleLocator(1))
            ax.tick_params(which="both", direction="in", top=True, right=True)
            ax.grid(visible=True, which="both", alpha=0.2)

    def updateLabel(self, iPdc):
        """
        remove a label when pdc is not valid so it is not shown in the legend
        """
        if not self.pdcValid[iPdc] != self.pdcValidPrev[iPdc]:
            if self.pdcValid[iPdc]:
                # PDC is valid, add a label
                self.lineTCR[iPdc].set_label(s=self.lineTCRlabel[iPdc])
                self.lineUCR[iPdc].set_label(s=self.lineUCRlabel[iPdc])
                self.lineCCR[iPdc].set_label(s=self.lineCCRlabel[iPdc])
            else:
                # pdc is not valid, remove the label
                self.lineTCR[iPdc].set_label(s='')
                self.lineUCR[iPdc].set_label(s='')
                self.lineCCR[iPdc].set_label(s='')

    def updateLegend(self, updatePrev=True):
        """
        show/update legends with proper parameters
        """
        if self.pdcValid != self.pdcValidPrev:
            # update legend only if a label has changed
            self.axes.flat[self.axTCR].legend()
            self.axes.flat[self.axUCR].legend()
            self.axes.flat[self.axCCR].legend()
            if updatePrev:
                self.pdcValidPrev = self.pdcValid

    def updatePlot(self):
        """
        set new data on the plot without stealing the focus
        """
        # set new data
        if self.current_pixel_index >= 0:
            for iPdc in range(self.nPdcMax):
                self.updateLabel(iPdc=iPdc)
                if self.pdcValid[iPdc]:
                    # PDC is valid, add it to the legend
                    self.lineTCR[iPdc].set_ydata(self.getTCR(iPdc, getSpad=False))
                    self.lineUCR[iPdc].set_ydata(self.getUCR(iPdc, getSpad=False))
                    self.lineCCR[iPdc].set_ydata(self.getCCR(iPdc, getSpad=False))

            # set TCR and UCR on the same scale
            TCR_ALL = self.flattenAllPDcs(self.getTCR)
            UCR_ALL = self.flattenAllPDcs(self.getUCR)
            CCR_ALL = self.flattenAllPDcs(self.getCCR)
            TCR_UCR_ALL = TCR_ALL + UCR_ALL # used to get the maximum value of UCR or TCR and use it to scale axis
            set_lim(self.axes.flat[self.axTCR], TCR_UCR_ALL)
            set_lim(self.axes.flat[self.axUCR], TCR_UCR_ALL)
            set_lim(self.axes.flat[self.axCCR], CCR_ALL)

        self.updateLegend()
        self.pausePlot(pauseTime=0.001)
        self.savePlot()
        self.checkExit()


    def newData(self, iSpad, zppList):
        """
        add new data to the class
        """
        for iPdc, zpp in enumerate(zppList):
            if zpp == None or zpp.isEmpty():
                # no valid data
                self.pdcValid[iPdc] = False
                continue
            self.pdcValid[iPdc] = True
            self.zpp[iPdc][iSpad] = zpp

    def saveData(self):
        """
        save data to a CSV file
        """
        dateStr=datetime.datetime.now().strftime("%Y%m%d_%Hh%Mm%S")
        pdcStr=""
        df = pd.DataFrame()
        for iPdc in range(0, self.nPdcMax):
            # per PDC data
            if self.pdcValid[iPdc]:
                # only if data is valid
                if pdcStr != "":
                    pdcStr+='_'
                pdcStr+=f"PDC{iPdc}"

                # extract data from class
                SPAD, TCR = self.getTCR(iPdc, getSpad=True)
                UCR = self.getUCR(iPdc, getSpad=False)
                CCR = self.getCCR(iPdc, getSpad=False)

                # add data to dataFrame
                dfNew = pd.DataFrame(data=SPAD, columns=[f"PDC{iPdc}_SPAD_idx"])
                df = pd.concat([df, dfNew], axis=1)
                dfNew = pd.DataFrame(data=TCR, columns=[f"PDC{iPdc}_TCR (cps)"])
                df = pd.concat([df, dfNew], axis=1)
                dfNew = pd.DataFrame(data=UCR, columns=[f"PDC{iPdc}_UCR (cps)"])
                df = pd.concat([df, dfNew], axis=1)
                dfNew = pd.DataFrame(data=CCR, columns=[f"PDC{iPdc}_CCR (%)"])
                df = pd.concat([df, dfNew], axis=1)

                # save all other parameters
                paramList = self.zppListName = [PDC_ZPP_NAME[item] for item in self.zppList]
                for param in paramList:
                    paramData = self.getZPPparam(iPdc, item=param, getSpad=False)
                    dfNew = pd.DataFrame(data=paramData, columns=[f"PDC{iPdc}_{param}"])
                    df = pd.concat([df, dfNew], axis=1)


        if df.size > 0:
            # if there are data to export
            filename = f"{dateStr}_ZPP_{pdcStr}_{int(measTime*1000):d}ms.csv"
            self.dataPath.mkdir(parents=True, exist_ok=True)
            datafile = os.path.join(self.dataPath, filename)
            print(f"{fgColors.green}Saving data to file {datafile}{fgColors.endc}")
            df.to_csv(datafile, sep=';', index=False, float_format="%.3E")

    def savePlot(self, iPdc=None):
        """
        save plot to a png file
        """
        if self.fig and self.doSavePlot:
            if iPdc == None or self.pdcValid[iPdc]:
                filename = f"ZPP_{self.plotIdx:06d}.png"
                self.plotPath.mkdir(parents=True, exist_ok=True)
                datafile = os.path.join(self.plotPath, filename)
                print(f"{fgColors.green}Saving plot to file {datafile}{fgColors.endc}")
                self.fig.savefig(datafile)
                self.plotIdx += 1

    def checkExit(self):
        """
        check figure by name if it still exists
        """
        if not plt.fignum_exists(self.figName):
            print("\nFigure closed...")
            #sys.exit()
            raise SystemExit


    def pausePlot(self, pauseTime=0.001):
        """
        let user interact with a plot while waiting for new data
        """
        #plt.pause(pauseTime)  # steal the focus
        self.fig.canvas.draw_idle()
        self.fig.canvas.start_event_loop(pauseTime)


def set_lim(ax, data):
    """
    set the limit on ax based on the values
    """
    dataValid = [val for val in data if val > 0]
    if dataValid:
        yMin = min(dataValid)/2.0
        yMax = max(dataValid)*2.0
        if (yMin != yMax):
            # auto limits
            ax.set_ylim(yMin, yMax)

# --------------------------------------------------
# --- Function to get ZPP of each PDC from h5 file
# --------------------------------------------------
def waitForH5File(timeOutSec=10):
    """
    function to wait for a new HDF5 file
    """
    t0 = datetime.datetime.now()
    while 1:
        db = h5Reader(deleteAfter=True,
                      hfAbsPath=zynq.h5Path,
                      hfFile="")

        if db.newFileReady():
            return db
        else:
            if datetime.datetime.now()-t0 > datetime.timedelta(seconds=timeOutSec):
                print(f"{fgColors.red}ERROR: Timeout while waiting for HDF5 data ({timeOutSec} seconds){fgColors.endc}")
                sys.exit()

def getZppOptimalPeriod(allPdcsZppData, nSpad):
    """
    get the average TCR for all PDCs to set the ZPP module period
    """
    avgTcrAll = 0
    nb = 0
    for zpp in allPdcsZppData:
        if zpp != None and zpp.TCR != -1:
            avgTcrAll += zpp.TCR
            nb += 1
    if nb > 0:
        avgTcrAll /= nb
    if avgTcrAll != 0:
        avgPrdAll = 1.0/avgTcrAll
    else:
        print(f"{fgColors.red}ERROR: no TCR detected on any SPAD. Is the HV ok ? Is the data link ok ?{fgColors.endc}")
        sys.exit()
    print(f"{fgColors.blue}Average total count rate over {nb} PDCs is {avgTcrAll:.1f} for {nSpad} SPADs.{fgColors.endc}")
    avgPrd1Spad = avgPrdAll*nSpad
    print(f"{fgColors.blue}Average period per SPAD is {avgPrd1Spad:.3E}{fgColors.endc}")
    zppPrd = avgPrd1Spad/5.0
    print(f"{fgColors.blue}Using ZPP period of {zppPrd:.3E}{fgColors.endc}")
    return zppPrd

def measCntRate(spadEnPattern, measTime, printTcrOnly=False):
    # 1- enable spads based on 64 bits pattern given and return to acquisition mode
    # 2- reset ZPP module
    # 3- wait for measTime for stats to build up
    # 4- send a Controller data packet with ZPP data
    # 5- wait to receive the file, fetch the ZPP data and close it
    if type(spadEnPattern) == int:
        # single pattern value, same setting for everyone
        N_SPAD = [bin(spadEnPattern).count("1")]*icp.nPdcMax
        client.runPrint(f"pdcSpad --pattern 0x{spadEnPattern:016x} --mode NONE")
    else:
        # array of patterns
        N_SPAD = [0]*icp.nPdcMax
        for iPdc in range(0, icp.nPdcMax):
            N_SPAD[iPdc] = bin(spadEnPattern[iPdc]).count("1")
            client.runPrint(f"pdcSpad --pattern 0x{spadEnPattern[iPdc]:016x} --spdc {iPdc} --mode NONE")

    client.runPrint("ctlCmd -c MODE_ACQ; " \
                    "ctlCmd -c RSTN_ZPP; " \
                    f"sleep {measTime:.06f}; " \
                    "ctlCmd -c PACK_TRG_A; ")

    # wait for the HDF5 result file
    db = waitForH5File()
    db.h5Open()

    # get ZPP results
    ZPP = [{}]*icp.nPdcMax
    for iPdc in range(0, icp.nPdcMax):
        ZPP[iPdc] = db.getPdcZPP(iPdc=iPdc, zppList=zp.zppList)
        if (ZPP[iPdc] != None) and (ZPP[iPdc].AVG != -1):
            # process here ZPP infos
            ZPP[iPdc].process()
            if printTcrOnly:
                print(f"PDC{iPdc}: TCR={ZPP[iPdc].TCR: <10.1f}")
            else:
                print(f"PDC{iPdc: <2}: NUL={ZPP[iPdc].NUL: <10.1f} BIN={ZPP[iPdc].BIN: <10.1f} PRD={ZPP[iPdc].PRD: <10.1f}")
                print(f"       TCR={ZPP[iPdc].TCR: <10.1f} UCR={ZPP[iPdc].UCR: <10.1f} CCR={100*ZPP[iPdc].CCR: <4.2f}%")

    db.h5Close()

    return ZPP

# ---------------------------------------
# --- data acquisition as a thread
# ---------------------------------------
def test_all_pixels(zp: zppPlotter, update=False):
    if type(zp) == type(None):
        print(f"{fgColors.red}zppPlotter object must be created first{fgColors.endc}")
        sys.exit()

    # 1 - enable all pixels and get count rate (for comparison only)
    spadEnPattern = 0xFFFFFFFFFFFFFFFF
    nSpad = bin(spadEnPattern).count("1")
    allPdcsZppData = measCntRate(spadEnPattern=spadEnPattern,
                                 measTime=measTime,
                                 printTcrOnly=True)

    for iSpad in range(0, icp.nSpad):
        sectionPrint(f"Testing SPAD index {iSpad}")
        spadEnPattern = 0x1<<iSpad

        # 2.1 - set ZPP module period to default measTime period to get TCR of the single SPAD
        setZppModule(client,
                 sysClkPrdSec=icp.sysClkPrd,
                 onTimeSec=measTime,
                 offTimeSec=icp.sysClkPrd)

        # 2.2 - from the ZPP module, get the measurements of the SPAD
        zppDataForOptimalPrd = measCntRate(spadEnPattern=spadEnPattern,
                                           measTime=measTime,
                                           printTcrOnly=True)


        # 2.3 - from the ZPP module measurements, get the optimal period to use, based on the TCR of the SPAD
        zppPrd = getZppOptimalPeriod(allPdcsZppData=zppDataForOptimalPrd, nSpad=1)

        # 2.4 - Set the optimal zpp period to use on the ZPP module
        setZppModule(client,
                 sysClkPrdSec=icp.sysClkPrd,
                 onTimeSec=zppPrd,
                 offTimeSec=icp.sysClkPrd)

        # 2.5 - from the ZPP module, get the measurements of the SPAD with the optimal period
        print(f"{fgColors.blue}Stats for SPAD {iSpad}:{fgColors.endc}")
        ZPP = measCntRate(spadEnPattern=spadEnPattern, measTime=zppPrd*nZppCycles)

        # put new data into zp data object
        zp.newData(iSpad, ZPP)

        # indicate which pixels are tested
        zp.current_pixel_index = iSpad

        # update plot (once per measure for all the PDCs)
        if update:
            zp.updatePlot()

    # indicate all tests are completed
    zp.done_test_all_pixels = True


# ---------------------------------------
# --- Script main execution
# ---------------------------------------
sectionPrint("Script main execution")
try:
    # ---------------------------------------
    # --- Object to hold the plots
    # ---------------------------------------
    # doSavePlot will save the plot at each measure.
    # It will then increase the test time.
    # Use it only to generate a .gif of the measures
    zp = zppPlotter(figName="ZPP PLOTTER",
                    nPdcMax=icp.nPdcMax,
                    nSpad=icp.nSpad,
                    doSavePlot=False)

    # ---------------------------------------
    # --- SPAD count rate logic
    # ---------------------------------------
    sectionPrint("SPAD count rate logic")
    run_thread = True
    if run_thread:
        # running as a thread
        thread_test = threading.Thread(target=test_all_pixels, args=[zp])
        thread_test.start()

        while not zp.done_test_all_pixels:
            zp.updatePlot()
        # update a last time
        zp.updatePlot()
    else:
        # not running as a thread
        test_all_pixels(zp=zp, update=True)

    # export data
    zp.saveData()

    # total execution time
    test_stop_time = time.time()
    print(f"{fgColors.bBlue}Test took {test_stop_time-test_start_time:.3f} seconds{fgColors.endc}")
    print(f"{fgColors.bBlue}Test completed, to exit, close figure{fgColors.endc}")
    plt.show(block=True)
    print("\nFigure closed... exit program")

except (KeyboardInterrupt, SystemExit) as ex:
    if "zp" in locals():
        zp.run = False
    if "thread_test" in locals():
        thread_test.join()
    if isinstance(ex, KeyboardInterrupt):
        print(f"\n{fgColors.yellow}Keyboard Interrupt: exit program{fgColors.endc}")
    else:
        print(f"\n{fgColors.yellow}Program interrupted: exit program{fgColors.endc}")

finally:
    if not 'test_stop_time' in locals():
        test_stop_time = time.time()
        print(f"{fgColors.bBlue}Test took {test_stop_time-test_start_time:.3f} seconds{fgColors.endc}")








