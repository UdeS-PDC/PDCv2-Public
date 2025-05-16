#----------------------------------------------------------------------------------
#-- Company: GRAMS
#-- Designer: Tommy Rossignol
#--
#-- Create Date: 2024-12-04
#-- Description:
#--     Using python ssh libraries to send remote commands to the ZCU102
#--     The script enables each pixel one at the time.
#--     For each pixel, the external trigger triggers N times the quenching circuit.
#--     If the quenching circuit is working properly, it should trigger N times.
#--
#-- Dependencies:
#-- Revision:
#-- Revision 1.0 - File Created
#-- Revision 2.0 - Updated for sharing on pdcv2-public
#-- Additional Comments:
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
tp = None

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
icp = initCtlPdcFromClient(client=client, sysClkPrd=10e-9, pdcEn=0xF)


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

# -----------------------------------------------
# --- Testing all the pixels,
# --- set the number of SPADS to 4096
# -----------------------------------------------
icp.nSpad = 4096
#icp.nPdcMax = 1 # NOTE: comment this line to use all PDCs
pixStep = 1 # 1 = test all pixels, 64 = test 1 out of 64

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
HOLD_TIME = 150.0
RECH_TIME = 10.0
FLAG_TIME = 10.0
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
#FLAG_FUNC = OUT_MUX.CFG_CLK
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



# --------------------------------------
# --- configure the external trigger ---
# --------------------------------------
sectionPrint("configure the external trigger")
# system clock period (in seconds)
CLK_PRD = icp.sysClkPrd

# minimum setting from proper operation with ZPP:
MIN_ZPP_RECOMMENDED_PERIOD = 2e-3

# trigger parameters
    # NOTE: N_TRG: 32767 is the maximum possible value
    # NOTE: N_TRG: since min measure time is MIN_ZPP_RECOMMENDED_PERIOD,
    #       for any setting where N_TRG*TRG_PRD is less than MIN_ZPP_RECOMMENDED_PERIOD,
    #       the execution time won't be any faster
N_TRG = 100  # number of trigger per loop
TRG_ON = 50e-9  # seconds
TRG_PRD = 10e-6  # seconds NOTE 655.36e-6 is the maximum possible value

N_TRG_CYC = int(TRG_PRD/CLK_PRD)
N_TRG_ON = int(TRG_ON/CLK_PRD)
N_TRG_OFF = int(N_TRG_CYC-N_TRG_ON)

measTime = 3*max(N_TRG*TRG_PRD, MIN_ZPP_RECOMMENDED_PERIOD) # ZPP period in seconds
    # NOTE: Using 3* to make sure the triggers are in the center of the ZPP period
    #       | measTime (wait) | measTime (N_TRG) | measTime (wait) |

# make sure to disable before changing settings
client.runPrint(f"ctlCfg -a TRGN -r 0x0000 -g")

client.runPrint(f"ctlCfg -a TRG0 -r 0x{icp.pdcEnUser&0xFFFF:04x} -g")
client.runPrint(f"ctlCfg -a TRG1 -r 0x{(icp.pdcEnUser>>16)&0xFFFF:04x} -g")
client.runPrint(f"ctlCfg -a TRG2 -r 0x0000 -g")
client.runPrint(f"ctlCfg -a TRG3 -r 0x0000 -g")
client.runPrint(f"ctlCfg -a TRGS -r 0x0000 -g")

client.runPrint(f"ctlCfg -a TRGH -r 0x{N_TRG_ON:04x} -g")
client.runPrint(f"ctlCfg -a TRGL -r 0x{N_TRG_OFF:04x} -g")


# ---------------------------------------
# --- configure Controller ZPP module ---
# ---------------------------------------
sectionPrint("configure Controller ZPP module")
CLK_PRD = icp.sysClkPrd
class ZppModuleSetMethod(IntEnum):
    app=0,
    registers=1

method = ZppModuleSetMethod.app
if method == ZppModuleSetMethod.app:
    # new application available from 20250509 image
    client.runPrint(f"set-ctl-zpp-prd {measTime}")
elif method == ZppModuleSetMethod.registers:
    print("  Configure ZPP Timer High Period")
    ZPP_HIGH_PRD=measTime # seconds
    ZPP_HIGH_REG=int(ZPP_HIGH_PRD/CLK_PRD)
    client.runPrint(f"ctlCfg -a ZPH0 -r 0x{ZPP_HIGH_REG&0xFFFF:04x} -g")
    client.runPrint(f"ctlCfg -a ZPH1 -r 0x{(ZPP_HIGH_REG>>16)&0xFFFF:04x} -g")

    print("  Configure ZPP Timer Low Period")
    ZPP_LOW_PRD=CLK_PRD
    ZPP_LOW_REG=int(ZPP_LOW_PRD/CLK_PRD)
    client.runPrint(f"ctlCfg -a ZPL0 -r 0x{ZPP_LOW_REG&0xFFFF:04x} -g")
    client.runPrint(f"ctlCfg -a ZPL1 -r 0x{(ZPP_LOW_REG>>16)&0xFFFF|0x8000:04x} -g")  # |0x8000 to enable ZPP



# ------------------------------------------------
# --- Prepare Controller FSM for the acquisition
# ------------------------------------------------
client.run(f"ctlCfg -a FSMM -r 0x0101 -g"); # triggered by a COMMAND

# --------------------------------------------------
# --- Class to generate the display of the results
# --------------------------------------------------
class tcrPlotter:
    def __init__(self, figName, nPdcMax, nSpad, doSavePlot=False, dataPath="default"):
        """
        create an empty object with no data, but with figure properly formatted
        """
        self.figName = figName
        self.nPdcMax = nPdcMax
        self.nSpad = nSpad

        # if PDc is enabled and gives valid data
        self.pdcValid = [False]*self.nPdcMax

        # index of tested pixel
        self.current_pixel_index = -1
        self.done_test_all_pixels = False
        self.run = True

        # data
        self.spadIdx = range(0, self.nSpad)
        self.spadTcr = [[0]*self.nSpad for iPdc in range(self.nPdcMax)]
        self.spad100 = [[] for iPdc in range(self.nPdcMax)]
        self.spadPop = [[] for iPdc in range(self.nPdcMax)]

        self.spadCumul100 = []
        self.spadCumulPop = []

        self.spadEn = [self.nSpad]*self.nPdcMax
        self.spadDis = [0]*self.nPdcMax

        # plot constants
        self.axTcr = 0
        self.axPop = 1

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

    def initPlot(self):
        """
        create properly formatted plot
        """
        plt.close('all')
        plt.ion()
        self.fig, self.axes = plt.subplots(nrows=1, ncols=2,
                                           figsize=(16,9), constrained_layout=True,
                                           num=self.figName)
        self.fig.get_layout_engine().set(w_pad=0.1, h_pad=0.1, hspace=0.05, wspace=0.05)

        # colors for plots
        self.colors = plt.rcParams['axes.prop_cycle'].by_key()['color']

        # show empty data for all axes
        self.hscatterTcr = [None]*self.nPdcMax
        self.linePopu = [None]*(self.nPdcMax+1)
        self.label = [""]*self.nPdcMax
        for iPdc in range(self.nPdcMax):
            self.label[iPdc] = f"PDC{iPdc}"
            # scatter of the TCR as a function of the SPAD index
            self.hscatterTcr[iPdc] = self.axes.flat[self.axTcr].scatter(self.spadIdx,
                                                                     self.spadTcr[iPdc],
                                                                     facecolors='none',
                                                                     edgecolors=self.colors[iPdc],
                                                                     linewidth=1.5,
                                                                     label=self.label[iPdc])
            # sorted population for each PDC
            self.linePopu[iPdc] = (self.axes.flat[self.axPop].plot(self.spad100[iPdc],
                                                                  self.spadPop[iPdc],
                                                                  label=self.label[iPdc]))[0]
        # cumulative population of all PDCs
        self.lineCumulLabel = f"All PDCs"
        self.lineCumul = (self.axes.flat[self.axPop].plot(self.spadCumul100,
                                                          self.spadCumulPop,
                                                          label=self.lineCumulLabel,
                                                          linewidth=2.0))[0]
        # statistics of the population
        self.lineCumulAvgLabel = f"{'All PDCs': <12} {'avg': <14}"
        self.lineCumulAvg = (self.axes.flat[self.axPop].plot([-1, 101], [0, 0], '--',
                                                             label=self.lineCumulAvgLabel,
                                                             linewidth=2.0))[0]
        self.lineCumulMedLabel = f"{'All PDCs': <12} {'': <17} {'med': <14}"
        self.lineCumulMed = (self.axes.flat[self.axPop].plot([-1, 101], [0, 0], '--',
                                                             label=self.lineCumulMedLabel,
                                                             linewidth=2.0))[0]

        # set titles
        self.axes.flat[self.axTcr].title.set_text("TCR as a function of the SPAD index")
        self.axes.flat[self.axPop].title.set_text("Histogram of TCR")

        # show legends
        self.updateLegend()

        # log y axis
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.axes.flat[self.axTcr].set_yscale('log')
            self.axes.flat[self.axPop].set_yscale('log')

        # scatter ticks
        self.axes.flat[self.axTcr].set_xticks(np.arange(0, 4097, 500))
        self.axes.flat[self.axTcr].set_xlim(-100, 4200)
        self.axes.flat[self.axTcr].xaxis.set_minor_locator(mticker.MultipleLocator(100))
        self.axes.flat[self.axTcr].tick_params(which="both", direction="in", top=True, right=True)

        # population ticks
        self.axes.flat[self.axPop].set_xticks(np.arange(0, 101, 10))
        self.axes.flat[self.axPop].set_xlim(-5, 105)
        self.axes.flat[self.axPop].xaxis.set_minor_locator(mticker.MultipleLocator(5))
        self.axes.flat[self.axPop].tick_params(which="both", direction="in", top=True, right=True)
        self.axes.flat[self.axPop].grid(visible=True, which="both", alpha=0.2)
        self.axes.flat[self.axPop].set_axisbelow(True)

        # limits
        set_lim(self.axes.flat[self.axTcr], self.spadTcr)
        set_lim(self.axes.flat[self.axPop], self.spadPop)

    def updateLegend(self):
        """
        show/update legends with proper parameters
        """
        self.axes.flat[self.axTcr].legend()
        self.axes.flat[self.axPop].legend(loc="upper left", title=f"{'': <26} {'avg': <14} {'med': <14}")

    def updatePlot(self, iPdc=None):
        """
        set new data on the plot without stealing the focus
        """
        if self.current_pixel_index >= 0:
            if iPdc == None:
                pdcRange = range(self.nPdcMax)
            else:
                pdcRange = [iPdc]

            for iPdc in pdcRange:
                if not self.pdcValid[iPdc]:
                    # PDC is not valid, remove it from the legend
                    self.hscatterTcr[iPdc].set_label(s='')
                    self.linePopu[iPdc].set_label(s='')

                else:
                    # update scatter
                    self.hscatterTcr[iPdc].set_label(s=self.label[iPdc])
                    scatterMax = min(len(self.spadIdx), len(self.spadTcr[iPdc])) # thread safe helper
                    self.hscatterTcr[iPdc].set_offsets(np.c_[self.spadIdx[:scatterMax], self.spadTcr[iPdc][:scatterMax]]) #

                    # update histo
                    avg = np.mean(self.spadPop[iPdc])
                    med = statistics.median(self.spadPop[iPdc])
                    self.linePopu[iPdc].set_label(s=f"{self.label[iPdc]: <12} {avg: <12.1f} {med: <12.1f}")
                    popMax = min(len(self.spad100[iPdc]), len(self.spadPop[iPdc])) # thread safe helper
                    self.linePopu[iPdc].set_data(np.array(self.spad100[iPdc][:popMax]),
                                                np.array(self.spadPop[iPdc][:popMax])) #

            # cumul of all PDCs
            avg = np.mean(self.spadCumulPop)
            med = statistics.median(self.spadCumulPop)
            self.lineCumul.set_label(s=f"{self.lineCumulLabel: <12} {avg: <12.1f} {med: <12.1f}")
            cumulIdx = min(len(self.spadCumul100), len(self.spadCumulPop)) # thread safe helper
            self.lineCumul.set_data(np.array(self.spadCumul100[:cumulIdx]),
                                    np.array(self.spadCumulPop[:cumulIdx])) #
            # cumul stats lines
            avgCumul = np.mean(self.spadCumulPop)
            self.lineCumulAvg.set_ydata([avgCumul, avgCumul])
            medCumul = statistics.median(self.spadCumulPop)
            self.lineCumulMed.set_ydata([medCumul, medCumul])

            # set new limits
            set_lim(self.axes.flat[self.axTcr], self.spadTcr)
            set_lim(self.axes.flat[self.axPop], self.spadPop)

        self.updateLegend()
        self.pausePlot(pauseTime=0.001)
        self.savePlot(iPdc=iPdc)
        self.checkExit()


    def newData(self, iPdc, iSpad, avg, avgTh):
        """
        add new data to the class
        """
        if avg < 0:
            # no valid data
            self.pdcValid[iPdc] = False
            self.spadEn[iPdc] -= 1
            self.spadDis[iPdc] += 1
            return
        self.pdcValid[iPdc] = True

        # add only valid data
        self.spadTcr[iPdc][iSpad] = avg
        self.spadPop[iPdc].append(avg)
        self.spadPop[iPdc].sort()
        self.spad100[iPdc] = np.linspace(0, 100.0, len(self.spadPop[iPdc]))

        self.spadCumulPop.append(avg)
        self.spadCumulPop.sort()
        self.spadCumul100 = np.linspace(0, 100.0, len(self.spadCumulPop))

        if avg != avgTh:
            # SPAD count is different than threshold
            print(f"{fgColors.red}Disabling SPAD {iSpad} on PDC {iPdc}{fgColors.endc}")
            self.spadEn[iPdc] -= 1
            self.spadDis[iPdc] += 1

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

                dfNew = pd.DataFrame(data=self.spadIdx, columns=[f"SPAD_idx{iPdc}"])
                df = pd.concat([df, dfNew], axis=1)
                dfNew = pd.DataFrame(data=self.spadTcr[iPdc], columns=[f"SPAD_TCR{iPdc}"])
                df = pd.concat([df, dfNew], axis=1)
                dfNew = pd.DataFrame(data=self.spad100[iPdc], columns=[f"SPAD_percent{iPdc}"])
                df = pd.concat([df, dfNew], axis=1)
                dfNew = pd.DataFrame(data=self.spadPop[iPdc], columns=[f"SPAD_distribution{iPdc}"])
                df = pd.concat([df, dfNew], axis=1)

        if df.size > 0:
            # if there are data to export
            filename = f"{dateStr}_TRG_{pdcStr}_{int(measTime*1000):d}ms.csv"
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
                filename = f"TRG_{self.plotIdx:06d}.png"
                self.plotPath.mkdir(parents=True, exist_ok=True)
                datafile = os.path.join(self.plotPath, filename)
                print(f"saving plot to file {datafile}")
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
    # flatten 2D array and remove zeroes and negative values
    dataFlatValid = [val for data1D in data for val in data1D if val > 0]
    if len(dataFlatValid) == 0:
        return

    yMin = min(dataFlatValid)/2.0
    yMax = max(dataFlatValid)*2.0
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

def measTrgRate(iPix, measTime, numPdc):
    """
    measTrgRate: send cmd and cfg to Controller and PDC to get the TRG count rate
    1- enable pixels, one at the time
    2- reset ZPP module
    3- wait for measTime for stats to build up
    4- send a Controller data packet with ZPP data
    5- wait to receive the file, fetch the ZPP data and close it
    """
    client.runPrint(f"pdcPix --dis --index {iPix} --mode NONE; sleep 0.001")
    N_SPAD = [1]*numPdc

    # all regrouped into a single command to remove host system execution time
    client.runPrint("ctlCmd -c MODE_TRG; " \
                    "ctlCmd -c RSTN_ZPP; " \
                    f"sleep {measTime/3.0:.06f}; " \
                    f"ctlCfg -a TRGN -r 0x{0x8000+N_TRG:04x}; " \
                    f"sleep {2.0*measTime/3.0:.06f}; " \
                    "ctlCmd -c MODE_ACQ; " \
                    "ctlCmd -c PACK_TRG_A; ")

    # wait for the HDF5 result file
    db = waitForH5File()
    db.h5Open()

    # get ZPP results
    TOT = [-1]*numPdc
    for iPdc in range(0, numPdc):
        ZPP = db.getPdcZPP(iPdc=iPdc, zppSingle=PDC_ZPP_ITEM.TOT)
        if (ZPP != None) and (ZPP.TOT != -1):
            # use ZPP value and normalize count rate to 1 sec to have cps
            TOT[iPdc] = ZPP.TOT
            print(f"  PDC {iPdc}, PIXEL {iPix}, TOT = {TOT[iPdc]}")

    db.h5Close()

    return TOT


# ---------------------------------------
# --- data acquisition as a thread
# ---------------------------------------
run_thread = True
def test_all_pixels(tp: tcrPlotter, update=False):
    if type(tp) == type(None):
        print(f"{fgColors.red}ERROR: tcrPlotter object must be created first{fgColors.endc}")
        sys.exit()

    # acquire data
    for iPix in range(0, icp.nSpad, pixStep):
        if not tp.run:
            break
        TOT = measTrgRate(iPix=iPix,
                        measTime=measTime,
                        numPdc=icp.nPdcMax)
        for iPdc in range(0, icp.nPdcMax):
            # put new data into data object
            tp.newData(iPdc, iPix, TOT[iPdc], N_TRG)

        tp.current_pixel_index = iPix

        if update:
            tp.updatePlot()

    # indicate all tests are completed
    tp.done_test_all_pixels = True



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
    tp = tcrPlotter(figName="PIX TRG PLOTTER",
                    nPdcMax=icp.nPdcMax,
                    nSpad=icp.nSpad,
                    doSavePlot=False)

    # ---------------------------------------
    # --- Pixel responsiveness logic
    # ---------------------------------------
    sectionPrint("Pixel responsiveness logic")

    # 1 - loop for each pixel to get its number of triggers and list pixels to disable
    if run_thread:
        # running as a thread
        thread_test = threading.Thread(target=test_all_pixels, args=[tp])
        thread_test.start()

        while not tp.done_test_all_pixels:
            tp.updatePlot()
    else:
        # running everything in the main
        test_all_pixels(tp=tp, update=True)

    # 2- Number of good pixel per PDC
    sectionPrint("Number of good pixel per PDC")
    for iPdc in range(0, icp.nPdcMax):
        if tp.pdcValid[iPdc]:
            print(f"PDC {iPdc} has {tp.spadDis[iPdc]} disabled SPADs")

    # export data
    tp.saveData()


    # total execution time
    test_stop_time = time.time()
    print(f"{fgColors.bBlue}Test took {test_stop_time-test_start_time:.3f} seconds{fgColors.endc}")
    print(f"{fgColors.bBlue}Test completed, to exit, close figure{fgColors.endc}")
    plt.show(block=True)
    print("\nFigure closed... exit program")

except (KeyboardInterrupt, SystemExit) as ex:
    if "tp" in locals():
        tp.run = False
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







