#----------------------------------------------------------------------------------
#-- Company: GRAMS
#-- Designer: Tommy Rossignol
#--
#-- Create Date: 2024-06-17
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
import modules.pixMap as pixMap
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

# time to wait for each setting
measTime = 0.2  # second

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
# --- 4096 for a complete 3D SPAD array
# --- 64 for the embedded 2D CMOS SPADs
# -----------------------------------------------
icp.nSpad = 64


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
        self.pdcTcrAll = [0]*self.nPdcMax   # all pixels
        self.pdcTcrNS = [0]*self.nPdcMax    # no screamers
        self.spadIdx = range(0, self.nSpad)
        self.spadTcr = [[0]*self.nSpad for iPdc in range(self.nPdcMax)]
        self.spad100 = [[] for iPdc in range(self.nPdcMax)]
        self.spadPop = [[] for iPdc in range(self.nPdcMax)]

        self.spadCumul100 = []
        self.spadCumulPop = []

        self.spadEn  = [[0]*self.nSpad for iPdc in range(self.nPdcMax)]
        self.spadValid = [[False]*self.nSpad for iPdc in range(self.nPdcMax)]

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
        if self.nSpad <= 64:
            self.axes.flat[self.axTcr].set_xticks(np.arange(0, 65, 8))
            self.axes.flat[self.axTcr].xaxis.set_minor_locator(mticker.MultipleLocator(1))
        else:
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


    def newData(self, iPdc, iSpad, avg):
        """
        add new data to the class
        """
        if avg < 0:
            # no valid data
            self.pdcValid[iPdc] = False
            return
        self.pdcValid[iPdc] = True
        self.spadValid[iPdc][iSpad] = True

        # add only valid data
        self.pdcTcrAll[iPdc] += avg
        self.spadTcr[iPdc][iSpad] = avg
        self.spadPop[iPdc].append(avg)
        self.spadPop[iPdc].sort()
        self.spad100[iPdc] = np.linspace(0, 100.0, len(self.spadPop[iPdc]))

        self.spadCumulPop.append(avg)
        self.spadCumulPop.sort()
        self.spadCumul100 = np.linspace(0, 100.0, len(self.spadCumulPop))


    def countValidSpads(self, iPdc):
        """
        count the number of valid SPADs for a given PDC.
        To be valid, a SPAD must have a ZPP packet associated to it.
        """
        return self.spadValid[iPdc].count(True)

    def countEnabledSpads(self, iPdc):
        """
        return the number of enabled SPADs
        """
        return self.spadEn[iPdc].count(1)

    def getPerSpadAvgTcr(self, iPdc, screamersEnabled=True):
        """
        get the average TCR per SPAD using total TCR
        divided by the number of valid/enabled SPADs
        """
        if not self.pdcValid[iPdc]:
            return 0
        if screamersEnabled:
            return self.pdcTcrAll[iPdc]/(1.0*self.countValidSpads(iPdc))
        else:
            return self.pdcTcrNS[iPdc]/(1.0*self.countEnabledSpads(iPdc))

    def getSpadMedTcr(self, iPdc):
        """
        get the median TCR value
        """
        return statistics.median(self.spadTcr[iPdc])

    def getClosestPctPop(self, iPdc, percent):
        """
        get the TCR value closest to the specified percentage in the population
        """
        lst = self.spad100[iPdc]
        return self.spadPop[iPdc][min(range(len(lst)), key = lambda i: abs(lst[i]-percent))]

    def getSpadEnFromThreshold(self, iPdc, threshold, methodStr="",
                               doPrint=True, addToScatter=False):
        """
        From a given count rate threshold, detect if a SPAD must be enabled or not
        """
        self.pdcTcrNS[iPdc] = 0 # reset value
        for iSpad in range(0, self.nSpad):
            if self.spadValid[iPdc][iSpad] and self.spadTcr[iPdc][iSpad] <= threshold:
                # smaller of equal to threshold, keep it enabled
                self.spadEn[iPdc][iSpad] = 1
                self.pdcTcrNS[iPdc] += self.spadTcr[iPdc][iSpad]
            else:
                # larger than threshold, disable it
                self.spadEn[iPdc][iSpad] = 0

        nSpadEnabled = self.countEnabledSpads(iPdc=iPdc)
        nSpadDisabled = self.nSpad - nSpadEnabled
        if doPrint:
            print(f"  PDC {iPdc} disabled {nSpadDisabled} SPAD with threshold of {threshold:.01f} {methodStr}")

        if addToScatter:
            self.axes.flat[self.axTcr].plot((min(self.spadIdx), max(self.spadIdx)),
                                             (threshold, threshold), "--",
                                             color=self.colors[iPdc],
                                             label=f"PDC{iPdc} th ({threshold})")

        return nSpadEnabled, self.spadEn[iPdc]

    def getSpadPattern(self, iPdc):
        """
        get an hexadecimal pattern to enable SPADs.
        Works only for less then 64 SPADs with app pdcSpad.
        """
        pattern = 0;
        if self.nSpad <= 64 and self.pdcValid[iPdc]:
            for iSpad in range(0, self.nSpad):
                if self.spadEn[iPdc][iSpad]:
                    pattern += (0x1<<iSpad)
        return pattern


    def getSpadRegister(self, iPdc):
        """
        get a list of registers to enable SPADs.
        """
        registerList = [0]*256 # (4096 pixels / 16 bits registers)
        if self.pdcValid[iPdc]:
            for iSpad in range(0, self.nSpad):
                if self.spadEn[iPdc][iSpad]:
                    addr = int(np.floor(iSpad/16))
                    registerList[addr] += (0x1<<(iSpad-16*addr))

        return registerList


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
            filename = f"{dateStr}_TCR_{pdcStr}_{int(measTime*1000):d}ms.csv"
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
                filename = f"TCR_{self.plotIdx:06d}.png"
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

def measCntRate(measTime, numPdc,
                spadRow=None, spadCol=None,
                spadIndex=None):
    """
    measCntRate: send cmd and cfg to Controller and PDC to get the SPAD count rate
    1- enable spads based on 64 bits pattern given and return to acquisition mode
    2- reset ZPP module
    3- wait for measTime for stats to build up
    4- send a Controller data packet with ZPP data
    5- wait to receive the file, fetch the ZPP data and close the file
    """
    if type(spadRow) != type(None) and type(spadRow) != type(None):
        # pixel/SPAD specified by row and column
        client.runPrint(f"pdcPix --dis --row {spadRow} --col {spadCol} --mode NONE")

    elif type(spadIndex) != type(None):
        # pixel/SPAD specified by index
        client.runPrint(f"pdcPix --dis --index {spadIndex} --mode NONE")

    else:
        # do not change pixels settings
        print("pixel/SPAD not specified")


    # send commands to the Controller/PDC
    client.runPrint("ctlCmd -c MODE_ACQ; " \
                    "ctlCmd -c RSTN_ZPP; " \
                    f"sleep {measTime:.06f}; " \
                    "ctlCmd -c PACK_TRG_A; ")

    # wait for the HDF5 result file
    db = waitForH5File()
    db.h5Open()

    # get ZPP results
    AVG = [-1]*numPdc
    for iPdc in range(0, numPdc):
        ZPP = db.getPdcZPP(iPdc=iPdc, zppSingle=PDC_ZPP_ITEM.AVG)
        if (ZPP != None) and (ZPP.AVG != -1):
            # use ZPP value and normalize count rate to 1 sec to have cps
            AVG[iPdc] = ZPP.AVG / measTime
            print(f"  PDC {iPdc} TCR = {AVG[iPdc]}")

    db.h5Close()

    return AVG


# ---------------------------------------
# --- data acquisition as a thread
# ---------------------------------------
class LoopingMethod(IntEnum):
    rows_cols=1,
    index=2,

def test_all_pixels(tp: tcrPlotter, update=False):
    if type(tp) == type(None):
        print(f"{fgColors.red}tcrPlotter object must be created first{fgColors.endc}")
        sys.exit()

    # acquire data
    if icp.nSpad == 4096:
        # testing a full array
        rows = range(0, 64)
        cols = range(0, 64)
        lMethod = LoopingMethod.rows_cols

    elif icp.nSpad == 64:
        # testing only 2D CMOS SPADs
        rows = [63]
        cols = range(0, icp.nSpad)
        lMethod = LoopingMethod.rows_cols
    else:
        # testing based on index (fallback case, should not be used)
        rows = [0]
        cols = range(0, icp.nSpad)
        lMethod = LoopingMethod.index

    for iRow in rows:
        for iCol in cols:
            if not tp.run:
                break
            if lMethod == LoopingMethod.index:
                spadIndex = iCol
                spadRow = None
                spadCol = None
                iSpad = spadIndex
            elif lMethod == LoopingMethod.rows_cols:
                spadIndex = None
                spadRow = iRow
                spadCol = iCol
                if icp.nSpad == 4096:
                    iSpad = pixMap.idx_map(y=spadRow, x=spadCol)
                else:
                    iSpad = iCol
            # enable the proper pixels and measure the total countrate
            AVG_TCR = measCntRate(spadIndex=spadIndex,
                                  spadRow=spadRow,
                                  spadCol=spadCol,
                                  measTime=measTime,
                                  numPdc=icp.nPdcMax)

            # add new data for each PDC
            for iPdc in range(0, icp.nPdcMax):
                # put new data into data object
                tp.newData(iPdc=iPdc,
                           iSpad=iSpad,
                           avg=AVG_TCR[iPdc])

            tp.current_pixel_index = iSpad

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
    tp = tcrPlotter(figName="TCR PLOTTER",
                    nPdcMax=icp.nPdcMax,
                    nSpad=icp.nSpad,
                    doSavePlot=False)

    # ---------------------------------------
    # --- SPAD count rate logic
    # ---------------------------------------
    # 1 - loop for each SPAD to measure its TCR
    sectionPrint("Loop for each SPAD to measure its TCR")
    run_thread = True
    if run_thread:
        # running as a thread
        thread_test = threading.Thread(target=test_all_pixels, args=[tp])
        thread_test.start()

        while not tp.done_test_all_pixels:
            tp.updatePlot()
        # update a last time
        tp.updatePlot()

    else:
        test_all_pixels(tp=tp, update=True)

    # 2- estimate the TCR of the array with all pixels enabled
    sectionPrint("Estimate the TCR of the array (all pixels)")
    # NOTE: This estimation does not consider that the TCR is obtained with the flag.
    #       If the SPADs TCR is too high, the flag will underestimate the TCR (pixels overlap)
    for iPdc in range(0, icp.nPdcMax):
        if tp.pdcValid[iPdc]:
            nSpad = tp.countValidSpads(iPdc)
            print(f"  PDC {iPdc} total TCR = {tp.pdcTcrAll[iPdc]: <8} " \
                  f"({tp.getPerSpadAvgTcr(iPdc): <8.01f} per SPAD for {nSpad} SPADs)")

    # 3- identify the screamer pixels
    sectionPrint("Identify the screamer pixels")
    # NOTE: Select here a method to identify the screamers.
    class ScreamerMethod(IntEnum):
        threshold=0,
        average=1,
        percent=2,
        medianFactor=3,
        medianToMin=4

    # store all tested threshold methods to later select the one to use
    thresholdList = [[0]*len(ScreamerMethod) for _ in range(0, icp.nPdcMax)]

    for iPdc in range(0, icp.nPdcMax):
        if not tp.pdcValid[iPdc]:
            continue

        # enable/disable the pixels based on a fixed threshold
        # NOTE: change threshold to be more or less selective
        thresholdList[ScreamerMethod.threshold][iPdc] = 400
        nSpadEnabled, spadEnabled = \
            tp.getSpadEnFromThreshold(  iPdc,
                                        threshold=thresholdList[ScreamerMethod.threshold][iPdc],
                                        methodStr="(fixed)")

        # enable/disable the pixels based on the average
        thresholdList[ScreamerMethod.average][iPdc] = tp.getPerSpadAvgTcr(iPdc)
        nSpadEnabled, spadEnabled = \
            tp.getSpadEnFromThreshold(  iPdc,
                                        threshold=thresholdList[ScreamerMethod.average][iPdc],
                                        methodStr="(average)")

        # enable/disable the pixels based on the percentage of the population
        # NOTE: change threshold to be more or less selective
        percent = 90
        thresholdList[ScreamerMethod.percent][iPdc] = tp.getClosestPctPop(iPdc, percent=percent)
        nSpadEnabled, spadEnabled = \
            tp.getSpadEnFromThreshold(  iPdc,
                                        threshold=thresholdList[ScreamerMethod.percent][iPdc],
                                        methodStr=f"({percent}%)")

        # enable/disable the pixels based on a factor to the median
        # NOTE: change factor to be more or less selective
        factor = 1.5
        thresholdList[ScreamerMethod.medianFactor][iPdc] = tp.getSpadMedTcr(iPdc)*factor
        nSpadEnabled, spadEnabled = \
            tp.getSpadEnFromThreshold(  iPdc,
                                        threshold=thresholdList[ScreamerMethod.medianFactor][iPdc],
                                        methodStr="(factor to median)")

        # enable/disable the pixels based on the distance from min to the median
        # NOTE: (median + (median - min)) = 2*median - min
        median = tp.getSpadMedTcr(iPdc)
        thresholdList[ScreamerMethod.medianToMin][iPdc] = 2.0*median-min(tp.spadTcr[iPdc])
        nSpadEnabled, spadEnabled = \
            tp.getSpadEnFromThreshold(  iPdc,
                                        threshold=thresholdList[ScreamerMethod.medianToMin][iPdc],
                                        methodStr="(distance from median to min)")
        print("")

    # 4- estimate the TCR of the array with the screamers pixels disabled
    sectionPrint("Estimate the TCR of the array (no screamers)")
    # NOTE: change here the method to use
    method = ScreamerMethod.percent
    print(f"selected method is {method.name}")

    # estimate the TCR using the given parameters
    for iPdc in range(0, icp.nPdcMax):
        if not tp.pdcValid[iPdc]:
            continue
        threshold = thresholdList[method][iPdc]
        nSpadEnabled, spadEnabled = \
                tp.getSpadEnFromThreshold(  iPdc,
                                            threshold=threshold,
                                            methodStr=method.name,
                                            addToScatter=True,
                                            doPrint=False)

        nSpad = tp.countEnabledSpads(iPdc)
        print(f"  PDC {iPdc} total TCR = {tp.pdcTcrNS[iPdc]: <8} " \
                f"({tp.getPerSpadAvgTcr(iPdc, False): <8.01f} per SPAD for {nSpad} SPADs at threshold = {threshold})")

    # refresh plot with selected thresholds
    tp.updateLegend()
    tp.pausePlot(pauseTime=0.001)

    # 5- enable the selected SPADs
    sectionPrint("Enable the selected SPADs")
    for iPdc in range(0, icp.nPdcMax):
        if not tp.pdcValid[iPdc]:
            continue
        # enable only the selected SPADs and print command (to copy to another script)
        if tp.nSpad <= 64:
            # for less than or 64 SPADs (here the embedded 2D CMOS SPADs), use a pattern
            pdcSpadCmd = f"pdcSpad --dis --pattern 0x{tp.getSpadPattern(iPdc):016x} --spdc {iPdc} --mode NONE"
            client.runPrint(pdcSpadCmd)
        else:
            # for more than 64 SPADs (here the 3D SPADs), specify each register to enable
            registerList = tp.getSpadRegister(iPdc)
            pdcSpadDisCmd = f"pdcPix --dis --spdc {iPdc} --mode NONE"
            client.runPrint(pdcSpadDisCmd)
            for addr, register in enumerate(registerList):
                if register != 0:
                    # skippeing empty registers since previously disabled all the SPADs
                    pdcSpadCmd = f"pdcPix --addr {addr} --reg 0x{register:04x} --spdc {iPdc} --mode NONE"
                    client.runPrint(pdcSpadCmd)

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







