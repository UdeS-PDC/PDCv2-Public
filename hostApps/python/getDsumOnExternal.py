#----------------------------------------------------------------------------------
#-- Company: GRAMS
#-- Designer: Tommy Rossignol
#--
#-- Create Date: 2025-03-24
#-- Description:
#--      Using python ssh libraries to send remote commands to the ZCU102
#--      This script prepare the Controller and the PDCs for an acquisition.
#--      The acquisition is triggered by an external signal.
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

# -----------------------------------------------
# --- open a connection with the ZCU102 board
# -----------------------------------------------
sectionPrint("open a connection with the ZCU102 board")
# parameters of the ZCU102 board
# open a client based on its name in the ssh config file
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
# NOTE: select here the PDC to use:
#       pdcEn=0x1 -> PDC0
#       pdcEn=0x2 -> PDC1
#       pdcEn=0x4 -> PDC2
#       pdcEn=0x8 -> PDC3
#       pdcEn=0xF -> PDC0, PDC1, PDC2, PDC3
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
SCSA = 0x1000
# configure CTL_DATA_A
SCDA = 0x0000
# configure PDC_DATA_A
    # 0x0100 = DSUM
    # 0x00F7 = ZPP
SPDA = 0x0100
icp.setCtlPacket(bank=packetBank.BANKA, SCS=SCSA, SCD=SCDA, SPD=SPDA)

# -----------------------------------------------
# --- configure the Controller FSM
# -----------------------------------------------
sectionPrint("configure the Controller FSM")
# NOTE: Always start by configuring FSM registers with the higher register addresses first.
#       This way, the FSM will not start while being configured.
# NOTE: All registers with a CANCEL_MODE selection (FACQ, FSM_ACQ_CANCEL_MODE), use the following bits:
#       bit 0 - NO_COINCIDENCE          NOCN
#       bit 1 - SUM THRESHOLD AND       STHA
#       bit 2 - SUM THRESHOLD ANY (OR)  STHO
#       bit 3 - EXTERNAL 1              EXT1
#       bit 4 - EXTERNAL 2              EXT2
#       bit 5 - PDC_FUNC_ALL            PDCA
#       bit 6 - PDC_FUNC_ANY (OR)       PDCO


# Number of samples to acquire
# NOTE: nAcqSamples is the sum of nAcqSamplesFast and nAcqSamplesSlow.
#       This value should not be larger than 128.
#       nAcqSamplesFast = 128, means 128 samples each Controller clock cycle (e.g. default is 10 ns)
#       nAcqSamplesSlow = 128, means 128 samples, but with a spacing defined by Controller register SLW0 and SLW1
#       User can decide to split between fast and slow (e.g. 28 fast and 100 slow).
#       nAcqSamplesHistory is the number of samples before the trigger of the acquisition.
#       To get nAcqSamplesHistory, acquisition must run continuously to get samples before the trigger.
nAcqSamplesFast = 0
nAcqSamplesSlow = 128
nAcqSamples = nAcqSamplesFast + nAcqSamplesSlow
nAcqSamplesHistory = 28
print(f"nAcqSamplesFast     = {nAcqSamplesFast}")
print(f"nAcqSamplesSlow     = {nAcqSamplesSlow}")
print(f"nAcqSamples         = {nAcqSamples}")
print(f"nAcqSamplesHistory  = {nAcqSamplesHistory}")
print("")
# depending if ACQ is fast or slow or both, this is the trigger to start the acquistion
FSM_ACQ_TRG_MODE = 2    # 0 = FLAG,
                        # 1 = COINCIDENCE_OK
                        # 2 = EXTERNAL 1
                        # 3 = EXTERNAL 2
                        # 4 = SUM_THRESHOLD_AND
                        # 5 = SUM_THRESHOLD_OR
                        # 6 = COUNTER
                        # 7 = BYPASS


# configure FSM timeout register (TOUT)
FSM_TIMEOUT_AUTO = 0    # 1 = timeout period is automatically calculated
FSM_TIMEOUT_A1O0 = 1    # 1 = All PDC must timeout to sent a timeout, 0 = only 1 PDC to sent a timeout
FSM_TIMEOUT_RETRY = 1   # number of timeout retries before sending a timeout
FSM_TIMEOUT_PRD = 48    # period in clock cycles of the timeout
toutReg = ((FSM_TIMEOUT_AUTO&0x1)<<15) | ((FSM_TIMEOUT_A1O0&0x1)<<14) | \
          ((FSM_TIMEOUT_RETRY&0x3)<<12) | (FSM_TIMEOUT_PRD&0x3FF)
client.runPrint(f"ctlCfg -a TOUT -r 0x{toutReg:04x} -g")


# configure FSM END register (FEND)
FSM_END_EN = 1      # 1 = enable end state of the FSM
FSM_END_MODE = 2    # 0 = END_DONE_CNT, 1 = END_DONE_DAQ, 2 = END_DONE_PCK
FSM_END_DELAY = 0   # delay to use to leave the FSM end state. (used with END_DONE_CNT)
fendReg = ((FSM_END_EN&0x1)<<15) | ((FSM_END_MODE&0x3)<<8) | (FSM_END_DELAY&0xFF)
client.runPrint(f"ctlCfg -a FEND -r 0x{fendReg:04x} -g")


# configure FSM transmit register 1 (FTX1)
FSM_TX_CNL_SEND_RSTN = 0    # 1 = when cancelling the transmit, sent a RSTN command to the PDCs
FSM_TX_CANCEL_MODE = 0      # bitwise setting to set the conditions on which to sent a RSTN command to the PDCs
FSM_TX_N = nAcqSamples      # number of samples of the digital sum to transmit from the PDC to the Controller
ftx1Reg = ((FSM_TX_CNL_SEND_RSTN&0x1)<<15) | ((FSM_TX_CANCEL_MODE&0x7F)<<8) | (FSM_TX_N&0xFF)
client.runPrint(f"ctlCfg -a FTX1 -r 0x{ftx1Reg:04x} -g")

# configure FSM transmit register 0 (FTX0)
FSM_TX_EN = 1           # 1 = enable transmit state of the Controller FSM
FSM_TX_CMD_MODE = 1     # 0 = done with a counter after the command, 1 = done when TX command is sent
FSM_TX_MODE = 2         # 0 = DONE_PDC_FUNC_ALL,
                        # 1 = DONE_PDC_FUNC_ANY,
                        # 2 = DONE_CNT, (completion from counter FSM_TX_N)
                        # 3 = DONE_AUTO (completion auto calculated from given settings)
FSM_TX_CMD_DELAY = 0    # delay in clock cycle to wait after sending the command
ftx0Reg = ((FSM_TX_EN&0x1)<<15) | ((FSM_TX_CMD_MODE&0x1)<<10) | ((FSM_TX_MODE&0x3)<<8) | (FSM_TX_CMD_DELAY&0xFF)
client.runPrint(f"ctlCfg -a FTX0 -r 0x{ftx0Reg:04x} -g")


# configure FSM acquisition-transmission register 1 (ATX1)
FSM_ATX_CNL_SEND_RSTN = 0    # 1 = when cancelling the acquisition-transmission, sent a RSTN command to the PDCs
FSM_ATX_CANCEL_MODE = 0      # bitwise setting to set the conditions on which to sent a RSTN command to the PDCs
FSM_ATX_N = 0                # number of samples of the digital sum to acquire and transmit from the PDC
atx1Reg = ((FSM_ATX_CNL_SEND_RSTN&0x1)<<15) | ((FSM_ATX_CANCEL_MODE&0x7F)<<8) | (FSM_ATX_N&0xFF)
client.runPrint(f"ctlCfg -a ATX1 -r 0x{atx1Reg:04x} -g")

# configure FSM acquisition-transmission register 0 (ATX0)
FSM_ATX_EN = 0          # 1 = enable acquisition-transmission state of the Controller FSM
FSM_ATX_MODE = 0        # 0 = DONE_PDC_FUNC_ALL,
                        # 1 = DONE_PDC_FUNC_ANY,
                        # 2 = DONE_CNT, (completion from counter FSM_TX_N)
                        # 3 = DONE_AUTO (completion auto calculated from given settings)
FSM_ATX_CMD_MODE = 0    # 0 = FLAG,
                        # 1 = COINCIDENCE_OK
                        # 2 = EXTERNAL 1
                        # 3 = EXTERNAL 2
                        # 4 = SUM_THRESHOLD_AND
                        # 5 = SUM_THRESHOLD_OR
                        # 6 = COUNTER
                        # 7 = BYPASS
FSM_ATX_CMD_DELAY = 0   # delay in clock cycle to wait after sending the command
atx0Reg = ((FSM_ATX_EN&0x1)<<15) | ((FSM_ATX_MODE&0x3)<<11) | \
          ((FSM_ATX_CMD_MODE&0x7)<<8) |  (FSM_ATX_CMD_DELAY&0xFF)
client.runPrint(f"ctlCfg -a ATX0 -r 0x{atx0Reg:04x} -g")


# configure FSM slow acquisition register 1 (SLW1)
FSM_ACQ_SLOW_PRD = 0; # NOTE: change PRD setting here, any setting between 0 and 255 is valid
FSM_ACQ_SLOW_N = nAcqSamplesSlow
slw1Reg = ((FSM_ACQ_SLOW_PRD&0xFF)<<8) | (FSM_ACQ_SLOW_N&0xFF)
client.runPrint(f"ctlCfg -a SLW1 -r 0x{slw1Reg:04x} -g")

# configure FSM slow acquisition register 0 (SLW0)
FSM_ACQ_SLOW_EN = 1 if nAcqSamplesSlow > 0 else 0   # enable slow acquisition state the the Controller FSM
FSM_ACQ_SLOW_MODE = 0   # 0 = FLAG,
                        # 1 = COINCIDENCE_OK
                        # 2 = EXTERNAL 1
                        # 3 = EXTERNAL 2
                        # 4 = SUM_THRESHOLD_AND
                        # 5 = SUM_THRESHOLD_OR
                        # 6 = COUNTER
                        # 7 = BYPASS
FSM_ACQ_SLOW_DELAY = 0  # delay in clock cycle to wait at the end of the state
if nAcqSamplesFast == 0:
    FSM_ACQ_SLOW_MODE = FSM_ACQ_TRG_MODE
    FSM_ACQ_SLOW_DELAY = nAcqSamplesSlow - nAcqSamplesHistory
slw0Reg = ((FSM_ACQ_SLOW_EN&0x1)<<15) | ((FSM_ACQ_SLOW_MODE&0x7)<<8) | (FSM_ACQ_SLOW_DELAY&0xFF)
client.runPrint(f"ctlCfg -a SLW0 -r 0x{slw0Reg:04x} -g")


# configure FSM fast acquisition register 1 (FST1)
FSM_ACQ_FAST_N = nAcqSamplesFast
fst1Reg = (FSM_ACQ_FAST_N&0xFF)
client.runPrint(f"ctlCfg -a FST1 -r 0x{fst1Reg:04x} -g")

# configure FSM fast acquisition register 0 (FST0)
FSM_ACQ_FAST_EN = 1 if nAcqSamplesFast > 0 else 0   # enable fast acquisition state the the Controller FSM
FSM_ACQ_FAST_MODE = FSM_ACQ_TRG_MODE
                        # 0 = FLAG,
                        # 1 = COINCIDENCE_OK
                        # 2 = EXTERNAL 1
                        # 3 = EXTERNAL 2
                        # 4 = SUM_THRESHOLD_AND
                        # 5 = SUM_THRESHOLD_OR
                        # 6 = COUNTER
                        # 7 = BYPASS
FSM_ACQ_FAST_DELAY = nAcqSamplesFast - nAcqSamplesHistory # delay in clock cycle to wait at the end of the state
fst0Reg = ((FSM_ACQ_FAST_EN&0x1)<<15) | ((FSM_ACQ_FAST_MODE&0x7)<<8) | (FSM_ACQ_FAST_DELAY&0xFF)
client.runPrint(f"ctlCfg -a FST0 -r 0x{fst0Reg:04x} -g")


# configure FSM acquisition register (FACQ)
FSM_ACQ_CNL_SEND_RSTN = 0       # 1 = when cancelling the acquisition, sent a RSTN command to the PDCs
FSM_ACQ_CANCEL_MODE = 0         # bitwise setting to set the conditions on which to sent a RSTN command to the PDCs
FSM_ACQ_OVERLAP = 0             # number of samples to overlap while sending the next command
FSM_ACQ_CMD_MODE = 7            # 0 = FLAG
                                # 1 = COINCIDENCE_OK
                                # 2 = EXTERNAL_1
                                # 3 = EXTERNAL_2
                                # 7 = BYPASS
facqReg = ((FSM_ACQ_CNL_SEND_RSTN&0x1)<<15) | ((FSM_ACQ_CANCEL_MODE&0x7F)<<8) | \
          ((FSM_ACQ_OVERLAP&0xF)<<4) | (FSM_ACQ_CMD_MODE&0x7)
client.runPrint(f"ctlCfg -a FACQ -r 0x{facqReg:04x} -g")


# configure FSM misc register (FSMM)
# NOTE: keep FSM disabled for now, will be enabled when everything else is ready
FSM_TX_ALL_PDC = 1      # force to transmit from all PDCs, not only those with data
FSM_ACQ_ALL_PDC = 1     # force to acquire from all PDCs, not only those with events
FSM_SEQ_START_MODE = 0  # 0 = DISABLED
                        # 1 = COMMAND only
                        # 2 = TIMER
                        # 3 = BYPASS (as soon as possible)
fsmmReg = ((FSM_TX_ALL_PDC&0x1)<<8) | ((FSM_ACQ_ALL_PDC&0x1)<<4) | (FSM_SEQ_START_MODE&0x3)
client.runPrint(f"ctlCfg -a FSMM -r 0x{fsmmReg:04x} -g")






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
# --- Using all of the 4096 pixels,
# --- set the number of SPADS to 4096
# -----------------------------------------------
# NOTE: Default icp.nSpad is 64 (2D CMOS SPAD)
#       To user all of the 4096 pixels,
#       uncomment the following line
#icp.nSpad = 4096

# --------------------------
# --- configure the PDCs ---
# --------------------------
sectionPrint("configure the PDCs")
PDC_SETTING = pdc_setting()
client.runPrint("ctlCmd -c MODE_CFG")  # set PDCs to configuration mode

# === PIXL REGISTER ===
print("\n=== PIXL REGISTER ===")
# active quenching of the front-end
ACTIVE_QC_EN = 1; # 0=disabled/passive, 1=enabled/active
# trigger using QC front-end (FE) or digital only (DGTL)
TRG_DGTL_FEN = 0; # 0=FE, 1=DGTL
# enable flag output of the pixel
FLAG_EN = 1; # 1=enabled, 0=disabled
# EDGE_LVLN and DIS_MEM on synchronizer
EDGE_LVLN = 1
DIS_MEM = 1
PIXL = ((DIS_MEM<<13) + (EDGE_LVLN<<12) + (FLAG_EN<<8) + (TRG_DGTL_FEN<<4) + (ACTIVE_QC_EN<<1))
client.runPrint(f"pdcCfg -a PIXL -r 0x{PIXL:04x} -g")  # configure pixel register
PDC_SETTING.PIXL = PIXL

# === TIME REGISTER ===
print("\n=== TIME REGISTER ===")
HOLD_TIME = 50.0
RECH_TIME = 10.0
FLAG_TIME =  2.0
client.runPrint(f"pdcTime --hold {HOLD_TIME} --rech {RECH_TIME} --flag {FLAG_TIME} -g")
PDC_SETTING.TIME = client.runReturnSplitInt('pdcTime -g')

# === ANLG REGISTER ===
print("\n=== ANLG REGISTER ===")
#ANLG = 0x0000; # disabled
ANLG = 0x001F; # full amplitude (~30 µA)
client.runPrint(f"pdcCfg -a ANLG -r 0x{ANLG:04x} -g")  # set analog monitor
PDC_SETTING.ANLG = ANLG

# === XXXX REGISTER ===
# skipping registers ACQA to DBGC

# === FIFO REGISTER ===
print("\n=== FIFO REGISTER ===")
OVERWRITE_DIS = 0   # disable overwriting of the FIFO during acquisition
FIFO_EN = 1         # to enable the FIFO
SEL_FIFO_IN = 0     # 0 = digital sum
                    # 1 = debug counter
                    # 2 = debug register
                    # 3 = unused (zeros)
FIFO_DEPTH = 127    # the size of the FIFO - 1. (e.g. FIFO_DEPTH=127 -> FIFO has 128 bins)
FIFO = ((OVERWRITE_DIS&0x1)<<13) | ((FIFO_EN & 0x1)<<12) | ((SEL_FIFO_IN&0x3)<<8) | (FIFO_DEPTH&0x7F)
client.runPrint(f"pdcCfg -a FIFO -r 0x{FIFO:04x} -g")
PDC_SETTING.FIFO = FIFO

# === OUTD REGISTER ===
print("\n=== OUTD REGISTER ===")
#DATA_FUNC = OUT_MUX.FLAG
#DATA_FUNC = OUT_MUX.TRG
DATA_FUNC = OUT_MUX.DATA
#DATA_FUNC = OUT_MUX.VSS
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
client.runPrint(f"pdcCfg -a TRGC -r 0x{TRGC:04x} -g")  # disable trigger command
PDC_SETTING.TRGC = TRGC

# === DISABLE ALL THE PIXELS ===
print("\n=== DISABLE ALL THE PIXELS ===")
    # NOTE: pdcPix returns the PDC to acquisition mode,
    #       if mode is not specified
client.runPrint("pdcPix --dis --mode NONE")

# === VALIDATE CONFIGURATIONS ===
print("\n=== VALIDATE CONFIGURATIONS ===")
icp.validPdcCfg()

# === OUTC REGISTER ===
print("\n=== OUTC REGISTER ===")
    # disable configuration output last once configuration was validated
FLAG_CFG_FUNC = OUT_MUX.CLK_CS     # default function
#FLAG_CFG_FUNC = OUT_MUX.VSS        # disabled
DATA_CFG_FUNC = OUT_MUX.CFG_VALID  # default function
#DATA_CFG_FUNC = OUT_MUX.VSS        # disabled
OUTC = (DATA_CFG_FUNC & 0x1F) + ((FLAG_CFG_FUNC & 0x1F)<<6)
client.runPrint(f"pdcCfg -a OUTC -r 0x{OUTC:04x} -g")
PDC_SETTING.OUTC = OUTC

# print the settings of all the PDCs
print("\n=== PDC SETTINGS ===")
PDC_SETTING.print()


# ------------------------
# --- enable PDC SPADs ---
# ------------------------
sectionPrint("enable PDC SPADs")
# NOTE: run command pdcSpad --help on Zynq to get more options to enable SPADs
# NOTE: This method enables N_SPAD_TO_ENABLE for each PDC from the center.
N_SPAD_TO_ENABLE = 64
client.runPrint(f"pdcSpad --verbose 3 --ncenter {N_SPAD_TO_ENABLE} --mode NONE")

# NOTE: To enable different SPADs for each PDC, use the following method:
# NOTE: Use another script to identify screamers and disable them.
#       Here, it is hardcoded as an example, user must find the appropriated values for each Head board.
#client.runPrint(f"pdcSpad --verbose 4 --pattern {0xdfdff7ffffffffff} --spdc 0 --mode NONE;") # PDC0
#client.runPrint(f"pdcSpad --verbose 4 --pattern {0xeffeffbeffffffff} --spdc 1 --mode NONE;") # PDC1
#client.runPrint(f"pdcSpad --verbose 4 --pattern {0xffffffffffffffff} --spdc 2 --mode NONE;") # PDC2
#client.runPrint(f"pdcSpad --verbose 4 --pattern {0xffffffffffffffff} --spdc 3 --mode NONE;") # PDC3
#client.runPrint(f"pdcSpad --verbose 4 --pattern {0xffbfdffd97ffddff} --spdc 4 --mode NONE;") # PDC4
#client.runPrint(f"pdcSpad --verbose 4 --pattern {0xffbbdfffffffdfff} --spdc 5 --mode NONE;") # PDC5
#client.runPrint(f"pdcSpad --verbose 4 --pattern {0xffff7bfffff7efff} --spdc 6 --mode NONE;") # PDC6
#client.runPrint(f"pdcSpad --verbose 4 --pattern {0xff7ffbdfbfeffff3} --spdc 7 --mode NONE;") # PDC7

# ---------------------------------------
# --- return PDCs to acquisition mode ---
# ---------------------------------------
sectionPrint("return PDCs to acquisition mode")
client.runPrint("ctlCmd -c MODE_ACQ")


# ---------------------------------------
# --- notify user of manual steps
# ---------------------------------------
try:
    print(f"{fgColors.bYellow}Apply HV here{fgColors.endc}")
    input("Press [enter] key to continue")
except KeyboardInterrupt:
    print("\nKeyboard Interrupt: exit program")
    sys.exit()

# ------------------------------------------------
# --- start Controller FSM acquisition
# ------------------------------------------------
sectionPrint("start Controller FSM acquisition")
client.runPrint(f"ctlCfg -a FSMM -r 0x{fsmmReg|0x3:04x} -g"); # starts the FSM

# ---------------------------------------------------
# --- Use AUXI 0 (DBG IN1) as EXTERNAL_1 for FSM ---
# ---------------------------------------------------
sectionPrint("Use AUXI 0 (DBG IN1) as EXTERNAL_1 for FSM")
AUX_FUNC_SEL = 0; # EXT1 (Controller internal signal)
AUX_REV_POL = 0;  # reverse polarity of input signal
AUX_CH_SEL = 4;   # AUXI 0 (External input, PCB DBG IN1 on adaptor 0)
auxiReg = ((AUX_FUNC_SEL&0xF)<<12) | ((AUX_REV_POL&0x1)<<8) | (AUX_CH_SEL&0xF)
client.runPrint(f"ctlCfg -a AUXI -r 0x{auxiReg:04x} -g")


# use strobe timer
client.runPrint("auxOut --ch 1 --func 0x17 -g")

# DEBUG strobe timer
INT_TIMER = 100000000
client.runPrint(f"ctlCfg -a TMR0 -r 0x{INT_TIMER&0xFFFF:04x} -g")
client.runPrint(f"ctlCfg -a TMR1 -r 0x{0x8000|(INT_TIMER>>16)&0xFFFF:04x} -g")

# ------------------------
# --- ready to operate ---
# ------------------------
print("\n=== READY TO OPERATE ===")
# NOTE: Implement here a specific routine
try:
    input("Press [enter] key to exit")
except KeyboardInterrupt:
    print("\nKeyboard Interrupt: exit program")

finally:
    client.runPrint("stop")
    sys.exit()





