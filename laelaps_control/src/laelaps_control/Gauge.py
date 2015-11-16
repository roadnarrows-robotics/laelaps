###############################################################################
#
#
# Package:   RoadNarrows Robotics Laelaps Robotic Mobile Platform ROS Package
#
# Link:      https://github.com/roadnarrows-robotics/laelaps
#
# ROS Node:  laelaps_*
#
# File:      Gauge.py
#
## \file 
##
## $LastChangedDate$
## $Rev$
##
## \brief Laelaps gauge widgets.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright:
##   (C) 2015.  RoadNarrows LLC.\n
##   (http://www.roadnarrows.com)\n
##   All Rights Reserved
##
# @EulaBegin@
# @EulaEnd@
#
###############################################################################

import sys
import os
import time
import random

from Tkinter import *
from Tkconstants import *
from tkFileDialog import *
import tkFont

from laelaps_control.Utils import *


# ------------------------------------------------------------------------------
# Class Dial
# ------------------------------------------------------------------------------

#
## \brief Laelaps about dialog.
##
class Dial(Frame):
  GaugeMinAngle = -135
  GaugeMaxAngle =  135
  GaugeRange    =  GaugeMaxAngle - GaugeMinAngle

  #
  ## \brief Constructor.
  ##
  ## \param cnf     Configuration dictionary.
  ## \param kw      Keyword options.
  #
  def __init__(self, master=None, cnf={}, **kw):
    self.setDefaults()

    kw = self.configGauge(kw)

    Frame.__init__(self, master=master, cnf=cnf, **kw)

    self.createGauge()

  #
  def setDefaults(self):
    # defaults
    self.m_gaugeLabel     = ""        ## show fixed label on gauge face
    self.m_gaugeShowVal   = False     ## do [not] show value on gauge face
    self.m_gaugeValMin    = 0.0       ## gauge minimum value
    self.m_gaugeValMax    = 100.0     ## gauge maximum value
    self.m_gaugeValHome   = 0.0       ## gauge home value
    self.m_gaugeValFmt    = "%.1f"    ## gauge value display format
    self.m_gaugeTextColor = "#ff8822" ## gauge label and value display color
    self.m_gaugeSize      = 300       ## gauge size (pixels)
    self.m_gaugeRes       = 1.0       ## gauge needle resolution (degrees)
    self.m_gaugeMovingWin = 1         ## moving window smoothing size 
    self.m_gaugeImgDial   = 'GaugeDialGreenRed.png' # dial image file name
    self.m_gaugeImgNeedle = 'GaugeNeedleAmber.png'  # needle image file name

  #
  ## \brief Configure gauge from keyword option values.
  ##
  ## \param kw      Keyword options.
  ##
  ## \return Modified keywords sans this specific class keywords.
  ##
  def configGauge(self, kw):
    passthru = {}

    for k,v in kw.iteritems():
      if k == "gauge_label":
        self.m_gaugeLabel = v
      elif k == "gauge_show_val":
        self.m_gaugeShowVal = bool(v)
      elif k == "gauge_val_min":
        self.m_gaugeValMin = float(v)
      elif k == "gauge_val_max":
        self.m_gaugeValMax = float(v)
      elif k == "gauge_val_home":
        self.m_gaugeValHome = float(v)
      elif k == "gauge_val_fmt":
        self.m_gaugeValFmt = str(v)
      elif k == "gauge_text_color":
        self.m_gaugeTextColor = str(v)
      elif k == "gauge_size":
        self.m_gaugeSize = int(v)
      elif k == "gauge_res":
        self.m_gaugeRes = float(v)
      elif k == "gauge_moving_win":
        self.m_gaugeMovingWin = int(v)
      elif k == "gauge_dial_image":
        self.m_gaugeImgDial = v
      elif k == "gauge_needle_image":
        self.m_gaugeImgNeedle = v
      else:
        passthru[k] = v

    self.m_gaugeValRange = self.m_gaugeValMax - self.m_gaugeValMin
    self.m_value  = self.m_gaugeValHome
    self.m_filter = self.m_gaugeMovingWin * [self.m_value/self.m_gaugeMovingWin]
    self.m_angle  = self.toGauge(self.m_value)
    self.m_dir    = 1

    return passthru

  #
  ## \brief Partial repurposing of the dial.
  ##
  ## The gauge dial images and sizes do not change.
  ##
  ## \param kw      Keyword options.
  #
  def repurpose(self, **kw):
    self.configGauge(kw)
    self.m_canvas.itemconfig(self.m_idLabel, fill=self.m_gaugeTextColor)
    self.m_canvas.itemconfig(self.m_idValue, fill=self.m_gaugeTextColor)
    self.showLabel(self.m_gaugeLabel)
    if self.m_gaugeShowVal:
      self.showValue(self.m_value)

  #
  ## \brief Create gui widgets with supporting data and show.
  #
  def createGauge(self):
    imageLoader         = ImageLoader(py_pkg="laelaps_control.images")
    scale               = 1.0
    wSize               = (300, 300)
    iOrigin             = (wSize[0]/2, wSize[1]/2)
    self.m_photoDial    = None
    self.m_photoNeedle  = None

    #
    # Dial face
    #
    self.m_imgDial = imageLoader.openImage(self.m_gaugeImgDial)

    if self.m_imgDial is not None:
      imgSize = self.m_imgDial.size
      if imgSize[0] != self.m_gaugeSize:
        scale   = float(self.m_gaugeSize) / float(imgSize[0])
        wSize   = (int(imgSize[0] * scale), int(imgSize[1] * scale))
        iOrigin  = (wSize[0]/2, wSize[1]/2)
        self.m_imgDial = self.m_imgDial.resize(wSize, Image.BICUBIC)
      self.m_photoDial = ImageTk.PhotoImage(self.m_imgDial)

    #
    # Dial needle
    #
    self.m_imgNeedle = imageLoader.openImage(self.m_gaugeImgNeedle)

    if self.m_imgNeedle is not None:
      self.m_imgNeedle = self.m_imgNeedle.convert('RGBA')
      if scale != 1.0:
        imgSize = self.m_imgNeedle.size
        size = (int(imgSize[0] * scale), int(imgSize[1] * scale))
        self.m_imgNeedle = self.m_imgNeedle.resize(size, Image.BICUBIC)
      self.m_photoNeedle = ImageTk.PhotoImage(self.m_imgNeedle)

    #
    # Canvas
    #
    self.m_canvas = Canvas(self, width=wSize[0], height=wSize[1])
    self.m_canvas.grid(row=0, column=0, padx=0, pady=0)

    # dial face
    if self.m_photoDial is not None:
      self.m_canvas.create_image(iOrigin, image=self.m_photoDial)

    # dial needle
    if self.m_photoNeedle is not None:
      self.m_idNeedle = self.m_canvas.create_image(iOrigin,
                                                image=self.m_photoNeedle)

    if scale > 0.35:
      fontSize = int(20.0 * scale) 
    else:
      fontSize = int(24.0 * scale) 
    if fontSize <= 10:
      fontWeight = "normal"
    else:
      fontWeight = "bold"
    helv = tkFont.Font(family="Helvetica", size=fontSize, weight=fontWeight)

    # dial label
    origin = (iOrigin[0], wSize[1]*0.82)
    self.m_idLabel = self.m_canvas.create_text(origin, text="",
                font=helv, justify=CENTER, fill=self.m_gaugeTextColor)

    if len(self.m_gaugeLabel) > 0:
      self.m_canvas.itemconfig(self.m_idLabel, text=self.m_gaugeLabel)

    # dial numeric value
    origin = (iOrigin[0], wSize[1]*0.70)
    self.m_idValue = self.m_canvas.create_text(origin, text="",
                font=helv, justify=CENTER, fill=self.m_gaugeTextColor)

    if self.m_gaugeShowVal:
      s = self.m_gaugeValFmt % (self.m_value)
      self.m_canvas.itemconfig(self.m_idValue, text=s)

  #
  def update(self, val):
    a = self.toGauge(val)
    #print 'val,angle', val, a
    self.rotate(a)
    if self.m_gaugeShowVal:
      self.showValue(self.m_value)

  #
  def toGauge(self, val):
    if val < self.m_gaugeValMin:
      val = self.m_gaugeValMin
    elif val > self.m_gaugeValMax:
      val = self.m_gaugeValMax
    self.m_value = self.smooth(val)
    # 0.0 - 1.0
    r = (self.m_value - self.m_gaugeValMin) / self.m_gaugeValRange
    # 135, -135
    return Dial.GaugeMaxAngle - Dial.GaugeRange * r

  #
  def smooth(self, val):
    if self.m_gaugeMovingWin <= 1:
      return float(val)
    else:
      lastVal = self.m_filter.pop(self.m_gaugeMovingWin - 1)
      newVal  = float(val) / self.m_gaugeMovingWin
      val     = self.m_value + newVal - lastVal
      self.m_filter.insert(0, newVal)
      return val

  #
  def rotate(self, angle):
    #print 'angle', angle
    if (angle >= self.m_angle - self.m_gaugeRes/2.0) and \
       (angle <= self.m_angle + self.m_gaugeRes/2.0):
      return
    if self.m_imgNeedle is not None:
      img = self.m_imgNeedle.rotate(angle, resample=Image.BICUBIC)
      self.m_photoNeedle = ImageTk.PhotoImage(img)
      self.m_canvas.itemconfig(self.m_idNeedle, image=self.m_photoNeedle)
    self.m_angle = angle

  #
  def showValue(self, val):
    s = self.m_gaugeValFmt % (self.m_value)
    self.m_canvas.itemconfig(self.m_idValue, text=s)

  #
  def showLabel(self, text):
    if len(self.m_gaugeLabel) > 0:
      self.m_canvas.itemconfig(self.m_idLabel, text=self.m_gaugeLabel)
    else:
      self.m_canvas.itemconfig(self.m_idLabel, text="")

  #
  def testGauge(self):
    if random.random() > 0.75:
      val = random.uniform(self.m_gaugeValMin, self.m_gaugeValMax)
      #print 'random1', val
    else:
      val = random.uniform(self.m_value-5.0, self.m_value+5.0)
      #print 'random2', val
    self.update(val)
    self.master.after(1000, self.testGauge)

  #
  def testGaugeRange(self, inc=5.0):
    if (self.m_dir > 0) and (self.m_angle+inc > Dial.GaugeMaxAngle):
      self.m_dir = -1
    elif (self.m_dir < 0) and (self.m_angle-inc < Dial.GaugeMinAngle):
      self.m_dir = 1
    a = self.m_angle + self.m_dir * inc
    self.rotate(a)
    self.master.after(1000, self.testGaugeRange)


# ------------------------------------------------------------------------------
# Guage Unit Test
# ------------------------------------------------------------------------------

if __name__ == '__main__':
  DialExtern = None

  def testRepurp():
    DialExtern.repurpose(gauge_label="Power", gauge_val_max=225.0,
        gauge_val_fmt="%.1f")

  root = Tk()

  root.protocol('WM_DELETE_WINDOW', root.destroy)

  win = Frame(root)
  win.master.title("Gauge Unit Test")
  win.grid(row=0, column=0, padx=5, pady=5)

  #
  # (Near) Defaults
  #
  dialDft = Dial(win,
      gauge_label="Gauge Dft", gauge_show_val=True, gauge_val_fmt="%d");
  dialDft.grid(row=0, column=0),

  #
  # Dashboard
  #
  wframe = LabelFrame(win, text="DashBoard", fg="#0000aa", relief="ridge",
      borderwidth=3)
  wframe.grid(row=0, column=1)

  dialPower = Dial(wframe,
      gauge_val_min=0.0, gauge_val_max=255.0, gauge_val_home=0,
      gauge_size=150, gauge_res=0.5, gauge_label="watts",
      gauge_show_val=True, gauge_moving_win=4,
      gauge_dial_image="GaugeDialGreenRed.png",
      gauge_needle_image="GaugeNeedleWhite.png",
      gauge_text_color="#ffffff")
  dialPower.grid(row=0, column=0);

  dialSpeed = Dial(wframe,
      gauge_val_min=0.0, gauge_val_max=25.0, gauge_val_home=0,
      gauge_size=150, gauge_res=0.5, gauge_label="m/s",
      gauge_show_val=True, gauge_moving_win=4,
      gauge_dial_image="GaugeDialGreenRed.png",
      gauge_needle_image="GaugeNeedleWhite.png",
      gauge_text_color="#ffffff")
  dialSpeed.grid(row=0, column=1);

  dialTemp = Dial(wframe,
      gauge_val_min=0.0, gauge_val_max=100.0, gauge_val_home=0,
      gauge_size=100, gauge_res=0.5, gauge_label=u"C\u00b0",
      gauge_show_val=True, gauge_moving_win=4,
      gauge_dial_image="GaugeDialBlueRed.png",
      gauge_needle_image="GaugeNeedleWhite.png",
      gauge_text_color="#ffffff")
  dialTemp.grid(row=1, column=0);

  #
  # Powertrain
  #
  wframe = LabelFrame(win, text="Powertrain", fg="#0000aa", relief="ridge",
      borderwidth=3)
  wframe.grid(row=0, column=2)

  dialPtPower = Dial(wframe,
      gauge_val_min=0.0, gauge_val_max=255.0, gauge_val_home=0,
      gauge_size=100, gauge_res=0.5, gauge_label="watts",
      gauge_show_val=True, gauge_moving_win=4,
      gauge_dial_image="GaugeDialGreenRed.png",
      gauge_needle_image="GaugeNeedleWhite.png",
      gauge_text_color="#ffffff")
  dialPtPower.grid(row=0, column=0);

  dialPtSpeed = Dial(wframe,
      gauge_val_min=0.0, gauge_val_max=25.0, gauge_val_home=0,
      gauge_size=100, gauge_res=0.5, gauge_label="qpps",
      gauge_show_val=True, gauge_moving_win=4,
      gauge_dial_image="GaugeDialRedGreenRed.png",
      gauge_needle_image="GaugeNeedleWhite.png",
      gauge_text_color="#ffffff")
  dialPtSpeed.grid(row=0, column=1);

  #
  # Test operaton
  #
  #dial.testGaugeRange()
  #dialDft.testGauge()
  win.after(1000, dialDft.testGauge)
  win.after(1000, dialPower.testGauge)
  win.after(1000, dialSpeed.testGauge)

  DialExtern = dialDft
  win.after(10000, testRepurp)


  win.mainloop()
