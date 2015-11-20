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
## \brief Laelaps dial gauge class
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
  ## \brief Create gui widgets.
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
# Class Counter
# ------------------------------------------------------------------------------

#
## \brief Laelaps counter gauge class.
##
class Counter(Frame):

  #
  ## \brief Constructor.
  ##
  ## \param cnf     Configuration dictionary.
  ## \param kw      Keyword options.
  #
  def __init__(self, master=None, cnf={}, **kw):
    self.m_imgFileName = {
      ' ':  'OdBlank.png', '-': 'OdMinus.png', '+': 'OdPlus.png',
      '.':  'OdDot.png',
      0: 'Od0.png', 1: 'Od1.png', 2: 'Od2.png', 3: 'Od3.png',
      4: 'Od4.png', 5: 'Od5.png', 6: 'Od6.png', 7: 'Od7.png',
      8: 'Od8.png', 9: 'Od9.png' }

    self.m_img          = {}      ## loaded images
    self.m_photo        = {}      ## converted to tkinter photo images
    self.m_idImg        = {}      ## canvas image ids
    self.m_tumbler      = {}      ## current counter tumbler values
    self.m_numIDigits   = 2       ## number of integer part digits
    self.m_numFDigits   = 0       ## number of fraction part digits
    self.m_isCreated    = False   ## counter is [not] created

    self.setDefaults()

    kw = self.configGauge(kw)

    Frame.__init__(self, master=master, cnf=cnf, **kw)

    self.createGauge()

  #
  def setDefaults(self):
    # defaults
    self.m_gaugeLabel     = ""        ## show fixed label below gauge counter
    self.m_gaugeTextColor = "#000000" ## gauge label display color
    self.m_gaugeValMin    = 0.0       ## gauge minimum value
    self.m_gaugeValMax    = 100.0     ## gauge maximum value
    self.m_gaugeValHome   = 0.0       ## gauge home value
    self.m_gaugeSize      = None      ## gauge component (width,height)
    self.m_gaugeRes       = 1.0       ## gauge fraction resolution
                                      ##    (1.0, 0.1, 0.01, ...)

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
      elif k == "gauge_text_color":
        self.m_gaugeTextColor = str(v)
      elif k == "gauge_val_min":
        self.m_gaugeValMin = float(v)
      elif k == "gauge_val_max":
        self.m_gaugeValMax = float(v)
      elif k == "gauge_val_home":
        self.m_gaugeValHome = float(v)
      elif k == "gauge_size":
        self.m_gaugeSize = v
      elif k == "gauge_res":
        self.m_gaugeRes = float(v)
      else:
        passthru[k] = v

    self.m_value  = self.m_gaugeValHome

    return passthru

  #
  ## \brief Create gui widgets.
  #
  def createGauge(self):
    imageLoader         = ImageLoader(py_pkg="laelaps_control.images")
    chkSize             = False

    #
    # Counter blank
    #
    if self.m_gaugeSize is not None and len(self.m_gaugeSize) == 2:
      tgtSize = (self.m_gaugeSize[0], self.m_gaugeSize[1])
      chkSize = True

    #
    # Load and transform counter digit images
    #
    for key,fname in self.m_imgFileName.iteritems():
      self.m_img[key] = imageLoader.openImage(fname)
      if self.m_img[key] is None:
        print "Error: Gauge.Counter: Cannot open image %s." % (fname)
        return
      if chkSize:
        imgSize = self.m_img[key].size
        if key == '.':
          scale = float(tgtSize[1]) / float(imgSize[1])
          tgtDotSize = (int(imgSize[0] * scale), int(imgSize[1] * scale))
          if imgSize != tgtDotSize:
            self.m_img[key] = self.m_img[key].resize(tgtDotSize, Image.BICUBIC)
        elif imgSize != tgtSize:
          self.m_img[key] = self.m_img[key].resize(tgtSize, Image.BICUBIC)
      self.m_photo[key] = ImageTk.PhotoImage(self.m_img[key])

    #print self.m_img.keys()
    #print self.m_photo.keys()

    #
    # Number of integer digits to display.
    #
    if self.m_gaugeValMin != 0.0:
      ndigits = int(math.ceil(math.log10(abs(self.m_gaugeValMin))))
    else:
      ndigits = 1
    if self.m_gaugeValMax != 0.0:
      pdigits = int(math.ceil(math.log10(abs(self.m_gaugeValMax))))
    else:
      pdigits = 1
    if pdigits >= ndigits:
      self.m_numIDigits = pdigits
    else:
      self.m_numIDigits = ndigits
    if self.m_numIDigits < 1:
      self.m_numIDigits = 1

    #
    # Number of fraction digits to display.
    #
    self.m_numFDigits = int(math.ceil(abs(math.log10(abs(self.m_gaugeRes)))))
    
    #
    # Do [not] include sign.
    #
    if self.m_gaugeValMin < 0.0:
      self.m_hasSign = True
    else:
      self.m_hasSign = False

    #
    # Dimensions
    #
    tumblerSize   = self.m_img[' '].size
    canvasWidth   = tumblerSize[0] * self.m_numIDigits
    canvasHeight  = tumblerSize[1]

    if self.m_hasSign:
      canvasWidth += tumblerSize[0]

    if self.m_numFDigits > 0:
      dotSize = self.m_img['.'].size
      canvasWidth += dotSize[0]
      canvasWidth += tumblerSize[0] * self.m_numFDigits

    if len(self.m_gaugeLabel) > 0:
      #label_width = len(self.m_gaugeLabel) * 13.0 # approx. 10pt
      textLen = len(self.m_gaugeLabel)
      fontMargin  = 1
      fontSize = int(canvasHeight * 0.67)
      if fontSize <= 10:
        fontWeight = "normal"
      else:
        fontWeight = "bold"
      labelWidth = textLen * fontSize + 2 * fontMargin
      helv = tkFont.Font(family="Helvetica", size=-fontSize, weight=fontWeight)
      canvasWidth += labelWidth
    else:
      fontSize    = 0
      fontMargin  = 0

    #
    # Canvas
    #
    self.m_canvas = Canvas(self, width=canvasWidth, height=canvasHeight)
    self.m_canvas.grid(row=0, column=0, padx=0, pady=0)

    # starting image origin
    x = tumblerSize[0] / 2
    y = tumblerSize[1] / 2

    #
    # Create canvas counter tumblers, left to right
    #

    # sign
    if self.m_hasSign:
      self.m_idImg['sign'] = self.m_canvas.create_image((x, y),
                                                image=self.m_photo['+'])
      self.m_tumbler['sign'] = '+'
      x += tumblerSize[0]

    # integer digits
    place = self.m_numIDigits - 1
    for i in range(0, self.m_numIDigits):
      self.m_idImg[place] = self.m_canvas.create_image((x, y),
                                                image=self.m_photo[0])
      self.m_tumbler[place] = 0
      #print 'i', place
      x  += tumblerSize[0]
      place -= 1

    # decimal point
    if self.m_numFDigits > 0:
      x -= tumblerSize[0] / 2
      x += dotSize[0] / 2
      self.m_idImg['.'] = self.m_canvas.create_image((x, y),
                                                image=self.m_photo['.'])
      x += dotSize[0] / 2 + tumblerSize[0] / 2

    # faction digits
    place = -1
    for i in range(0, self.m_numFDigits):
      #print 'f', place
      self.m_idImg[place] = self.m_canvas.create_image((x, y),
                                                image=self.m_photo[0])
      self.m_tumbler[place] = 0
      x  += tumblerSize[0]
      place -= 1

    #print self.m_idImg.keys()
    #print self.m_tumbler.keys()

    # counter label
    if len(self.m_gaugeLabel) > 0:
      x = canvasWidth - labelWidth/2
      y = fontMargin + fontSize/2
      self.m_idLabel = self.m_canvas.create_text((x, y), text=self.m_gaugeLabel,
                font=helv, justify=CENTER, fill=self.m_gaugeTextColor)

    self.m_isCreated  = True

  #
  def update(self, val):
    if self.m_isCreated:
      self.turn(val)

  #
  def turn(self, val):
    if val < self.m_gaugeValMin:
      val = self.m_gaugeValMin
    elif val > self.m_gaugeValMax:
      val = self.m_gaugeValMax
    if (val >= self.m_value - self.m_gaugeRes/2.0) and \
       (val <= self.m_value + self.m_gaugeRes/2.0):
      return

    self.m_value = val
    iPart = int(val)
    fPart = val - iPart
    #print self.m_value, iPart, fPart

    # optional sign
    if self.m_hasSign:
      if self.m_value >= 0:
        sign = '+'
      else:
        sign = '-'
      if self.m_tumbler['sign'] != sign:
        self.m_canvas.itemconfig(self.m_idImg['sign'], image=self.m_photo[sign])
        self.m_tumbler['sign'] = sign

    # integer part
    place = 0
    for i in range(0, self.m_numIDigits):
      tmp   = iPart/10
      d     = iPart - tmp * 10
      iPart = tmp
      if self.m_tumbler[place] != d:
        self.m_canvas.itemconfig(self.m_idImg[place], image=self.m_photo[d])
        self.m_tumbler[place] = d
      place += 1

    # faction part
    place = -1
    for i in range(0, self.m_numFDigits):
      tmp = fPart * 10.0
      d = int(tmp)
      fPart = tmp - d
      if self.m_tumbler[place] != d:
        self.m_canvas.itemconfig(self.m_idImg[place], image=self.m_photo[d])
        self.m_tumbler[place] = d
      place -= 1

  #
  def testGauge(self):
    val = random.uniform(self.m_gaugeValMin, self.m_gaugeValMax)
    self.update(val)
    self.master.after(1000, self.testGauge)

  #
  def testGaugeCountingUp(self):
    delta = random.uniform(0.0, 100.0)
    self.update(self.m_value+delta)
    self.master.after(1000, self.testGaugeCountingUp)


# ------------------------------------------------------------------------------
# Class Battery
# ------------------------------------------------------------------------------

#
## \brief Laelaps battery charge gauge class.
##
class Battery(Frame):
  GaugeMinCharge =   0.0
  GaugeMaxCharge = 100.0
  GaugeResCharge =  20.0

  #
  ## \brief Constructor.
  ##
  ## \param cnf     Configuration dictionary.
  ## \param kw      Keyword options.
  #
  def __init__(self, master=None, cnf={}, **kw):
    self.m_imgFileName = {
      'batt_0':       'Battery0.png',
      'batt_20':      'Battery20.png',
      'batt_40':      'Battery40.png',
      'batt_60':      'Battery60.png',
      'batt_80':      'Battery80.png',
      'batt_100':     'Battery100.png',
      'batt_chg_0':   'BatteryCharge0.png',
      'batt_chg_20':  'BatteryCharge20.png',
      'batt_chg_40':  'BatteryCharge40.png',
      'batt_chg_60':  'BatteryCharge60.png',
      'batt_chg_80':  'BatteryCharge80.png',
      'batt_chg_100': 'BatteryCharge100.png',
    }

    self.m_img          = {}      ## loaded images
    self.m_photo        = {}      ## converted to tkinter photo images
    self.m_idBattImg    = None    ## canvas image ids
    self.m_keyBatt      = None    ## battery key
    self.m_value        = 0.0     ## battery charge value
    self.m_isCharging   = False   ## battery is [not] charging
    self.m_isCreated    = False   ## counter is [not] created

    self.m_testDir      = 1

    self.setDefaults()

    kw = self.configGauge(kw)

    Frame.__init__(self, master=master, cnf=cnf, **kw)

    self.createGauge()

  #
  def setDefaults(self):
    # defaults
    self.m_gaugeLabel     = ""        ## show fixed label below gauge counter
    self.m_gaugeTextColor = "#000000" ## gauge label display color
    self.m_gaugeSize      = None      ## gauge size (width,height)

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
      elif k == "gauge_text_color":
        self.m_gaugeTextColor = str(v)
      elif k == "gauge_size":
        self.m_gaugeSize = v
      else:
        passthru[k] = v

    return passthru

  #
  ## \brief Create gui widgets.
  #
  def createGauge(self):
    imageLoader         = ImageLoader(py_pkg="laelaps_control.images")
    chkSize             = False

    #
    # Counter blank
    #
    if self.m_gaugeSize is not None and len(self.m_gaugeSize) == 2:
      tgtSize = (self.m_gaugeSize[0], self.m_gaugeSize[1])
      chkSize = True

    #
    # Load and transform battery images
    #
    for key,fname in self.m_imgFileName.iteritems():
      self.m_img[key] = imageLoader.openImage(fname)
      if self.m_img[key] is None:
        print "Error: Gauge.Counter: Cannot open image %s." % (fname)
        return
      if chkSize:
        imgSize = self.m_img[key].size
        if imgSize != tgtSize:
          self.m_img[key] = self.m_img[key].resize(tgtSize, Image.BICUBIC)
      self.m_photo[key] = ImageTk.PhotoImage(self.m_img[key])

    self.m_keyBatt = 'batt_0'

    #
    # Dimensions
    #
    battImgSize   = self.m_img[self.m_keyBatt].size
    canvasWidth   = battImgSize[0]
    canvasHeight  = battImgSize[1]

    if len(self.m_gaugeLabel) > 0:
      label_width = len(self.m_gaugeLabel) * 13.0 # approx. 10pt
      scale = canvasWidth / label_width * 0.90
      fontSize = int(13.0 * scale) 
      if fontSize <= 10:
        fontWeight = "normal"
      else:
        fontWeight = "bold"
      fontMargin  = 1
      helv = tkFont.Font(family="Helvetica", size=-fontSize, weight=fontWeight)
      canvasHeight += fontSize + 2 * fontMargin
    else:
      fontSize    = 0
      fontMargin  = 0

    #
    # Canvas
    #
    self.m_canvas = Canvas(self, width=canvasWidth, height=canvasHeight)
    self.m_canvas.grid(row=0, column=0, padx=0, pady=0)

    # battery
    x = canvasWidth / 2
    y = canvasHeight / 2
    self.m_idImgBatt = self.m_canvas.create_image((x, y),
                                            image=self.m_photo[self.m_keyBatt])

    self.m_isCreated  = True

  #
  def update(self, val, isCharging=False):
    if self.m_isCreated:
      val = self.toGauge(val)
      key = self.toKey(val, isCharging)
      if key != self.m_keyBatt:
        self.m_canvas.itemconfig(self.m_idImgBatt, image=self.m_photo[key])
        self.m_keyBatt = key
      self.m_value      = val
      self.m_isCharging = isCharging

  #
  def toGauge(self, val):
    if val < Battery.GaugeMinCharge:
      val = Battery.GaugeMinCharge
    elif val > Battery.GaugeMaxCharge:
      val = Battery.GaugeMaxCharge
    return val

  #
  def toKey(self, val, isCharging):
    key = self.m_keyBatt
    rngmin = int(Battery.GaugeMinCharge)
    rngmax = int(Battery.GaugeMaxCharge+Battery.GaugeResCharge)
    for res in range(rngmin, rngmax, int(Battery.GaugeResCharge)):
      if val <= res:
        if isCharging:
          key = "batt_chg_%d" % (int(res))
        else:
          key = "batt_%d" % (int(res))
        return key
    return key

  #
  def testGauge(self):
    ndir = 0
    if self.m_value >= Battery.GaugeMaxCharge:
      ndir = -1
      isCharging = False
    elif self.m_value <= Battery.GaugeMinCharge:
      ndir = 1
      isCharging = True
    delta = random.uniform(0, Battery.GaugeResCharge/2)
    if ndir == 1:
      self.m_testDir = ndir
      isCharging = True
    elif ndir == -1:
      self.m_testDir = ndir
      isCharging = False
    else:
      isCharging = self.m_isCharging
    val = self.m_value + self.m_testDir * delta
    self.update(val, isCharging)
    self.master.after(1000, self.testGauge)


# ------------------------------------------------------------------------------
# Class Indicator
# ------------------------------------------------------------------------------

#
## \brief Laelaps indicator light gauge class.
##
class Indicator(Frame):

  #
  ## \brief Constructor.
  ##
  ## \param cnf     Configuration dictionary.
  ## \param kw      Keyword options.
  #
  def __init__(self, master=None, cnf={}, **kw):
    self.m_imgFileName = {
      'gray':   'IndicatorGray.png',
      'green':  'IndicatorGreen.png',
      'yellow': 'IndicatorYellow.png',
      'amber':  'IndicatorAmber.png',
      'red':    'IndicatorRed.png',
    }

    self.m_indValues  = ['gray', 'green', 'yellow', 'amber', 'red']

    self.m_img        = {}      ## loaded images
    self.m_photo      = {}      ## converted to tkinter photo images
    self.m_idIndImg   = None    ## canvas image ids
    self.m_value      = 'gray'  ## battery charge value
    self.m_fontText   = tkFont.Font(family="Helvetica", size=-11, weight="bold")
    self.m_isCreated  = False   ## counter is [not] created

    self.setDefaults()

    kw = self.configGauge(kw)

    Frame.__init__(self, master=master, cnf=cnf, **kw)

    self.createGauge()

  #
  def setDefaults(self):
    # defaults
    self.m_gaugeLabel     = ""        ## show fixed label below gauge counter
    self.m_gaugeTextColor = "#ffffff" ## gauge label display color
    self.m_gaugeValHome   = 'gray'    ## gauge home value
    self.m_gaugeSize      = 48        ## gauge size (pixels)

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
      elif k == "gauge_text_color":
        self.m_gaugeTextColor = str(v)
      elif k == "gauge_size":
        self.m_gaugeSize = int(v)
      elif k == "gauge_val_home":
        self.m_gaugeValHome = str(v)
      else:
        passthru[k] = v

    self.m_value  = self.m_gaugeValHome

    return passthru

  #
  ## \brief Create gui widgets.
  #
  def createGauge(self):
    imageLoader         = ImageLoader(py_pkg="laelaps_control.images")

    #
    # Load and transform battery images
    #
    for key,fname in self.m_imgFileName.iteritems():
      self.m_img[key] = imageLoader.openImage(fname)
      if self.m_img[key] is None:
        print "Error: Gauge.Counter: Cannot open image %s." % (fname)
        return
      imgSize = self.m_img[key].size
      if imgSize[0] != self.m_gaugeSize:
        tgtSize = (self.m_gaugeSize, self.m_gaugeSize)
        self.m_img[key] = self.m_img[key].resize(tgtSize, Image.BICUBIC)
      self.m_photo[key] = ImageTk.PhotoImage(self.m_img[key])

    canvasWidth   = imgSize[0]
    canvasHeight  = imgSize[1]

    #
    # Canvas
    #
    self.m_canvas = Canvas(self, width=canvasWidth, height=canvasHeight)
    self.m_canvas.grid(row=0, column=0, padx=0, pady=0)

    # indicator 
    x = canvasWidth / 2
    y = canvasHeight / 2

    self.m_idIndImg = self.m_canvas.create_image((x, y),
                                            image=self.m_photo[self.m_value])

    # label
    text = ''
    if len(self.m_gaugeLabel) > 0:
      text = self.m_gaugeLabel

    x = canvasWidth/2
    y = canvasHeight/2
    self.m_idLabel = self.m_canvas.create_text((x, y), text=text,
                font=self.m_fontText, justify=CENTER,
                fill=self.m_gaugeTextColor)

    self.m_isCreated  = True

  #
  def update(self, val, new_label=None):
    if self.m_isCreated:
      if val not in self.m_imgFileName.keys():
        print "Warning: %s: unknown indicator." % (val)
        val = 'gray'
      if val != self.m_value:
        self.m_canvas.itemconfig(self.m_idIndImg, image=self.m_photo[val])
        self.m_value = val
      if new_label is not None and (new_label != self.m_gaugeLabel):
        self.m_canvas.itemconfig(self.m_idLabel, text=new_label)
        self.m_gaugeLabel = new_label

  #
  def testGauge(self):
    val = random.choice(self.m_imgFileName.keys())
    self.update(val, val)
    self.master.after(1000, self.testGauge)


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
  wframe = LabelFrame(win, text="Defaults", fg="#0000aa", relief="ridge",
      borderwidth=3)
  wframe.grid(row=0, column=0)

  dialDft = Dial(wframe,
      gauge_label="Gauge Dft", gauge_show_val=True, gauge_val_fmt="%d");
  dialDft.grid(row=0, column=0),

  counterDft = Counter(wframe, gauge_size=(100, 150))
  counterDft.grid(row=1, column=0);

  indDft = Indicator(wframe, gauge_label='Test')
  indDft.grid(row=0, column=4)

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

  subframe = Frame(wframe, relief="flat")
  subframe.grid(row=1, column=1)

  counterMeters = Counter(subframe, gauge_size=(20, 30),
      gauge_val_min=0.0, gauge_val_max=1000000.0, gauge_res=0.01,
      gauge_label="m")
  counterMeters.grid(row=0, column=0);

  counterTrip = Counter(subframe, gauge_size=(20, 30),
      gauge_val_min=0.0, gauge_val_max=1000.0, gauge_res=0.01,
      gauge_label="trip")
  counterTrip.grid(row=1, column=0);

  batt = Battery(wframe, gauge_size=(75, 100))
  batt.grid(row=0, column=3)

  indMode = Indicator(wframe, gauge_label='Manual')
  indMode.grid(row=0, column=4)

  indMotors = Indicator(wframe, gauge_label='Motors')
  indMotors.grid(row=1, column=4)

  indMoving = Indicator(wframe, gauge_label='Moving')
  indMoving.grid(row=0, column=5)

  indMotors = Indicator(wframe, gauge_label='Alarms')
  indMotors.grid(row=1, column=5)

  indEStop = Indicator(wframe, gauge_label='EStop')
  indEStop.grid(row=0, column=6)

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

  counterPulses = Counter(wframe, gauge_size=(10, 15),
      gauge_val_min=-4000000000, gauge_val_max=4000000000,
      gauge_label="p")
      #relief='ridge', borderwidth=1, bg="#000000")
  counterPulses.grid(row=1, column=0, columnspan=2);

  #
  # Test operaton
  #
  #dial.testGaugeRange()
  #dialDft.testGauge()
  win.after(1000, dialDft.testGauge)
  win.after(1200, counterDft.testGauge)
  win.after(3013, indDft.testGauge)

  win.after(900, dialPower.testGauge)
  win.after(2033, dialSpeed.testGauge)
  win.after(193, counterMeters.testGaugeCountingUp)
  win.after(2193, batt.testGauge)

  win.after(894, counterPulses.testGauge)

  DialExtern = dialDft
  win.after(10000, testRepurp)


  win.mainloop()
