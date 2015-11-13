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
  #
  ## \brief Constructor.
  ##
  ## \param cnf     Configuration dictionary.
  ## \param kw      Keyword options.
  #
  def __init__(self, master=None, cnf={}, **kw):
    # initialize dialog data
    kw = self.initData(kw)

    Frame.__init__(self, master=master, cnf=cnf, **kw)

    self.createGauge()

  #
  ## \brief Initialize class state data.
  ##
  ## \param kw      Keyword options.
  ##
  ## \return Modified keywords sans this specific class.
  ##
  def initData(self, kw):
    # defaults
    self.m_gaugeLabel     = ""
    self.m_gaugeShowVal   = False
    self.m_gaugeMin       = 0
    self.m_gaugeMax       = 100
    self.m_gaugeHome      = 0
    self.m_gaugeSize      = 300

    for k,v in kw.iteritems():
      if k == "gauge_label":
        self.m_gaugeLabel = v
      elif k == "gauge_show_val":
        self.m_gaugeMin = v
      elif k == "gauge_min":
        self.m_gaugeMin = v
      elif k == "gauge_max":
        self.m_gaugeMax = v
      elif k == "gauge_home":
        self.m_gaugeHome = v
      elif k == "gauge_size":
        self.m_gaugeSize = v
      else:
        continue
      del kw[k]
    return kw

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
    self.m_angle        = 0.0

    #
    # Dial face
    #
    self.m_imgDial = imageLoader.openImage("GaugeDialGreenRed.png")

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
    self.m_imgNeedle = imageLoader.openImage("GaugeNeedleWhite.png")

    if self.m_imgNeedle is not None:
      self.m_imgNeedle = self.m_imgNeedle.convert('RGBA')
      if scale != 1.0:
        imgSize = self.m_imgDial.size
        size = (int(imgSize[0] * scale), int(imgSize[1] * scale))
        self.m_imgDial = self.m_imgDial.resize(size, Image.BICUBIC)
      self.m_photoNeedle = ImageTk.PhotoImage(self.m_imgNeedle)

    #
    # Canvas
    #
    self.m_canvas = Canvas(self, width=wSize[0], height=wSize[1])
    self.m_canvas.grid(row=0, column=0, padx=0, pady=0)

    if self.m_photoDial is not None:
      self.m_canvas.create_image(iOrigin, image=self.m_photoDial)
    if self.m_photoNeedle is not None:
      self.m_idNeedle = self.m_canvas.create_image(iOrigin,
                                                image=self.m_photoNeedle)

  #
  def rotate(self, angle):
    if self.m_imgNeedle is not None:
      img = self.m_imgNeedle.rotate(angle)
      self.m_photoNeedle = ImageTk.PhotoImage(img)
      self.m_canvas.itemconfig(self.m_idNeedle, image=self.m_photoNeedle)

  #
  def testGauge(self):
    self.m_angle += 1.0
    print 'testGauge', self.m_angle
    self.rotate(self.m_angle)
    self.master.after(1000, self.testGauge)

# ------------------------------------------------------------------------------
# Guage Unit Test
# ------------------------------------------------------------------------------

if __name__ == '__main__':
  root = Tk()

  root.protocol('WM_DELETE_WINDOW', root.destroy)

  win = Frame(root)
  win.master.title("Gauge Unit Test")
  win.grid(row=0, column=0)

  dial = Dial(master=win)
  dial.grid(row=0, column=0);

  dial.testGauge()

  win.mainloop()
