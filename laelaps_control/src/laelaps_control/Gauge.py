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
    self.m_images         = {}    # must keep loaded images referenced
    self.m_gaugeLabel     = ""
    self.m_gaugeMin       = 0
    self.m_gaugeMax       = 100
    self.m_gaugeHome      = 0
    for k,v in kw.iteritems():
      if k == "gauge_min":
        self.m_guageMin = v
      elif k == "gauge_max":
        self.m_guageMax = v
      else:
        continue
      del kw[k]
    return kw

  #
  ## \brief Create gui widgets with supporting data and show.
  #
  def createGauge(self):
    imageLoader = ImageLoader(py_pkg="laelaps_control.images")

    self.m_canvas = Canvas(self, width=300, height=300)
    self.m_canvas.grid(row=0, column=0)

    self.m_imgDial = imageLoader.openImage("GaugeDialGreenRed.png")

    if self.m_imgDial is not None:
      #self.m_imgDial = self.m_imgDial.resize((50,50), Image.BICUBIC)
      self.m_photoDial = ImageTk.PhotoImage(self.m_imgDial)

    # dial
    if self.m_photoDial is not None:
      self.m_canvas.create_image(150, 150, image=self.m_photoDial)

    self.m_imgNeedle = imageLoader.openImage("GaugeNeedleDarkGray.png")

    if self.m_imgNeedle is not None:
      self.m_imgNeedle = self.m_imgNeedle.convert('RGBA')
      #self.m_imgNeedle = self.m_imgNeedle.resize((50,50))

      self.m_angle = 0.0

      self.m_photoNeedle = ImageTk.PhotoImage(self.m_imgNeedle)
      self.m_idNeedle = self.m_canvas.create_image(150, 150,
          image=self.m_photoNeedle)

  def rotate(self, angle):
    if self.m_imgNeedle is not None:
      img = self.m_imgNeedle.rotate(angle)
      self.m_photoNeedle = ImageTk.PhotoImage(img)
      self.m_canvas.itemconfig(self.m_idNeedle, image=self.m_photoNeedle)

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
