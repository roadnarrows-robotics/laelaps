###############################################################################
#
# Package:   RoadNarrows Robotics Laelaps Robotic Mobile Platform ROS Package
#
# Link:      https://github.com/roadnarrows-robotics/laelaps
#
# ROS Node:  laelaps_panel, laelaps_range
#
# File: RangeSensorWin.py
#
## \file 
##
## \brief Laelaps range sensor group panel.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright:
##   (C) 2017  RoadNarrows LLC.\n
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
import math
import getopt

from Tkinter import *
from Tkconstants import *
from tkFileDialog import *
import tkFont

import random

import roslib; roslib.load_manifest('laelaps_control')
import rospy

from industrial_msgs.msg import TriState

from laelaps_control.msg import RangeState          # message

from laelaps_control.Utils import *


# ------------------------------------------------------------------------------
# Globals
# ------------------------------------------------------------------------------

## \brief Additional image search paths.
imagePath = [
  "/prj/share/appkit/images",
  "/usr/local/share/appkit/images"
]

## \brief Common foreground colors.
fgColors = {
  'normal':   'black',
  'ok':       '#008800',
  'focus':    '#0000aa',
  'warning':  '#aa6600',
  'error':    '#cc0000'
}

## \brief Laelaps relevant sensor info.
#
# Tuple:
#   name      Name and key of Laelaps range sensor.
#   beamDir   Direction of beam, with 0 degrees being the top and front of
#             Laelaps.
#   boff      Pixel offset from a conceptualized Laelaps centered at 0,0 and
#             with pixel dimensions of 300x300. Scale and translate as needed.
#   toff      Text position offset from calculated beam pixel origin.
# 
SensorInfo = [
  ('front',       degToRad(0.0),    (   0, -120), (  0, -10)),
  ('left_front',  degToRad(10.0),   ( -50, -120), (  0, -10)),
  ('left',        degToRad(90.0),   ( -90,    0), (-20,   0)),
  ('left_rear',   degToRad(170.0),  ( -50,  130), (  0,  10)),
  ('rear',        degToRad(180.0),  (   0,  130), (  0,  10)),
  ('right_rear',  degToRad(190.0),  (  50,  130), (  0,  10)),
  ('right',       degToRad(270.0),  (  90,    0), ( 20,   0)),
  ('right_front', degToRad(350.0),  (  50, -120), (  0, -10))
]

## \brief Laelaps sensor visualization 'structure'.
#
class SensorViz:
  BeamMinDist       = 0.0         # beam minimum distance (meters)
  BeamMaxDist       = 0.200       # beam maximum distance (meters)
  BeamNoObj         = -1.0        # no object detected
  BrightMax         = 100         # maximum beam brightness as percent
  BrightMin         =   0         # minimum beam brightness as percent
  BrightIncStepSize =  10         # increment beam brightness step size
  BrightDecStepSize =  10         # decrement beam brightness step size
  SlidingWinMaxSize =   5         # sliding window maximum size
  CanvasBg          = '#333333'   # canvas background color
  BeamColor         = [           # sensor beam color
    #'#333333',  #   0%
    '#403333',  #   0%
    '#473333',  #  10%
    '#5b3333',  #  20%
    '#703333',  #  30%
    '#843333',  #  40%
    '#993333',  #  50%
    '#ad3333',  #  60%
    '#c13333',  #  70%
    '#d63333',  #  80%
    '#ea3333',  #  90%
    '#ff3333'   # 100%
  ]
  TextBgColor = '#333333'
  TextFgColor = '#ffffff'
  RobotBodyDim = (0.350, 0.250) # Laelaps body length x width dimensions(meters)


# ------------------------------------------------------------------------------
# Class RangeSensorWin
# ------------------------------------------------------------------------------

## \brief Range sensor group window class.
#
class RangeSensorWin(Toplevel):
  #
  ## \brief Constructor.
  ##
  ## \param master  Window parent master widget.
  ## \param cnf     Configuration dictionary.
  ## \param kw      Keyword options.
  #
  def __init__(self, master=None, cnf={}, **kw):
    self.m_isCreated = True

    Toplevel.__init__(self, master=master, cnf=cnf, **kw)

    self.title("laelaps_ranges")

    self.wm_protocol("WM_DELETE_WINDOW", lambda: self.onDeleteChild(self))

    self.m_win = RangeSensorFrame(master=self, cnf=cnf, **kw)

    self.m_win.grid(row=0, column=0, padx=5, pady=5)

    # close button
    w = Button(self, width=10, text='Close',
        command=lambda: self.onDeleteChild(self), anchor=CENTER)
    w.grid(row=1, column=0, sticky=N, pady=5)

    self.m_bttnClose = w

    self.lift()

  #
  ## \brief On delete callback.
  ##
  ## \param w   Widget (not used).
  #
  def onDeleteChild(self, w):
    if self.m_isCreated:
      self.m_isCreated = False
      self.destroy()


# ------------------------------------------------------------------------------
# Class RangeSensorFrame
# ------------------------------------------------------------------------------

## \brief Range sensor group frame class.
#
class RangeSensorFrame(Frame):
  #
  ## \brief Constructor.
  ##
  ## \param master  Window parent master widget.
  ## \param cnf     Configuration dictionary.
  ## \param kw      Keyword options.
  #
  def __init__(self, master=None, cnf={}, **kw):
    self.m_isCreated = True

    self.m_parent = master

    # intialize window data
    kw = self.initData(kw)

    self.m_imageLoader = ImageLoader(py_pkg='laelaps_control.images',
                                      image_paths=imagePath)

    Frame.__init__(self, master=master, cnf=cnf, **kw)

    self.m_icons['app_icon'] = \
                  self.m_imageLoader.loadImage("icons/LaelapsRangeIcon.png")

    if self.m_icons['app_icon'] is not None:
      self.master.tk.call('wm', 'iconphoto',
          self.master._w, self.m_icons['app_icon'])

    # craete and show widgets
    self.createWidgets()

    self.grid(row=0, column=0, padx=5, pady=5)

    # subscribe to extended robot status data
    self.m_sub = rospy.Subscriber("laelaps_control/range_state", 
                     RangeState, 
                     callback=self.updateSensorData) 

  #
  ## \brief Initialize class state data.
  ##
  ## Any keywords for this application specific window that are not supported 
  ## by the Frame Tkinter class must be removed.
  ##
  ## \param kw      Keyword options.
  ##
  ## \return Modified keywords sans this specific class.
  #
  def initData(self, kw):
    self.m_debug          = False # default debug level
    self.m_icons          = {}    # must keep loaded icons referenced
    self.m_range          = {}    # range sensor state

    if kw.has_key('debug'):
      self.m_debug = kw['debug']
      del kw['debug']

    # variables only used for debugging
    if self.m_debug:
      pass

    for sensor in SensorInfo:
      self.m_range[sensor[0]] = {
          'raw_value':      SensorViz.BeamNoObj,
          'filtered_value': SensorViz.BeamNoObj,
          'sliding_win':    [],
          'sum_total':      0,
          'brightness':     SensorViz.BrightMin,
          'beam_dir':       sensor[1],
          'boff':           sensor[2],
          'toff':           sensor[3]}

    return kw

  #
  ## \brief Create gui widgets with supporting data and show.
  #
  def createWidgets(self):
    self.createHeading(self, 0, 0)
    self.createSensorPanel(self, 1, 0)

  #
  ## \brief Create top gui heading.
  ##
  ## \param parent    Parent container widget.
  ## \param row       Row in parent widget.
  ## \param col       Column in parent widget.
  #
  def createHeading(self, parent, row, col):
    # top heading
    w = Label(parent)
    w['font']   = ('Helvetica', 16)
    w['text']   = 'Laelaps Range Sensor Group'
    w['anchor'] = CENTER
    w.grid(row=row, column=col, sticky=E+W)

  #
  ## \brief Create subsystem state lower center panel headers.
  ##
  ## \param parent  Parent widget
  ## \param row     Row in parent widget.
  ## \param col     Column in parent widget.
  #
  def createSensorPanel(self, parent, row, col):
    wframe = Frame(parent)
    wframe['borderwidth'] = 2
    wframe['relief'] = 'ridge'
    wframe.grid(row=row, column=col, padx=1, pady=3, sticky=N+W+E)

    helv  = tkFont.Font(family="Helvetica",size=10,weight="bold")
    padx  = 10
    pady  = 3
    row   = 0
    col   = 0

    # Center top-down view
    self.m_icons['laelaps_top_down'] = \
        self.m_imageLoader.loadImage("LaelapsTopDown300.png")

    self.m_canvas = Canvas(wframe, width=600, height=700)
    self.m_canvas['bg'] = SensorViz.CanvasBg
    self.m_canvas.grid(row=0, column=1, rowspan=2, padx=0, pady=0, sticky=E+W)

    origin = (300, 350)
    size   = (300, 300)

    if self.m_icons['laelaps_top_down'] is not None:
      self.m_canvas.create_image(origin, image=self.m_icons['laelaps_top_down'])

    for key in self.m_range:
      sensor = self.m_range[key]

      # create beam placeholder
      sensor['origin'] = (origin[0]+sensor['boff'][0], 
                          origin[1]+sensor['boff'][1])
      sensor['idBeam'] = None

      # create sensor value text
      sensor['idText'] = self.m_canvas.create_text(
          (sensor['origin'][0]+sensor['toff'][0],
           sensor['origin'][1]+sensor['toff'][1]),
          fill = SensorViz.TextFgColor)

      #
      # Determine the parameterized lines coefficients for the beam rays. 
      #
      # x = x_0 + a * t, y = y_0 + b * t  where t = dist_meas / max
      #
      # Note: Robot is oriented up, so need to rotate by 90 degrees.
      #
      radius = size[0] * SensorViz.BeamMaxDist / SensorViz.RobotBodyDim[0]
      rot90  = degToRad(90.0)
      sensor['coef'] = [(0.0, 0.0)]   # origin a,b coefficients
      for ray in [-20.0, -10.0, 0.0, 10.0, 20.0]:
        angle = sensor['beam_dir'] + rot90 + degToRad(ray)
        sensor['coef'].append((radius*math.cos(angle), radius*math.sin(angle)))
    
  #
  ## \brief Update all alarms from received status message.
  ##
  ## \param status    Robot extended status message.     
  #
  def updateSensorData(self, sensedData):
    i = 0
    for name in sensedData.name:
      rawVal = sensedData.range[i]
      i += 1
      if self.m_range.has_key(name):
        if rawVal >= SensorViz.BeamMinDist and rawVal <= SensorViz.BeamMaxDist:
          self.detectedObj(self.m_range[name], rawVal)
        else:
          self.noObj(self.m_range[name])

  def detectedObj(self, sensor, rawVal):
    sensor['raw_value'] = rawVal
    filteredVal         = self.filter(sensor)
    brightness          = sensor['brightness']
    if brightness < SensorViz.BrightMax:
      brightness += SensorViz.BrightIncStepSize
    self.showBeam(sensor, filteredVal, brightness)
    sensor['filtered_value'] = filteredVal
    sensor['brightness']     = brightness

  def noObj(self, sensor):
    sensor['raw_value'] = SensorViz.BeamNoObj
    filteredVal         = sensor['filtered_value']
    brightness          = sensor['brightness']
    if brightness > SensorViz.BrightMin:
      brightness -= SensorViz.BrightDecStepSize
    if brightness <= SensorViz.BrightMin:
      filteredVal           = SensorViz.BeamNoObj
      sensor['sliding_win'] = []
      sensor['sum_total']   = 0
    self.showBeam(sensor, filteredVal, brightness)
    sensor['filtered_value'] = filteredVal
    sensor['brightness']     = brightness
      
  def filter(self, sensor):
    sensor['sum_total'] += sensor['raw_value']
    sensor['sliding_win'].append(sensor['raw_value'])
    if len(sensor['sliding_win']) > SensorViz.SlidingWinMaxSize:
      v = sensor['sliding_win'].pop(0)
      sensor['sum_total'] -= v
    return sensor['sum_total'] / len(sensor['sliding_win'])

  def showBeam(self, sensor, newFilteredVal, newBrightness):
    curFilteredVal = sensor['filtered_value']
    curBrightness  = sensor['brightness']

    # same beam and value
    if newFilteredVal == curFilteredVal and newBrightness == curBrightness:
      return

    # brightness indices
    ibright = (int)(curBrightness / len(SensorViz.BeamColor))
    jbright = (int)(newBrightness / len(SensorViz.BeamColor))

    #
    # Create a new beam when new object is detected or its distance has changed.
    #
    if  newFilteredVal != SensorViz.BeamNoObj and \
        (newFilteredVal < curFilteredVal - 0.002 or \
         newFilteredVal > curFilteredVal + 0.002):
      if sensor['idBeam'] != None:
        self.m_canvas.delete(sensor['idBeam'])
      x0, y0 = sensor['origin']
      t = newFilteredVal / SensorViz.BeamMaxDist
      poly = []
      for a,b in sensor['coef']:
        poly.append((x0 + a * t, y0 - b * t))
      sensor['idBeam'] = self.m_canvas.create_polygon(
          poly, fill=SensorViz.BeamColor[jbright])

    #
    # Set a new brightness for the current beam.
    #
    elif jbright != ibright: 
      self.m_canvas.itemconfig(sensor['idBeam'],
                                fill=SensorViz.BeamColor[jbright])
      # special case
      if jbright == 0:
        if sensor['idBeam'] != None:
          self.m_canvas.delete(sensor['idBeam'])
        x0, y0 = sensor['origin']
        t = 1.0
        poly = []
        for a,b in sensor['coef']:
          poly.append((x0 + a * t, y0 - b * t))
        sensor['idBeam'] = self.m_canvas.create_polygon(
          poly, fill=SensorViz.BeamColor[jbright])
        self.m_canvas.tag_lower(sensor['idText'])

    # new filtered value
    if newFilteredVal != curFilteredVal:
      if newFilteredVal == SensorViz.BeamNoObj:
        self.m_canvas.itemconfig(sensor['idText'], text='')
      else:
        self.m_canvas.itemconfig(sensor['idText'],
                                  text="%.3lf" % (newFilteredVal))
    
    # make sure value if visable
    self.m_canvas.tag_raise(sensor['idText'])


# ------------------------------------------------------------------------------
# Unit Test Main
# ------------------------------------------------------------------------------
if __name__ == '__main__':
  # create root 
  root = Tk()

  win = RangeSensorWin(master=root)

  win.mainloop()
