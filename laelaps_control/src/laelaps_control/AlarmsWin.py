###############################################################################
#
# Package:   RoadNarrows Robotics Laelaps Robotic Mobile Platform ROS Package
#
# Link:      https://github.com/roadnarrows-robotics/laelaps
#
# ROS Node:  laelaps_panel
#
# File: AlarmsWin.py
#
## \file 
##
## $LastChangedDate: 2012-12-06 16:33:18 -0700 (Thu, 06 Dec 2012) $
## $Rev: 330 $
##
## \brief Laelaps alarms sub-panel.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright:
##   (C) 2016.  RoadNarrows LLC.\n
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

from laelaps_control.msg import Alarms              # message
from laelaps_control.msg import MotorCtlrHealth     # message
from laelaps_control.msg import MotorHealth         # message
from laelaps_control.msg import RobotStatusExtended # subscribe

from laelaps_control.Utils import *
from laelaps_control.Gauge import *


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

alarmState = {'none':0, 'warning':1, 'alarm':2, 'critical':3}
alarmColor = {
  'none':'gray', 'warning':'yellow', 'alarm':'red', 'critical':'red'
}


# ------------------------------------------------------------------------------
# Class AlarmsWin
# ------------------------------------------------------------------------------

##
## \brief Alarms window class.
##
class AlarmsWin(Toplevel):
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

    Toplevel.__init__(self, master=master, cnf=cnf, **kw)
    self.title("Laelaps Alarms Panel")

    # craete and show widgets
    self.createWidgets()

    self.wm_protocol("WM_DELETE_WINDOW", lambda: self.onDeleteChild(self))

    # subscribe to extended robot status data
    rospy.Subscriber("laelaps_control/robot_status_ex", 
                     RobotStatusExtended, 
                     self.updateAlarms) 
    self.lift()

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
    self.m_alarms         = {}    # alarm state

    self.m_imageLoader = ImageLoader(py_pkg='laelaps_control.images',
                                      image_paths=imagePath)

    if kw.has_key('debug'):
      self.m_debug = kw['debug']
      del kw['debug']

    # variables only used for debugging
    if self.m_debug:
      pass

    return kw

  #
  ## \brief Create gui widgets with supporting data and show.
  #
  def createWidgets(self):
    self.createSysAlarmPanel(self, 0)
    self.createMotorCtlrAlarmPanel(self, 1)

    # close button
    w = Button(self, width=10, text='Close',
        command=lambda: self.onDeleteChild(self), anchor=CENTER)
    w.grid(row=2, column=0, sticky=N, pady=5)
    self.m_bttnClose = w

  #
  ## \brief Create system alarms panel.
  #
  def createSysAlarmPanel(self, parent, row):
    wframe = LabelFrame(parent, text='System Alarms')
    wframe['font'] =('Helvetica', 12)
    wframe['fg'] = fgColors['focus']
    wframe['borderwidth'] = 2
    wframe['relief'] = 'ridge'
    wframe.grid(row=row, column=0, padx=3, pady=3, sticky=N+W+E)

    key = 'system'
    self.m_alarms[key] = {}

    row = 0
    col = 0

    w = Indicator(wframe, gauge_label='Battery')
    w.grid(row=row, column=col)
    self.m_alarms[key]['battery'] = {
        'val':      alarmState['none'],
        'var':      None,
        'widget':   w,
        'type':     'indicator'}

    col += 1

    w = Indicator(wframe, gauge_label='Temp')
    w.grid(row=row, column=col)
    self.m_alarms[key]['temperature'] = {
        'val':      alarmState['none'],
        'var':      None,
        'widget':   w,
        'type':     'indicator'}

    col += 1

    w = Indicator(wframe, gauge_label='EStop')
    w.grid(row=row, column=col)
    self.m_alarms[key]['estop'] = {
        'val':      alarmState['none'],
        'var':      None,
        'widget':   w,
        'type':     'indicator'}

    col += 1

    w = Indicator(wframe, gauge_label='Critical')
    w.grid(row=row, column=col)
    self.m_alarms[key]['critical'] = {
        'val':      alarmState['none'],
        'var':      None,
        'widget':   w,
        'type':     'indicator'}

  #
  ## \brief Create motor controller alarms panel.
  #
  def createMotorCtlrAlarmPanel(self, parent, row):
    wframe = Frame(parent)
    wframe['relief'] = 'flat'
    wframe.grid(row=row, column=0, padx=3, pady=3, sticky=N+W+E)

    row = 0
    for motorctlr in ['front', 'rear']:
      self.createMotorCtlrAlarmSubPanel(wframe, motorctlr, row)
      row += 1

  #
  ## \brief Create specific motor controller alarms subpanel.
  #
  def createMotorCtlrAlarmSubPanel(self, parent, key, row):
    title = key.capitalize() + " Motor Controller Alarms"
    wframe = LabelFrame(parent, text=title)
    wframe['font'] =('Helvetica', 12)
    wframe['fg'] = fgColors['focus']
    wframe['borderwidth'] = 2
    wframe['relief'] = 'ridge'
    wframe.grid(row=row, column=0, padx=1, pady=3, sticky=N+W+E)

    self.m_alarms[key] = {}

    row = 0
    col = 0

    w = Indicator(wframe, gauge_label='Battery\nHigh')
    w.grid(row=row, column=col)
    self.m_alarms[key]['batt_high'] = {
        'val':      alarmState['none'],
        'var':      None,
        'widget':   w,
        'type':     'indicator'}

    col += 1

    w = Indicator(wframe, gauge_label='Battery\nLow')
    w.grid(row=row, column=col)
    self.m_alarms[key]['batt_low'] = {
        'val':      alarmState['none'],
        'var':      None,
        'widget':   w,
        'type':     'indicator'}

    col += 1

    w = Indicator(wframe, gauge_label='Logic\nHigh')
    w.grid(row=row, column=col)
    self.m_alarms[key]['logic_high'] = {
        'val':      alarmState['none'],
        'var':      None,
        'widget':   w,
        'type':     'indicator'}

    col += 1

    w = Indicator(wframe, gauge_label='Logic\nLow')
    w.grid(row=row, column=col)
    self.m_alarms[key]['logic_low'] = {
        'val':      alarmState['none'],
        'var':      None,
        'widget':   w,
        'type':     'indicator'}

    col += 1

    w = Indicator(wframe, gauge_label='Temp')
    w.grid(row=row, column=col)
    self.m_alarms[key]['temperature'] = {
        'val':      alarmState['none'],
        'var':      None,
        'widget':   w,
        'type':     'indicator'}

    row += 1
    col += 1

    self.createMotorAlarmPanel(wframe, row, col, key)

  #
  ## \brief Create motor alarms panel.
  #
  def createMotorAlarmPanel(self, parent, row, colspan, ctlr):
    wframe = Frame(parent)
    wframe['relief'] = 'flat'
    wframe.grid(row=row, column=0, columnspan=colspan, 
        padx=1, pady=3, sticky=N+W+E)

    if ctlr == 'front':
      motorList = ['left_front', 'right_front']
    else:
      motorList = ['left_rear', 'right_rear']

    col = 0

    for key in motorList:
      self.createMotorAlarmSubPanel(wframe, key, ctlr, row, col)
      col += 1

  #
  ## \brief Create specific motor alarms subpanel.
  #
  def createMotorAlarmSubPanel(self, parent, key, ctlr, row, col):
    title = key.replace("_"+ctlr, " ")
    title = title.capitalize() + " Motor"
    wframe = LabelFrame(parent, text=title)
    wframe['font'] =('Helvetica', 10)
    wframe['fg'] = fgColors['focus']
    wframe['borderwidth'] = 2
    wframe['relief'] = 'ridge'
    wframe.grid(row=row, column=col, padx=10, pady=3, sticky=N+W+E)

    self.m_alarms[key] = {}

    row = 0
    col = 0

    w = Indicator(wframe, gauge_label='Fault')
    w.grid(row=row, column=col)
    self.m_alarms[key]['fault'] = {
        'val':      alarmState['none'],
        'var':      None,
        'widget':   w,
        'type':     'indicator'}

    col += 1

    w = Indicator(wframe, gauge_label='Over\nCurrent')
    w.grid(row=row, column=col)
    self.m_alarms[key]['over_current'] = {
        'val':      alarmState['none'],
        'var':      None,
        'widget':   w,
        'type':     'indicator'}

  def updateAlarms(self, status):

  def onDeleteChild(self, w):
    self.m_isCreated = False
    self.destroy()


# ------------------------------------------------------------------------------
# Unit Test Main
# ------------------------------------------------------------------------------
if __name__ == '__main__':
  # create root 
  root = Tk()

  win = AlarmsWin(master=root)

  win.mainloop()
