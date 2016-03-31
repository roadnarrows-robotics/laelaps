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
## \brief Laelaps alarms panel.
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

## \brief Alarm severity keys.
alarmSeverity = ['none', 'warning', 'alarm', 'critical']

## \brief Alarm severity to indicator color map.
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

    self.m_icons['app_icon'] = \
                  self.m_imageLoader.loadImage("icons/LaelapsAlarmsIcon.png")
    if self.m_icons['app_icon'] is not None:
      self.master.tk.call('wm', 'iconphoto', self.master._w,
          self.m_icons['app_icon'])


    # subscribe to extended robot status data
    self.m_sub = rospy.Subscriber("laelaps_control/robot_status_ex", 
                     RobotStatusExtended, 
                     callback=self.updateAlarms) 

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
    self.createHeading(self, 0, 0)
    self.createSysAlarmPanel(self, 1, 0)
    self.createAllMotorCtlrAlarmPanel(self, 2, 0)

    # close button
    w = Button(self, width=10, text='Close',
        command=lambda: self.onDeleteChild(self), anchor=CENTER)
    w.grid(row=3, column=0, sticky=N, pady=5)
    self.m_bttnClose = w

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
    w['text']   = 'Laelaps Alarms Panel'
    w['anchor'] = CENTER
    w.grid(row=row, column=col, sticky=E+W)

  #
  ## \brief Create system alarms panel.
  ##
  ## \param parent    Parent container widget.
  ## \param row       Row in parent widget.
  ## \param col       Column in parent widget.
  #
  def createSysAlarmPanel(self, parent, row, col):
    wframe = LabelFrame(parent, text='System Alarms')
    wframe['font'] =('Helvetica', 12)
    wframe['fg'] = fgColors['focus']
    wframe['borderwidth'] = 2
    wframe['relief'] = 'ridge'
    wframe.grid(row=row, column=col, padx=3, pady=3, sticky=N+W+E)

    subsys = 'system'
    self.m_alarms[subsys] = {}
    self.m_alarms[subsys]['alarm_bits']   = 0x0
    self.m_alarms[subsys]['warning_bits'] = 0x0

    row = 0
    col = 0

    w = Indicator(wframe, gauge_label='General')
    w.grid(row=row, column=col)
    self.m_alarms[subsys]['general'] = {
        'val':      'none',
        'var':      None,
        'widget':   w,
        'type':     'indicator'}

    col += 1

    w = Indicator(wframe, gauge_label='Battery')
    w.grid(row=row, column=col)
    self.m_alarms[subsys]['battery'] = {
        'val':      'none',
        'var':      None,
        'widget':   w,
        'type':     'indicator'}

    col += 1

    w = Indicator(wframe, gauge_label='Temp')
    w.grid(row=row, column=col)
    self.m_alarms[subsys]['temperature'] = {
        'val':      'none',
        'var':      None,
        'widget':   w,
        'type':     'indicator'}

    col += 1

    w = Indicator(wframe, gauge_label='EStop')
    w.grid(row=row, column=col)
    self.m_alarms[subsys]['estop'] = {
        'val':      'none',
        'var':      None,
        'widget':   w,
        'type':     'indicator'}

    col += 1

    w = Indicator(wframe, gauge_label='Critical')
    w.grid(row=row, column=col)
    self.m_alarms[subsys]['critical'] = {
        'val':      'none',
        'var':      None,
        'widget':   w,
        'type':     'indicator'}

  #
  ## \brief Create all motor controller and motor alarms panels.
  ##
  ## \param parent    Parent container widget.
  ## \param row       Row in parent widget.
  ## \param col       Column in parent widget.
  #
  def createAllMotorCtlrAlarmPanel(self, parent, row, col):
    wframe = Frame(parent)
    wframe['relief'] = 'flat'
    wframe.grid(row=row, column=col, padx=3, pady=3, sticky=N+W+E)

    row = 0
    col = 0

    for motorctlr in ['front', 'rear']:
      self.createMotorCtlrAlarmPanel(wframe, motorctlr, row, col)
      row += 1

  #
  ## \brief Create specific motor controller and motor alarms panels.
  ##
  ## \param parent    Parent container widget.
  ## \param subsys    Alarm subsystem key.
  ## \param row       Row in parent widget.
  ## \param col       Column in parent widget.
  #
  def createMotorCtlrAlarmPanel(self, parent, subsys, row, col):
    title = subsys.capitalize() + " Motor Controller Alarms"
    wframe = LabelFrame(parent, text=title)
    wframe['font'] =('Helvetica', 12)
    wframe['fg'] = fgColors['focus']
    wframe['borderwidth'] = 2
    wframe['relief'] = 'ridge'
    wframe.grid(row=row, column=col, padx=1, pady=3, sticky=N+W+E)

    self.m_alarms[subsys] = {}
    self.m_alarms[subsys]['alarm_bits']   = 0x0
    self.m_alarms[subsys]['warning_bits'] = 0x0

    row = 0
    col = 0

    w = Indicator(wframe, gauge_label='Battery\nHigh')
    w.grid(row=row, column=col)
    self.m_alarms[subsys]['batt_high'] = {
        'val':      'none',
        'var':      None,
        'widget':   w,
        'type':     'indicator'}

    col += 1

    w = Indicator(wframe, gauge_label='Battery\nLow')
    w.grid(row=row, column=col)
    self.m_alarms[subsys]['batt_low'] = {
        'val':      'none',
        'var':      None,
        'widget':   w,
        'type':     'indicator'}

    col += 1

    w = Indicator(wframe, gauge_label='Logic\nHigh')
    w.grid(row=row, column=col)
    self.m_alarms[subsys]['logic_high'] = {
        'val':      'none',
        'var':      None,
        'widget':   w,
        'type':     'indicator'}

    col += 1

    w = Indicator(wframe, gauge_label='Logic\nLow')
    w.grid(row=row, column=col)
    self.m_alarms[subsys]['logic_low'] = {
        'val':      'none',
        'var':      None,
        'widget':   w,
        'type':     'indicator'}

    col += 1

    w = Indicator(wframe, gauge_label='Temp')
    w.grid(row=row, column=col)
    self.m_alarms[subsys]['temperature'] = {
        'val':      'none',
        'var':      None,
        'widget':   w,
        'type':     'indicator'}

    colspan = col + 1
    row    += 1
    col     = 0

    self.createCtlrMotorAlarmPanels(wframe, subsys, row, col, colspan)

  #
  ## \brief Create motor controller's motor alarms panels.
  ##
  ## \param parent    Parent container widget.
  ## \param ctlr      Motor controller subsystem.
  ## \param row       Row in parent widget.
  ## \param col       Column in parent widget.
  ## \param colspan   Column span.
  #
  def createCtlrMotorAlarmPanels(self, parent, ctlr, row, col, colspan):
    wframe = Frame(parent)
    wframe['relief'] = 'flat'
    wframe.grid(row=row, column=col, columnspan=colspan, 
        padx=1, pady=3, sticky=N+W+E)

    if ctlr == 'front':
      motorList = ['left_front', 'right_front']
    else:
      motorList = ['left_rear', 'right_rear']

    row = 0
    col = 0

    for subsys in motorList:
      self.createMotorAlarmPanel(wframe, ctlr, subsys, row, col)
      col += 1

  #
  ## \brief Create specific motor alarms panel.
  ##
  ## \param parent    Parent container widget.
  ## \param ctlr      Motor controller subsystem.
  ## \param subsys    Motor subsystem.
  ## \param row       Row in parent widget.
  ## \param col       Column in parent widget.
  #
  def createMotorAlarmPanel(self, parent, ctlr, subsys, row, col):
    title = subsys.replace("_"+ctlr, " ")
    title = title.capitalize() + " Motor"
    wframe = LabelFrame(parent, text=title)
    wframe['font'] =('Helvetica', 10)
    wframe['fg'] = fgColors['focus']
    wframe['borderwidth'] = 2
    wframe['relief'] = 'ridge'
    wframe.grid(row=row, column=col, padx=10, pady=3, sticky=N+W+E)

    self.m_alarms[subsys] = {}
    self.m_alarms[subsys]['alarm_bits']   = 0x0
    self.m_alarms[subsys]['warning_bits'] = 0x0

    row = 0
    col = 0

    w = Indicator(wframe, gauge_label='Fault')
    w.grid(row=row, column=col)
    self.m_alarms[subsys]['fault'] = {
        'val':      'none',
        'var':      None,
        'widget':   w,
        'type':     'indicator'}

    col += 1

    w = Indicator(wframe, gauge_label='Over\nCurrent')
    w.grid(row=row, column=col)
    self.m_alarms[subsys]['over_current'] = {
        'val':      'none',
        'var':      None,
        'widget':   w,
        'type':     'indicator'}

  #
  ## \brief Update all alarms from received status message.
  ##
  ## \param status    Robot extended status message.     
  #
  def updateAlarms(self, status):
    self.updateSysAlarms(status)
    self.updateAllMotorCtlrAlarmPanels(status)
    self.updateAllMotorAlarmPanels(status)

  #
  ## \brief Update system alarms from received status message.
  ##
  ## \param status    Robot extended status message.     
  #
  def updateSysAlarms(self, status):
    subsys = 'system'

    if  status.alarms.alarms   != self.m_alarms[subsys]['alarm_bits'] or \
        status.alarms.warnings != self.m_alarms[subsys]['warning_bits']:
      self.m_alarms[subsys]['alarm_bits']   = status.alarms.alarms
      self.m_alarms[subsys]['warning_bits'] = status.alarms.warnings
      #print "DBG: %s: alarms=0x%x warnings=0x%x" % \
      #    (subsys, status.alarms.alarms, status.alarms.warnings)

    alarm = 'general'
    severity = self.getSeverity(status.alarms, Alarms.ALARM_GEN,
                                               Alarms.WARN_NONE)
    if self.m_alarms[subsys][alarm]['val'] != severity:
      self.m_alarms[subsys][alarm]['widget'].update(alarmColor[severity])
      self.m_alarms[subsys][alarm]['val'] = severity

    alarm = 'battery'
    severity = self.getSeverity(status.alarms, Alarms.ALARM_BATT,
                                               Alarms.WARN_BATT)
    if self.m_alarms[subsys][alarm]['val'] != severity:
      self.m_alarms[subsys][alarm]['widget'].update(alarmColor[severity])
      self.m_alarms[subsys][alarm]['val'] = severity

    alarm = 'temperature'
    severity = self.getSeverity(status.alarms, Alarms.ALARM_TEMP,
                                               Alarms.WARN_TEMP)
    if self.m_alarms[subsys][alarm]['val'] != severity:
      self.m_alarms[subsys][alarm]['widget'].update(alarmColor[severity])
      self.m_alarms[subsys][alarm]['val'] = severity

    alarm = 'estop'
    severity = self.getSeverity(status.alarms, Alarms.ALARM_ESTOP,
                                               Alarms.WARN_NONE)
    if self.m_alarms[subsys][alarm]['val'] != severity:
      self.m_alarms[subsys][alarm]['widget'].update(alarmColor[severity])
      self.m_alarms[subsys][alarm]['val'] = severity

    alarm = 'critical'
    if status.alarms.is_critical:
      severity = 'critical'
    else:
      severity = 'none'
    if self.m_alarms[subsys][alarm]['val'] != severity:
      self.m_alarms[subsys][alarm]['widget'].update(alarmColor[severity])
      self.m_alarms[subsys][alarm]['val'] = severity

  #
  ## \brief Update all motor controller alarms from received status message.
  ##
  ## \param status    Robot extended status message.     
  #
  def updateAllMotorCtlrAlarmPanels(self, status):
    for health in status.motor_ctlr_health:
      self.updateMotorCtlrAlarmPanel(health)

  #
  ## \brief Update specific motor controller alarms.
  ##
  ## \param health    Motor controller health status.
  #
  def updateMotorCtlrAlarmPanel(self, health):
    subsys = health.name

    if not self.m_alarms.has_key(subsys):
      return

    if  health.alarms.alarms   != self.m_alarms[subsys]['alarm_bits'] or \
        health.alarms.warnings != self.m_alarms[subsys]['warning_bits']:
      self.m_alarms[subsys]['alarm_bits']   = health.alarms.alarms
      self.m_alarms[subsys]['warning_bits'] = health.alarms.warnings
      #print "DBG: %s: alarms=0x%x warnings=0x%x" % \
      #    (subsys, health.alarms.alarms, health.alarms.warnings)

    alarm = 'batt_high'
    severity = self.getSeverity(health.alarms, Alarms.ALARM_MOTCTLR_BATT_V_HIGH,
                                               Alarms.WARN_MOTCTLR_BATT_V_HIGH)
    if self.m_alarms[subsys][alarm]['val'] != severity:
      self.m_alarms[subsys][alarm]['widget'].update(alarmColor[severity])
      self.m_alarms[subsys][alarm]['val'] = severity

    alarm = 'batt_low'
    severity = self.getSeverity(health.alarms, Alarms.ALARM_MOTCTLR_BATT_V_LOW,
                                               Alarms.WARN_MOTCTLR_BATT_V_LOW)
    if self.m_alarms[subsys][alarm]['val'] != severity:
      self.m_alarms[subsys][alarm]['widget'].update(alarmColor[severity])
      self.m_alarms[subsys][alarm]['val'] = severity

    alarm = 'logic_high'
    severity = self.getSeverity(health.alarms,
                                            Alarms.ALARM_MOTCTLR_LOGIC_V_HIGH,
                                            Alarms.WARN_NONE)
    if self.m_alarms[subsys][alarm]['val'] != severity:
      self.m_alarms[subsys][alarm]['widget'].update(alarmColor[severity])
      self.m_alarms[subsys][alarm]['val'] = severity

    alarm = 'logic_low'
    severity = self.getSeverity(health.alarms, Alarms.ALARM_MOTCTLR_LOGIC_V_LOW,
                                               Alarms.WARN_NONE)
    if self.m_alarms[subsys][alarm]['val'] != severity:
      self.m_alarms[subsys][alarm]['widget'].update(alarmColor[severity])
      self.m_alarms[subsys][alarm]['val'] = severity

    alarm = 'temperature'
    severity = self.getSeverity(health.alarms, Alarms.ALARM_TEMP,
                                               Alarms.WARN_TEMP)
    if self.m_alarms[subsys][alarm]['val'] != severity:
      self.m_alarms[subsys][alarm]['widget'].update(alarmColor[severity])
      self.m_alarms[subsys][alarm]['val'] = severity

  #
  ## \brief Update all motor alarms from received status message.
  ##
  ## \param status    Robot extended status message.     
  #
  def updateAllMotorAlarmPanels(self, status):
    for health in status.motor_health:
      self.updateMotorAlarmPanel(health)

  #
  ## \brief Create specific motor alarms subpanel.
  ##
  ## \param health    Motor health status.
  #
  def updateMotorAlarmPanel(self, health):
    subsys = health.name

    if not self.m_alarms.has_key(subsys):
      return

    if  health.alarms.alarms   != self.m_alarms[subsys]['alarm_bits'] or \
        health.alarms.warnings != self.m_alarms[subsys]['warning_bits']:
      self.m_alarms[subsys]['alarm_bits']   = health.alarms.alarms
      self.m_alarms[subsys]['warning_bits'] = health.alarms.warnings
      #print "DBG: %s: alarms=0x%x warnings=0x%x" % \
      #    (subsys, health.alarms.alarms, health.alarms.warnings)

    alarm = 'fault'
    severity = self.getSeverity(health.alarms, Alarms.ALARM_MOT_FAULT,
                                               Alarms.WARN_NONE)
    if self.m_alarms[subsys][alarm]['val'] != severity:
      self.m_alarms[subsys][alarm]['widget'].update(alarmColor[severity])
      self.m_alarms[subsys][alarm]['val'] = severity

    alarm = 'over_current'
    severity = self.getSeverity(health.alarms, Alarms.ALARM_MOT_OVER_CUR,
                                               Alarms.WARN_MOT_OVER_CUR)
    if self.m_alarms[subsys][alarm]['val'] != severity:
      self.m_alarms[subsys][alarm]['widget'].update(alarmColor[severity])
      self.m_alarms[subsys][alarm]['val'] = severity

  #
  ## \brief Determine severity of alarm.
  ##
  ## \param alarms      Alarms.
  ## \param alarmBit    Alarm bit.
  ## \param warnBit     Warning bit.
  ##
  ## \return Severity key.
  #
  def getSeverity(self, alarms, alarmBit, warnBit):
    if alarms.alarms & alarmBit:
      severity = 'alarm'
    elif alarms.warnings & warnBit:
      severity = 'warning'
    else:
      severity = 'none'
    return severity

  #
  ## \brief On delete callback.
  ##
  ## \param w   Widget (not used).
  #
  def onDeleteChild(self, w):
    self.m_isCreated = False
    self.m_sub.unregister()
    self.destroy()


# ------------------------------------------------------------------------------
# Unit Test Main
# ------------------------------------------------------------------------------
if __name__ == '__main__':
  # create root 
  root = Tk()

  win = AlarmsWin(master=root)

  win.mainloop()
