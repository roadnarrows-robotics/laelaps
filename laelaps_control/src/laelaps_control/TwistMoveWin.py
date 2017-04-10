###############################################################################
#
# Package:   RoadNarrows Robotics Laelaps Robotic Mobile Platform ROS Package
#
# Link:      https://github.com/roadnarrows-robotics/laelaps
#
# ROS Node:  laelaps_panel, laelaps_range
#
# File: TwistMoveWin.py
#
## \file 
##
## \brief Laelaps twist move effector panel.
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

from geometry_msgs.msg import Twist # message

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



# ------------------------------------------------------------------------------
# Class TwistMoveWin
# ------------------------------------------------------------------------------

## \brief Range sensor group window class.
#
class TwistMoveWin(Toplevel):
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

    self.title("laelaps_twist")

    self.wm_protocol("WM_DELETE_WINDOW", lambda: self.onDeleteChild(self))

    self.m_frame = TwistMoveFrame(master=self, cnf=cnf, **kw)

    self.m_frame.grid(row=0, column=0, padx=5, pady=5)

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
      self.m_frame.cleanup()
      self.destroy()


# ------------------------------------------------------------------------------
# Class TwistMoveFrame
# ------------------------------------------------------------------------------

## \brief Range sensor group frame class.
#
class TwistMoveFrame(Frame):
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
                  self.m_imageLoader.loadImage("icons/BotTwistIcon.png")

    if self.m_icons['app_icon'] is not None:
      self.master.tk.call('wm', 'iconphoto',
          self.master._w, self.m_icons['app_icon'])

    # craete and show widgets
    self.createWidgets()

    self.grid(row=0, column=0, padx=5, pady=5)

    # publish twist message
    self.m_pub_twist = \
        rospy.Publisher("laelaps_control/cmd_vel",
        Twist, queue_size=2)

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

    if kw.has_key('debug'):
      self.m_debug = kw['debug']
      del kw['debug']

    # variables only used for debugging
    if self.m_debug:
      pass

    self.m_velLin = None
    self.m_velAng = None

    return kw

  #
  ## \brief Create gui widgets with supporting data and show.
  #
  def createWidgets(self):
    self.createHeading(self, 0, 0)
    self.createEffectorPanel(self, 1, 0)

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
    w['text']   = 'Laelaps Twist Move'
    w['anchor'] = CENTER
    w.grid(row=row, column=col, sticky=E+W)

  #
  ## \brief Create subsystem state lower center panel headers.
  ##
  ## \param parent  Parent widget
  ## \param row     Row in parent widget.
  ## \param col     Column in parent widget.
  #
  def createEffectorPanel(self, parent, row, col):
    wframe = Frame(parent)
    wframe['borderwidth'] = 2
    wframe['relief'] = 'ridge'
    wframe.grid(row=row, column=col, padx=1, pady=3, sticky=N+W+E)

    helv  = tkFont.Font(family="Helvetica",size=10,weight="bold")
    padx  = 10
    pady  = 3
    row   = 0
    col   = 0

    #
    # Linear
    #
    subframe = Frame(wframe)
    subframe.grid(row=row, column=col, padx=1, pady=3, sticky=N+W+E)
    subframe.columnconfigure(1, minsize=105)

    # twist button
    w = self.createButton(subframe, 'Twist', 'icons/icon_bot_twist.png',
                  command=lambda: self.cbTwist(self))
    w.grid(row=0, column=0, sticky=W, pady=5)

    w = Scale(subframe, from_=-2.0, to=2.0, orient=VERTICAL, resolution=0.1,
        tickinterval=0.5, length=200, font=helv,
        command=self.cbVelLin)
    w.grid(row=0, column=1, sticky=N+E)
    self.m_wVelLin = w

    w = Label(subframe, text="  linear velocity\n  (meters/s)", font=helv)
    w.grid(row=0, column=2)

    row += 1

    #
    # Angular
    #
    subframe = Frame(wframe)
    subframe.grid(row=row, column=col, padx=1, pady=3, sticky=N+W+E)

    w = Scale(subframe, from_=-45.0, to=45.0, orient=HORIZONTAL, resolution=1,
        tickinterval=10.0, length=400,
        font=helv, command=self.cbVelAng)
    w.grid(row=0, column=0)
    self.m_wVelAng = w

    w = Label(subframe, text="angular velocity (degrees/s)",
        font=helv)
    w.grid(row=1, column=0)

  def cbVelLin(self, val):
    val = float(val)
    # init 
    if self.m_velLin is None:
      self.m_velLin = val
    elif val != self.m_velLin:
      self.m_velLin = val

  #
  ## \brief Create button.
  ##
  ## \param parent    Parent widget.
  ## \param text      Button text.
  ## \param imagefile Image file name. None for no image.
  ## \param command   Callback for button push.
  ## \param fg        Foreground text color.
  ##
  ## \return Button widget.
  #
  def createButton(self, parent, text, imagefile, command, fg='black'):
    key = str.lower(text.replace("\n", "_"))
    self.m_icons[key] = self.m_imageLoader.loadImage(imagefile)
    w = Button(parent)
    w['text']     = text
    if self.m_icons[key]:
      w['image']    = self.m_icons[key]
      w['compound'] = LEFT
      w['padx']     = 0
      w['pady']     = 0
      w['anchor']   = W
      w['width']    = 105
    else:
      w['anchor']   = CENTER
      w['width']    = 10
    w['fg']       = fg
    w['command']  = command
    return w

  def cbVelAng(self, val):
    val = float(val)
    if self.m_velAng is None:
      self.m_velAng = val
    elif val != self.m_velAng:
      self.m_velAng = val

  def cbTwist(self, w):
    self.publishTwist(self.m_velLin, degToRad(self.m_velAng))

  #
  ## \brief Publish twist.
  ##
  ## \param velLinear   Robot linear velocity (m/s).
  ## \param velAngular  Robot angular velocity (radians/s).
  #
  def publishTwist(self, velLinear, velAngular):
    cmd = Twist()
    cmd.linear.x = velLinear
    cmd.linear.y = 0.0
    cmd.linear.z = 0.0
    cmd.angular.x = 0.0
    cmd.angular.y = 0.0
    cmd.angular.z = velAngular
    self.m_pub_twist.publish(cmd)

  def cleanup(self):
    pass


# ------------------------------------------------------------------------------
# Unit Test Main
# ------------------------------------------------------------------------------
if __name__ == '__main__':
  # create root 
  root = Tk()

  win = TwistMoveWin(master=root)

  win.mainloop()
