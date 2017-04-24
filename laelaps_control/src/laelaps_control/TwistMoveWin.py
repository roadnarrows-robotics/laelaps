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

from laelaps_control.srv import Stop              # service

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

    self.wm_protocol("WM_DELETE_WINDOW", lambda: self.onDelete(self))

    self.m_frame = TwistMoveFrame(master=self, cnf=cnf, **kw)

    self.m_frame.grid(row=0, column=0, padx=5, pady=5)

    # load close icon
    self.m_iconClose = self.m_frame.loadImage('icons/icon_close_32.png')

    # close button
    k, w = createCompoundButton(self, text='Close', image=self.m_iconClose,
        command=lambda: self.onDelete(self), width=80)
    w.grid(row=1, column=0, sticky=N, pady=5)
    self.m_bttnClose = w

    self.lift()

  #
  ## \brief On delete callback.
  ##
  ## \param w   Widget (not used).
  #
  def onDelete(self, w):
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

    self.m_icons['app_icon'] = self.loadImage("icons/BotTwistIcon.png")

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
  ## \brief Open image from file and convert to PhotoImage.
  ##
  ## \param filename    Image file name.
  ##
  ## \return Returns image widget on success, None on failure.
  #
  def loadImage(self, filename):
    return self.m_imageLoader.loadImage(filename)

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
    # Button subframe
    #
    subframe = Frame(wframe)
    #subframe['borderwidth'] = 2
    #subframe['relief'] = 'ridge'
    subframe.grid(row=row, column=col, padx=1, pady=1, sticky=N+W)

    # twist button
    w = self.createButton(subframe, 'Twist', 'icons/icon_bot_twist.png',
                  command=self.cbTwist)
    w.grid(row=0, column=0, sticky=W, padx=1, pady=5)

    # stop button
    w = self.createButton(subframe, 'Stop', 'icons/icon_bot_brake.png',
                  command=self.cbStop)
    w.grid(row=1, column=0, sticky=W, padx=1, pady=5)

    col += 1

    #
    # Linear subframe
    #
    subframe = Frame(wframe)
    #subframe['borderwidth'] = 2
    #subframe['relief'] = 'ridge'
    subframe.grid(row=row, column=col, padx=3, pady=(5,1), sticky=N+W)
    subframe.columnconfigure(0, minsize=100)
    subframe.columnconfigure(1, minsize=200)

    # linear velocity scale (slider)
    w = Scale(subframe, from_=2.0, to=-2.0, orient=VERTICAL, resolution=0.1,
        tickinterval=0.5, length=220, font=helv,
        command=self.cbVelLin)
    #w['borderwidth'] = 2
    #w['relief'] = 'ridge'
    w.grid(row=0, column=0, sticky=W+E, padx=(25,0))
    self.m_wVelLin = w
    self.m_maxLin  = 2.0

    w = Label(subframe, text="linear velocity\n  (meters/s)", font=helv)
    #w['borderwidth'] = 2
    #w['relief'] = 'ridge'
    w.grid(row=0, column=1, sticky=W)

    col += 1

    #
    # Canvas subframe
    #
    subframe = Frame(wframe)
    subframe.grid(row=row, column=col, rowspan=2, padx=1, pady=3, sticky=N+W+E)

    #dim = 220
    dim = 300
    self.m_canvas = Canvas(subframe, width=dim, height=dim)
    self.m_canvas.grid(row=0, column=3, rowspan=2, sticky=N+E)
    halfdim = dim/2
    self.m_x0 = halfdim
    self.m_y0 = halfdim
    self.m_r  = halfdim - 10

    self.idXAxis = self.m_canvas.create_line(
        self.m_x0, self.m_y0+halfdim, self.m_x0, self.m_y0-halfdim,
        arrow=LAST)
    self.idYAxis = self.m_canvas.create_line(
        self.m_x0+halfdim, self.m_y0, self.m_x0-halfdim, self.m_y0,
        arrow=LAST)
    self.idTwist  = None
    self.idBot    = None
    self.idExtras = None
    self.idDbgBox = None
    self.idDbgAng = None
    self.showBot(self.m_x0, self.m_y0)

    row += 1
    col = 0

    #
    # Angular subframe
    #
    subframe = Frame(wframe)
    subframe.grid(row=row, column=col, columnspan=2, padx=1, pady=3,
        sticky=N+W+E)

    # angular velocity scale (slider)
    w = Scale(subframe, from_=-300.0, to=300.0, orient=HORIZONTAL, resolution=1,
        tickinterval=60.0, length=420,
        font=helv, command=self.cbVelAng)
    w.grid(row=0, column=0, sticky=N+W+E)
    self.m_wVelAng = w

    w = Label(subframe, text="angular velocity (degrees/s)",
        font=helv)
    w.grid(row=1, column=0)

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
    self.m_icons[key] = self.loadImage(imagefile)
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

  def cbVelLin(self, val):
    val = float(val)
    # init 
    if self.m_velLin is None:
      self.m_velLin = val
    elif val != self.m_velLin:
      self.m_velLin = val
      self.showArc()

  def cbVelAng(self, val):
    val = float(val)
    if self.m_velAng is None:
      self.m_velAng = val
    elif val != self.m_velAng:
      self.m_velAng = val
      self.showArc()

  def showArc(self):
    t     = 1.0             # seconds
    color = '#0000aa'       # arc color
    rot90 = degToRad(90.0)  # rotate 90 degrees
    x0    = self.m_x0       # x origin in pixel coordinates
    y0    = self.m_y0       # y origin in pixel coordinates
    rmax  = self.m_r        # maximum radius in pixels

    dist = self.m_velLin * t # distance traveled in real world coordinates
    deg  = self.m_velAng * t # degrees turned

    d = rmax * dist / self.m_maxLin  # distance in pixel coordinates

    if self.idTwist is not None:
      self.m_canvas.delete(self.idTwist)  # bye bye
      self.idTwist = None
    if self.idExtras is not None:
      self.m_canvas.delete(self.idExtras) # red shirt
      self.idExtras = None

    # no movement
    if math.fabs(dist) < 0.1 and math.fabs(deg) < 2.0: 
      x1 = x0
      y1 = y0
      pass

    # linear movement only
    elif math.fabs(deg) < 2.0:
      x1 = x0
      y1 = y0 - d
      self.idTwist = self.m_canvas.create_line(x0, y0, x1, y1,
          fill=color, width=3)

    # angular movement only
    elif math.fabs(dist) < 0.1: 
      x1      = x0
      y1      = y0
      bbox    = (x0 - 10, y0 - 10, x0 + 10, y0 + 10)
      if deg > 0.0:
        start   = 0.0
        extent  = 270.0
        xs,ys   = x0, y0 + 10
        xe,ye   = x0 + 10, y0 + 10
      else:
        start   = 270.0
        extent  = 270.0
        xs,ys   = x0, y0 + 10
        xe,ye   = x0 - 10, y0 + 10
      self.idTwist = self.m_canvas.create_arc(bbox,
          start=start, extent=extent,
          style=ARC, width=3, outline=color, fill=color)
      self.idExtras = self.m_canvas.create_line(
        xs, ys, xe, ye, arrow=LAST,
        width=3, fill=color)

    # blended movement
    else:
      if self.idDbgBox is not None:
        self.m_canvas.delete(self.idDbgBox)  # hasta la vista
        self.idDbgBox = None
      if self.idDbgAng is not None:
        self.m_canvas.delete(self.idDbgAng)  # and you too
        self.idDbgAng = None

      #
      # Calculate a circular arc about x0,y0
      #
      a = degToRad(deg)

      # radius for fixed arc length
      r = math.fabs(d) / math.fabs(a)

      # arc start point in pixel coordinates
      xs = x0 + r
      ys = y0

      # arc end point in pixel coordinates
      x1 = x0 + r * math.cos(a)
      y1 = y0 - r * math.sin(a) 

      extent = math.fabs(deg)

      if deg >= 0.0 and dist >= 0.0:
        q     = 'QI --> QII'    # label
        x1    = x1 - r          # translate arc in negative x direction
        bbox  = (x0 - 2 * r, y0 - r, x0, y0 + r)
        start = 0.0
      elif deg < 0.0 and dist >= 0.0:
        q     = 'QIV --> QI'    # label
        x1    = x0 + xs - x1    # reflect
        y1    = y0 - y1 + y0    # translate
        bbox  = (x0, y0 - r, x0 + 2 * r, y0 + r)
        start = 180.0 - extent
      elif deg >= 0.0 and dist < 0.0:
        q     = 'QI --> QIV'    # label
        x1    = x0 + xs - x1    # reflect
        y1    = y0 - y1 + y0    # translate
        bbox  = (x0, y0 - r, x0 + 2 * r, y0 + r)
        start = 180.0
      elif deg < 0.0 and dist < 0.0:
        q     = 'QI --> QIV'    # label
        x1    = x1 - r          # translate arc in negative x direction
        bbox  = (x0 - 2 * r, y0 - r, x0, y0 + r)
        start = 360.0 - extent
      else:
        q     = "Q? --> Q?"
        x1 = x0
        y1 = y0
        bbox  = (x0 - 2 * r, y0 - r, x0, y0 + r)

      #print 'DBG', q, bbox, start, extent

      self.idTwist = self.m_canvas.create_arc(bbox,
          start=start, extent=extent,
          style=ARC, width=3, outline=color, fill=color)

      # debug 
      #self.idDbgBox = self.m_canvas.create_rectangle(bbox)
      #self.idDbgAng = self.m_canvas.create_line(
      #    x0, y0, x0 + d * math.cos(a), y0 - d * math.sin(a))

    self.showBot(x1, y1)
    
  # show your bot
  def showBot(self, x0, y0):
    if self.idBot is not None:
      self.m_canvas.delete(self.idBot)
      self.idBot = None
    self.idBot = self.m_canvas.create_oval(x0-4, y0-4, x0+4, y0+4,
          outline='#000000', fill='#fed700')

  def center(self, x0, y0, x1, y1, r):
    q = ((x0 + x1)/2.0, (y0 + y1)/2.0)    # midpoint of p0 and p1
    dx = x1 - x0
    dy = y1 - y0
    R = math.sqrt(dx * dx + dy * dy)      # distance between p0 and p1
    m = (-dy, dx)                         # orthogonal slope
    t = math.sqrt(r/R * r/R - 1/4.0)
    cx = q[0] + t * m[0]
    cy = q[1] + t * m[1]
    return (cx, cy)


  def cbTwist(self):
    self.publishTwist(self.m_velLin, degToRad(self.m_velAng))

  #
  ## \brief Brake to stop callback.
  #
  def cbStop(self):
    try:
      rospy.wait_for_service("laelaps_control/stop", timeout=1)
    except rospy.ROSException, e:
      return
    try:
      stop = rospy.ServiceProxy('laelaps_control/stop', Stop)
      stop()
    except rospy.ServiceException, e:
     return
    self.m_wVelLin.set(0.0)
    self.m_wVelAng.set(0.0)

  
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
    self.cbStop()


# ------------------------------------------------------------------------------
# Unit Test Main
# ------------------------------------------------------------------------------
if __name__ == '__main__':
  # create root 
  root = Tk()

  win = TwistMoveWin(master=root)

  win.mainloop()
