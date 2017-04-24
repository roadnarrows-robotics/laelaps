###############################################################################
#
#
# Package:   RoadNarrows Robotics ROS Laelaps Robot Package
#
# Link:      https://github.com/roadnarrows-robotics/laelaps
#
# ROS Node:  laelaps_*
#
# File:      Utils.py
#
## \file 
##
## $LastChangedDate$
## $Rev$
##
## \brief Utilities.
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
import math

from pkg_resources import *

from Tkinter import *
from Tkconstants import *
from tkFileDialog import *
import tkFont

from PIL import Image, ImageTk

# ------------------------------------------------------------------------------
# Class ImageLoader
# ------------------------------------------------------------------------------

#
## \brief Class to handle image loading.
#
class ImageLoader:

  #
  ## \brief Constructor
  ##
  ## \param py_pkg      Python resource (e.g. "laelaps_control.images").
  ## \param image_paths List of directory paths to search for the image.
  #
  def __init__(self, py_pkg=None, image_paths=[]):
    self.m_pyPkg = py_pkg
    if len(image_paths) > 0:
      self.m_imagePaths = image_paths
    else:
      self.m_imagePaths = ['.']
  
  #
  ## \brief Open image from file and convert to PhotoImage.
  ##
  ## \param filename    Image file name.
  ##
  ## \return Returns image widget on success, None on failure.
  #
  def loadImage(self, filename):
    img = self.openImage(filename)
    if img is not None:
      return ImageTk.PhotoImage(img)
    else:
      return None

  """
    # no file name
    if filename is None or len(filename) == 0:
      return None;
    # absolute file name
    if filename[0] == os.path.sep:
      try:
        return ImageTk.PhotoImage(Image.open(filename))
      except IOError:
        return None
    # relative file name - try python resource(s) first
    if self.m_pyPkg:
      try:
        fqname = resource_filename(self.m_pyPkg, filename)
        try:
          return ImageTk.PhotoImage(Image.open(fqname))
        except IOError:
          pass
      except ImportError:
        pass
    # relative file name - search path for file
    for path in self.m_imagePaths:
      fqname = path + os.path.sep + filename
      try:
        return ImageTk.PhotoImage(Image.open(fqname))
      except IOError:
        continue
    return None
  """

  #
  ## \brief Open image from file.
  ##
  ## \param filename    Image file name.
  ##
  ## \return Returns image widget on success, None on failure.
  #
  def openImage(self, filename):
    # no file name
    if filename is None or len(filename) == 0:
      return None;
    # relative file name - try python resource(s) first
    if self.m_pyPkg:
      try:
        fqname = resource_filename(self.m_pyPkg, filename)
        try:
          return Image.open(fqname)
        except IOError:
          pass
      except ImportError:
        pass
    # relative file name - search path for file
    for path in self.m_imagePaths:
      fqname = path + os.path.sep + filename
      try:
        return Image.open(fqname)
      except IOError:
        continue
    return None


# ------------------------------------------------------------------------------
# Misc. Widget Utilities
# ------------------------------------------------------------------------------

#
## \brief Create compound button.
##
## \param parent    Parent widget.
## \param text      Button text.
## \param image     Loaded image. None for no image.
## \param command   Callback for button push.
## \param fg        Foreground text color.
## \param width     Button width. If icon, then in pixels, else in characters.
##
## \return Returns 'nice' text button key and button widget (key, w).
#
def createCompoundButton(parent, text='', image=None, command=None,
                                  fg='black', width=105):
  key = str.lower(text.replace("\n", "_"))
  w = Button(parent)
  w['text'] = text
  if image:
    w['image']    = image
    w['compound'] = LEFT
    w['padx']     = 0
    w['pady']     = 0
    w['anchor']   = W
    w['width']    = width
  else:
    w['anchor']   = CENTER
    w['width']    = width
  w['fg']       = fg
  w['command']  = command
  return key, w


# ------------------------------------------------------------------------------
# Misc. Utilities
# ------------------------------------------------------------------------------

#
#
## Round to nearest 100th.
#
def round100th(x):
  return math.floor((x + 0.005) * 100.0) / 100.0

#
## Round to nearest 10th.
#
def round10th(x):
  return math.floor((x + 0.05) * 10.0) / 10.0

#
## Degrees to radians.
#
def degToRad(deg):
  return deg / 180.0 * math.pi

#
## Radians to degrees.
#
def radToDeg(rad):
  return rad / math.pi * 180.0
