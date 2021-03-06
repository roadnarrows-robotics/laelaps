###############################################################################
#
#
# Package:   RoadNarrows Robotics Laelaps Robotic Mobile Platform ROS Package
#
# Link:      https://github.com/roadnarrows-robotics/laelaps
#
# ROS Node:  laelaps_panel
#
# File:      PanelConfig.py
#
## \file 
##
## $LastChangedDate$
## $Rev$
##
## \brief Laelaps panel configuration dialog and XML classes.
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

import xml.parsers.expat as expat

from laelaps_control.Utils import *

# ------------------------------------------------------------------------------
# Class ConfigDlg
# ------------------------------------------------------------------------------

#
## \brief Laelaps Panel configuration dialog.
#
class ConfigDlg(Toplevel):
  ## \brief User's home directory.
  Home = os.path.expanduser("~")

  ## \brief User-specific configuration directory (in home directory).
  UserDirName = ".roadnarrows"

  ## \brief laelaps_panel application configuration file name.
  ConfigFileName = "laelaps_panel.xml"

  PathNameDft = Home + os.path.sep + \
                UserDirName + os.path.sep + \
                ConfigFileName

  ## \brief Configuration default.
  ConfigDft = \
  {
    'governor': 0.2,
  }

  #
  ## \brief Constructor.
  ##
  ## \param cnf     Configuration dictionary.
  ## \param kw      Keyword options.
  #
  def __init__(self, master=None, cnf={}, **kw):

    # initialize dialog data
    kw = self.initData(kw)

    Toplevel.__init__(self, master=master, cnf=cnf, **kw)

    self.title("laelaps_panel configuration")

    # parent widget's window geometry
    if master is not None:
      self.m_parentGeo = [master.winfo_width(), master.winfo_height(),
                          master.winfo_rootx(), master.winfo_rooty()]
    else:
      self.m_parentGeo = [400, 400, 400, 400]

    #print 'DBG: Parent geometry = {0}x{1}+{2}+{3}'.format(*self.m_parentGeo)

    # Set a good location for this dialog overlaying on top of the parent's
    # geometry. This is a compromise in that this dialog's geometry has not
    # been determined yet.
    glist= [self.m_parentGeo[2] + self.m_parentGeo[0]/4,
            self.m_parentGeo[3] + self.m_parentGeo[1]/6]
    self.geometry('+{0}+{1}'.format(*glist))

    # create and show widgets
    self.createWidgets()

    # allows the enter button to fire either button's action
    self.m_bttnCancel.bind('<KeyPress-Return>', func=self.close)

    # allows us to customize what happens when the close button is pressed
    self.protocol("WM_DELETE_WINDOW", self.close)

    #
    # Modal diagle settings.
    #
    # set the focus on dialog window (needed on Windows)
    self.focus_set()

    # make sure events only go to our dialog
    self.grab_set()

    # make sure dialog stays on top of its parent window (if needed)
    self.transient(master)

    # display the window and wait for it to close
    self.wait_window(self)

  #
  ## \brief Initialize class state data.
  ##
  ## \param kw      Keyword options.
  ##
  ## \return Modified keywords sans this specific class.
  ##
  def initData(self, kw):
    self.m_saved    = False
    self.m_filename = None
    if kw.has_key('config'):
      self.m_config = kw['config']
      del kw['config']
    else:
      self.m_config = ConfigDft;
    return kw

  #
  ## \brief Create gui widgets with supporting data and show.
  #
  def createWidgets(self):
    frame = Frame(self)
    frame.grid(row=0, column=0)

    row = 0

    # top heading
    w = Label(frame)
    helv = tkFont.Font(family="Helvetica",size=24,weight="bold")
    w['font']   = helv
    w['text']   = 'Configuration'
    w['anchor'] = CENTER
    w.grid(row=row, column=0, sticky=E+W)

    row += 1
    wframe = Frame(frame)
    wframe.grid(row=row, column=0)

    #
    # Nada
    #
    subrow = 0
    w = Label(wframe, text="No configuration")
    w.grid(row=subrow, column=0, padx=0, pady=0, sticky=E)

    #
    # buttons
    #
    row += 1
    wframe = Frame(frame)
    wframe.grid(row=row, column=0)

    # cancel button
    w = Button(wframe, width=10, text='Cancel', command=self.close)
    w.grid(row=0, column=0, padx=2, pady=5)
    w['anchor']  = CENTER
    self.m_bttnCancel = w

    # save button
    w = Button(wframe, width=10, text='Save', command=self.ok)
    w.grid(row=0, column=1, padx=2, pady=5)
    w['anchor']  = CENTER
    self.m_bttnOk = w

  #
  ## \brief Destroy window callback.
  #
  def ok(self):
    val = self.m_varGovernor.get()
    val /= 100.0
    if val < 0.0:
      val = 0.0
    elif val > 1.0:
      val = 1.0
    self.m_config['governor'] = val

    dirname = ConfigDlg.Home + os.path.sep + ConfigDlg.UserDirName
    if not os.path.isdir(dirname):
      try:
        os.mkdir(dirname, 0755)
      except OSError, err:
        print "%s: %s" % (dirname, err)
        return
    self.m_filename = dirname + os.path.sep + ConfigDlg.ConfigFileName
    xml = ConfigXml()
    xml.save(self.m_filename, self.m_config)
    self.close()
    self.m_saved = True

  #
  ## \brief Destroy window callback.
  #
  def close(self):
    self.destroy()


# ------------------------------------------------------------------------------
# Class ConfigXml
# ------------------------------------------------------------------------------

##
## \brief Application laelaps_panel configuration xml class.
##
class ConfigXml():
  def __init__(self):
    self.m_curData      = ""
    self.m_config       = None

  def parse(self, pathname=None):
    if pathname is None:
      pathname = ConfigDlg.PathNameDft;
    self.m_config = ConfigDlg.ConfigDft
    try:
      fp = open(pathname, 'r')
    except IOError, err:
      return self.m_config
    parser = expat.ParserCreate()
    parser.returns_unicode      = False
    parser.StartElementHandler  = self.onElemStart
    parser.CharacterDataHandler = self.onElemData
    parser.EndElementHandler    = self.onElemEnd
    self.m_stack = []
    try:
      parser.ParseFile(fp)
    except expat.ExpatError as e:
      print "%s: %s" % (pathname, expat.ErrorString(e.code))
    fp.close()
    return self.m_config

  def save(self, pathname, config):
    try:
      fp = open(pathname, 'w')
    except IOError, err:
      print "%s: %s." % (pathname, err)
      return
    fp.write("""\
<?xml version="1.0" encoding="utf-8"?>
<?xml-stylesheet type="text/xsl" href="http://roadnarrows.com/xml/PanTilt/1.0/laelaps.xsl"?>

<!-- RoadNarrows Laelaps Top-Level Configuration -->
<laelaps xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:noNamespaceSchemaLocation="http://roadnarrows.com/xml/PanTilt/1.0/laelaps.xsd">

  <!-- laelaps_panel configuration -->
  <laelaps_panel>
""")

    self.writeTree(fp, 4, config);

    fp.write("""\
  </laelaps_panel>

</laelaps>
""")

    fp.close()

  def writeTree(self, fp, indent, config):
    s = ' ' * indent
    for key,data in config.iteritems():
      if type(data) is dict:
        fp.write("{0}<{1}>\n".format(s, key))
        self.writeTree(fp, indent+2, data)
        fp.write("{0}</{1}>\n".format(s, key))
      else:
        fp.write("{0}<{1}>{2}</{3}>\n".format(s, key, config[key], key))

  def onElemStart(self, elem, attrs):
    #print "start-of-element", "<%s> attrs=%s" % (elem, repr(attrs))
    self.m_stack.append(elem)
    #print 'onElemStart', self.m_stack
    self.m_curData  = ""

  def onElemData(self, data):
    #print "char-data", repr(data)
    self.m_curData += data

  def onElemEnd(self, elem):
    #print "end-of-element", "<\%s>" % (elem)
    # <laelaps> <laelaps_panel> <x>
    if len(self.m_stack) == 3:
      elem = self.m_stack[2]
      if self.m_config.has_key(elem):
        self.m_config[elem] = self.cvtToFloat(self.m_curData.strip())
    try:
      self.m_stack.pop()
    except:
      pass
    #print 'onElemEnd', self.m_stack
    self.m_curData  = ""

  def cvtToBool(self, data):
    if data.lower() == 'true':
      return True
    elif data.lower() == 'false':
      return False
    else:
      try:
        i = int(data)
        if i:
          return True
        else:
          return False
      except ValueError:
        return False

  def cvtToFloat(self, data):
    try:
      return float(data)
    except (TypeError, ValueError):
      return 0.0
