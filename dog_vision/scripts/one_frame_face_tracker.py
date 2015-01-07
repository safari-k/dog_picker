# -*- coding: utf-8 -*-
"""
Created on Thu Jan  1 16:12:10 2015

@author: keith
"""

import rospy
import cv
import sys
from face_tracker import PatchTracker

class OneFrame(PatchTracker):
    def __init__(self, node_name):
        PatchTracker.__init__(self, node_name)
       

        
def image_callback(self, data):
    self.image_sub.unregister()
    #super(OneFrame, self).image_callback(self, data)
    
def depth_callback(self, data):
    self.depth_sub.unregister()
    #super(OneFrame, self).image_callback(self, data)   
    
def main(args):
    """ Display a help message if appropriate """
    help_message =  "Hot keys: \n" \
          "\tq - quit the program\n" \
          "\tc - delete current features\n" \
          "\tt - toggle text captions on/off\n" \
          "\tf - toggle display of features on/off\n" \
          "\tn - toggle \"night\" mode on/off\n" \
          "\ta - toggle auto face tracking on/off\n"

    print help_message
    
    """ Fire up the Face Tracker node """
    OneFrame("one_frame_face_tracker")

    try:
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down face tracker node."
      cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)        