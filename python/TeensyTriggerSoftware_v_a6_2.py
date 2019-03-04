# -*- coding: utf-8 -*-
"""
Created on Tue Jul  3 14:26:12 2018

@author: Garwing
"""

##  Version a5
#   Based on serialecho_ui3.ui and serialecho_ui3.py implementations by BH. 
#   This version is intended to interface with Teensy running TeensyTriggerFirmware_v_a5.ino
#
#
#
##  Changelog:
#   28 Aug 2018:    GWT. Grouped UI connect commands in init into logical groups according to function
#                   Deleted old echo functions previously used for speed testing serial comms
#                   Deleted commented out code blocks that seemed obsolete
#                   Deleted _translate("Form", "Input Enabled: Disable") etc. Unclear why _translate is needed
#   29 Aug 2018     BH. Changed comport to 3 (was set to 5 and needed it on 3... might be a problem when switching computers)
#
##  TO DO:
#           - What if default enable/disable state was changed? Would inputDisablePushButtondisabled based on button_clicks counter still work?
#               Better to change to a state variable instead of using a counter?


import serial, time, sys, io
from PyQt5 import QtCore, QtGui, QtWidgets
import TeensyTriggerSoftwareUI_v_a6_2 as seui
import threading
import numpy as np

class teensyTriggerApp(QtWidgets.QWidget, seui.Ui_Form):                           #set up daughter class of two imported classes from imported code
    def __init__(self, teensyTrigger):                                          #run when class initiated
        super(self.__class__, self).__init__()                                  #initialize the Qt widget code. Dies on second call.
        self.setupUi(self)                                                      #call parameters to create Qt widget code
                    
        self.t = teensyTrigger                                                  #create a shortcut name to call the Teensy Trigger Class
        self.button_clicks=0                                                    #establish variable to keep track of number of times the inputDisablePushButton has been pushed to use as a refence      
        
        #SET LEVEL
        self.thresholdLevelDoubleSpinBox.setKeyboardTracking(False)                                             #only send value entered into Spin box once enter has been sent
        self.thresholdLevelDoubleSpinBox.valueChanged.connect(self.thresholdLevelDoubleSpinBoxvalueChanged)     #trigger thresholdLevelDoubleSpinBoxvalueChanged when spin box value changed on the Qt widget
#        self.thresholdLevelDoubleSpinBox.valueChanged.connect(self.readLevelPushButtonClicked)                  #read threshold level after new level set
        self.thresholdLevelVerticalSlider.valueChanged.connect(self.thresholdLevelVerticalSlidervalueChanged)   #trigger thresholdLevelVerticalSlidervalueChanged when slider value changed on the Qt widget
#        self.thresholdLevelVerticalSlider.sliderReleased.connect(self.readLevelPushButtonClicked)               #read threshold level after setting it
                                                              
        #READ LEVEL                                                    
        self.readLevelPushButton.clicked.connect(self.readLevelPushButtonClicked)                               #trigger readLevelPushButtonClicked when the read level button is clicked on the Qt widget
        
        #OPTIMISE LEVEL
        self.optimisePushButton.clicked.connect(self.doOptimise)                                        
                                                
        #INPUT ENABLE/DISABLE
        self.inputDisablePushButton.pressed.connect(self.inputDisablePushButtondisabled)                        #connect ui button to function that controls if triggers are enabled or not
        self.inputDisablePushButton.setText("Enabled")                       #set starting text of ui inputDisablePushButton
                                           
        #SET GAIN
        self.Gaindropdown.activated[str].connect(self.gainChanged)                                              #when new level selected from drop down list, activate function to send this value to the teensy
        
        #Request Frequency
        self.FreqPushButton.clicked.connect(self.th_measTriggerFreq)                        #measures average freq over an user set amount of time
                                           
        #display output
        self.show()
        
        #Update vals when connecting to already-running Teensy
        self.readLevelPushButtonClicked()
        self.readGain()
        
    def closeEvent(self, event):
        self.t.close()
        
                    
    def thresholdLevelDoubleSpinBoxvalueChanged(self):  
        th = self.thresholdLevelDoubleSpinBox.value()               #set th (threshold value) equeal to spinbox value  in volts
        self.thresholdLevelVerticalSlider.setValue(th*1000.0)       # verticalSlider only takes int positions. Slider range is set to 0...5000., set level on slider to same as spinbox proportionally
        self.setThresholdRef(th)                                    #trigger setThresholdRef with set threshold value
                                                  
    def thresholdLevelVerticalSlidervalueChanged(self):
        th = self.thresholdLevelVerticalSlider.value()              #set th (threshold value) equeal to slider value in volts
        self.thresholdLevelDoubleSpinBox.setValue(th/1000.0)        # verticalSlider only takes int positions. Slider range is set to 0...5000., set level on spinbox to same as slider proportionally
        self.setThresholdRef(th)                                    #trigger setThresholdRef with set threshold value
        
    def setThresholdRef(self, th):
        #th is the threshold value in V
#        self.lcdNumber.display(self.t.query("Set "+str(th)))         #send Set instruction plus the threshold value to serial port through the echo fucntion set set the threshold on the teensy, read back the set threshold value and  display it on the lcd display on the Qt widget
        self.t.query("Set "+str(th))
        
    def readLevelPushButtonClicked(self):
        setValStr = self.t.query(str("Set ?"))[0:5]
        setVal = float(setValStr)
#        self.lcdNumber.display(setValStr)    #send the Thresh intruction to the serail port through the query function and read back the set threshold value and  display it on the lcd display on the Qt widget
        self.thresholdLevelDoubleSpinBox.setValue(setVal)    # update spinbox
        self.thresholdLevelVerticalSlider.setValue(setVal*1000.0)

    def inputDisablePushButtondisabled(self):
        self.button_clicks +=1                                      #each time the inputDisablePushButton is pressed the value of self.button_clicks inceases by one to keep track of button state
#        print(self.button_clicks)
        if (self.button_clicks%2 == 0): #check state of inputDisablePushButton based on the number of clicks
            rstring = self.t.query(str("Enab 1")) #enables interupts on teensy
#            print("enable")
            self.inputDisablePushButton.setText("Enabled")#changes text on ui to match state of teensy
        else:
            rstring = self.t.query(str("Enab 0")) #disables interupts on teensy
#            print("Disable")
            self.inputDisablePushButton.setText("Disabled")#changes text on ui to match state of teensy
        self.errorChk(rstring)
    
    def gainChanged(self):
        text=self.Gaindropdown.currentText() #take the value form the drop down list
        rstring = self.t.query(str("Gain "+str(text))) #send this value to the teensy
        self.errorChk(rstring)
        
    def readGain(self):
        rstring = self.t.query('GAIN ?')
        if rstring == '00': g='1'
        if rstring == '01': g='2'
        if rstring == '10': g='4'
        if rstring == '11': g='8'
        ind = self.Gaindropdown.findText(g,QtCore.Qt.MatchFixedString)
        self.Gaindropdown.setCurrentIndex(ind)
        
    def updateStatusText(self, string):                             #lets user know if the commands are successful even if there is no clear/ immediate function response
        self.statusLabel.setText(string)
        self.statusLabel.adjustSize()
        
    def errorChk(self, replyStr):  
        # expect "0" from Teensy for operations that would otherwise not return anything in event of success
        if replyStr == '0': 
            self.updateStatusText("Success.")
        else:
            self.updateStatusText("Error: "+replyStr)
            
    def th_measTriggerFreq(self):                                   #runs the freq measurement on a "seperate" string from the rest of the functions so it doesnt hang the programs
        thread = threading.Thread(target=self.measTriggerFreq)
        thread.daemon = True
        thread.start()
    
    def measTriggerFreq(self):
        self.FreqPushButton.setEnabled(False)                       #disables Freq Button to prevent multiples freq timers going at once, will work, but report incorrect freq
        self.t.query(str("FREQ S"))                                 #starts freq measurement
        time.sleep(self.TimeSetDoubleSpinBox.value())               #delays for time user wants to measure
        r = self.t.query(str("FREQ M"))
        self.Freqdisplay.display(int(float(r)))                            #ends freq measurement and displays freq
        self.FreqPushButton.setEnabled(True)                        #renables Freq button so user can check frequency again
        return float(r)
        
    def th_doOptimise(self):                                        #doesnt work. 
        thread = threading.Thread(target=self.doOptimise)
        thread.daemon = True
        thread.start
    
    def doOptimise(self):                                           #hangs program until done. 
        o = optimiser(self)
        o.optimiseByMax(minFreq=self.minFreqSpinBox.value())
         
class optimiser():
    #optimise the preamp gain and trigger level
    def __init__(self, ttapp):
        self.ttapp = ttapp
        
    def sweep(self, params):
        # params is a dict of form:
        #    params = dict(
        #        minLev = 0.02,       # [V]. Min sweep limit
        #        maxLev = 4.98,       # [V]. Max sweep limit
        #        levRes = 0.02,       # [V]. Step size
        #        gate   = 0.10,         # [s]. gate time per freq read
        #        )
        
        #sweep params
        start = params['minLev']
        stop = params['maxLev']
        res = params['levRes']
        numpnts = int(np.floor((stop-start)/res))
        
        # override front panel gate time
        gate = params['gate']
        self.ttapp.TimeSetDoubleSpinBox.setValue(gate)
        print('Optimiser: Gate time set to '+str(gate)+' s.')
        
        # disable measure button
        self.ttapp.FreqPushButton.setEnabled(False)

        # array to hold the results
        r = np.zeros((numpnts,2))    
        r[:,0] = np.linspace(start, stop, numpnts)
        
        print('Optimiser: ')
        for i in r:
            self.ttapp.thresholdLevelDoubleSpinBox.setValue(i[0])       # set spindbox value
            self.ttapp.thresholdLevelDoubleSpinBoxvalueChanged()        # update display
            #i[1] = np.random.random()
            i[1] = self.ttapp.measTriggerFreq()                         
            #print('\t'+str(i[0])+'V\t'+str(i[1])+'Hz')
            print('\t %1.3f V,\t %5i Hz' % (i[0], i[1]))
            sys.stdout.flush()
        
        
        self.ttapp.FreqPushButton.setEnabled(True)
        return r
            
    
    def optimiseByMax(self, minFreq=1, doCheck=True, doPreAmp=True):
        # sweep through range of threshold levels. 
        # determine max threshold voltage that gives non-zero trigger freq
        # check if trigger freq is at least minFreq
        #
        # TODO:
        #   - handle saturation (ie max trigger rate for all values of threshold)
        
        
        # sweep params
        params = dict(
                minLev = 0.02,       # [V]. Min sweep limit
                maxLev = 4.98,       # [V]. Max sweep limit
                levRes = 0.04,       # [V]. Step size
                gate   = 1/minFreq,      # [s]. gate time per freq read
                )

        r = self.sweep(params)                  # returns an array [[v1, f1],[v2, f2]...] where v is the threshold voltage, f is the trigger freq
        
        # process result of sweep              
        f = r[:,1]                              # get freqs
        f = f-minFreq                           # make all freqs below the min val negative
        f = [0 if i < 0 else i for i in f]      # make all negative vals zero
        try:
            idx = np.amax(np.nonzero(f))        # get index of last non-zero freq
        except:
            print('Optimiser exit code: -1')    # no triggers. All values in f are zero.
            print('\tNo triggers.')
            return -1
        
        buffFactor = 0.9                        # set the threshold level just a little under max value found during sweep
        t = r[idx,0]*buffFactor
        
        if doPreAmp:
            #check whether it is worth increasing preamp gain
            # TODO: Must also check current gain level?
            m = params['maxLev']
            if t < (m/8):   g=8
            elif t < (m/4): g=4
            elif t < (m/2): g=2
            
            t = t*g                                             # increase trigger val
            self.ttapp.t.query(str("Gain "+str(g)))             # send this value to the teensy
            self.ttapp.readGain()                               # update display
            print('Optimiser: ')
            print('\tPreamp gain set to %i'% (g))
            
             
        self.ttapp.thresholdLevelDoubleSpinBox.setValue(t)      # set spindbox value
        self.ttapp.thresholdLevelDoubleSpinBoxvalueChanged()    # update display  
        
        v = self.ttapp.measTriggerFreq()
        if doCheck and v < minFreq:
            #check if trig freq is indeed above minFreq
            print('Optimiser exit code: -2.')
            print('\tMin freq not attained.')
            return -2
        print('Optimiser success.')
        print('\tThreshold set to %1.3f'% (t))
        return 0
            
        
        



class COM_fastTeensy:
    """Functions for talking to a generic COM port
    """
    
    def __init__(self, portnum, baud=9600, timeout=.3, opendelay=2):
        self.ser = serial.Serial('COM'+str(portnum), baud, timeout=timeout)     #immediately opens the com port. Nb. baud is ignored for USB
        time.sleep(opendelay)                                                   #wait for the serial port to open. Arduino re-loads code onto board EVERYTIME com port is opened! Needs 2 seconds!
        print(str(self.ser.port)+' opened.')
                  
    def close(self):
        self.ser.close()                                                        #closes com port
        print(str(self.ser.port)+' closed.')
        
    def write(self, sendStr):  
        self.ser.write((sendStr+'\r').encode())                                 # Arduino code (SetThresholdArduino) is explicitly waiting for '\r' as EOM character
                      
    def query(self, sendStr, readdelay=0):
        self.write(sendStr)                             #write enter value to serial port (should be a defined command or nothing will happen on the arduino)
        if (readdelay!= 0): time.sleep(readdelay)       #if delay is needed to properly read input from ui, wait right amaound                          
        data = self.ser.readline()                      # default eom char is '\n' ??                   
        return data.decode().rstrip('\n\r ')            # convert to ascii and strip trailing NL, CR and whitespace
      


class TeensyTrigger(COM_fastTeensy):
    pass    

        

        
   

if __name__ == '__main__':
    Teensy_addr = 5                     #choose which comport to use
    try:
        t = TeensyTrigger(Teensy_addr)      
    except:
        print("Teensy not found at COM "+str(Teensy_addr))

# pulls up widget
    app = QtWidgets.QApplication(sys.argv)
    form = teensyTriggerApp(t)                                          
    app.exec_()  



