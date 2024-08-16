        # Program Multi_4Chan_MQ4_K96_Hawk_INIR
        # Prog Version V3 Additional Temperature Arduino
        # Data are received automatically from INIR
        # Results are displayed on PC screen and written to File
        # Filename ID1,COMX,yrs,mths,dys.txt
        # V3 Lets the user to selectSensor ID
        # Data format ID1 Com port, Time INIR,Hawk
        # This version allows user to select individual Com Ports
        # Only one INIR device is written to and read from

from datetime import date
from datetime import datetime
import serial
import time
import sys
import signal
import binascii

def signal_handler(signal, frame):
        print("closing program")
        SerialPort.close()
        sys.exit(0)
        
SerialPort1 = serial.Serial("COM7",9600,stopbits=1,timeout=1.5) # MQ4
SerialPort5 = serial.Serial("COM3",9600,stopbits=1,timeout=5) # Temperature probe
SerialPort4 = serial.Serial("COM4",38400,stopbits=1,timeout=1.5) # INIR
time.sleep(4)

INIR_OutgoingData="[H]"  # Set INIR in Read/Demand Mode
SerialPort4.write(bytes(INIR_OutgoingData,'utf-8'))
time.sleep(1)


Flag = 0
while (1):
        try:
                SerialPort2 = serial.Serial("COM5",115200,stopbits=2,timeout=1.5) # K96
                OutgoingData= (0x68,0x04,0x00,0x00,0x00,0x08,0xf8,0xf5)  # Device Address 0x44
                
                #SerialPort1.write(bytes('S','utf-8'))
        except KeyboardInterrupt:
                print("Closing and exiting the program")
                #SerialPort1.close()
                sys.exit(0)
#  Read from MQ4 Sensor   *** Sort out Results
        SerialPort1.write(bytes('S','utf-8'))
        Flag = 0
        while(Flag == 0):
                InData = SerialPort1.readline().decode('utf-8').rstrip()
                Flag = 1
                #SerialPort1.close()

        x= InData.split()
        try:
                mq1 = (x[1])
                mq2 = (x[2])
                mq3 = (x[3])
               
        except:
                mq1 = -9999
                mq2 = -9999
                mq3 = -9999 
        print('Got TGS')
        
#   Data collection  reads 20 bytes of K96 data

        SerialPort2.write(OutgoingData)
        Flag = 0
        line = SerialPort2.read(21)   # read 16
        a = (line[3]*256)+line[4]
        b = (line[5]*256)+line[6]
        c = (line[7]*256)+line[8]
        d = (line[9]*256)+line[10]
        e = (line[11]*256)+line[12]
        f = (line[13]*256)+line[14]
        g = (line[15]*256)+line[16]
        h = (line[17]*256)+line[18]
        SerialPort2.close()
        print('Got K96')
        
#  Get Hawk Data

        SerialPort3 = serial.Serial("COM6",9600,stopbits=1,timeout=5) # Hawk
        Indata = SerialPort3.readline()
        #print('Hawk ',Indata.decode('utf-8'))  # Test code
        In1 = Indata[2:10]
        #print(In1)    #Test code
        In2 = In1.lstrip()
        In3 = In2.rstrip()
        In4 = In3.decode('utf-8')

        try:
                Hawk = int(In4)
        except:
                Hawk = -9999
        SerialPort3.close()        
        print('Got Hawk')
        
# Read Temperature probe
        #SerialPort5 = serial.Serial("COM19",9600,stopbits=1,timeout=5) # Temperature probe
        Indata = SerialPort5.readline().decode('utf-8')
        y = Indata.split()
        try:
                RH = (y[1])
                TTT = (y[2])         
        except:
                RH = -9999
                TTT = -9999
                
        print('Got Temperature')

        
# Read INIR
        INIR_Cmd = "[Q]"
        SerialPort4.write(bytes(INIR_Cmd,'utf-8'))
        time.sleep(1)                  
        Flag = 0
        while(Flag == 0):
                 
                IncomingData=SerialPort4.readline()   #  [
                if( IncomingData == b'\r0000005b\n'):  #THIS IS HEADER STRING
                        Flag = 1
        In1=SerialPort4.readline() # Methane
        In2=SerialPort4.readline() # Error flag
        In3 = SerialPort4.readline()#Temperature
        In4=SerialPort4.readline() # Reference
        In5=SerialPort4.readline()
        In6=SerialPort4.readline()
        In7=SerialPort4.readline()
        
# Sort out input string to 3 individual samples
        if(In1):
                D2 = In1[1:9]
                T1 = In3[1:9]
                R1 = In4[1:9]
                #print(D2)   # Test code
                #print(T1)
# Convert data from Hex to dacimal
              
                try:
                        
                        D2I = int(D2,16)
                        TT = (int(T1,16))/100
                        Ref = int(R1,16)
                except:
                        D2I = -9999
                        TT = -9999
                        Ref = -9999
                        
        print('Got INIR',' ',D2I)
# Get PC time                
        current_date = date.today()
        current_time = datetime.now()
        #print('Got time')   # Test code
       
# extracting the current year, month and day
       
        dys = current_date.day
        mths = current_date.month
        yrs = current_date.year
        yrs = yrs - 2000
        hh = current_time.hour
        mm = current_time.minute
        ss = current_time.second
#  Sorting out filenames and add 0 if < 10
        yrsa = str(yrs)
        mthsa = str(mths)
        if mths < 10:
                mthsa = '0'+ mthsa
        dysa = str(dys)
        if dys < 10:
                dysa = '0'+dysa
        
        fname1 = "MULTI_Sensor "+yrsa+mthsa+dysa+".txt"
       
        #print(fname1)   #  Test code
       
# Assign filename
        text_file1 = open(fname1,'a')

# Write results to file	        
        text_file1.write  (str(hh)+":"+str(mm)+':'+str(ss)+' '+str(mq1)+' '+str(mq2)+' '+str(mq3)+' '+str(a)+" "+str(b)+" "+str(c)+" "+str(d)+" "+str(e)+" "+str(f)+" "+str(g)+" "+str(h)+" "+str(Hawk)+" "+str(D2I)+" "+str(TT)+" "+str(RH)+" "+str(TTT)+ '\n')
        text_file1.close()
# Display results on PC
        print('Time ',str(hh),':',str(mm),':',str(ss),' ',str(mq1),' ',str(mq2),' ',str(mq3),' ',str(a),' ',str(b),' ',str(c),' ',str(d),' ',str(e),' ',str(f),' ',str(g),' ',str(h),' ',str(Hawk),' ',str(D2I),' ',str(TT),' ',str(RH),' ',str(TTT))       
    
# Pause until next sample this value may require changing depending on output speed of INIR        
        time.sleep(2)

