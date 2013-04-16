{{
  Simple test code.

  This code sets the PID Cog to mode 0 in case of Hall sensor failure on the encoder wheels.

  License: MIT (see included LICENSE file) 


}}
CON
  ' clock settings
  _clkmode=xtal1+pll16x
  _xinfreq = 5000000 
        
  Led = 27

        
  'PID constants
  nPIDLoops = 2
  MotorCnt = nPIDLoops
  MotorIndex = MotorCnt - 1
'  Drive0   = 10           ' Drive 0 address, 2 motors per address
'  Drive1   = Drive0 +1    ' Drive 1

  PIDCTime = 10           ' PID Cycle time ms

  'Serial pins QiK
  TXQ      = 26           ' Serial out to QiC
  RXQ      = 25           ' Serial in
  
  ' Quadrature encoders
  Enc0Pin  = 0            'Start pin
  EncCnt   = 2            'Number of encoders

  '' Serial port 
   CR = 13                      
   LF = 10
   CE = 11                 'CE: Clear to End of line
   TXD = 30
   RXD = 31
   Baud = 115200 '256000 '115200 '250000 '1000000 '115200 '230400 '115200
  
   ' Misc characters
   ST = 36 'dollar sign
   SEP = 44 'comma, used to seperate command params.
   MINUS = 45 'minus
   DOT = 46 'dot

   SETP0 = 10   'Setpoint values for the pid cog, PID will try to maintain this value as Encoder counts per PID cycle. Change if Needed. Set between 1 and 120
   SETP1 = 10

   DEBUG = 1
   
     
OBJ
  t             : "Timing"
  PID           : "PID Connect V5_2"                    ' PID contr. 4 loops. for wheels
  num           : "simple_numbers"                      ' Number to string conversion
  serial        : "FullDuplexSerial_rr005"              ' PC command and debug interface
  STRs          : "STRINGS2hk"
  F32           : "F32"                                 ' float32 support 

var 
  Long error 'Error value, error is filled if there are problems with reading / parsing the message. If the error = 0,
             'was parsed succesfully.          

  Byte SerialCog      'Cog for serial communication
  Byte PIDCog         'Cog for the PID control system
  Byte QiKCog         'Cog for the QIK motor drivers
  Byte F32cog         'cog for floating point calculations
  
    
  Long Setp[MotorCnt] 'Shared with the PID cog, contais the setpoint for each motor.
  Long enco[MotorCnt] 'stores the current motor count
  Long prevActPos[MotorCnt] 'Buffer to store Actual velocity reported by PID cog in cnts/pidcycle
  
pub main  | i
  {{
    Entry point for this system. Starts the serial, PID and floating point cogs.

    This method will continuously call handleSerial 
    Parameters:
      none
    Returns: none and never.

  }} 
  SerialCog:=serial.start(RXD, TXD, 0, Baud)

  serial.str(string("Starting..."))

  PIDCog:=PID.Start(PIDCTime, @Setp,  Enc0Pin, EncCnt, TxQ, RxQ, nPIDLoops) 'thr4, 5 and 6

  pid.setallpidmode(1) '' 1= velocity control 2 = position control


  repeat while PID.GetPIDStatus<>2                      'Wait while PID initializes

  SetPIDPars
  waitcnt(500_000_000+cnt) ''Wait a few seconds for the powersupply to stabelize. 
    
  serial.str(string("Now accepting commands..."))

  repeat until serial.rx == ST                          'Wait until char '&' has been recieved before turning on motors.
  setp[0] := SETP0
  setp[1] := SETP1
  prevActPos[0] := PID.GetActPos(0)
  prevActPos[1] := PID.GetActPos(1)
  t.Pause1s(2)

  repeat while error <> 1
    t.Pause1ms(5)

    Repeat i from 0 to 1
      if setp[i] <> 0
        if setp[i] > 0
          if (PID.GetActPos(i) - prevActPos[i]) < 1
            PID.setallpidmode(0)
            error := 1
        if setp[i] < 0
          if (PID.GetActPos(i) - prevActPos[i]) > 1
            PID.setallpidmode(0)
            error := 1

      if DEBUG    
        Serial.DEC(i)
        Serial.str(string(": Setp: "))
        Serial.DEC(setp[i])
        Serial.str(string(" Actpos: "))
        Serial.DEC(PID.GetActPos(i))
        Serial.str(string(" prevpos: "))
        Serial.DEC(prevActPos[i])
        Serial.str(string(" Actvel: "))
        Serial.DEC(PID.GetActvel(i))
        Serial.TX(13)
    
      prevActPos[i] := PID.GetActPos(i)

  Serial.str(string("PID COG STOPPED DUE TO HALL SENSOR FAILURE"))
  Serial.TX(13)
  
  

PRI SetPIDPars
  {{
    Sets various PID parameters of the PID cog.
    Parameters: none
    Returns: none
    Modifies: none.
  }}
  'Set control parameters wheels
  PID.SetKi(0,800)
  PID.SetK(0,1000)
  PID.SetKp(0,1000)
  PID.SetIlimit(0,1000)
  PID.SetPosScale(0,1)
  PID.SetFeMax(0,200)
  PID.SetMaxCurr(0,4500)
  
  PID.SetKi(1,800)
  PID.SetK(1,1000)
  PID.SetKp(1,1000)
  PID.SetIlimit(1,1000)
  PID.SetPosScale(1,1)
  PID.SetFeMax(1,200)
  PID.SetMaxCurr(1,4500)

                                     