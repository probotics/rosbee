{{
  Entry point for the Rosbee II Parallax control board.

  This is the cog which communicates with the on board notebook with ROS, using the standard protocol for serial communication.
  See http://lectoraatmechatronica.wikispaces.com/file/view/Serieel%20protocol%20voor%20robotica%20r1.2.pdf/381442566/Serieel%20protocol%20voor%20robotica%20r1.2.pdf
  (in Dutch, will be translated soon)
  This implementation uses a subset of available commands of the protocol. The messages implemented are:
  '$0\r' -> halt robot
  '$1\r' -> start robot
  '$2,vx,rot\r', ie '$2,100,-100\r' -> Turn / move robot with specified values vx and rot.

  vx is the forward velocity, in mm/s.
  rot is the rotational speed in mrad/s.

  This implementation should echo the current values using the same protocol (to be implemented).

  Heartbeat / watchdog timer: tbd.

  Reversions:
  0.1: DV - basic implementation, without watchdog or command echos.

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

   SERIAL_MESSAGE_MAX_LENGTH = 64

   WHEEL_BASE_WIDTH = 0.368
   MM_PER_S_TO_CNTS_PER_PIDCYCLE = 20.00
   DEBUG = 0 'Debug flag for additional spam on the serial terminal.
   REPORT = 0 'Flag to enable or disable reporting setPL setPR actVL actVR
  
OBJ
  t             : "Timing"
  PID           : "PID Connect V5_2"             ' PID contr. 4 loops. for wheels
  num           : "simple_numbers"                      ' Number to string conversion
  serial        : "FullDuplexSerial_rr005"              ' PC command and debug interface
  STRs          : "STRINGS2hk"
  F32           : "F32" ' float32 support 

var 
  Byte SerialCog ' Cog for serial communication
  Byte SerialMSG[SERIAL_MESSAGE_MAX_LENGTH] ' Buffer for the incoming string
  Byte SerialMSGOUT[SERIAL_MESSAGE_MAX_LENGTH] ' Buffer for the outgoing string
  Byte ParserBuffer[SERIAL_MESSAGE_MAX_LENGTH] ' Buffer for the parser
  Byte p ' Pointer to a position in the incoming string buffer
  Long error 'Error value, error is filled if there are problems with reading / parsing the message. If the error = 0,
             'was parsed succesfully.          
  
  Byte PIDCog 'Cog for the PID control system
  Byte QiKCog 'Cog for the QIK motor drivers
  Byte F32cog ''cog for floating point calculations
  
    
  Long Setp[MotorCnt] 'Shared with the PID cog, contais the setpoint for each motor.
  Long enco[MotorCnt] 'stores the current motor count
  Long prevActPos[MotorCnt] 'Buffer to store Actual velocity reported by PID cog in cnts/pidcycle

  Long actVelMMS 'Long used to store actual speed in mm/s for feedback over serial
  Long actVelRadS 'Long used to store actual rotation speed in mrad/s for feedback over serial 

  

pub main
  {{
    Entry point for this system. Starts the serial, PID and floating point cogs.

    This method will continuously call handleSerial 
    Parameters:
      none
    Returns: none and never.

  }} 
  SerialCog:=serial.start(RXD, TXD, 0, Baud)
  F32cog:=f32.start
  
  serial.str(string("Starting..."))
  PIDCog:=PID.Start(PIDCTime, @Setp,  Enc0Pin, EncCnt, TxQ, RxQ, nPIDLoops) 'thr4, 5 and 6

  pid.setallpidmode(1) '' 1= velocity control 2 = position control


  repeat while PID.GetPIDStatus<>2                      'Wait while PID initializes

  SetPIDPars
  waitcnt(500_000_000+cnt) ''Wait a few seconds for the powersupply to stabelize. 
  'setp[0] :=0
  'setp[1] := 10
  'prevActPos[1] := PID.GetActPos(1)


  
  serial.str(string("Now accepting commands..."))

  repeat until serial.rx == ST
  setp[0] :=0
  setp[1] := 10
  prevActPos[1] := PID.GetActPos(1)
  t.Pause1s(2)



  
  repeat
    
    'setp[1] := 10
    't.Pause1ms(1000)
     
    't.Pause1s(5)
    'setp[1] :=-14
    't.Pause1s(5)
    't.Pause1ms(20)
   ' if PID.GetActVel(1) < 1
    ' PID.setallpidmode(0)
    t.Pause1ms(10)
    if (PID.GetActPos(1) - prevActPos[1]) < 1
      PID.setallpidmode(0)
    Serial.str(string("Setp: "))
    Serial.DEC(setp[1])
    Serial.str(string(" Actpos: "))
    Serial.DEC(PID.GetActPos(1))
    Serial.str(string(" prevpos: "))
    Serial.DEC(prevActPos[1])
    
    Serial.str(string(" Actvel: "))
    Serial.DEC(PID.GetActvel(1))
    Serial.TX(13)
    
    prevActPos[1] := PID.GetActPos(1)
  
  

PRI handleSerial | val, i, j, messageComplete
  {{
    
  }}

  bytefill(@SerialMSG,0,SERIAL_MESSAGE_MAX_LENGTH) ''empty SerialMSG     
  '!outa[LED]
  i := 0 ' iterator counter for filling SerialMSG
  j := 0  'iterator for sending back te contents of SerialMSG
  error:=0 'resets the error value to 0, as this is a new serial message 
  messageComplete := false
  repeat until serial.rx == ST 'Busy wait until a '$' has been received
  repeat until messageComplete 'Busy wait until a termination char has received or an error has occurred   
    val := serial.rx
    if (val == CR)
      messageComplete := true
      error := 0
    if(i == SERIAL_MESSAGE_MAX_LENGTH and not messageComplete)' message buffer full and no termination char found.
      messageComplete := true
      error |= (2 << 0)
    SerialMSG[i] := val
    
    i++
  ''SerialMSG[i+1] := 0 ''String afsluiten  
  '!outa[LED]  

  if(DEBUG)
    repeat until i == j ''echo Serial MSG
      serial.tx(SerialMSG[j])
      j++
          
  if(error == 0)
    case SerialMSG[0] ' 0 is the first character after the '$' in the message.
      "0":  'halt robot
        serial.str(string("$0")) 
        setp[0] :=0
        setp[1] :=0
      "1":  'start robot
        serial.str(string("$1"))
        'setp[0] :=14
        setp[1] :=14                       
      "2":
        'serial.str(string("$2")) ''This requires further string parsing.              
      other:
        serial.str(string("Unexpected message. Halting."))
        '' disableWheels //todo
  else
    serial.str(string("Error with errno: "))
    serial.dec(error)
  
  
       

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

                                     