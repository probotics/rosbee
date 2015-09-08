CON
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
   CmdLen = 10
   ' Misc characters
   ST = 36 'dollar sign
   SEP = 44 'comma, used to seperate command params.
   MINUS = 45 'minus
   DOT = 46 'dot

   SERIAL_MESSAGE_MAX_LENGTH = 64   
  
OBJ
  t             : "Timing"
  PID           : "PID Connect V5_2"             ' PID contr. 4 loops. for wheels
  num           : "simple_numbers"               ' Number to string conversion
  serial        : "FullDuplexSerial_rr005"       ' PC command and debug interface
  STRs          : "STRINGS2hk"

var 
  Byte SerialCog ' Cog for serial communication
  Byte SerialMSG[SERIAL_MESSAGE_MAX_LENGTH] ' Buffer for the incoming string
  Byte ParserBuffer[SERIAL_MESSAGE_MAX_LENGTH] ' Buffer for the parser
  Byte p ' Pointer to a position in the incoming string buffer
  
  Byte PIDCog
  Byte QiKCog
    
  byte sercog

  Long Setp[MotorCnt]
  Long pidmode
  Long setpAdd[MotorCnt]
  Long timeVal, timeValAdd, DirRamp, SpeedRamp


pub main
  SerCog:=serial.start(RXD, TXD, 0, Baud)
  serial.str(string("Starting..."))
  serial.char(13)
  PIDCog:=PID.Start(PIDCTime, @Setp,  Enc0Pin, EncCnt, TxQ, RxQ, nPIDLoops) 'thr4, 5 and 6
  pid.setallpidmode(1)
  'pid.setpidmode(0,1)
  'pid.setpidmode(1,1)
  PIDMode:=PID.GetPIDMode(0)                            
  repeat while PID.GetPIDStatus<>2                      'Wait while PID initializes

  SetPIDPars


  t.Pause1ms(5000)
  
  serial.str(string("Ready press $ to start."))
  serial.char(13)
  repeat until serial.rx == ST

  
  setp[0] :=1
  setp[1] :=1
  setpAdd[0] :=1
  setpAdd[1] :=-1
  timeVal := 10
  timeValAdd := 10

  'serial.str(string("Ready press $ to start."))
  'serial.char(13)
  
  
  
  repeat
    'SetSetPoint(0)
    'genWaitTime
    'SetSetPoint(1)
    setp[0] :=14    '~ 0.73 rps
    setp[1] :=-14   '~ 0.73 rps
    serial.dec(setp[0])
    serial.char(13)
    serial.dec(setp[1])
    serial.char(13)
    serial.str(string("Setpoint0: "))
    serial.dec(PID.getSetp(0))
    serial.char(13)
    serial.str(string("Setpoint1: "))
    serial.dec(PID.getSetp(1))
    serial.char(13)
    serial.str(string("PID Time: "))
    serial.dec(pid.GetPIDTime)
    serial.char(13)
    serial.str(string("PID Lead Time: "))
    serial.dec(pid.GetPIDLeadTime)
    serial.char(13)
    serial.str(string("PID Out: "))
    serial.dec(pid.GetPIDout(0))
    serial.str(string(", "))
    serial.dec(pid.GetPIDout(1))
    serial.char(13)
    t.Pause1ms(50)



PRI SetPIDPars | i
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

  'PlatformID:=2001
  DirRamp:=10
  SpeedRamp:=1

{{
PRI SetSetpoint(i) 

  if setp[i] == 10 or setp[i] == -10
    setp[i] := setp[i]*-1
  
}}

PRI SetSetpoint(i) 

  setp[i] := setp[i] + setpAdd[i]

  if setp[i] > 20
    setpAdd[i] := setpAdd[i]*-1


  if setp[i] < -20
    setpAdd[i] := setpAdd[i]*-1

PRI genWaitTime

  timeVal += TimeValAdd

  if timeVal > 1000
    TimeValAdd := TimeValAdd*-1

  if timeVal < 10
    TimeValAdd := TimeValAdd*-1

  