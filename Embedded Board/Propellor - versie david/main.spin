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

   'Wheel base width
   WHEEL_BASE_WIDTH = 0.385   
  
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
  Byte ParserBuffer[SERIAL_MESSAGE_MAX_LENGTH] ' Buffer for the parser
  Byte p ' Pointer to a position in the incoming string buffer
  Long error
  
  Byte PIDCog
  Byte QiKCog
    
  Long Setp[MotorCnt]
  Long pidmode

  Byte F32cog ''cog for floating point calculations

pub main 
  SerialCog:=serial.start(RXD, TXD, 0, Baud)
  F32cog:=f32.start
  
  serial.str(string("Starting..."))
  PIDCog:=PID.Start(PIDCTime, @Setp,  Enc0Pin, EncCnt, TxQ, RxQ, nPIDLoops) 'thr4, 5 and 6
  pid.setallpidmode(1) ''snelheidsregeling
  repeat while PID.GetPIDStatus<>2                      'Wait while PID initializes

  SetPIDPars
  waitcnt(500_000_000+cnt) ''wacht een paar sec om de voeding te stabiliseren
  setp[0] :=0
  setp[1] :=0


  
  serial.str(string("Now accepting commands..."))
  repeat
    handleSerial
  



  
pri move(vx, rot) | setpL, setpR
  Serial.str(string(" vx: "))
  Serial.dec(vx)
  Serial.str(string(" rot: "))
  Serial.dec(rot)

  setpL := f32.fround(f32.fdiv(f32.fsub(f32.ffloat(vx) , f32.fmul( constant(WHEEL_BASE_WIDTH) , f32.ffloat(rot) )),17.5)) ' = round(500 - (wheel base width * 100))
  setpR := f32.fround(f32.fdiv(f32.fadd(f32.ffloat(vx) , f32.fmul( constant(WHEEL_BASE_WIDTH) , f32.ffloat(rot) )),17.5))
  
  if setpL < 129 and setpL > -129      
    setp[0] := setpL
  if setpR < 129 and setpR > -129  
    setp[1] := setpR * -1 ''rechter wiel andere kant op :D
  Serial.str(string(" Setpl: "))
  Serial.dec(setp[0])
  Serial.str(string(" Stepr: "))
  Serial.dec(setp[1])
  Serial.str(string(13," Error: "))
  Serial.dec(error)    

PRI handleSerial | val, i, j, messageComplete
  wordfill(@SerialMSG,0,SERIAL_MESSAGE_MAX_LENGTH)     
  '!outa[LED]
  i := 0
  j := 0
  error:=0  
  messageComplete := false
  repeat until serial.rx == ST 'Busy wait until a '$' has been received
  repeat until messageComplete   
    val := serial.rx
    if (val == CR)
      messageComplete := true
      error := 0
    if(i == SERIAL_MESSAGE_MAX_LENGTH and ~messageComplete)
      messageComplete := true
      error |= (2 << 1)
    SerialMSG[i] := val
    
    i++
  ''SerialMSG[i+1] := 0 ''String afsluiten  
  '!outa[LED]  

  
  repeat until i == j
    serial.tx(SerialMSG[j])
    j++    
  
  case SerialMSG[0]
    "0":  'halt robot
      serial.str(string("$0")) 
      move(0,0)
    "1":  'start robot
      serial.str(string("$1"))
      '' enable wheels / todo                        
    "2":
      serial.str(string("$2")) ''This requires further string parsing.
      parseParam       
    other:
      serial.str(string("Unexpected message. Halting."))
      '' disableWheels //todo
      
PRI parseParam | vx, vy, rot
  vx := 0
  vy := 0   
  rot := 0
  p := 1 ''pointer binnen de parser 
  if(serialMSG[p] == SEP)
    p++
    vx := parseNumber(SEP)
    if(serialMSG[p] == SEP)
      p++
      vy := parseNumber(SEP)
      if(serialMSG[p] == SEP)
        p++
        rot := parseNumber(CR)
      else
        error |= (2 << 1)      
    else
      error |= (2<<2)
  else
    error|= (2 << 3)
  move(vx,rot)
       
PRI parseNumber(term) : value | n, i, done, hasError
  value := 0 ''uiteindelijke returnwaarde
  i := 0 ''pointer voor de tijdelijke array
  done := false
  hasError := false
  
  repeat until done
    if serialMSG[p] == term or serialMSG[p] == 0
      done := true
    if (p == SERIAL_MESSAGE_MAX_LENGTH and not done)
      done := true
      hasError := true
      error |= (2<<4)
    if(not done)
      ParserBuffer[i] := serialMSG[p]
      i++
      p++
  if(not hasError)  
    value := serial.strToDec(@ParserBuffer)   
  
  serial.str(string("Value: "))
  serial.dec(value)
  return value

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
                                              