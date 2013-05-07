CON
        _clkmode=xtal1+pll16x
        _xinfreq = 5000000 

        'Time related Constants
        CLK_FREQ = ((_clkmode-xtal1)>>6)*_xinfreq
        MS_001 = CLK_FREQ / 1_000

        
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
  num             : "simple_numbers"                      ' Number to string conversion
  serial           : "FullDuplexSerial_rr005"              ' PC command and debug interface
  STRs          : "STRINGS2hk"
  QiK           : "QiKCommands"                         ' Standard serial for drives
  PosEnc        : "Quadrature Encoder"                  ' Position Encoder object

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
  Long speedramp
  Long dirramp

  Long speed
  Long increase
  
pub main | T1

  t.Pause1s(10)

  QikCog:=QiK.Init(rxq, txq)
  'QiK.AutoBaud
  QiK.SetProtocol(0)

  SerialCog:=serial.start(RXD, TXD, 0, Baud)

  serial.str(string("QiK Error: "))
  serial.bin(QiK.Geterror(10),8)
  serial.tx(13)

  speed := 0
  increase := 5
  'qik.setspeedm0(10,127)
  'qik.setspeedm1(10,127)
    
  Serial.str(string("QiK Error: "))
  serial.bin(QiK.Geterror(10),8)
  serial.tx(13)
  serial.str(string("Firmware Version: "))
  serial.tx(QiK.GetFirmWare(10))
  serial.tx(13)

  repeat

    't.Pause1ms(100)
    T1 := cnt
    {{
    serial.str(string("Speed: "))
    serial.dec(speed)
    serial.tx(13)
    qik.setspeedm0(10,20)
    serial.str(string("QiK Current: "))
    serial.dec(qik.getcurrentm0(10))
    serial.tx(13)
    serial.str(string("QiK Error: "))
    serial.bin(QiK.Geterror(10),8)
    serial.tx(13)
    }}
    '{{
    't.Pause1ms(8)
    qik.setspeedm0(10,20)
    qik.getcurrentm0(10)
    QiK.Geterror(10)

    qik.setspeedm1(10,-20)
    qik.getcurrentm1(10)
    QiK.Geterror(10)
    
    'serial.str(string("Loop Time: "))
    serial.dec(elapsedms(T1))
    serial.tx(13)
    '}}
  

    if speed > 100
     increase *= -1

    if speed < -100
      increase *= -1

    speed += increase
    
      
    
PRI elapsedms(tstart)

return ||(cnt - tstart) / MS_001  