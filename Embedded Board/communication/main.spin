'' Test application for serial communication
'' Author: David Vollmar
'' Date: 13-dec-2012
''
'' Serial communication according to Serial Protocol for Robotics r1.2 
'' http://lectoraatmechatronica.wikispaces.com/file/view/Serieel%20protocol%20voor%20robotica%20r1.2.pdf/381442566/Serieel%20protocol%20voor%20robotica%20r1.2.pdf
'' (in Dutch)

'' For this setup, the platform dependent command
'' $2,vx,vy,rotCR is used, with vx in mm/s, vy in mm/s and rot in mrad/s

'' Version 0.1

CON

  _clkmode=xtal1+pll16x   '80MHz
  _xinfreq = 5000000      'MRS1

  LED = 23

  'Serial misc characters
  CR = 13 
  ST = 36 'dollar sign
  SEP = 44 'comma, used to seperate command params.
  MINUS = 45 'minus
  DOT = 46 'dot

 'Serial Settings
  BAUDRATE = 115200
  RXPIN = 31
  TXPIN = 30

  'Max length of a serial message to be parsed. 
  SERIAL_MESSAGE_MAX_LENGTH = 64


OBJ
  serial           : "FullDuplexSerial_rr005"  

VAR
  Byte SerialCog
  Byte SerialMSG[SERIAL_MESSAGE_MAX_LENGTH]
  Byte ParserBuffer[SERIAL_MESSAGE_MAX_LENGTH]
  Byte p


PUB main
  setup
  repeat    
    loop

PRI setup

  dira[LED] := 1 'LED as output
  SerialCog := serial.start(RXPIN,TXPIN,0,BAUDRATE)


PRI loop | val, i, j, messageComplete, error  
  '!outa[LED]
  i := 0
  j := 0  
  messageComplete := false
  repeat until serial.rx == ST 'Busy wait until a '$' has been received
  repeat until messageComplete   
    val := serial.rx
    if (val == CR)
      messageComplete := true
      error := 0
    if(i == SERIAL_MESSAGE_MAX_LENGTH and ~messageComplete)
      messageComplete := true
      error := 1
    SerialMSG[i] := val
    i++
  !outa[LED]  

  
  repeat until i == j
    serial.tx(SerialMSG[j])
    j++    
  
  case SerialMSG[0]
    "0":
      serial.str(string("$0"))
      machineStop
    "1":
      serial.str(string("$1"))
      machineStart                        
    "2":
      serial.str(string("$2")) ''This requires further string parsing.
      parseParam       
    other:
      serial.str(string("Unexpected message. Halting."))
      machineStop

' Parses a message String into a forward speed, sidewards speed, a rotation and a possible error
' vx: forward speed in mm/s
' vy: sidewards speed in mm/s, in this application always 0
' rot: rotation in mmrad/s       
PRI parseParam | vx, vy, rot, error
  vx := 0
  vy := 0
  rot := 0
  p := 1 ''pointer binnen de parser 
  if(serialMSG[p] == SEP)
    p++
    vx := parseNumber(SEP, error)
    if(serialMSG[p] == SEP)
      p++
      vy := parseNumber(SEP, error)
      if(serialMSG[p] == SEP)
        p++
        rot := parseNumber(CR, error)
      else
        error += 4
    else
      error += 3
      {{
    p++
    vx := parseNumber(CR, error)}}
  else
    error += 2
  serial.str(string("Error: "))
  serial.dec(error)
  serial.str(string("Carret: "))
  serial.dec(p)
  serial.str(string("val: "))
  serial.char(serialMSG[p])
  serial.str(string("vx: "))
  serial.dec(vx)
  serial.str(string("vy: "))
  serial.dec(vy)
  serial.str(string("rot: "))
  serial.dec(rot)  
  machineOperate(vx,vy,rot)

PRI machineStop

PRI machineStart

PRI machineOperate(vx, vy, rot)

PRI parseNumber( term, error) : value | n, i, done, hasError
  value := 0
  i := 0 
  done := false
  hasError := false
  
  repeat until done
    if serialMSG[p] == term or serialMSG[0] == 0
      done := true
    if (p == SERIAL_MESSAGE_MAX_LENGTH and not done)
      done := true
      hasError := true
      error += (2<<4)
    if(not done)
      ParserBuffer[i] := serialMSG[p]
      i++
      p++
  if(not hasError)  
    value := serial.strToDec(@ParserBuffer)   
  
  serial.str(string("Value: "))
  serial.dec(value)
  return value  