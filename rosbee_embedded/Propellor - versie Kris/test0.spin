CON
  _clkmode=xtal1+pll16x
  _xinfreq = 5000000 

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

OBJ
  serial : "FullDuplexSerial_rr005"              ' PC command and debug interface

VAR
  Byte SerialCog
  
pub main | error
  SerialCog :=serial.start(RXD, TXD, 0, Baud)
  error := 0
  error |= (2<<3)
  repeat
    serial.str(string("asdf")) 
    serial.dec(error)
