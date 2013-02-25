OBJ
  console : "FullDuplexSerial_rr005"


PUB foo | lch 
   repeat     'loop possible input
     lch := console.RxCheck                      ' Check serial port
     
     if lch==36 ' equal to $ dollar sign
        DoCommand      
        lch:=0 'set lch to zero
      else
        'console.dec(lch)
        if(lch>0 AND lch<>13)'weird input, not starting anything like a dollar sign or leftovers of a sent command.
             'can occur when there is a bad cable connection, bad information is sent.
          move(0,0)
          DisableSteering
          DisableWheels
          console.str(string("$E-stop"))                                

PRI DoCmdInit


  MaxWaitTime := 4000                    'ms wait time for incoming string

  PcComActive:=0                         'Reset communication state var's   
  
  ByteFill(@StrBuf,0,MaxStr)             ' clear variable
  ByteFill(@cStrBuf,0,MaxStr)            ' clear variable

PRI DoCommand 

    StrCnt++
    'console.str(string("detected_command"))''debug
    console.StrInMaxTime(@StrBuf,MaxStr,MaxWaitTime)   'Non blocking max wait time
    if Strsize(@StrBuf)>0                           'Received string must be larger than n char's skip rest
      ByteMove(@StrBuf,@StrBuf,strsize(StrBuf))             'Copy received string in display buffer for debug

     'console.str(string("detected_command 2"))''debug
      
      XStat:=DoXCommand                             'Check input string for new commands
      
PRI DoXCommand | i, j, Par1, Par2, lCh, c1,MoveSpeed,MoveDir,MoveMode,num_arguments 

'  console.position(0,10)
 ' console.str(string("Debug XB ")) 'debug

  Sender:=0
  StrLen:=strsize(@StrBuf)  
  console.str(@StrBuf)''debug 
  StrP:=0
  num_arguments:=0
  Sender:=sGetPar
  if(Sender<0 OR Sender>999999)
    Sender:=0 'fail, or incorrect data received, shutdown movement and control
  else
    i:=0
    c1:=0
    repeat while(i<MAX_ARGUMENTS AND i<>MAX_ARGUMENTS)'store arguments, arguments are separated by 0
      arguments[i]:=sGetPar'get argument, split by , (comma) or zero terminator
      'console.dec(arguments[i])
      if(arguments[i]==600)
        num_arguments:=i
        i:=MAX_ARGUMENTS
      else
          i++
  'console.dec(Sender)''debug
  'console.char("-")''debug     
  'console.dec(num_arguments)''debug
  'console.char(13)''debug
  
  console.str(string(" - "))
  console.dec(arguments[0])
  console.char(13)''debug 

  console.dec(Sender)
  Case Sender

    0:'Disable platform    
       move(0,0)'Disable movement, set speed to zero
       console.str(string("$D-stop"))'feedback to serial terminal: Device stop
       DisableSteering
       DisableWheels
       'enabled:=0
       console.char(13) 'newline'end feedback

    1:'enable platform
       EnableSteering
       move(0,0)
       EnableWheels
       'All wheels are reset to a width of 1500 (centered)   

    2: 'set drive speed and wheel angles (back wheels have different setup)
       if(num_arguments>1 AND enabled==1) 
         corresponding_wheel_angle:=0
         SetWheelAngle((arguments[1]+7))
         move((arguments[0]*-1),0)'value is inverted   
       else
         console.str(string("$D-move_2-NA"))'feedback to serial terminal: system not enabled
         console.char(13) 'newline'end feedback

    3: 'set drive speed and wheel angles (back wheels have same setup) 
       if(num_arguments>1 AND enabled==1)
         corresponding_wheel_angle:=1
         SetWheelAngle((arguments[1]+7))
         move((arguments[0]*-1),0)'value is inverted
       else
         console.str(string("$D-move_3-NA"))'feedback to serial terminal: system not enabled
         console.char(13) 'newline'end feedback

    4: 'set drive speed and angle for a specified wheel
       if(num_arguments>1 AND enabled==1)
         corresponding_wheel_angle:=1
         move((arguments[1]*-1),arguments[0])'value is inverted
         corresponding_wheel_angle:=1
       else
         console.str(string("$D-move_4-NA"))'feedback to serial terminal: system not enabled
         console.char(13) 'newline'end feedback
         
    5: 'set drive speed and angle for wheels, but individually
       if(num_arguments>7 AND enabled==1)
         corresponding_wheel_angle:=1
         move((arguments[4]*-1),arguments[0])'value is inverted
         move((arguments[5]*-1),arguments[1])'value is inverted
         move((arguments[6]*-1),arguments[2])'value is inverted
         move((arguments[7]*-1),arguments[3])'value is inverted
         corresponding_wheel_angle:=1
       else
         console.str(string("$D-move_5-NA"))'feedback to serial terminal: system not enabled
         console.char(13) 'newline'end feedback

    9:
      console.str(string("test parameter:"))
      'console.str(arguments[0])
      'console.str(arguments[1])
      console.dec(arguments[0])
      console.dec(num_arguments)

  suppress_feedback[0]:=0
  suppress_feedback[1]:=0 
  suppress_feedback[2]:=0 
  suppress_feedback[3]:=0

  do_check:=1 
  
  Bytefill(@StrBuf,0,MaxStr)'clear strbuf
  return 1

  
PRI sGetCh | lch 'Get next character from commandstring
   lch:=Byte[@StrBuf][StrP++]
'   console.tx("\")          
'   console.tx(lch)
 '  Cmd[Lp++]:=lch
Return lch

{{PRI sGetPar | j, ii, lPar, Ch
  j:=0
  Bytefill(@LastPar1,0,CmdLen)   'Clear buffer

'  console.str(string(" In GetPar : " ))''debug 
'  console.tx(Ch)
  repeat strsize(StrBuf)
    ch:=sGetch
   ' console.tx(Ch) ''debug   

  if Ch == 0
'    console.tx(">")
'    console.str(string(" 1: Unexpected end of str! " ))
    Return -99  'error unexpected end of string
   
 }}
PRI sGetPar | j, ii, lPar, Ch, DO_INT
  j:=0
  DO_INT:=true
  Bytefill(@LastPar1,0,CmdLen)   'Clear buffer

'  console.str(string(" In GetPar : " ))''debug 
'  console.tx(Ch)
  repeat until (Ch => "0" and Ch =< "9") or (Ch => "a" and Ch =< "Z") or Ch == "-" or Ch == 0 or Ch == 13    
   ' console.tx("{") ''debug
    Ch:=sGetch
   ' console.tx(Ch) ''debug   
  if Ch => "a" and Ch =< "Z"
     DO_INT:=false
  console.dec(Ch) 
  if (Ch == 0 OR Ch == 13)  
'    console.tx(">")
'    console.str(string(" 1: Unexpected end of str! " ))
    Return 600  'error unexpected end of string
    
  'console.str(string(" GetPar : " ))''debug                             
  repeat while  Ch => "0" and Ch =< "9"  or Ch == "-" or (Ch => "a" and Ch =< "Z")
    if Ch == 0
'      console.tx(">")
'      console.str(string(" 2: Unexpected end of str! " ))
      Return -98  'error unexpected end of string
    if ch<>"," and j<CmdLen
'      console.tx("|")
'      console.dec(j)
'      console.tx(">")
'      console.tx(Ch)
      byte[@LastPar1][j]:=ch
      j++
      ' console.str(@LastPar1)''debug   
     Ch:=sGetch           'skip next
 
  'console.str(@LastPar1)  ''debug
  if(DO_INT)   
    LPar:=console.strtodec(@LastPar1)
    Return Lpar
  'console.tx("=") ''debug   
  'console.dec(lPar) ''debug   
  'console.tx(" ") ''debug   
Return Lpar