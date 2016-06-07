{{
File: IRRemote.spin
Description: Uses an IR receiver to read data from a standard NEC remote
}}

VAR
  byte pin

PUB Start(ir_pin)
  pin := ir_pin
      
PUB GetIRData : IRCode | pulse, pulse1, pulse2, data, BitNumber, cnt2
  dira[pin]~
  ctra[5..0] := pin
  frqa := 1
  ctra[30..26] := %01100

  'Wait for start pulse (length in us)
  repeat until pulse1 > 8500 and pulse1 < 9500 
    waitpeq(0<<pin, |<pin, 0)    'Wait for low
    cnt2 := cnt                       'Reset pulse timer
    waitpeq(0<<pin, |<pin, 0)    
    waitpeq(|<pin, |<pin, 0)     'Wait for high
    pulse1 := (cnt-cnt2) / (clkfreq/1_000_000)     'Measure duration of low pulse in us
    waitpeq(|<pin, |<pin, 0)
    cnt2 := cnt
    waitpeq(|<pin, |<pin, 0)
    waitpeq(0<<pin, |<pin, 0)
    pulse2 := (cnt-cnt2) / (clkfreq/1_000_000)    'Measure high pulse

  if pulse2 > 2000 and pulse2 < 2500    'Repeat code
    waitpeq(0<<pin, |<pin, 0)
    waitpeq(|<pin, |<pin, 0)    'Capture short 560us burst
    return $FFFFFFFF
  elseif not (pulse2 > 4250 and pulse2 < 4750)
    return $00000000   'Invalid code? 

  data~
  repeat BitNumber from 0 to 31
    waitpeq(0<<pin, |<pin, 0)
    cnt2 := cnt
    waitpeq(0<<pin, |<pin, 0)
    waitpeq(|<pin, |<pin, 0)
    waitpeq(0<<pin, |<pin, 0)    'Must go high, then back low
    pulse := (cnt - cnt2) / (clkfreq/1_000_000)    'Pulse length in us
    if pulse > 2000 and pulse < 2500   'Represents a 1
      'pst.dec(pulse)
      'pst.newline
      data := data + (1 << BitNumber)   'Record data bit
    IRCode := data  'Return all data collected