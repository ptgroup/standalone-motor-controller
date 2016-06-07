{{
File: Timer.spin
Description: A simple timer to count milliseconds (like a stopwatch)
}}

VAR
  long millis, last_millis

PUB Init(strobe_pin)
  'Set up PHSB
  dira [strobe_pin]~~        'set output for phsa[31]        
  ctra [30..26] := %00100     'output phsa[31]     
  ctra [5..0] := strobe_pin  'specify output pin 
  frqa := $8000_0000 / ( clkfreq / 2_000 ) '1K Hz pos edge freq 
  ctrb [30..26]:= %01010      'count pos edges on pin
  ctrb [5..0] := strobe_pin
  frqb := 1

PUB Update
  millis := phsb

PUB Lap
  last_millis := millis

PUB GetMillis
  return millis

PUB GetLapTime
  return millis - last_millis