{{
File: File_IO.spin
Description: Handles various IO tasks for the controller, i.e. getting polarization values from the file
}}

CON
  SEND_FREQUENCY = $11
  CONFIRM_CONNECTION = $33
  READ_EVENT_NUM = $77
  SEND_DIRECTION = $88
  SEND_POL_RATE = $BB
  READ_POLARIZATION = $FF

OBJ
  serial_file : "FullDuplexSerial"

PUB Init(rx_pin, tx_pin)
  'Start serial connections 
  serial_file.Start(rx_pin, tx_pin, %0000, 9600)

PUB ConfirmConnection | b1, b2            'Confirms that a serial connection has been established to collect data
  repeat
    serial_file.RxFlush
    serial_file.tx(CONFIRM_CONNECTION)       '0x33 == "Return confirmation code"
    waitcnt(clkfreq/5 + cnt)     'Make sure control byte is accepted before trying to receive data
    b1 := serial_file.RxCheck
    b2 := serial_file.RxCheck

    if (b1 == $BE AND b2 == $EF)
      quit

PUB SendFrequency(frequency)          'Sends the current frequency to the simulation (will have no effect in real setup)
  SendU32(SEND_FREQUENCY, frequency)
   
PUB SendDirection(direction)
  SendU32(SEND_DIRECTION, direction)

PUB SendPolRate(rate)        'Sends the given polarization rate (in standard IEEE float format, big-endian)
  SendU32(SEND_POL_RATE, rate)
  
PUB ReadPolarization : polarization    'Reads and returns the current polarization
  return ReadU32(READ_POLARIZATION)

PUB ReadEventNum : eventNum | b1, b2, b3, b4           'Reads and returns the current event number (timestamp, in seconds)
  return ReadU32(READ_EVENT_NUM)

PRI SendU32(control_byte, u32)
  'Sends an unsigned 32-bit integer, MSB first, using the given control byte
  serial_file.tx(control_byte)
  serial_file.tx(u32 >> 24)
  serial_file.tx(u32 >> 16)
  serial_file.tx(u32 >> 8)
  serial_file.tx(u32)

PRI ReadU32(control_byte) : u32 | b1, b2, b3, b4
  'Reads an unsigned 32-bit integer, MSB first, using the given control byte as a trigger
  repeat  
    serial_file.RxFlush
    serial_file.tx(control_byte)     '0x77 == "Return an event number"
    waitcnt(clkfreq/5 + cnt)   'Wait for control byte to be processed
    b4 := serial_file.RxTime(250)
    b3 := serial_file.RxTime(250)
    b2 := serial_file.RxTime(250)
    b1 := serial_file.RxTime(250)
  while b1 == -1 OR b2 == -1 OR b3 == -1 OR b4 == -1
  

  u32.byte[0] := b1
  u32.byte[1] := b2
  u32.byte[2] := b3
  u32.byte[3] := b4