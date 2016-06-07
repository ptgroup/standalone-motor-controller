{TODO

}

CON
  _xinfreq = 6_250_000 
  _clkmode = xtal1 + pll16x
  
  'Motor control
  DIR0_PIN = 10
  DIR1_PIN = 9
  PWM_PIN= 8
   
  'Display Pins
  LCD_PIN=13
   
  'File Read Serial Pins
  FILE_RX_PIN=14
  FILE_TX_PIN=15

  'Other input pins
  MENU_MAIN_PIN = 22     'Main menu button
  MENU_SELECT_PIN = 21     'Menu select button (enter)
  SELECT_UP_PIN = 19
  SELECT_DOWN_PIN = 20
  IR_PIN = 12       'IR receiver

    'Reserved pins
  STROBE_PIN = 7

  'Configuration values 
  FILTER_FREQUENCY = 500'200'80 'Filter amount (samples per reading of motor position)....400 samples seems to be good for the AD7680     
  FILTER_FREQUENCY_FAST = 50'25 'Filter amount for fast updates
  HISTORY_LENGTH = 3 'Length of polarization history (for determining outliers)............50 samples seems to be good for the AD7680

  'Which information to display on the LCD (ex. STAGE_SEEK will display the interface for automatic/seek mode)
  'Menus
  STAGE_MAINMENU      = 0
  STAGE_SETTINGS      = 1
  'Major modes of operation
  STAGE_SEEK          = 2
  STAGE_MANUAL        = 3
  'Calibration (motor or frequency)
  STAGE_MOTORCALIB    = 4
  STAGE_MOVEUP_INITIAL= 20 
  STAGE_FREQ1         = 5
  STAGE_MOVEMOTOR     = 6
  STAGE_FREQ2         = 7
  STAGE_MOVEMOTORBACK = 18
  STAGE_BACKLASH      = 19
  'Setting adjustments
  STAGE_ADJUST_RATE   = 8
  STAGE_CHANGE_SIGN   = 13
  STAGE_CHOOSE_PRESET = 14
  STAGE_CHOOSE_FIELD  = 15
  STAGE_DISPLAY_PRESET= 17
  'Errors and warnings
  STAGE_CONFIRMATION  = 9
  STAGE_WAIT          = 10
  STAGE_REPOSITION    = 11 
  STAGE_ERROR         = 12
  STAGE_PRESET_ERROR  = 16

  'IR remote buttons
  BUTTON_POWER   = $E41BF708
  BUTTON_A       = $E01FF708
  BUTTON_B       = $E11EF708
  BUTTON_C       = $E51AF708
  BUTTON_UP      = $FA05F708
  BUTTON_DOWN    = $FF00F708
  BUTTON_LEFT    = $F708F708
  BUTTON_RIGHT   = $FE01F708
  BUTTON_CENTER  = $FB04F708
  BUTTON_REPEAT  = $FFFFFFFF
  BUTTON_UNKNOWN = $00000000

  'Seek algorithm preset materials
  PRESET_NONE    = 0
  PRESET_AMMONIA = 1

  'Optimal frequencies for presets
  AMMONIA5T_FREQ_POS = 140145
  AMMONIA5T_FREQ_NEG = 140470

  IR_TIMEOUT = 200     'Timeout in ms for IR remote

  'Misc. constants
  BOUNDARY_SIZE = 1       'Percent near edge to be considered "boundary"
  TOLERANCE_LOW = 0.2      'Size of "band" around zero polarization to be counted as an error (relative to the average)
  TOLERANCE_HIGH = 5.0     'Upper band of polarization for error checking (same as above, but high instead of low)
  ERROR_TRIGGER = 6    'Number of consecutive polarization reading errors that will trigger an error status (forcing user interaction)
   
OBJ
  lcd : "FullDuplexSerial"             'LCD display serial
  f : "Float32"                        'Floating point engine

  fs : "FloatString"                   'Utility to convert floats to strings
  serial_file: "FullDuplexSerial"      'Serial connection to file reader or simulation
  eep : "Propeller Eeprom"             'For saving persistent data after shutdown (i.e. motor calibration constants, etc.)
  ir : "IRRemote"                      'For working with the IR remote

  adc : "ADC_New"                      'Communicates with the ADC
  timer : "Timer"                      'For keeping track of time between IR events






VAR
  'Stacks for separate cogs
  long Stack1[100], Stack2[100], Stack3[100]

  'Motor configuration and calibration values 
  long motor_position                     'The current motor position (ADC value from 0 to 65535)
  long correction_factor                  'Correction factor for backlash effect (to be added to motor_position to get "real positon")
  
  'Temporary (for now)
  long old_motor_position
  long correction_factor_max
  
  long motor_pos_min,motor_pos_max        'The motor postion boundaries for this particular motor     
  byte dir0_high                          'Whether moving the motor in "direction 0" will increase the position
  long direction                          'Global variable for the motor's direction (either 0 or 1)
  long old_direction

  'Frequency calibration values
  long intercept, slope10000              'Intercept for frequency calculation and slope*10000 (done to avoid floating point stuff)
  long eip1, motor1, eip2, motor2         'Keep track of the two points selected
  long eip3, motor3                       'Used when calculating the backlash
  long b                                  'Backlash, (slack between wiper value and acutal EIP value) (in units of motor position)
  long new_freq                           'freq after a motor movement
  'long old_freq                           'freq before a motor movement                                   

  'Remote control stuff
  byte remote_select_up, remote_select_down 'Whether the IR remote is signalling to move up or down (takes into account repeat code)
  byte remote_select_left, remote_select_right

  'Seek method globals
  long num_samples                       'How many samples to take at each frequency
  long seeking_positive                  'Whether we are seeking positive polarization (false would be for negative)
  long max_freq_global, max_rate_global  'Maximum position (frequency) and rate of polarization increase (global values for printing)
  long readings_taken                    'Number of polarization readings taken so far
  long preset_material                   'Which material is selected as a preset
  long field_strength                    'Strength of field (in mT) for use with preset
  byte preset_selected                   'Whether the seek method needs to process a new preset selection (should be deset after preset is activated)
  byte motor_running 

  'Menu stuff
  long display_stage, option_select      'Stage of display (what to write on LCD) and current menu option selected
  long menu_page                         'Current "page" of the menu
  long current_digit, current_number[6]  'A number selector utility thing (allows selecting digits individually)
  long main_options[4], settings_options[5], sign_options[2], preset_options[5]  'Options arrays for menus

  'Misc. display stuff
  long dots                              'Number of dots to print after waiting (gives some feedback to impatient users)
  long last_display_stage                'Last display stage before encountering an "out of range" error so we can return to it after fixing

  'Error checking (for polarization errors)
  long n_errors, n_readings              'Number of errors encountered so far in a row; number of (valid) readings taken
  long pol_history[HISTORY_LENGTH]       'History of previous (valid) polarization readings, for detecting outliers
  byte play_sound                        'Whether to play error sound

PUB Main    | i                       'Main function (for setup and running seek)
  'Initialization of global variables
  
  'Motor and frequency calibration presets (for debug purposes)
  '------------------------
  'EIO1(?) presets
  'dir0_high:=1
  'motor_pos_max:=33428
  'motor_pos_min:=21610
  'intercept := 148224
  'slope10000 := -3173
  '------------------------  
  'EIO2 presets
  'dir0_high := 1
  'motor_pos_max := 48300
  'motor_pos_min := 36600
  'intercept := 129403
  'slope10000 := 2457
  'b:=35*10000/slope10000
  '------------------------  
  'EIO3 presets (OLD ADC)
  'dir0_high:=0
  'motor_pos_max:= 46500                                                                                                               
  'motor_pos_min:=34350
  'intercept := 132370
  'slope10000 := 2237
  'b:=-37 
  '------------------------  
  'EIO3 PRESETS USING AD7680 ADC CHIP
  dir0_high:=0
  motor_pos_max:= 46500                                                                                                               
  motor_pos_min:=34350
  intercept:= 132641
  slope10000:=2156
  b:= -22 
  '------------------------ 
  
  'Defaults (only initialize if EEPROM does not already have values stored)
  if motor_pos_min == 0
    motor_pos_max := 65535

  'Other global variables
  remote_select_up := false
  remote_select_down := false

  num_samples := 3       'Default of 5 samples per position (approx. 1 minute on typical sweep).....Changed to 10 then 16 on 10/5/15...12 on 10/6/15
  seeking_positive~~     'By default we should look for positive polarization

  ResetNumber
  dots := 1
  last_display_stage := STAGE_MAINMENU
  n_errors := 0
  n_readings := 0
  play_sound := TRUE

  'Initialization 
  f.Start             'Start floating point engine

  'Initialize cogs (for PWM, display, and input)
  cognew(DisplayLoop, @Stack1)
  cognew(InputLoop, @Stack2)
  cognew(IRInputLoop, @Stack3) 

  
  'Start serial connections 
  serial_file.Start(FILE_RX_PIN, FILE_TX_PIN, %0000, 9600)

  'Motor setup
  dira[DIR0_PIN]~~
  dira[DIR1_PIN]~~
  dira[PWM_PIN]~~

  outa[PWM_PIN]~~

  'Navigate to main menu
  MainMenu
  'Seek method should run on this cog, even in the background
  Seek  
PRI GenericInitialFreq | Initial_freq_pos, Initial_freq_neg

  '"Best Guess" at optimal initial frequencies for uncharacterized target materials (based on SANE data analysis). Subtract ~40, because motor always moves up in freq on first iteration. 
  Initial_freq_pos:= (140145 - 40) '* (field_strength / 5000) '[MHz]
  Initial_freq_neg:= (140470 - 40) '* (field_strength / 5000) '[MHz]


  'Move the motor to a good position at the start, so the polarization "ramp-up" will not be ill-effected as when starting seek too far out of the ideal range
  if seeking_positive
    Motor(FreqToMotorPos(Initial_freq_pos))
  elseif NOT seeking_positive
    Motor(FreqToMotorPos(Initial_freq_pos))

  
PUB Seek | time_step, min_time_step, max_rate, max_pos, curr_rate, curr_pos, prev_rate, prev_pos, steps, old_freq, f_counter      'Simple seek method that attempts to find optimum frequency for (positive) polarization

  'Initial values for seek (seems to work pretty well this way, steps aren't too big or too small)
  time_step := 250 'changed from 3000 to 2000 on 10/1/2015  because it was just moving too much at the beginning...changed to 700 then 400 on 10/5/15...600 on 10/6/15....500 on 11/11/15
  direction := 0
  old_direction := direction
  steps := 1
  f_counter:= 0

  'Take initial values as soon as we're ready
  repeat
    if display_stage == STAGE_SEEK
      'If no preset selected, make sure motor is at least in a generic, acceptable frequency range. Otherwise, pol. ramp up can be negatively affected. 
      if NOT preset_selected
        'GenericInitialFreq
      'If a preset has been selected, activate it (move motor to position)
      if preset_selected
        if preset_material <> PRESET_NONE   'If we actually have a preset to go to
          motor(FreqToMotorPos(GetPresetFrequency))
        preset_selected~
        
      'Take initial values
      old_freq := GetFrequency
      SendFrequency(old_freq)
      prev_pos := motor_position
      prev_rate := GetControlParam
      SendPolRate(prev_rate)
      max_rate := prev_rate
      max_pos := prev_pos
      max_freq_global := GetFrequency
      max_rate_global := max_rate
      quit   'Break out of repeat loop

  repeat
    'Don't run the algorithm while the motor is trying to calibrate or something (or an error)
    'It's OK to run in manual mode, to collect data and such, but it won't move the motor
    if display_stage <> STAGE_SEEK AND display_stage <> STAGE_MANUAL
      next

    'If a preset has been selected, activate it (move motor to position and adjust time step)
    if preset_selected
      if preset_material <> PRESET_NONE   'If we actually have a preset to go to
        motor(FreqToMotorPos(GetPresetFrequency)) 
      else
        preset_selected~     'Don't reprocess next cycle

    'Move the motor as directed and take a polarization rate sample at the new position
    UpdateCorrectionFactor 'We're going to move the motor, so make sure to update the correction factor for motor_position
        
    if steps <> 0                                        'For each iteration of the seek method:
      if direction == old_direction
        MoveMotor(direction,time_step)                     'Move the motor in the determined direction,
      else
        MoveMotor(direction, time_step*2)
      waitcnt(clkfreq*6+cnt)                             'wait 6 seconds to allow ADC values to settle, 
      new_freq:=GetFrequency                             'and sample the new frequency.
      repeat while NOT FrequencyLogicCheck(old_freq)     'If the new average frequency isn't logical (i.e. didn't go up when motor did):
        MoveMotor(direction,time_step)               'move the motor (triple the normal timestep to ensure ADC values change)
        waitcnt(clkfreq*2+cnt)                         'allow ADC to settle after movement
        new_freq:=GetFrequency                           'resample the frequency,

    'if old_direction <> direction
      'waitcnt(clkfreq*60+cnt)
      

    old_freq := new_freq
    SendFrequency(new_freq)
      
    FrequencyRangeCheck  'After moving motor, make sure the it is in an allowable range
 
    n_readings := 0
    curr_pos := motor_position
    curr_rate := GetControlParam
    SendPolRate(curr_rate)
    SendDirection

    'Check to see if we have found a new max rate
    if max_pos == 0 OR (seeking_positive AND f.FCmp(curr_rate, max_rate) > 0) OR (NOT seeking_positive AND f.FCmp(curr_rate, max_rate) < 0)
      max_rate := curr_rate
      max_pos := curr_pos
      max_freq_global := GetFrequency
      max_rate_global := max_rate

    'If the rate decreased, switch directions
    old_direction := direction
    if (seeking_positive AND f.FCmp(curr_rate, prev_rate) < 0) OR (NOT seeking_positive AND f.FCmp(curr_rate, prev_rate) > 0)
      direction := 1 - direction

    'Prepare for next iteration
    prev_pos := curr_pos
    prev_rate := curr_rate
    steps++

    waitcnt(clkfreq/5 + cnt)

PUB FrequencyRangeCheck | PosFreqMin, PosFreqMax, NegFreqMin, NegFreqMax, CurrentFreq, GoodPosition  'Checks to ensure the motor will not wander out of an appropriate frequency range

  'These values are from the Dose vs. Frequency plot (SANE data), showing the frequency ranges to obtain optimal positive or negative polarization
  PosFreqMin:= 140000 'MHz
  PosFreqMax:= 140250 'MHz
  NegFreqMin:= 140350 'MHz
  NegFreqMax:= 140600 'MHz
  
  waitcnt(2*clkfreq+cnt)                            'wait a second to allow ADC values to stabilize after motor movement 
  CurrentFreq:= GetFrequency                        'The current frequency is defined in a variable called "CurrentFreq"
  GoodPosition:= FreqToMotorPos(max_freq_global)    'Define the motor position where the frequency was at its highest and call it "GoodPosition"
  
  if seeking_positive
    if CurrentFreq =< PosFreqMin OR CurrentFreq => PosFreqMax    'if the motor has wandered out of the acceptable frequency range:
      lcd.Tx($D7)
      lcd.Tx($D3)
      lcd.Tx($E3)              'play a series of tones,
      lcd.Tx($E3)
      lcd.Tx($E3)
      lcd.Tx($E3)
      lcd.Tx($E3)           
      if max_freq_global => PosFreqMin AND max_freq_global =< PosFreqMax
        motor(GoodPosition)      'return to "GoodPosition" if it's in an acceptable range. 
      else                       'otherwise: just send it to 140145 MHz
        motor(FreqToMotorPos(140145)) 

  if NOT seeking_positive
    if CurrentFreq =< NegFreqMin OR CurrentFreq => NegFreqMax    'if the motor has wandered out of the acceptable frequency range: 
      lcd.Tx($D7)
      lcd.Tx($D3)
      lcd.Tx($E3)             'play a series of tones,
      lcd.Tx($E3)
      lcd.Tx($E3)
      lcd.Tx($E3)
      lcd.Tx($E3)
      if max_freq_global => NegFreqMin AND max_freq_global =< NegFreqMax                    
        motor(GoodPosition)     'return to "GoodPosition" if it's in an acceptable range. 
      else                      'otherwise: just send it to 140470 MHz 
        motor(FreqToMotorPos(140470))
        
PUB FrequencyLogicCheck(old_freq) | b_freq                                                                                                                                                                            
    'This set of conditions ensures that an up or down motor movement will receive a logical corresponding frequency change.
    'Because of the extreme resolution of the ADC, and fairly noisy signal off of the motor's wiper, sometimes the frequency values don't average out quite as predicted.
    
    if (direction == 0 AND dir0_high == 1) OR (direction == 1 AND dir0_high == 0)
      'Needed to increase frequency
      {repeat while new_freq =< old_freq
      new_freq++}
      if new_freq =< old_freq
        return false
    elseif (direction == 1 AND dir0_high == 1) OR (direction == 0 AND dir0_high == 0)
      'Needed to decrease frequency
      {repeat while new_freq => old_freq
      new_freq--}
      if new_freq => old_freq
        return false

    return true

        
PRI ConfirmConnection | b1, b2            'Confirms that a serial connection has been established to collect data
  repeat
    serial_file.RxFlush
    serial_file.tx($33)       '0x33 == "Return confirmation code"
    waitcnt(clkfreq/5 + cnt)     'Make sure control byte is accepted before trying to receive data
    b1 := serial_file.RxCheck
    b2 := serial_file.RxCheck

    if (b1 == $BE AND b2 == $EF)
      quit

  'display_stage := STAGE_SEEK

PRI Calibrate | last_position        'Calibrates motor (figures out which direction is high, and min/max positions)
  display_stage := STAGE_MOTORCALIB
  
  'Initial values (so ADC filtering doesn't prevent accurate calibration)
  motor_pos_min := 0
  motor_pos_max := 1 << 16
  last_position:=motor_position

  'Move motor for 10 seconds; the change in position tells us which direction is which (dir0_high)
  MoveMotor(0,10000)
  if(motor_position>last_position)
    dir0_high:=1
  else
    dir0_high:=0

  'Move the motor until it can't move any more (reached an extreme position)  
  repeat while ||(motor_position-last_position)>40
    last_position:=motor_position
    outa[DIR0_PIN]~~
    waitcnt(clkfreq*5+cnt)
    outa[DIR0_PIN]~

  'Set maximum or minimum accordingly 
  if(dir0_high==1)
    motor_pos_max:=motor_position
  else
    motor_pos_min:=motor_position

  'Try to get motor "unstuck" if we've moved it too far 
  outa[DIR1_PIN]~~
  waitcnt(clkfreq*5+cnt)
  outa[DIR1_PIN]~

  'Move the motor the other direction and find the remaining limit 
  repeat while ||(motor_position-last_position)>40
    last_position:=motor_position
    outa[DIR1_PIN]~~
    waitcnt(clkfreq*5+cnt)
    outa[DIR1_PIN]~
   
  if(dir0_high==1)
    motor_pos_min:=motor_position
  else
    motor_pos_max:=motor_position

  'Save calibration data so it can be loaded on the next startup 
  eep.VarBackup(@motor_pos_min, @motor_pos_min + 4)
  eep.VarBackup(@motor_pos_max, @motor_pos_max + 4)
  eep.VarBackup(@dir0_high, @dir0_high + 1)

  'Move motor to 50% (good starting position)
  MotorFull(motor_pos_min + (motor_pos_max - motor_pos_min) / 2)

  'return to the main menu once calibrated
  waitcnt(clkfreq+cnt)
  display_stage:=STAGE_MAINMENU

PRI MoveMotor(move_direction,time) | cnt2      'Moves motor in a direction = 0 or 1 and for a certain time (in milli seconds)
  'Prepare motor for motion (make sure it's not moving, and set full duty cycle)
  outa[DIR0_PIN]~
  outa[DIR1_PIN]~

  'Don't move the motor if we're not seeking or calibrating 
  if display_stage <> STAGE_SEEK AND display_stage <> STAGE_MOTORCALIB
    return
  'Keep the specified direction pin on for the given time (moves motor)
  {if(direction==0)
    outa[DIR0_PIN]~~
    waitcnt(clkfreq/1000*time+cnt)  
    outa[DIR0_PIN]~
  elseif(direction == 1)
    outa[DIR1_PIN]~~
    waitcnt(clkfreq/1000*time+cnt)
    outa[DIR1_PIN]~}

  cnt2 := clkfreq/1000*time+cnt
  repeat while cnt < cnt2
    if move_direction == 0
      outa[DIR0_PIN]~~
      outa[DIR1_PIN]~
    else
      outa[DIR0_PIN]~
      outa[DIR1_PIN]~~
    waitcnt(clkfreq/900+cnt) 
    outa[DIR0_PIN]~
    outa[DIR1_PIN]~
    waitcnt(clkfreq/450+cnt)

    'changed from 500-high / 500-low (50% duty cycle) on 11/2/15 (to 33%) 

PRI Motor(set_motor_position) | error, threshold       'Moves the motor to the specified position
  motor_running~~
  threshold:=25     'Threshold for "correct" motor position     ...25 works fine (on EIO2); changed to 20 on 10/7/15...having problems, changed to 30 on 10/28/15 
  error := set_motor_position - motor_position

  repeat while ( ||(error) > threshold)
    'Don't move the motor if we're not seeking or calibrating
    if display_stage <> STAGE_SEEK AND display_stage <> STAGE_MOTORCALIB
      outa[DIR0_PIN]~
      outa[DIR1_PIN]~
      error := set_motor_position - motor_position
      next

    if ((error>0 AND dir0_high == 1) OR (error<0 AND dir0_high == 0))        'Move in direction 0
      outa[DIR0_PIN]~~
      outa[DIR1_PIN]~
    elseif((error>0 AND dir0_high == 0) OR (error<0 AND dir0_high == 1))     'Move in direction 1
      outa[DIR0_PIN]~
      outa[DIR1_PIN]~~

    '"Fake PWM" generation: 33% duty cycle  (1/900 high, 1/450 low: period = 1/300 sec....f=300 Hz).....changed to 25% (Period = 1/400, f = 400 Hz)
    waitcnt(clkfreq/900+cnt)
    outa[DIR0_PIN]~
    outa[DIR1_PIN]~
    waitcnt(clkfreq/450+cnt)
    error := set_motor_position - motor_position

  'Stop moving motor once we get to the desired position
  outa[DIR0_PIN]~
  outa[DIR1_PIN]~
  motor_running~

PRI MotorFull(set_motor_position) | error, threshold       'Moves the motor to the specified position at full duty cycle
  threshold:=40     'Threshold for "correct" motor position
  error := set_motor_position - motor_position

  repeat while ( ||(error) > threshold)
    'Don't move the motor if we're not seeking or calibrating
    if display_stage <> STAGE_SEEK AND display_stage <> STAGE_MOTORCALIB
      outa[DIR0_PIN]~
      outa[DIR1_PIN]~
      error := set_motor_position - motor_position
      next

    if ((error>0 AND dir0_high == 1) OR (error<0 AND dir0_high == 0))        'Move in direction 0
      outa[DIR0_PIN]~~
      outa[DIR1_PIN]~
    elseif((error>0 AND dir0_high == 0) OR (error<0 AND dir0_high == 1))     'Move in direction 1
      outa[DIR0_PIN]~
      outa[DIR1_PIN]~~

    error := set_motor_position - motor_position

  'Stop moving motor once we get to the desired position
  outa[DIR0_PIN]~
  outa[DIR1_PIN]~

PRI MotorAtBoundary : at_bound      'Whether the motor is too close to an edge of its position and should be moved
  return MotorPositionPercent < BOUNDARY_SIZE OR MotorPositionPercent > 100 - BOUNDARY_SIZE
    
PRI PWM(pin,address,frequency) | period     'Generates a PWM pulse (address = duty_cycle)
  dira[pin]~~
  ctra[5..0]:=pin
  ctra[30..26]:=%00100
   
  frqa:=1
  long[address]:=0
  repeat
    repeat while long[address]==0
    period:=8000+cnt
    phsa:=-(80*long[address])
    waitcnt(period-1425)

PUB DisplayLoop      'Loop to continuously update the display
  'Initialize LCD display
  dira[LCD_PIN]~~
  outa[LCD_PIN]~ 
   
  'Initialize ADC
  adc.Init

  lcd.Start(LCD_PIN, LCD_PIN, %1000, 9600)
  waitcnt(clkfreq/10+cnt)

  {lcd.Tx($D7)
  lcd.Tx($D3)
  lcd.Tx($E3)
  lcd.Tx($E3)
  lcd.Tx($E3)
  lcd.Tx($E3)
  lcd.Tx($E3)}

  'Define custom characters
  'Up arrow (character 3)
  lcd.Tx($FB)
  lcd.Tx(%00000)
  lcd.Tx(%00100)
  lcd.Tx(%01110)
  lcd.Tx(%11111)
  lcd.Tx(%00100)
  lcd.Tx(%00100)
  lcd.Tx(%00100)
  lcd.Tx(%00000)
  lcd.Tx($03)
  
  'Down arrow (character 4)
  lcd.Tx($FC)
  lcd.Tx(%00000)
  lcd.Tx(%00100)
  lcd.Tx(%00100)
  lcd.Tx(%00100)
  lcd.Tx(%11111) 
  lcd.Tx(%01110)
  lcd.Tx(%00100)  
  lcd.Tx(%00000)
  lcd.Tx($04) 

  'Enable backlight and turn off cursor
  lcd.Tx($16)
  waitcnt(clkfreq/100+cnt)
  lcd.Tx($11)
  waitcnt(clkfreq/100+cnt)
  ClearDisplay
  waitcnt(clkfreq/10+cnt)

  'Initialize menu options arrays
  main_options[0] := @main_menu0
  main_options[1] := @main_menu1
  main_options[2] := @main_menu2
  main_options[3] := @main_menu3

  settings_options[0] := @settings_menu0
  settings_options[1] := @settings_menu1
  settings_options[2] := @settings_menu2
  settings_options[3] := @settings_menu3
  settings_options[4] := @settings_menu4

  sign_options[0] := @sign_menu0
  sign_options[1] := @sign_menu1

  preset_options[0] := @preset_menu0
  preset_options[1] := @preset_menu1
  preset_options[2] := @preset_menu2
  preset_options[3] := @preset_menu3
  preset_options[4] := @preset_menu4

  'Main loop (updates display 5 times/second) 
  repeat
    UpdateDisplay
    waitcnt(clkfreq/10+cnt)

PRI UpdateDisplay | freq, number          'Controls output to LCD display, as well as reading from the ADC  
  'Update motor position and send frequency update to simulation


  'UpdateMotorPosition      ....(9/14/15) changed to "MotorPresenceCheck" because screen is blank if motor cable not plugged in upon start-up
  ' Only update motor position if we're in a stage that needs it
  if display_stage == STAGE_SEEK OR display_stage == STAGE_MANUAL AND NOT motor_running
    UpdateMotorPosition
  else
    FastUpdateMotorPosition
    
  if (display_stage <> STAGE_CONFIRMATION)
    if (display_stage <> STAGE_SEEK)
      SendFrequency(GetFrequency)

    'Check motor for boundary
    if (display_stage <> STAGE_MOTORCALIB AND display_stage <> STAGE_REPOSITION AND display_stage <> STAGE_MOVEMOTOR AND MotorAtBoundary)
      last_display_stage := display_stage
      display_stage := STAGE_REPOSITION

    if (display_stage == STAGE_REPOSITION AND NOT MotorAtBoundary)
      display_stage := last_display_stage              

  
  'Check for errors and change stage if necessary
  if n_errors => ERROR_TRIGGER AND display_stage <> STAGE_MOTORCALIB
    display_stage := STAGE_ERROR
    'Test
    if play_sound
      lcd.Tx($D7)
      lcd.Tx($D3)
      lcd.Tx($E3)
      lcd.Tx($E3)
      lcd.Tx($E3)
      lcd.Tx($D5)
      lcd.Tx($DF)
      
      lcd.Tx($D3)
      lcd.Tx($E1)
      lcd.Tx($E1)
      lcd.Tx($E1)
      lcd.Tx($D5)
      lcd.Tx($DE)

      play_sound := FALSE

  ClearDisplay
  case display_stage
    STAGE_MAINMENU:
      'Main menu display
      lcd.str(string("MAIN MENU"))
      DisplayMenu(@main_options, 4)
      
    STAGE_SETTINGS:
      'Settings display
      lcd.str(string("SETTINGS/CONFIG"))
      DisplayMenu(@settings_options, 5)
      
    STAGE_SEEK:
      'Seek method display (automatic mode)
      lcd.str(string("AUTOMATIC MODE("))
      lcd.dec(readings_taken)
      lcd.str(string(")"))
      LCDNewline
      freq:=GetFrequency
      lcd.str(string("Motor pos: "))
      lcd.Dec(MotorPositionPercent)
      lcd.str(string(" %"))
      LCDNewline
      lcd.str(string("Freq: "))
      lcd.Dec(freq)
      lcd.str(string(" MHz"))
      LCDNewline
      fs.SetPrecision(3)
      'if max_freq_global == 0
      '  lcd.str(string("MAX:Not enough data "))
      'lse
        lcd.str(string("k_val: "))
        lcd.str(fs.FloattoString(max_rate_global))
       ' lcd.str(string(" @"))
        'lcd.dec(max_freq_global)
        lcd.Tx($BC)
        
    STAGE_MANUAL:
      'Manual mode display
      lcd.str(string("MANUAL MODE"))
      LCDNewline
      freq:=GetFrequency 
      lcd.str(string("Motor pos: "))
      lcd.Dec(MotorPositionPercent)
      lcd.str(string(" %"))
      LCDNewline
      lcd.str(string("Freq: "))
      lcd.Dec(freq)
      lcd.str(string(" MHz"))
      
    STAGE_MOTORCALIB:
      lcd.str(string("Motor Calibration"))
      LCDNewline
      lcd.str(string("Please wait"))
      PrintDots  'Waiting dots (. -> .. -> ... etc)
      
    STAGE_FREQ1:
      lcd.str(string("FREQ. CALIBRATION"))
      LCDNewline
      lcd.str(string("Select 1st frequency"))
      DisplayNumberFrequency

    STAGE_MOVEUP_INITIAL:
      lcd.str(string("FREQ. CALIBRATION"))
      LCDNewline
      lcd.str(string("Move UP a bit"))
      LCDNewline
      lcd.str(string("Motor pos: "))
      lcd.Dec(MotorPositionPercent)
      lcd.str(string(" %"))     
      
    STAGE_MOVEMOTOR:
      lcd.str(string("FREQ. CALIBRATION"))
      LCDNewline
      lcd.str(string("Move motor up..."))
      LCDNewline
      lcd.str(string("Motor pos: "))
      lcd.Dec(MotorPositionPercent)
      lcd.str(string(" %"))
      
    STAGE_FREQ2:
      lcd.str(string("FREQ. CALIBRATION"))
      LCDNewline
      lcd.str(string("Select 2nd frequency"))
      DisplayNumberFrequency

    STAGE_MOVEMOTORBACK:
      lcd.str(string("BACKLASH CALIBRATION"))
      LCDNewline
      lcd.str(string("Move motor down..."))
      LCDNewline
      lcd.str(string("Motor pos: "))
      lcd.Dec(MotorPositionPercent)
      lcd.str(string(" %"))

    STAGE_BACKLASH:
      lcd.str(string("BACKLASH CALIBRATION"))
      lcd.str(string("Enter EIP frequency"))
      LCDNewline
      DisplayNumberFrequency
      
    STAGE_WAIT:
      lcd.str(string("Please wait"))
      PrintDots
      
    STAGE_CONFIRMATION:
      lcd.str(string("Waiting for data"))
      LCDNewline
      lcd.str(string("Please make sure a"))
      LCDNewline
      lcd.str(string("data source is"))
      LCDNewline
      lcd.str(string("available"))
      
    STAGE_REPOSITION:
      lcd.str(string("WARNING"))
      LCDNewline
      lcd.str(string("Motor near boundary"))
      LCDNewline
      lcd.str(string("Please move motor"))
      LCDNewline
      lcd.str(string("Motor pos: "))
      lcd.Dec(MotorPositionPercent)
      lcd.str(string(" %"))
      
    STAGE_ADJUST_RATE:
      lcd.str(string("ADJUST SAMPLE RATE"))
      LCDNewline
      lcd.str(string("Samples: "))
      lcd.dec(num_samples) 
      LCDNewline
      lcd.str(string("Time: "))
      DisplayTime(13 * num_samples)
      LCDNewline
      lcd.str(string("(at 13 s per sweep)"))
      
    STAGE_CHANGE_SIGN:
      lcd.str(string("CHANGE DESIRED SIGN"))
      LCDNewline
      DisplayMenu(@sign_options, 2)
      
    STAGE_CHOOSE_PRESET:
      lcd.str(string("CHOOSE PRESET"))
      LCDNewline
      DisplayMenu(@preset_options, 5)
      
    STAGE_CHOOSE_FIELD:
      lcd.str(string("CHOOSE FIELD"))
      LCDNewline
      lcd.str(string("Preset field:"))
      LCDNewline
      DisplayNumberField
      
    STAGE_DISPLAY_PRESET:
      lcd.str(string("CURRENT PRESET"))
      LCDNewline
      lcd.str(string("Material: "))
      case preset_material
        PRESET_NONE:
          lcd.str(string("None"))
        PRESET_AMMONIA:
          lcd.str(string("Ammonia"))
      LCDNewline
      lcd.str(string("Field: "))
      if preset_material == PRESET_NONE
        lcd.str(string("N/A"))
      else
        lcd.dec(field_strength / 1000)
        lcd.str(string("."))
        LCDInt(field_strength // 1000, 3)
        lcd.str(string(" T"))
      LCDNewline
      lcd.str(string("Sign: "))
      if seeking_positive
        lcd.str(string("Positive"))
      else
        lcd.str(string("Negative"))
        
    STAGE_ERROR:
      lcd.str(string("POLARIZATION ERROR  "))
      lcd.str(string("Please fix and press"))
      lcd.str(string("ENTER to continue   "))
      if (cnt / clkfreq) // 2 == 0
        lcd.Tx($D9)
        lcd.Tx($D3)
        lcd.Tx($DC)
        lcd.Tx($E2)
        lcd.Tx($E2)
        lcd.Tx($DC)
      
    STAGE_PRESET_ERROR:
      lcd.str(string("PRESET ERROR        "))
      lcd.str(string("The selected preset "))
      lcd.str(string("is not in range.    "))
      lcd.str(string("Press ENTER to cont."))

PRI DisplayTime(time) | hours, minutes, seconds     'Displays time nicely to the LCD
  seconds := time // 60
  time := (time - seconds) / 60
  minutes := time // 60
  time := (time - minutes) / 60
  hours := time

  LCDInt(hours, 2)
  lcd.str(string(":"))
  LCDInt(minutes, 2)
  lcd.str(string(":"))
  LCDInt(seconds, 2)

PRI LCDInt(n, width) | n_tmp, padding, i       'Displays the given integer in a field of given width, padded by zeros if necessary
  n_tmp := n
  padding := width

  repeat while n_tmp > 0
    padding--
    n_tmp /= 10
  
  i := 0
  repeat while i < padding
    lcd.str(string("0"))
    i++
  if n <> 0
    lcd.dec(n) 

PRI DisplayMenu(options, num_options)     'Displays the menu with the given array of strings as its options
  if num_options =< 0
    return
    
  lcd.Tx($94)     'Move to row 1, col 0
  SelectOnOption(menu_page)
  lcd.str(long[options][menu_page])
  if menu_page > 0
    'Indicate that there are options above
    lcd.Tx($A7)
    lcd.Tx($03)

  if num_options == 1
    return
    
  lcd.Tx($A8)    'Row 2, col 0
  SelectOnOption(menu_page + 1)
  lcd.str(long[options][menu_page + 1])

  if num_options == 2
    return
  
  lcd.Tx($BC)     'Row 3, col 0
  SelectOnOption(menu_page + 2)
  lcd.str(long[options][menu_page + 2])
  if num_options - 1 > menu_page + 2
    'Indicate that there are options below
    lcd.Tx($CF)
    lcd.Tx($04)

PRI MainMenu         'Navigates to main menu 
  option_select := 0
  display_stage := STAGE_MAINMENU
  menu_page := 0

PRI SettingsMenu     'Navigates to settings menu
  option_select := 0
  display_stage := STAGE_SETTINGS
  menu_page := 0
  
PRI SelectOnOption(option_num)           'Displays a * before the menu option that is currently selected
  if (option_select == option_num)
    lcd.str(string("*"))
  else
    lcd.str(string(" "))

PRI IsMenu               'Whether the current display stage is a menu of some sort
  return display_stage == STAGE_MAINMENU OR display_stage == STAGE_SETTINGS OR display_stage == STAGE_CHANGE_SIGN OR display_stage == STAGE_CHOOSE_PRESET
  
PRI MenuMaxOption        'Max option number of current menu
  if display_stage == STAGE_MAINMENU
    return 3
  elseif display_stage == STAGE_SETTINGS
    return 4
  elseif display_stage == STAGE_CHANGE_SIGN
    return 1
  elseif display_stage == STAGE_CHOOSE_PRESET
    return 4
  else
    return -1

PRI AdjustMenuPage        'Change menu page if the current option isn't displayable
  if option_select < menu_page
    menu_page := option_select
  elseif option_select > menu_page + 2
    menu_page := option_select - 2

PRI DisplayNumberFrequency | i             'Displays the current frequency calibration number
  'Print out the number
  repeat i from 0 to 5
    lcd.dec(current_number[i])
  lcd.str(string(" MHz"))
  LCDNewline
  'Print a ^ under the digit that is currently selected
  i := 0
  repeat while i < current_digit
    lcd.str(string(" "))
    i++
  lcd.str(string("^"))

PRI DisplayNumberField | i            'Displays the current field selection number
  'Print out the number
  lcd.dec(current_number[0])
  lcd.dec(current_number[1])
  lcd.str(string("."))
  lcd.dec(current_number[2])
  lcd.dec(current_number[3])
  lcd.dec(current_number[4])
  lcd.str(string(" T"))
  LCDNewline
  'Print a ^ under the digit that is currently selected
  i := 0
  repeat while i < current_digit
    lcd.str(string(" "))
    i++
  if current_digit > 1
    lcd.str(string(" "))    'Don't count the decimal point
  lcd.str(string("^"))

PRI GetNumberFrequency        'Calculates the number in the current_number array (for frequency calibration)
  return 100_000 * current_number[0] + 10_000 * current_number[1] + 1_000 * current_number[2] + 100 * current_number[3] + 10 * current_number[4] + current_number[5]

PRI GetNumberField   'Calculates number for field selection (only involves 5 digits)
  return 10_000 * current_number[0] + 1_000 * current_number[1] + 100 * current_number[2] + 10 * current_number[3] + current_number[4]

PRI ResetNumber               'Resets the current_number array
  current_number[0] := current_number[1] := current_number[2] := current_number[3] := current_number[4] := current_number[5] := 0

  current_digit := 0

PRI MenuSelect | old_seeking, old_material, old_field        'Execute selection on the menu (choose option or do other stuff)
  case display_stage
    STAGE_MAINMENU:
      'Main menu options
      if option_select == 0
        'display_stage := STAGE_SEEK
        display_stage := STAGE_CONFIRMATION
        ConfirmConnection
        lcd.Tx($D8)
        lcd.Tx($D4)
        lcd.Tx($E3)
        display_stage := STAGE_SEEK
      elseif option_select == 1
        display_stage := STAGE_MANUAL
      elseif option_select == 2
        SettingsMenu
      elseif option_select == 3
        display_stage := STAGE_DISPLAY_PRESET       
    STAGE_SETTINGS:
      'Settings menu options
      if option_select == 0
        'Motor calibration
        Calibrate
        MainMenu
      elseif option_select == 1
        'Frequency calibration
        ResetNumber
        current_number[0] := 1
        current_number[1] := 4
        display_stage := STAGE_MOVEUP_INITIAL
      elseif option_select == 2
        'Adjust sample rate
        display_stage := STAGE_ADJUST_RATE
      elseif option_select == 3
        'Adjust polarization seek sign
        menu_page := 0
        if seeking_positive
          option_select := 0
        else
          option_select := 1
        display_stage := STAGE_CHANGE_SIGN
      elseif option_select == 4
        'Choose preset
        menu_page := 0
        option_select := 0
        display_stage := STAGE_CHOOSE_PRESET
        
    STAGE_MOVEUP_INITIAL:
      display_stage:=STAGE_FREQ1
    STAGE_FREQ1:
      if current_digit == 5
        'Calculate EIP input value and reset digits
        eip1 := GetNumberFrequency
        ResetNumber
        current_number[0] := 1
        current_number[1] := 4
        current_number[3] := 4
        'Take first data point for frequency calibration
        display_stage := STAGE_WAIT 
        motor1 := AverageMotorPosition(5)
        display_stage := STAGE_MOVEMOTOR
      else
        current_digit++
    STAGE_MOVEMOTOR:
      display_stage := STAGE_FREQ2
    STAGE_FREQ2:
      if current_digit == 5
        'Calculate EIP input value and reset digits
        eip2 := GetNumberFrequency
        ResetNumber
        'Take second data point for frequency calibration
        display_stage := STAGE_WAIT 
        motor2 := AverageMotorPosition(5)
        'Calculate new slope and intercept
        slope10000 := 10000*(eip2-eip1)/(motor2-motor1)
        intercept := eip2 - motor2*slope10000/10000
        'Save these calibration values in EEPROM so calibration is not required again on next startup
        eep.VarBackup(@intercept, @slope10000+ 4)
        'Move into backlash calibration stage
        current_number[0] := 1
        current_number[1] := 4
        display_stage:= STAGE_MOVEMOTORBACK
        'MainMenu 
      else
        current_digit++     
    STAGE_MOVEMOTORBACK:
      display_stage := STAGE_BACKLASH    
    STAGE_BACKLASH:
      if current_digit == 5
        'After moving motor back down, there will be disparity between EIP and box. 
        eip3 :=GetNumberFrequency
        ResetNumber
        display_stage := STAGE_WAIT
        motor3:= AverageMotorPosition(5)
        'calculate the backlash, b
        b:= (FreqToMotorPos(eip3) - motor3)
        eep.VarBackup(@b,@b + 4)
        MainMenu
        'Update correction factor here (we just calculated it, now apply it)
        correction_factor := b
      else
        current_digit++  
    STAGE_ADJUST_RATE:
      MainMenu
    STAGE_CHANGE_SIGN:
      old_seeking := seeking_positive
      'Change seeking toggle accordingly
      if option_select == 0
        seeking_positive~~
      else
        seeking_positive~
      'Go to the preset value for this new sign
      if preset_material == PRESET_NONE OR CanSelectPreset
        preset_selected~~
        MainMenu
      else
        seeking_positive := old_seeking  'Don't change sign because it could cause issues later on
        preset_selected~
        display_stage := STAGE_PRESET_ERROR
    STAGE_CHOOSE_PRESET:
      'Choose the preset
      old_material := preset_material
      old_field := field_strength
      case option_select
        0:
          'No change (keep current preset)
          preset_selected~
          MainMenu
        1:
          'No preset
          preset_material := PRESET_NONE
          preset_selected~~
          MainMenu
        2:
          'Ammonia at 5T
          preset_material := PRESET_AMMONIA
          field_strength := 5000
          'Select preset if possible (within range), otherwise just default back to no preset
          if CanSelectPreset
            preset_selected~~
            MainMenu
          else
            preset_material := old_material
            field_strength := old_field
            preset_selected~
            display_stage := STAGE_PRESET_ERROR
        3:
          'Ammonia at 2.5T
          preset_material := PRESET_AMMONIA
          field_strength := 2500
          if CanSelectPreset
            preset_selected~~
            MainMenu
          else
            preset_material := old_material
            field_strength := old_field
            preset_selected~
            display_stage := STAGE_PRESET_ERROR
        4:
          'Ammonia, choose field
          preset_material := PRESET_AMMONIA
          ResetNumber
          current_number[1] := 5
          display_stage := STAGE_CHOOSE_FIELD
    STAGE_CHOOSE_FIELD:
      if current_digit == 4
        'Done choosing all the digits; try to set the preset
        field_strength := GetNumberField
        if CanSelectPreset
          preset_selected~~
          MainMenu
        else
          preset_material := old_material
          field_strength := old_field
          preset_selected~
          display_stage := STAGE_PRESET_ERROR
        ResetNumber
      else
        current_digit++
    STAGE_DISPLAY_PRESET:
      'Return to main menu
      MainMenu
    STAGE_ERROR:
      'Error is fixed, return to normal operations
      play_sound := TRUE
      n_errors := 0   'Reset error count
      n_readings := 0
      MainMenu
    STAGE_PRESET_ERROR:
      'Error has been acknowledged
      MainMenu
    
PRI ClearDisplay     'Clears LCD display and ensures backlight is enabled
  lcd.Tx($0C)
  lcd.Tx($16)
  lcd.Tx($11)
  waitcnt(clkfreq/200+cnt)

PRI LCDNewline       'Moves to next line on LCD
  lcd.Tx($0D)

PRI PrintDots | i     'Prints dots (. , .., ...) to show user something is happening
  repeat i from 1 to dots
    lcd.str(string("."))
  dots++
  if dots > 4
    dots := 1

PRI AverageMotorPosition(iter) : average | i     'Returns an average of motor position values taken over a certain time (for greater precision)
  average := 0
  repeat i from 1 to iter
    average += motor_position
    waitcnt(clkfreq+cnt)
  average /= iter

{PRI UpdateMotorPosition | ADCValue, i, temp    'Updates motor position using ADC filtering method
  ADCValue := 0

  repeat i from 1 to FILTER_FREQUENCY
    'Read data
    temp := ReadADCData
      lcd.dec(temp)
      lcd.str(string(" "))
    'Make sure position is within allowable range (otherwise it's obviously wrong)
    repeat while temp > motor_pos_max OR temp < motor_pos_min
      temp := ReadADCData
    ADCValue += temp
  'Calculate an average of the collected positions
  motor_position := ADCValue / FILTER_FREQUENCY  }

PRI UpdateMotorPosition | ADCValue, i, temp    'This is just the "UpdateMotorPosition" with a catch to notify user if motor cable is plugged in. 
  ADCValue := 0

  repeat i from 1 to FILTER_FREQUENCY
    'Read data
    temp := adc.ReadData
    {if temp > 64224 OR temp < 1310   'If value is above 98% or below 2%, then likely reading a floating (unplugged) value....notify user of this condition:
      ClearDisplay
      lcd.Tx($D7)
      lcd.Tx($D3)
      lcd.Tx($E3)
      lcd.Tx($E3)
      lcd.Tx($E3)
      lcd.Tx($E3)
      lcd.Tx($E3)
      lcd.str(string("ERROR:Motor positionunreasonable.       *Please ensure motorcable is plugged in."))   
    'Make sure position is within allowable range (otherwise it's obviously wrong)
    repeat while temp > motor_pos_max OR temp < motor_pos_min
      temp := adc.ReadData}
    ADCValue += temp
  'Calculate an average of the collected positions
  motor_position := ADCValue / FILTER_FREQUENCY

PRI FastUpdateMotorPosition | ADCValue, i, temp    'This is just the "UpdateMotorPosition" with a catch to notify user if motor cable is plugged in. 
  ADCValue := 0

  repeat i from 1 to FILTER_FREQUENCY_FAST
    'Read data
    temp := adc.ReadData
    {if temp > 64224 OR temp < 1310   'If value is above 98% or below 2%, then likely reading a floating (unplugged) value....notify user of this condition:
      ClearDisplay
      lcd.Tx($D7)
      lcd.Tx($D3)
      lcd.Tx($E3)
      lcd.Tx($E3)
      lcd.Tx($E3)
      lcd.Tx($E3)
      lcd.Tx($E3)
      lcd.str(string("ERROR:Motor positionunreasonable.       *Please ensure motorcable is plugged in."))    
    'Make sure position is within allowable range (otherwise it's obviously wrong)
    repeat while temp > motor_pos_max OR temp < motor_pos_min
      temp := adc.ReadData}
    ADCValue += temp
  'Calculate an average of the collected positions
  motor_position := ADCValue / FILTER_FREQUENCY_FAST

PRI UpdateCorrectionFactor
  'Need to update correction factor
  if (dir0_high == 1 AND direction == 0) OR (dir0_high == 0 AND direction == 1)     'If moving up
    correction_factor := 0
  else
    correction_factor := b
      
{PUB BacklashCheck | PsuedoEIP, RealEIP

    PsuedoEIP:= AverageMotorPosition(5)
    
    if ((old_direction == 0 AND dir0_high == 1) OR (old_direction == 1 AND dir0_high == 0)) AND ((current_direction == 1 AND dir0_high == 1) OR (current_direction == 0 AND dir0_high == 0))
      'If you were going up, and now you are going down:
      'The EIP frequency will remain fixed (due to backlash) until you move down "b" MHz. 
      RealEIP:=PsuedoEIP - b 
        repeat while motor_position > RealEIP  
          PsuedoEIP:=PsuedoEIP
      'After the motor has moved down below the threshold "b" value, our wiper value (PsuedoEIP) should be correct now
      PsuedoEIP:=motor_potition + b

       
    elseif ((old_direction == 1 AND dir0_high == 1) OR (old_direction == 0 AND dir0_high == 0)) AND ((current_direction == 0 AND dir0_high == 1) OR (current_direction == 1 AND dir0_high == 0))
      'If you were going down, and now you are going up:
      'The EIP will remain fixed (due to backlash) until you movie up "b" MHz...and then the EIP will start to move again
      RealEIP:=PsuedoEIP + b 
        repeat while motor_position > RealEIP  
          PsuedoEIP:=PsuedoEIP

    return true}
PRI MotorPositionPercent      'Returns motor position as a percent of the total (eg. with min-max being the 0-100% range)
  return 100*((motor_position + correction_factor) - motor_pos_min) / (motor_pos_max - motor_pos_min)
    
PUB InputLoop | pressed, press_cnt, pressed2, press_cnt2, switch_displaced       'Loop to receive and process input from buttons and switches
  'Initialize input pins and motor control pins
  dira[DIR0_PIN]~~
  dira[DIR1_PIN]~~
  dira[PWM_PIN]~~
  dira[MENU_MAIN_PIN]~
  dira[MENU_SELECT_PIN]~

  outa[PWM_PIN]~~

  'Start timer
  timer.Init(STROBE_PIN)

  'Prevent detecting the same button press multiple times
  pressed := FALSE
  press_cnt := cnt
  pressed2 := FALSE
  press_cnt2 := cnt 

  repeat
    timer.Update
    
    'Main menu button logic
    if ina[MENU_MAIN_PIN] AND (NOT pressed) AND (cnt-press_cnt > clkfreq/4)
      pressed := TRUE
      press_cnt := cnt
      'Go back to main menu
      if display_stage == STAGE_SEEK OR display_stage == STAGE_MANUAL OR display_stage == STAGE_SETTINGS OR display_stage == STAGE_DISPLAY_PRESET
        MainMenu
    'Protection against bounces and holding down button    
    if NOT ina[MENU_MAIN_PIN]
      pressed := FALSE
      press_cnt := cnt


    'Select (enter) button logic  
    if ina[MENU_SELECT_PIN] AND (NOT pressed2) AND (cnt-press_cnt2 > clkfreq/4)
      pressed2 := TRUE
      press_cnt2 := cnt
      'What should happen when the button is pressed
       MenuSelect   
    'Protection against bounces and holding down button    
    if (NOT ina[MENU_SELECT_PIN])
      pressed2 := FALSE
      press_cnt2 := cnt

    
    'IR remote timeout
    if timer.GetLapTime > IR_TIMEOUT
      remote_select_up~
      remote_select_down~
      remote_select_left~
      remote_select_right~
    
    'Option select switch
    if SelectUp
      if IsMenu AND NOT switch_displaced
        'Change menu selection
        switch_displaced~~
        option_select--
        'Don't allow scrolling off the top of the menu (loop around)
        if option_select < 0
          option_select := MenuMaxOption
        'Make sure the user can see the option that's currently selected
        AdjustMenuPage
      elseif (display_stage == STAGE_MANUAL or display_stage == STAGE_MOVEUP_INITIAL OR display_stage == STAGE_MOVEMOTOR OR display_stage == STAGE_REPOSITION OR display_stage == STAGE_MOVEMOTORBACK)
        old_direction := direction
        'Move motor up
        if (dir0_high == 1)
          direction := 0
          outa[DIR0_PIN]~~
          outa[DIR1_PIN]~
          'waitcnt(clkfreq/1000+cnt)
          'outa[DIR0_PIN]~
          'outa[DIR1_PIN]~
          'waitcnt(clkfreq/1000+cnt)
        else
          direction := 1
          outa[DIR0_PIN]~
          outa[DIR1_PIN]~~
          'waitcnt(clkfreq/1000+cnt)
          'outa[DIR0_PIN]~
          'outa[DIR1_PIN]~
          'waitcnt(clkfreq/1000+cnt)
        'We moved the motor, so time to update the correction factor
        UpdateCorrectionFactor
      elseif (display_stage == STAGE_FREQ1 OR display_stage == STAGE_FREQ2 OR display_stage == STAGE_BACKLASH OR display_stage == STAGE_CHOOSE_FIELD) AND NOT switch_displaced
        switch_displaced~~
        'Change frequency selection
        current_number[current_digit]++
        if current_number[current_digit] > 9
          current_number[current_digit] := 0
      elseif display_stage == STAGE_ADJUST_RATE
        'Change sample rate
        if num_samples < 10
          num_samples += 1
        elseif num_samples < 30
          num_samples += 2
        elseif num_samples < 100
          num_samples += 5
        elseif num_samples < 500
          num_samples += 10
        else
          num_samples += 50
        waitcnt(clkfreq/5+cnt)
        remote_select_up~
          
    elseif SelectDown 
      if IsMenu AND NOT switch_displaced
        'Change menu selection
        switch_displaced~~
        option_select++
        'Don't allow scrolling past the last option in the menu
        if option_select > MenuMaxOption
          option_select := 0
        AdjustMenuPage
      elseif display_stage == STAGE_MANUAL or display_stage == STAGE_MOVEUP_INITIAL OR display_stage == STAGE_MOVEMOTOR OR display_stage == STAGE_REPOSITION OR display_stage == STAGE_MOVEMOTORBACK
        old_direction := direction
        'Move motor down
        if (dir0_high == 1)
          direction := 1
          outa[DIR0_PIN]~
          outa[DIR1_PIN]~~
          'waitcnt(clkfreq/1000+cnt)
          'outa[DIR0_PIN]~
          'outa[DIR1_PIN]~
          'waitcnt(clkfreq/1000+cnt)
        else
          direction := 0
          outa[DIR0_PIN]~~
          outa[DIR1_PIN]~
          'waitcnt(clkfreq/1000+cnt)
          'outa[DIR0_PIN]~
          'outa[DIR1_PIN]~
          'waitcnt(clkfreq/1000+cnt)
        'We moved the motor, so time to update the correction factor
        UpdateCorrectionFactor
      elseif (display_stage == STAGE_FREQ1 OR display_stage == STAGE_FREQ2 OR display_stage == STAGE_BACKLASH OR display_stage == STAGE_CHOOSE_FIELD) AND NOT switch_displaced
        switch_displaced~~
        'Change frequency selection
        current_number[current_digit]--
        if current_number[current_digit] < 0
          current_number[current_digit] := 9
      elseif display_stage == STAGE_ADJUST_RATE
        'Change sample rate
        if num_samples > 2    'Can't go lower than 2 samples
          if num_samples < 10
            num_samples -= 1
          elseif num_samples < 30
            num_samples -= 2
          elseif num_samples < 100
            num_samples -= 5
          elseif num_samples < 500
            num_samples -= 10
          else
            num_samples -= 50
          waitcnt(clkfreq/5+cnt)
          remote_select_down~

    elseif SelectLeft
      if (display_stage == STAGE_FREQ1 OR display_stage == STAGE_FREQ2 OR display_stage == STAGE_BACKLASH) AND NOT switch_displaced
        switch_displaced~~
        'Change digit selection
        current_digit--
        if current_digit < 0
          current_digit := 5
      elseif display_stage == STAGE_CHOOSE_FIELD AND NOT switch_displaced
        switch_displaced~~
        current_digit--
        if current_digit < 0
          current_digit := 4

    elseif SelectRight
      if (display_stage == STAGE_FREQ1 OR display_stage == STAGE_FREQ2 OR display_stage == STAGE_BACKLASH) AND NOT switch_displaced
        switch_displaced~~
        'Change digit selection
        current_digit++
        if current_digit > 5
          current_digit := 0
      elseif display_stage == STAGE_CHOOSE_FIELD AND NOT switch_displaced
        switch_displaced~~
        current_digit++
        if current_digit > 4
          current_digit := 0
          
    else
      switch_displaced~
      outa[DIR0_PIN]~
      outa[DIR1_PIN]~

    if display_stage <> STAGE_SEEK
      n_readings := 0       'Don't detect errors if we're not seeking

PUB IRInputLoop | ir_data                         'Loop for IR input
  dira[DIR0_PIN]~~
  dira[DIR1_PIN]~~
  dira[PWM_PIN]~~

  outa[PWM_PIN]~~
  
  'Start IR receiver
  ir.Start(IR_PIN)

  repeat
    'Get IR input
    timer.Lap
    ir_data := ir.GetIRData
    lcd.Tx($0B)   'Do nothing
    case ir_data
      BUTTON_POWER:
        'Go to main menu
        if display_stage == STAGE_SEEK OR display_stage == STAGE_MANUAL OR display_stage == STAGE_SETTINGS OR display_stage == STAGE_DISPLAY_PRESET
          MainMenu
      BUTTON_CENTER:
        MenuSelect
      BUTTON_UP:
        'Motor up
        remote_select_up~~
        remote_select_down~
      BUTTON_DOWN:
        'Motor down
        remote_select_up~
        remote_select_down~~
      BUTTON_LEFT:
        remote_select_left~~
        remote_select_right~
      BUTTON_RIGHT:
        remote_select_left~
        remote_select_right~~
      BUTTON_REPEAT:
        next

PRI SelectUp             'Whether the up direction is selected (for menu, manual motor control, etc.)
  return ina[SELECT_UP_PIN] OR remote_select_up

PRI SelectDown
  return ina[SELECT_DOWN_PIN] OR remote_select_down

PRI SelectLeft
  return remote_select_left

PRI SelectRight
  return remote_select_right

PRI SendPolRate(rate)        'Sends the given polarization rate (in standard IEEE float format, big-endian)
  serial_file.Tx($BB)    'Control byte: "expect a polarization rate"
  serial_file.Tx(rate >> 24)
  serial_file.Tx(rate >> 16)
  serial_file.Tx(rate >> 8)
  serial_file.Tx(rate)
  
PRI GetFrequency            'Calculates frequency based on the current motor position
  return (motor_position + correction_factor) * slope10000 / 10000 + intercept

PRI FreqToMotorPos(freq)    'Calculates motor position from frequency
  return (freq - intercept) * 10000 / slope10000

PRI GetPresetFrequency | freq_5t     'Calculates the optimal frequency at the current preset selection
  if preset_material == PRESET_AMMONIA
    if seeking_positive
      freq_5t := AMMONIA5T_FREQ_POS
    else
      freq_5t := AMMONIA5T_FREQ_NEG

  return field_strength * freq_5t / 5000  'Adjust optimal frequency for field strength

PRI CanSelectPreset | preset_motor          'Whether this preset can be selected at the current calibration settings
  preset_motor := FreqToMotorPos(GetPresetFrequency)   'The motor position of the optimal frequency

  return preset_motor > motor_pos_min AND preset_motor < motor_pos_max 

PRI SendFrequency(frequency)          'Sends the current frequency to the simulation (will have no effect in real setup)
  frequency:= GetFrequency
  serial_file.tx($11)      '0x11 is the control byte for "expect a frequency input"
  serial_file.tx(frequency >> 24)
  serial_file.tx(frequency >> 16)
  serial_file.tx(frequency >> 8)
  serial_file.tx(frequency)
  
PRI SendDirection
  serial_file.tx($88)   '0x88 is the control byte for "expect a direction input"
  serial_file.tx(direction >> 24)
  serial_file.tx(direction >> 16)
  serial_file.tx(direction >> 8)
  serial_file.tx(direction)

PRI SendString(strptr)
  serial_file.tx($EE)  '0xEE -> send a null-terminated string
  serial_file.str(strptr)
  serial_file.tx($00)
  
PRI ReadPolarization : polarization | b1, b2, b3, b4    'Reads and returns the current polarization
  repeat  
    serial_file.RxFlush
    serial_file.tx($FF)     '0xFF == "Return a polarization"
    waitcnt(clkfreq/5 + cnt)   'Wait for control byte to be processed
    b4 := serial_file.RxTime(250)
    b3 := serial_file.RxTime(250)
    b2 := serial_file.RxTime(250)
    b1 := serial_file.RxTime(250)
  while b1 == -1 OR b2 == -1 OR b3 == -1 OR b4 == -1
  

  polarization.byte[0] := b1
  polarization.byte[1] := b2
  polarization.byte[2] := b3
  polarization.byte[3] := b4

PRI ReadEventNum : eventNum | b1, b2, b3, b4           'Reads and returns the current event number (timestamp, in seconds)
  repeat  
    serial_file.RxFlush
    serial_file.tx($77)     '0x77 == "Return an event number"
    waitcnt(clkfreq/5 + cnt)   'Wait for control byte to be processed
    b4 := serial_file.RxTime(250)
    b3 := serial_file.RxTime(250)
    b2 := serial_file.RxTime(250)
    b1 := serial_file.RxTime(250)
  while b1 == -1 OR b2 == -1 OR b3 == -1 OR b4 == -1
  

  eventNum.byte[0] := b1
  eventNum.byte[1] := b2
  eventNum.byte[2] := b3
  eventNum.byte[3] := b4

PRI ProcessPolarization(pol) : success | avg, percent_error, i           'Processes a polarization reading (add to history, etc.) and return whether it is valid
  'Only check for validity if we have enough history to calculate a proper average
  if n_readings => HISTORY_LENGTH
    avg := 0.0
    repeat i from 0 to HISTORY_LENGTH - 1
      avg := f.FAdd(avg, pol_history[i])
    avg := f.FDiv(avg, f.FFloat(HISTORY_LENGTH))
    'Determine if measurement is valid (within specified tolerance range for polarization)
    if f.FCmp(f.FAbs(pol), f.FAbs(f.FMul(TOLERANCE_LOW, avg))) < 0 OR f.FCmp(f.FAbs(pol), f.FAbs(f.FMul(TOLERANCE_HIGH, avg))) > 0
      'Invalid; break out of function and add one to error count
      n_errors++
      return FALSE

  'Fall-through: measurement is valid, add it to history
  repeat i from 0 to HISTORY_LENGTH - 2
    pol_history[i] := pol_history[i+1]
  pol_history[HISTORY_LENGTH - 1] := pol
  n_readings++
  n_errors := 0
  return TRUE

PRI GetControlParam : rate | t1, t2, t3, p1, p2, p3, n_vals, k_val, a, c  'Calculates the "rate" (rate = A*k)
  k_val := 0  'Used as a running sum to compute the average
  n_vals := 0  'Number of k_vals which have been taken (divide the sum by this to get average)
  
  'Wait for rate to "settle down" a little bit before taking data
  waitcnt(clkfreq*5 + cnt)

  repeat
    t1 := ReadEventNum
    p1 := ReadPolarization
    'Try to get a valid polarization reading
    if NOT ProcessPolarization(p1)
      lcd.Tx($D8)
      lcd.Tx($D4)
      lcd.Tx($E3)
      waitcnt(clkfreq*3+cnt)
      next
   
    waitcnt(clkfreq*5+cnt)
  while display_stage <> STAGE_SEEK   'Disregard polarization samples taken outside seek method

  readings_taken := 1 
   
  repeat while readings_taken < 2
    if display_stage <> STAGE_SEEK   'Don't take samples outside seek method
      next
   
    t2 := ReadEventNum
    
    if t2 == t1
      next
    p2 := ReadPolarization
    
    if NOT ProcessPolarization(p2)
      lcd.Tx($D8)
      lcd.Tx($D4)
      lcd.Tx($E3)
      waitcnt(clkfreq*3+cnt)
      next
      
    'Make sure the data is new
    repeat while f.fcmp(p1,p2) == 0
      t2 := ReadEventNum
      p2 := ReadPolarization 
    
    'Wait before taking the next measurement
    waitcnt(clkfreq*5 + cnt)
    readings_taken++
   
  repeat while readings_taken < num_samples
    if display_stage <> STAGE_SEEK   'Don't take samples outside seek method
      next
   
    t3 := ReadEventNum
    p3 := ReadPolarization
   
    if t3 == t2 or t3 == t1
      next
    if NOT ProcessPolarization(p3)
      lcd.Tx($D8)
      lcd.Tx($D4)
      lcd.Tx($E3)
      waitcnt(clkfreq*3+cnt)
      next
      
    'Make sure the data is new
    repeat while f.fcmp(p2,p3) == 0
      t3 := ReadEventNum
      p3 := ReadPolarization 
    
    'Wait before taking the next measurement
    waitcnt(clkfreq*5 + cnt)
    readings_taken++

    'Do calculations here
    k_val += KValCalc(t1, t2, t3, p1, p2, p3)
    n_vals++
    
    'Shift all the variables to make room for a new (x3, y3) pair
    t1 := t2
    p1 := p2
    t2 := t3
    p2 := p3

  'Compute average of k_vals
  k_val := f.FDiv(k_val, f.FFloat(n_vals))
  SendString(string("Debug stuff"))
  SendString(fs.FloattoString(k_val))
  
  'Calculate the steady state value
  'Polarization is assumed to take the form p(t) = a + c*exp(-k*t)
  'C parameter ( C = (p2 - p1) / (exp(-k*t2) - 1) )
  c := f.FDiv(f.FSub(p2, p1), f.FSub(f.Exp(f.FMul(f.FNeg(k_val), f.FFloat(t2-t1))), 1.0))
  'A parameter (steady state) (A = p1 - c)
  a := f.FSub(p1, c)
  SendString(fs.FloattoString(p1))
  SendString(fs.FloattoString(c))  
  SendString(fs.FloattoString(a))

  '"control param" is A*k (the value of the first derivative at the beginning of ramp-up)
  rate := f.FMul(a, k_val)
  

  'for debugging purposes to the screen....
  max_rate_global := rate
  SendString(fs.FloattoString(rate))


PRI KValCalc(t1, t2, t3, p1, p2, p3) : k | d2, d3, p12, p13  'Calculates the k value using the formula
  d2 := f.FFloat(t2 - t1)
  d3 := f.FFloat(t3 - t1)
  p12 := f.FSub(p2, p1)
  p13 := f.FSub(p3, p1)
  'k = 2 * ((p2 - p1)*d3 - (p3 - p1)*d2) / ((p2 - p1)*d3^2 - (p3 - p1)*d2^2) 
  k := f.FMul(2.0, f.FDiv(f.FSub(f.FMul(p12, d3), f.FMul(p13, d2)), f.FSub(f.FMul(p12, f.FMul(d3, d3)), f.FMul(p13, f.FMul(d2, d2)))))

PRI PlayTone

      lcd.Tx($D7)
      lcd.Tx($D3)
      lcd.Tx($E3)
      lcd.Tx($E7)

DAT
'Main menu options
        main_menu0      BYTE "Automatic mode", 0
        main_menu1      BYTE "Manual mode", 0
        main_menu2      BYTE "Adjust settings", 0
        main_menu3      BYTE "Show current preset", 0
        
'Settings menu options
        settings_menu0  BYTE "Motor calibration", 0
        settings_menu1  BYTE "Frequency calib.", 0
        settings_menu2  BYTE "Change sample rate", 0
        settings_menu3  BYTE "Choose pos/neg", 0
        settings_menu4  BYTE "Choose preset", 0
        
'Change sign menu options
        sign_menu0      BYTE "Positive", 0
        sign_menu1      BYTE "Negative", 0
        
'Preset menu options
        preset_menu0    BYTE "Keep current", 0
        preset_menu1    BYTE "No preset", 0 
        preset_menu2    BYTE "Ammonia @ 5T", 0
        preset_menu3    BYTE "Ammonia @ 2.5T", 0
        preset_menu4    BYTE "Ammonia @ (choose)", 0