# notes

my own notes while learning this. plain language on purpose, i rewrite bits as i
understand them better.

two words that come up a lot:

  step      the bring-up order i followed, one piece of the board at a time
              0  usb console         5  loop paced by the IMU's data-ready line
              1  i2c sensors         6  display
              2  bench calibration   7  sd card
              3  gps                 8  the main loop
              4  pitch and roll      9  BSEC, 10 tuning - not done yet
  session   one power-on, one record in SESSIONS.JSON on the card
              5      19 sep         first drive, 18 min
              6-8    28 sep-3 oct   back on the bench with the fixes from 5
              17     3 oct          first long drive, 2 h 19 min, 118.5 km


## the layers

the code is stacked - each layer only talks to the one under it. the file list
and what each layer does is in the README; what matters here is why.

swap the IMU for a different chip and only imu.c and its glue file change,
everything above keeps working. it also keeps each file small enough to read in
one go.

three files sit above all of it and are stacked on nothing - attitude.c, trip.c
and elevation.c get handed plain numbers by main. nothing hardware-shaped goes
in, so all three run on the mac.

the usb console is the one stack that is mostly ST's:

  usb_serial.c             my module - printf lands here
  usbd_cdc_if.c            ST's "serial port" class
  usbd_core.c              ST's core - runs the usb interview
  usbd_conf.c              ST's glue down to the HAL
  HAL                      the usb hardware on PA11 and PA12

vendor drivers i re-download rather than patch by hand - the next download would
wipe the patch and i would not notice.


## what a driver actually is

a chip like the LSM6DSO is a box of numbered mailboxes called registers, one
byte each. write to 0x10 and the sample rate changes; read 0x28 to 0x2D and you
get the accelerometer. that is the whole interface.

a driver is code that knows which mailbox is which, so i don't memorise 100
numbers. instead of "put 0x40 into register 0x10" i write

  lsm6dso_xl_data_rate_set(..., LSM6DSO_XL_ODR_104Hz)

ST and Bosch publish these free, written to work on any microcontroller. which
is why the glue file exists.


## why the glue file exists

ST's driver has no idea what an STM32 is. it asks me for two functions instead -
one that reads bytes, one that writes them - and calls those when it needs the
bus.

handing over a function like that is a function pointer: a variable holding
where a function lives instead of a number.

so lsm6dso_platform.c has two small functions in the shape ST asked for, each
calling my i2c_bus_read_reg / i2c_bus_write_reg, bundled into a struct:

  stmdev_ctx_t lsm6dso_ctx = {
      .write_reg = lsm6dso_platform_write,
      .read_reg  = lsm6dso_platform_read,
      .mdelay    = lsm6dso_platform_delay,
      .handle    = &lsm6dso_i2c_addr,
  };

that struct is the context, and every ST call takes it first.

.handle is a spare pointer ST hands back untouched. i point it at the device
address, because my i2c_bus functions need to know which chip to talk to and
ST's driver has no other way of telling me. Bosch does the same and calls it
intf_ptr.


## I2C, enough to follow the code

two wires shared by every chip - SDA is data, SCL is clock. each chip has an
address so they don't talk over each other. BME680 is 0x76, LSM6DSO is 0x6A.

addresses are 7 bits. the HAL wants them shifted left by one because it uses the
freed bottom bit for read vs write. my wrapper does the shift inside, so i
always pass the plain datasheet number.

a normal read is "hey 0x6A, i want register 0x28" then "give me 6 bytes".
HAL_I2C_Mem_Read does that whole exchange in one call, which is why i used it
instead of building the sequence myself.


## what an IMU is

inertial measurement unit. one package holding:

  accelerometer   how hard it is being pushed
  gyroscope       how fast it is being turned

3 axes each, 6 numbers - hence 6-axis.

i named my module imu_ and not lsm6dso_ because ST's driver owns that prefix.
lsm6dso_status_t already exists in their header, so reusing it would not compile.
imu also names the job instead of the part.


## accelerometer numbers

measured in g, one g being earth's gravity.

the confusing bit: sitting still on a table it does not read zero. it reads 1 g
on whichever axis points up, because gravity is always being measured. that is
what makes tilt possible.

rough car numbers:

  sitting still            1 g on the vertical axis
  hard braking             about 1 g forwards
  emergency stop           up to 1.2 g
  normal cornering         0.3 to 0.8 g sideways
  pothole                  2 to 3 g extra straight up, very briefly


## gyroscope numbers

dps, degrees per second.

  90 dps    quarter turn every second
  360 dps   full spin every second

a bend at 50 km/h on a 30 m radius is about 26 dps. a tight hairpin taken slowly
is about 48 dps. normal driving basically never passes 60 dps.


## ODR - output data rate

how many measurements a second. 104 Hz = 104 samples a second, like frames per
second on a camera pointed at the car's movement.

why not pick a low number and save effort - aliasing. same effect as wagon wheels
in old films looking like they spin backwards: the camera samples too slowly, so
a fast motion gets recorded as a slow wrong one.

a car is full of fast vibration - engine, road, tyres. sample too slowly and that
vibration is not missed, it is recorded as slow fake movement that looks exactly
like braking. once it is in the data it cannot be removed.

how i got to 104 Hz:

  1. real car body movement is under about 15 Hz
  2. bare minimum is twice that, so over 30 Hz
  3. in practice 5-10x not 2x, so over 75 Hz
  4. the chip offers 12.5, 26, 52, 104, 208, 417, 833
  5. 104 is the first above 75

why not 208 - the built-in smoothing filter is tied to the ODR. at 104 it cuts
above 33 Hz, exactly what i want. at 208 it only cuts above 67, so more vibration
gets through. higher is worse here.


## full scale - the measuring range

the biggest value before it maxes out. like a bathroom scale: one that goes to
200 kg weighs more but each step it can tell apart is bigger. bigger range = less
detail per step. the smallest step is an LSB, one count of the raw number.

accel:

  ±2 g     0.061 mg
  ±4 g     0.122 mg
  ±8 g     0.244 mg
  ±16 g    0.488 mg

i took ±4 g. the range is per axis, not total, and the worst case is vertical -
1 g just sitting there plus 2-3 g on a pothole, which maxes ±2 g straight away.

the lost detail does not matter: the sensor's own zero error is up to ±20 mg,
164x bigger than the 0.122 mg step. step size is not what limits accuracy, the
zero error is. so ±4 g is basically free.

gyro:

  ±125 dps    4.375 mdps
  ±250 dps    8.75 mdps
  ±500 dps    17.50 mdps
  ±1000 dps   35 mdps
  ±2000 dps   70 mdps

±500 dps for the same reason. driving stays under 60 dps so ±250 would do, but
the zero error is ±1 dps = 1000 mdps, 28-57x bigger than either step. so i take
the wider range free and get headroom if the car ever slides.


## what each step of imu_init does

in order, and the order matters:

  1. WHO_AM_I - one register that always holds 0x6C. if that works, the bus, the
     address and the part are all confirmed at once. if it fails nothing else is
     worth trying.

  2. software reset - back to factory defaults, so a restart behaves like a real
     power cycle. the reset bit clears itself, so the code loops until it does,
     with a timeout so a dead sensor cannot hang the program.

  3. disable I3C - a newer protocol on the same pins. not used, and left on a
     glitch can knock the chip off the bus until it is power cycled.

  4. block data update - each reading is 2 bytes read one at a time. without this
     a new sample can land between the two and i get half of one reading glued to
     half of another. shows up as rare impossible spikes.

  5. auto increment - read 6 bytes in one go instead of asking for each register.

  6. full scale - the range for accel and gyro.

  7. ODR last - writing the sample rate is what wakes the sensor and starts it
     measuring, so everything else is already set by the first sample.


## what the BME680 measures

second sensor on the same bus:

  temperature      degrees C
  pressure         Pascal from the driver, i turn it into hPa
  humidity         % relative humidity
  gas resistance   ohms - not air quality yet, see below

env.c wraps it like imu.c wraps the LSM6DSO. called env for the role, not the
part.


## sleep and forced mode

only two modes:

  sleep    does nothing, almost no power, boots into this
  forced   one full measurement, then back to sleep on its own

there is no keep-measuring-forever mode on this chip. the BME688 has one called
parallel mode, and since Bosch ships one driver for both parts the constant
BME68X_PARALLEL_MODE sits in the header - it just does not work on a BME680.

so a read is three steps: trigger, wait, read out. that is the big difference
from the IMU, which free-runs at 104 Hz and lets me grab whatever is in its
registers whenever.

i do not guess the wait - bme68x_get_meas_dur() works it out from the current
settings. with mine that is about 43 ms plus 100 ms of heater, so env_read blocks
for roughly 143 ms.


## oversampling

the sensor takes several readings internally and averages them. averaging N cuts
random noise by sqrt(N), and costs time and power roughly in line with N.

different value for each of the three, because each runs out of usefulness for a
different reason.

temperature 2x. the part is only accurate to ±0.5 C and that is a calibration
limit, averaging cannot fix it - its noise is already 0.005 C at the lowest
setting, 100x smaller. i still do not skip it, because its value is used to
correct pressure and humidity.

pressure 16x. the one place the electronics really are the limit. at 16x the
noise is 0.12 Pa, about 1.7 cm of altitude. normally 16x gets avoided for being
slow, but i only measure every 3 seconds.

humidity 1x. accurate to ±3 % at best, and the sensing material needs 8 seconds
to react to 63% of a change. that limit is physical. averaging cannot speed up a
slow part.


## the IIR filter

smoothing inside the sensor, a running average:

  new = (old * (c - 1) + fresh reading) / c

bigger c is smoother but slower. temperature and pressure only, not humidity or
gas.

the datasheet says it is for "slamming of a door or wind blowing into the
sensor", which is exactly a car - doors slam, vents blow across the dash, windows
open at speed. all real pressure spikes i do not want on the display.

coefficients are 0, 1, 3, 7, 15, 31, 63, 127. i picked 3. the catch is it counts
samples, not seconds, and i measure every 3 s:

  c = 3     ~3 samples      ~9 seconds
  c = 15    ~15 samples     ~45 seconds
  c = 127   ~127 samples    over 6 minutes

the log wants temperature range and pressure per drive, so a heavy filter would
smear out climbing a mountain road or entering a tunnel. 3 kills door slams and
keeps up with real change.

a desk decision off the datasheet, not measured. if pressure looks jumpy in the
car i try 7; if it visibly lags going uphill, drop to 1.


## the gas sensor

a tiny plate inside that gets heated, with a sensing layer on top. the resistance
of that layer changes with the gases touching it.

300 C for 100 ms, Bosch's own example values. the driver caps at 400 C and
4032 ms.

the reading comes with a validity flag. env_read checks two status bits - one
says the measurement finished, the other says the plate actually reached target.
if either is missing the ohms mean nothing.

that is why gas_valid sits in env_sample_t next to the value instead of env_read
returning an error: temperature, pressure and humidity are still fine in that
same reading, so there is no reason to throw them away.

gas_valid is false on the first readings after power-on - the plate needs a few
cycles. confirmed on the board.

the heater is also what makes this sensor draw real current. the microamp figures
in the datasheet are for temperature, pressure and humidity only.


## ohms are not air quality

the resistance on its own does not mean much. it moves with humidity, with
temperature, and with the age of the sensor. IAQ, eCO2 and VOC need BSEC, a
closed binary library from Bosch.

that is its own job, and a bigger one than i first thought:

- not in the repo. needs a licence click-through, and the right prebuilt version
  for this exact chip and compiler
- it takes over the sensor config. instead of me setting oversampling and heater
  once in env_init, BSEC says what to use before each measurement. so adding it
  rewrites env.c rather than adding to it
- it wants nanosecond timestamps as 64-bit. HAL_GetTick gives 32-bit ms, which
  wraps after 49 days
- it runs on a fixed schedule and remembers state between calls. that state takes
  days of runtime before its accuracy rating climbs from 0 to 3, so it has to be
  saved to the SD card and reloaded on the next boot. without that every drive
  starts from scratch and the number never becomes useful

the last two are why BSEC cannot happen before main() exists - it needs the task
loop and the card first.


## the GPS is a different kind of device

the two sensors sit silent until i read a register. the GPS just starts talking
once it has power and never stops - a burst of text every second, forever,
whether i am listening or not.

it sends text, not numbers:

  $GNRMC,080608.000,A,3029.461489,N,11430.072002,E,0.00,148.41,210423,,,D,V*09

so the work is string parsing, not register decoding.

and the UART hardware holds exactly one byte, so if i am busy when the next
arrives the old one is gone. hence an interrupt filling a buffer in the
background instead of reading when convenient.


## what a NMEA sentence looks like

  $        every sentence starts with this
  GN       talker ID, 2 letters, which constellations gave the fix
  RMC      sentence type, 3 letters
  ,...,    the fields, comma separated
  *09      star then a 2 digit checksum
  CR LF    end of line

the checksum is an XOR of every byte between the $ and the *. cheap, and worth
doing - a corrupted sentence with a believable speed in it would quietly poison
the trip stats.

fields are left completely empty when there is no value, so i get two commas next
to each other and every field has to be checked before use.

i parse two types:

  RMC   position, speed, course, UTC date and time
  GGA   fix quality, satellites used, altitude, HDOP


## the talker ID trap

the two letters after the $ change with which constellations produced the fix.
from the Quectel spec:

  GPS + Galileo                 GN
  GPS + GLONASS + Galileo       GN
  GPS + BDS                     GP
  no fix yet                    always GP

so before a fix i get $GPRMC and after a fix $GNRMC. matching the whole "$GPRMC"
would make the GPS look like it stopped working at exactly the moment it started
working. so i match only the last three letters and ignore the talker.

GGA is odd - its talker stays GP even in multi constellation mode.


## speed is in knots

the spec says knots, not km/h. 1 knot is 1.852 km/h. conversion needed in main().


## why double type for latitude and longitude

NMEA gives ddmm.mmmm - degrees and minutes glued together. on the L76-L the
minutes have 4 decimals, so the smallest step reported is 0.0001 minutes, about
18 cm on the ground.

a float holds about 7 significant digits. at 45 degrees that is roughly 0.6 m,
3x coarser than what the module actually gives. so float would throw away real
precision. double it is.

the M4 has a hardware FPU for float only. double still works, the compiler just
calls software routines, maybe 10-50x slower. fine for two numbers once a second.
the rule is no doubles in a fast loop - not the 104 Hz IMU path, not per pixel.

one trap: a plain 1000.0 in C is a double, so float / 1000.0 does slow software
maths. that is why my code says 1000.0f and 100.0f.


## the ring buffer

the interrupt fires once per byte and drops it into a 2048 byte circular buffer
(256 until oct - see back on the bench). the main loop takes bytes out later.

head is only written by the interrupt, tail only by the main loop. the two never
write the same variable, so no locking.

256 because at 9600 baud only ~96 bytes can arrive in 100 ms, so polling every
100 ms leaves room. if uart_bus_overruns() is ever not 0, i am not polling often
enough.

easy to miss: HAL UART stops receiving after a framing or noise error and never
restarts on its own. a car is full of electrical noise, so without
HAL_UART_ErrorCallback restarting it, one glitch kills the GPS until the next
power cycle.


## what the fix age really says

gps_fix_age_ms counts from the last RMC that parsed, whether it carried a
position or not. on purpose - the module keeps sending RMC once a second even
with no sky, the sentence just says V for void instead of A:

  fix_valid false, age small    module alive, no sky - a tunnel
  age large                     nothing arriving - module or wiring dead

one rule for wiring main() later: hand trip_update_gps a fix only when
updated_tick has changed. every call restarts its between-fixes clock, so
repeating the same fix grinds the forward acceleration estimate down to zero and
the compensation quietly switches itself off.


## this one can actually be tested

nmea.c has no HAL and no hardware - string in, numbers out. so it compiles on the
mac and i can feed it the example sentences from the Quectel spec.

that passed: both talkers recognised, bad checksums rejected, lat/lon correct to
9 decimals, knots converted, south and west negative, empty fields handled, and
the clock still readable from a sentence with no fix yet.

attitude.c, trip.c and elevation.c are the same - 39, 31 and 15 checks. all four
are pure numbers, and that is the only reason any of this could be proven before
the board existed.

one thing learned the hard way, below: a test that builds its fake inputs using
the same assumption the code makes will pass while both are wrong.


## pitch and roll

two angles is all the car needs:

  pitch   nose up or down, going up a hill
  roll    leaning to one side, cornering or a cambered road

+ pitch is nose up, + roll is right side down, axes X forward, Y left, Z up.
that combination is ISO 8855, what the car industry uses - so anything i read
about vehicle dynamics matches my code without flipping signs in my head.

yaw is not in here. gravity cannot see it, for the reason in the mounting
section, and the GPS gives heading anyway.


## why one sensor is not enough

the accelerometer alone can do it. sitting still it measures gravity, so the
direction it feels is the tilt:

  pitch = atan2(ax, sqrt(ay² + az²))
  roll  = atan2(ay, az)

careful with the first - most code online has a minus in front of ax, and that is
for axes with Z pointing down. mine has Z up. i copied the minus, it was wrong,
and the section further down is how long it took to notice.

the problem is it measures every force, not just gravity, and cannot tell them
apart. not a cheap sensor problem, physics - a push and gravity feel the same.
brake at 0.5 g and it adds 0.5 g backwards onto 1 g down; the total points
26.6 degrees off vertical, and the maths cheerfully reports the nose diving
26.6 degrees on a flat road.

the gyro alone can also do it - turning speed x time added up gives angle, and
gravity is not involved, so braking does not bother it. the problem there is the
zero offset. up to ±1 dps while perfectly still: 1 degree after a second, 60
after a minute, and it never comes back.

so the accel is right in the long run and wrong for a moment, the gyro right for
a moment and wrong in the long run. exactly opposite, which is why the fix below
works.


## the complementary filter

one line per axis, per sample:

  angle = a * (angle + gyro_rate * dt) + (1 - a) * angle_from_accel
  a     = tau / (tau + dt)

mostly keep where the gyro says i am, nudge a little towards where the accel says
i am.

a is just under 1 - at tau = 2 s and dt = 1/104 s it is 0.995. so 99.5% gyro and
0.5% accel every sample. sounds like the accel does nothing, but it votes 104
times a second and those add up: gyro drift gets pulled out continuously, while a
bump lasting a tenth of a second barely moves anything.

"complementary" because the weights add to 1 and each sensor is used where the
other is bad. a low-pass on the accel and a high-pass on the gyro, added.

the fancier version is a Kalman filter. same job here, far more code and far more
to tune. not worth it.


## what happens to one sample

  1. take the gyro offset off, in the board's own axes
  2. rotate both vectors into car axes
  3. check the total length of the accel - gravity, or the car pushing?
  4. if gravity, work out pitch and roll from it
  5. step the previous angles forward with the gyro
  6. blend 5 with 4, or keep only 5 if step 3 said no


## tau, and how i settle it later

tau is the only knob, in seconds, and it is the crossover: shorter than tau
believe the gyro, longer believe the accel.

two errors pull opposite ways:

  gyro side    a leftover offset b gives a standing error of b x tau degrees.
               1 dps at tau 2 s is 2 degrees. wants tau small.

  accel side   a fake tilt of f degrees lasting T seconds leaks in as
               f x (1 - e^(-T/tau)). a 0.5 g brake held 3 s at tau 2 s puts 20 of
               those 26.6 degrees into the answer. wants tau big.

no value is acceptable for both. that is what makes the two fixes below not
optional - they shrink each error at its source instead of trading one for the
other: measure the gyro offset and subtract it so b stops being 1 dps, and gate
the accel so the big fake tilts never reach the filter.

with both in, anything from 1 to 5 s works and the exact number stops mattering.
2.0 s is in attitude.c as a starting point, not a measured value.

how i settle it properly: log raw accel, raw gyro and the filter output on one
drive, replay on the mac with different tau, keep whichever tracks a known
manoeuvre best - a speed bump taken slowly, or a car park ramp whose slope i can
measure.

sensor noise never comes into this. gyro noise is 27 mdps, accel noise 0.03
degrees of tilt. the road and the driver are thousands of times bigger. this
filter fights driving, not electronics.


## the mounting angle

the case will not sit flat - tilted maybe 40-45 degrees towards the driver so the
screen is readable, with the accelerometer soldered flat on the board inside it.

that does not matter, as long as the tilt never changes and i handle it right.

the wrong way is to measure pitch and roll at rest and subtract them from every
later reading. fine for a few degrees, broken at 45, because pitch and roll are
angles and not parts of a vector - they do not subtract. with the board tipped
back, a pure roll of the car shows up partly as pitch. about 70% of the signal
lands in the wrong channel at 45 degrees.

the right way is to rotate the readings before any angle maths. the board-to-car
relationship is one fixed 3D rotation R:

  a_car = R * a_board
  g_car = R * g_board       (turning speed is a vector too, same R)

then the two arctangents run on numbers that look like they came off a flat
board. 9 multiplies per vector, two vectors per sample. nothing on a 180 MHz chip
with an FPU.

where R comes from: park on flat ground, read the accelerometer once. that one
reading is the measurement of my mounting angle - including the couple of degrees
of slop from the case and the dash not being level, which i could never have
typed in by hand. attitude_set_mount builds R from it (Rodrigues' formula, closed
form, no looping). so what gets hardcoded is three readings, not nine matrix
entries.

it cleans up the accelerometer's own ±20 mg zero error for free too, worth 1.15
degrees of permanent tilt, because that error is sitting inside the recorded
reading.

one catch: gravity gives 2 of the 3 parts of a rotation, never the third. it says
which way is down, so tilt is pinned completely, but it says nothing about
turning about the vertical - the sensor cannot tell whether the case faces
forwards or sideways, both look identical to gravity. that part comes from how i
build the mount, and lives in the axis map, not in attitude.c.


## the gate

if the accelerometer is not reading about 1 g in total, the car is pushing on it,
so that sample's accel half gets skipped and the gyro carries the angle alone.

|a| = sqrt(ax² + ay² + az²), and outside 1 ± 0.1 g the accel gets no vote.

  hard braking 0.5 g     |a| = 1.118    26.6 deg of fake tilt    caught
  cornering 0.3 g        |a| = 1.044    16.7 deg                 missed
  gentle push 0.15 g     |a| = 1.011     8.5 deg                 missed
  pothole                all over       big                      caught

the misses happen because a sideways push added onto 1 g down grows the total
length only as sqrt(1 + x²), which barely moves for small x, while the angle
error grows straight away. tightening to ±3% would catch more, but then almost
nothing passes on a rough road and the gyro drifts unchecked.

so it is a partial fix and i know exactly how partial - the host test feeds 0.2 g
for 3 seconds and expects about 8.8 degrees of error. measured and written down
instead of hidden.

the shape of the error makes it liveable: it builds while the push lasts and
decays on its own once it stops. nothing accumulates, nothing needs resetting. on
the display it reads as the trace drooping for a few seconds during a long pull.

the full fix, later: i already know the car's real acceleration from elsewhere -
forwards from the change in GPS speed, sideways from speed x yaw rate off gyro Z.
subtract those and what is left really is gravity. the sideways one should work
well at 104 Hz; the forwards one only partly, GPS speed updates once a second.
bumps stay uncaught either way.


## the sign that will bite me

the accel formulas and the gyro axes do not line up the way i first expected.
roughly level it comes down to:

  pitch_rate = -gyro_y
  roll_rate  = +gyro_x

the minus is real. with Y pointing left, a nose-up rotation is a negative turn
about Y by the right hand rule.

writing it down because getting it backwards does not blow anything up - the gyro
half and the accel half just pull against each other and the output looks slow
and laggy rather than obviously broken. that kind of bug survives for weeks.

those two lines are only the level-ish version. once the car is properly tilted
its axes are tilted too and they mix. the full form is below, and it is what is
in the code.

the host test has a line for each sign: spin one axis at 10 dps for a second with
the accel gated out, check the angle comes out +10.


## off-road is where the shortcut breaks

good news first: off-road is the easy case for this filter. crawling over rocks
the car barely accelerates - 0.1 or 0.2 g against 0.5 g of braking on tarmac - so
the accelerometer is nearly telling the truth and the gate lets most samples
through. the regime it struggles in is motorway braking, which is exactly where
the angle does not matter.

also fine: atan2 is exact at 40 degrees the same as at 4, no small-angle
approximation anywhere. and the mount rotation saves me from a trap - with the
case tilted 45 back, a car pitched 40 puts the sensor at 85, right next to where
this maths falls apart. but the rotation into car axes happens first, so the
maths only ever sees the car's 40. the awkward point is 90 degrees of car pitch,
which is a rollover, not a hill.

what was not fine: those two gyro lines. the full version is

  pitch_rate = -cos(roll) x gyro_y + sin(roll) x gyro_z
  roll_rate  = gyro_x - tan(pitch) x (sin(roll) x gyro_y + cos(roll) x gyro_z)

both collapse to the short version at zero. what the short one costs:

  10 degrees of tilt      1.5% out    ~0.05 deg of error, invisible
  30 degree side slope    13% out     ~5 deg, and it stays
  40 degrees, turning     worse       yaw rate leaks into roll through tan()

that last one is climbing across a side slope while turning, which is normal
off-road. and the error does not fade - it lasts as long as the manoeuvre,
because the gyro keeps integrating wrong and the accel only pulls back a little
each sample.

fixing it costs two sin, two cos and one tan per sample - about 0.04% of the
processor at 104 Hz. tan() runs away at 90 degrees so the pitch fed into it is
capped at 80, already a rollover.

past about 60 degrees the right answer would be quaternions instead of angles,
but 60 degrees of pitch means the car is going over.

one more thing that matters more off-road: reading the sensor every 100 ms while
it produces 104 a second throws away 9 out of 10. on tarmac the angles barely
move between samples. off-road they do - at 60 dps the car moves 6 degrees
between two - and the thrown-away samples alias vibration into the kept ones. so
INT1 is not a nice-to-have for this.


## the sign i got wrong, and how the test hid it

worth writing down because it is the exact failure the section above warns about.

i had the accel formula as pitch = atan2(-ax, ...), which is what most code
online has. wrong for my axes; the right one has no minus.

why: an accelerometer at rest reads +1 g on whichever axis points up, so what it
hands back is the "up" direction as seen from the car. nose up tilts the forward
axis towards up, so the forward number goes positive. the version with the minus
comes from books using Z down, and it does not survive being copied across.

what makes it worse than a flipped display: the gyro half had the sign right and
the accel half had it wrong, so the two would have pulled against each other
forever and never settled - the "looks sluggish instead of broken" bug. i wrote
that warning while the bug was already sitting in the file.

how it hid: my test built its fake accelerometer readings with the same wrong
sign, so both halves agreed and everything passed. a test that shares an
assumption with the code cannot catch that assumption.

what caught it: a separate little program that rotated a vector directly, with no
angle formulas in it at all. that is the lesson - to check maths, check it
against something that does not work the same way, not a tidier version of
itself. the numbers from that program are now the expected values in the test.


## trip stats

everything that only makes sense across a whole drive. one session per power-on,
because the ignition cuts the power and there is nothing to resume.

  how long        total, and the part actually spent moving
  how far         speed x time
  how fast        the highest GPS speed
  how hard        peak push forwards, backwards and both sideways
  how far leaned  biggest pitch and roll each way
  how close       those two against what the car can actually take
  the weather     min, max and average of temperature, pressure, humidity

the module has no clock. time arrives as a dt on every motion update. same reason
as the parser - no HAL means it runs on the mac.


## a float that grows starts rounding

a float holds about 7 meaningful digits. plenty for one reading, but a running
total eats its own digits as it grows - the bigger the number, the coarser the
steps near that value.

the trip timer adds about 0.0096 s per sample. past two and a quarter hours the
smallest step a float can make up there is about 0.001 s, so every addition gets
rounded. and because the added amount is identical every sample, the rounding
error is identical too - it stops averaging out and the timer runs fast, up to a
minute per hour. distance has the same disease, worst on a motorway at constant
speed.

the fix: whole seconds and whole metres live in plain integers, which never
round, and only the part below 1.0 lives in a float, which never grows. every
update adds into the fraction and carries the whole part across, like a clock
carrying seconds into minutes.


## taking gravity out first

the accelerometer reading always has gravity in it. park on a 10% slope, which is
5.7 degrees, and the forward axis reads sin(5.7) = 0.1 g forever. leave that in
and the "hardest braking" of a mountain trip is the car sitting still on a hill.

so the first thing that happens to every sample is subtracting where gravity is
pointing, known because the attitude filter just worked out the angles:

  gravity_x = sin(pitch)
  gravity_y = cos(pitch) x sin(roll)

what is left is what the car is doing. also why the coasting below can work at
all - integrating a number with gravity still in it would have the car
accelerating up every hill.


## the accelerometer reads the push, not the feel

which side was the hardest corner on - easy to get backwards.

in a bend two directions are in play. the tyres push the car toward the inside -
that is the real force, the one that makes it turn. the feeling of being thrown
toward the outside is the fictitious one, same as being pressed into the seat
under acceleration.

the accelerometer measures the real one. same rule as reading +1 g up while
parked - that is the ground pushing up, not gravity pulling down. so a left-hand
bend, tyres pushing the car leftward, reads positive on Y and lands in
accel_peak_left_g; the right-hand bend is the negative number.


## why the peaks are smoothed first

at 104 Hz a pothole is a single sample reading 2 or 3 g. record peaks off the raw
numbers and that one sample is the hardest braking of the trip.

body movement is all under about 5 Hz, impacts are far above it. so a 5 Hz low
pass keeps the first and drops the second, and only then are peaks recorded:

  y += alpha x (x - y)      alpha = dt / (tau + dt)

with tau = 1/(2 pi x 5) = 32 ms. writing alpha from dt means changing the sample
rate later does not quietly change the filtering.

two speed gates on top:

  3 km/h   below this the odometer stops, so it does not creep at a red light
  5 km/h   below this no push is recorded - a door slam is not cornering

lean angles use the lower gate on purpose. crawling over a rock at 4 km/h is
exactly when the tilt matters most, while a 4 km/h shove is not a manoeuvre.


## coasting when the GPS goes

a tunnel or an underground car park and the sky is gone. speed then comes from
the accelerometer: speed = speed + acceleration x time.

that only works for a while, because the error grows with the guessing:

  5 s      0.05 m/s out    about 0.1 m of distance
  30 s     0.3 m/s         about 4 m
  120 s    1.2 m/s         about 70 m
  5 min    3 m/s           about 450 m

so it gives up after 2 minutes and freezes the distance rather than inventing
more.

the good bit: if the guessed speed reaches zero it stays there until something
clearly pushes again - 0.05 g held for half a second. stopping resets the
accumulated error to nothing, so a tunnel where the car actually stops comes out
more accurate than one where it crawls the whole way. that has a name in
navigation, a zero velocity update.

while coasting there is no independent measurement of acceleration any more, so
the compensation handed to the attitude filter is set to zero rather than fed the
accelerometer's own guess. feeding a number back into the thing it came from is
how a filter starts believing its own mistakes.


## what gets handed back to the attitude filter

the whole point is that the two numbers come from somewhere other than the
accelerometer:

  forwards   how the GPS speed changed since the last fix
  sideways   speed x how fast the car is turning, from gyro Z

neither touches the accelerometer, so subtracting them actually tells you
something. subtracting a number derived from the accelerometer would just be the
accelerometer agreeing with itself.

the turn rate gets clamped to what the car can physically do at that speed first -
see vehicle_info.md. one corrupt gyro sample would otherwise shove a huge false
correction into the filter.

there is a loop here: the attitude filter needs the compensation, and the
compensation needs the pitch to remove gravity. broken by using last cycle's
numbers, one sample old at 104 Hz, which is 10 ms. normal, and what every control
loop does.


## elevation

two ways to know how high the car is, both bad alone:

  GPS altitude   right on average, jumps around by tens of metres
  air pressure   very smooth, slides with the weather over hours

opposite errors again, so the same complementary filter as pitch and roll, only
much slower - tau 60 seconds instead of 2.

turning pressure into height needs today's sea level pressure, which changes with
the weather and cannot be known. so the absolute number from the barometer is
always wrong. its changes are right though, and changes are all the filter wants
from it - the GPS supplies the absolute part.

  height = height + (what the barometer moved) then leaned towards the GPS

climb is added up from the combined figure, never the barometer alone, because a
weather front would otherwise register as a mountain - about 8 m of false height
per hPa the weather moves.

the climb count has a 3 m deadband so noise wobbling either side of one height
adds nothing. without it a flat motorway slowly accumulates a mountain range. the
test has a case for exactly this, and my first attempt used a wobble of 1.5 hPa,
which is 12 m - a real hill, not noise. the deadband was right and my test was
wrong.


## the car itself is a number now

App/Inc/vehicle_axes.h holds everything specific to the car and the mount, so
moving the board to another car is one file and a rebuild.

the car is an audi a4 b6 avant 1.8T 190hp quattro, 2004, 6-speed manual.
everything except the two centre-of-gravity numbers comes off its own data
sheet - the full table is in vehicle_info.md.

the useful part is that the car's shape feeds the maths:

  track width and cog height     how much sideways g tips it over
  cog to the rear axle           how steep a climb tips it backwards
  turning radius                 the fastest it can possibly turn

so the trip stats do not just say "0.4 g of cornering", they say how close that
was to putting the car over - the number that means something off-road. mass,
length and height are in there too but nothing calculates with them; they are for
the log, listed separately so it stays obvious which ones matter. mass is 1650 kg
and not the 1530 kerb figure, because the car never drives empty.

formulas and worked examples: docs/vehicle_info.md.


## timers and pwm, enough to follow tim.c

a timer is a counter that ticks at some speed and wraps to zero at a limit. two
numbers set everything:

  prescaler   divides the clock feeding the timer; the result is the tick
  period      how many ticks make one cycle before the wrap

both are hardware "value plus one", so prescaler 83 divides by 84 and period 999
means 1000 ticks per cycle.

the timers do not all get the same clock. TIM2 and TIM5 sit on the slow bus and
are fed 84 MHz; TIM1 sits on the fast one and gets 168 MHz. that is why TIM1
needs prescaler 167 for the same 1 MHz tick the others get from 83.

pwm is the timer driving a pin: output stays high until the count passes a third
number, the compare value, then goes low until the wrap. the fraction of each
cycle spent high is the duty cycle, and that is the actual knob. the flipping is
done by the timer hardware, no code runs per cycle.

  TIM1   1 MHz tick, period 999     1 kHz pwm, 1000 duty steps - backlight
  TIM2   1 MHz tick, period 99999   10 Hz - bench value for the header pin
  TIM5   1 MHz tick, period 99999   10 Hz - same, the other header pin

the backlight numbers can stay: 1 kHz is far above visible flicker and 1000 steps
is plenty. the two header timers are placeholders - whatever daughterboard uses
them brings its own numbers.


## why the chip selects idle high

chip select tells one chip "this bus traffic is for you". active low - low means
selected, high means ignore the bus.

cubemx asks what level every output pin starts at, and the default is low. for
the three chip selects that would mean every chip wakes up already selected:

- an sd card in spi mode wants at least 74 clock pulses with its chip select held
  high before it will enter spi mode. boot with the pin low and that quietly
  fails until code fixes the pin
- a selected display treats whatever appears on the bus as commands, so stray
  startup traffic could leave the controller in a state the init code does not
  expect

so all three start high, set in cubemx so a regeneration keeps it.

the display reset pin is the opposite on purpose: it starts low, holding the
panel in reset until the init code wakes it into a known state.


## printf over usb

at this point the board could not tell me anything. i had no debug probe yet, so
no breakpoints and no watching variables, and the only output was an LED. enough
to prove the timers run; useless for "did the sensor answer" or "is z reading
1.00 or 0.98". (i got an ST-Link V2 clone later, after step 5 - see the swd
section near the end. the usb console still matters, it is the only channel that
exists in the car.)

the fix is the usb-c cable already plugged in. it already worked - dfu-util
flashes through it - so the connector, the traces and the 48 MHz usb clock were
proven before i started. what was missing was software.

a usb device has to say what kind of thing it is. there is a standard kind called
CDC, communication device class, meaning "i am a serial port", and every OS ships
a driver for it. once the firmware claims to be CDC the mac makes a file at
/dev/cu.usbmodem... and screen reads it like a terminal. no new wire.


## what happens when i plug it in

plugging in does not just start working. the host runs a short interview first,
called enumeration:

- it holds the data lines in reset
- it asks who are you, what class, how much power, what pipes do you have
- the device answers with descriptors - small fixed byte tables in the firmware
- only at the end does the host say "configured"

before that last word the device cannot send anything and my buffers do not exist
yet. that is why usb_serial_ready() checks for the configured state before every
send. printing earlier would not just be lost, it would crash - the cdc code
follows a pointer that is null until the host configures the device.

the port name comes out of that interview too. mine is
/dev/cu.usbmodem204A396852301 and that number is this chip's own serial number,
burned in at the factory. a second board would get a different name.

cubemx deleted Core/Src/usb_otg.c when i added the middleware. correct, not a
mistake - the hardware setup moved into usbd_conf.c because the usb stack owns
that layer now. keeping both would initialise the same peripheral twice.


## making printf go somewhere

on a pc printf hands its bytes to the operating system. there is none here, so
the chain has a missing bottom end:

  printf()          formats the text
  newlib            the c library
  _write()          "send this block somewhere"
  __io_putchar()    "send this one character somewhere"
  usb_serial.c      actually sends it

_write already exists in Core/Src/syscalls.c but it is marked weak, so the linker
throws it away the moment a real one turns up. it loops over __io_putchar, and
that is the hook i fill in - so i plug into the gap without editing a generated
file.

two things that bit:

- newlib keeps a buffer of its own above all this and holds output until about a
  kilobyte piles up. setvbuf(stdout, NULL, _IONBF, 0) turns that off
- the linker uses nano.specs, a cut-down c library that leaves floating point out
  of printf. %d works, %.2f prints nothing useful. -u _printf_float pulls it back
  in and costs 11.8 kB, measured on this toolchain. worth paying - every number
  the sensors give me is a float


## why there are two buffers

sending one character at a time over usb would be slower than the printf itself,
so __io_putchar only appends to a buffer and the whole line goes out on the
newline.

the catch is that CDC_Transmit_FS does not copy anything. it stores a pointer and
returns straight away; the real copying happens later inside the usb interrupt.
so this is wrong:

  CDC_Transmit_FS(buf, used);
  used = 0;                  // printf starts overwriting buf
                             // while the interrupt is still reading it

the result would be scrambled lines that come and go with timing. two buffers fix
it: hand one over, fill the other.

same family of trap as HAL_UART_ErrorCallback in uart_bus.c - a hardware job that
outlives the function call that started it.


## nobody is listening

usb devices never start a transfer. the host asks "anything for me?" and the
device either hands over a packet or says NAK. if no program has the port open
the mac stops asking, the queued packet sits there forever, and CDC_Transmit_FS
keeps answering busy.

so every send has a 10 ms timeout and throws the line away when it runs out.
without it the firmware would freeze in that loop the first time nothing is
attached - which is every time it runs in the car.

characters thrown away like that are invisible by nature, so they get counted,
same idea as uart_bus_overruns(). the first run showed drops 149 before screen
was attached, and 149 is exactly the banner plus the first tick line added up by
hand. the moment screen opened it froze and stayed frozen.


## the sensors answered

step 1 was the first time the firmware talked to another chip. up to here it was
all the stm32 talking to itself - timers, clocks, usb.

call imu_init() and env_init() once, print whether they worked, then read both
every second. both said OK first try.

i put a bus scan in front of them. it walks every 7-bit address from 0x08 to 0x77
and sends just the address with no data, then looks at whether anything pulls the
line low to answer - the ACK from the I2C section above.

the point is telling two failures apart. if imu_init() fails on its own i cannot
tell whether the part is dead, the address is wrong, or the whole bus is down:

  both 0x6A and 0x76 answer   bus and soldering fine; a register is the problem
  nothing answers at all      bus level - pull-ups, the two pins, or power
  only one answers            that one part's solder joints

worth ten lines because every flash means moving the boot jumper by hand.

the drop counter did its job again. it stood at 1352 before screen attached and
never moved after. counting by hand - the banner, then seven seconds of readings
at five lines each, with a carriage return on every line - comes to about 1363
bytes.


## a state and a rate

the thing that confused me watching the numbers.

the accelerometer measures a state: which way is down right now. tilt the board
and hold it, the numbers change and stay changed.

the gyroscope measures a rate: how fast am i turning right now. rotate it and it
reads maybe 80 dps while my hand moves; stop and it drops back to nearly zero, no
matter which way the board ended up facing. it has no idea where it is, only that
something is changing.

so a gyro trace is spikes while something moves and flat the rest of the time,
and a board sitting still reads zero whichever face is down. correct, not a
fault.

why keep both:

  accelerometer   honest over the long run, useless while moving; it cannot tell
                  gravity from acceleration, so braking hard looks like the world
                  tilted

  gyroscope       ignores that completely and reacts instantly, but the angle
                  only comes from adding the rate up, and adding up a wrong
                  number adds the wrongness up with it

my own readings show why. the board is dead still and the gyro reads about
+0.45 dps on X - in spec. but add it up: 0.45 x 60 = 27 degrees of turn per
minute that never happened. that is the drift the complementary filter exists to
remove, and this is the first time i have seen it as a number off my own board.


## sensor axes and car axes

two coordinate systems are live at once, and mixing them up cost me an argument
with myself.

  sensor frame   what the LSM6DSO reports. fixed by how the chip is soldered.
                 flat board reads -0.990 on Z, forever, and that is correct
  car frame      X forward, Y left, Z up. what everything above imu.c expects

vehicle_map() is the only thing that crosses between them. nothing below it knows
the car frame exists.

the 1 Hz debug line prints the sensor frame on purpose - raw truth, and if it
ever changed i would want to see it. the calibration block prints the car frame,
because that is where the constants live. so "Z still reads -0.990" is not a bug
report, it is the lower layer doing its job. the test is whether the mapped Z
reads positive, and it does.


## the chip is upside down

the pass criterion i wrote for step 1 said a flat board should read about +1 g on
Z. it reads -0.990.

an accelerometer at rest reads +1 g on whichever axis points up. the LSM6DSO's +Z
comes out of the top face of its package, and the chip is soldered to the bottom
of the board, so that face aims at the table. +Z points down, Z reads minus one.
the mounting, not a fault - i wrote that criterion before thinking about which
side the part is on.

what it changes: attitude.c wants Z up. feeding it the raw sensor Z would give
exactly the sluggish tracking the sign section warns about. the fix belongs in
the axis map, not in attitude.c.


## working out the axis map

step 2. one rule does all of it: an accelerometer reads +1 g on whichever axis
points up. everything else is bookkeeping.

  1. picked a forward - the edge opposite USB-C
  2. flat: sensor Z reads -0.990, so sensor +Z points at the table
  3. nose up: sensor X went to +0.71, so sensor +X points forward
  4. right side down: sensor Y went to -0.61, so sensor +Y points right

car axes want X forward, Y LEFT, Z up. so X survives, Y flips, Z flips.

the flip is not about how the board gets mounted in the car - it is only about
which side of the PCB the chip is on. take a book: arrow forward on the cover,
arrow left, arrow up out of the cover. flip it over about the forward arrow.
forward is still forward, up now points at the table, left now points right. one
motion, two arrows reversed. that is the chip.

flipping only Z would be a mirror, and no rigid rotation makes a mirror. the
check is the determinant: (+1)(-1)(-1) = +1 is a rotation, -1 would mean i made
an error.

then i checked it by running the three real tilt readings back through
vehicle_map() on the mac. nose up gave +46 degrees of pitch and right side down
gave +38 of roll, and attitude.h says + is nose up and + is right side down. both
right. drop the Y minus and roll reads -38 - the display leans the wrong way
while still looking plausible, which is the failure mode that hides.

the map only expresses quarter turns and flips. the actual mounting angle - the
40-ish degrees the case will sit at - is not in it at all; that goes in the rest
accel reading.


## what the bench numbers are worth

board flat, everything inside what the datasheet allows:

                        measured             datasheet allows
  accel X               +7 to +16 mg         +/-20 mg zero-g offset
  accel Y               +1 to +4 mg          same
  accel Z magnitude     0.988 to 0.992 g     +/-1% sensitivity tolerance
  gyro, all three       0.31 to 0.56 dps     +/-1 dps zero-rate level

nothing out of tolerance, which is the real result - the parts and the joints are
good.

then something i did not expect. the same board reads differently on a mouse pad
and on a wooden table:

  mouse pad    ax = +0.015     tilt = atan2(0.015, 0.990) = 0.87 degrees
  wood         ax = +0.0075    tilt = atan2(0.0075, 0.991) = 0.43 degrees

about 0.43 degrees flatter on wood - the foam squashes unevenly under the
through-hole solder tails.

the catch is that the absolute number is not trustworthy. up to 20 mg of that X
reading could be the sensor's own zero error rather than tilt, and 20 mg is worth
1.15 degrees. so i cannot say "the board is tilted 0.87 degrees", only "these two
surfaces differ by 0.43", because the zero error is the same in both and cancels
out of the difference.

which is exactly why the rest reading is stored as a calibration constant instead
of assumed to be zero. it measures the tilt and the zero error together, and for
cancelling them both out that is all that is needed.

practical: calibrate on the hard table.


## judging an average

the calibration prints two spreads and they answer different questions:

  peak-to-peak in one run   was the board still?
  spread between runs       is the thing i am measuring stable?

mine came out 0.003-0.007 g and 0.105-0.175 dps within a run, matching the
datasheet noise density (75 ug/sqrt(Hz) accel at ±4 g, 3.8 mdps/sqrt(Hz) gyro, at
about 52 Hz of bandwidth). so nothing bumped the table.

when the between-runs spread is larger than the within-run p-p, the thing is
drifting, not noisy. that is exactly what gyro X did - 0.19 dps between runs
against 0.12 within one. two spreads, two different diagnoses, and one number
alone cannot give you that.


## the gyro offset moves while the board warms up

the zero-rate level is not one fixed number.

  just powered      gyro X = +0.42 dps
  21 minutes in     gyro X = +0.61 dps      and still climbing

Y and Z settled straight away - 0.002 and 0.006 dps across four runs. only X
drifts.

so the board has to be warm before the 500 samples get averaged, and warm is
longer than i guessed. i said five minutes; twenty was not enough.

what i cannot do is blame a number on it. the BME680 only moved 2.3 C over that
window, which would make the drift 0.080 dps per degree, eight times the
datasheet's typical. but the BME680 sits in its own self-heated pocket that
saturates in the first minute, while the IMU die warms on a slower curve. wrong
thermometer, so that figure means nothing. the LSM6DSO has its own temperature
register if this ever needs doing properly.

why i stopped chasing it: the leftover 0.2 dps is 0.4 degrees of standing error
at tau 2 s, and the car's own 40 degree temperature swing causes 0.8 degrees
regardless. halving the smaller of two errors is tidiness, not engineering.


## the bme680 warms itself

the temperature reads high - about 30-31 degrees, above the room.

expected, and the datasheet says so - the reading tracks pcb temperature, the
sensor's own self-heating and the ambient together, and sits above ambient. no
surprise with a 300 degree plate firing for 100 ms every second a millimetre from
the die, next to a 168 MHz chip and an LDO.

the humidity follows it. relative humidity is a ratio against how much water the
air could hold, and warmer air could hold more, so the same water reads as a
lower percentage. the sensor is not wrong - it is correctly reporting its own
warm pocket, which is not the room.

i can back the room out of it. with the sensor at 31.0 C and 29.1 %RH:

  room 24 C  ->  43.9 %RH
  room 25 C  ->  41.4 %RH
  room 26 C  ->  39.0 %RH
  room 27 C  ->  36.7 %RH
  room 28 C  ->  34.7 %RH

nothing is broken - pressure is sane at 1005-1006 hPa, gas responds to breath,
and T and RH are consistent with each other.

not calibrating the offset yet though. at 1 Hz the heater runs 10% of the time;
BSEC's low-power mode measures every 3 s, about 3.3% duty and a third of the
heat. measuring an offset now would calibrate it against a duty cycle i am not
going to ship. that belongs with BSEC.

the gas resistance climbs the whole time a session runs - 16 kohm at eight
seconds, 27 kohm at a minute, plateauing near 51-57 kohm. the metal oxide layer
burning off what it picked up sitting in a drawer; higher resistance means
cleaner air. bosch tracks this in bsec with two flags, one for the very first use
after production and one for every switch-on after. more evidence that raw ohms
are not an air quality number.

breathing on it drops the resistance, which is the right direction - the vocs
make the layer more conductive. a finger on the package pushes temperature and
humidity up together, also right.

one thing i wondered about and then answered: the humidity kept falling while the
temperature was flat, which the ratio does not explain. it settled and stopped,
so it was the sensor equilibrating, not a runaway.


## the gps talked first time

one flash, one screen, the whole chain proved. i print counters every second:

  bytes     everything the UART received
  sent      complete $...* sentences
  rmc gga   what parsed
  other     good checksum, a type i ignore - GSV, GSA, VTG, GLL
  bad       checksum failed - noise on the line
  ovr       ring buffer overflowed

read it as a ladder. bytes stuck at 0 = no signal on the pin. bytes moving, sent
0 = wrong baud. sent moving, rmc 0 = parser. all moving = fine. first run: 261
bytes a second, exactly, rmc and gga +1 each, bad 0, ovr 0. wire, baud,
interrupt, ring and parser done in one go. the counters stay in for the car -
bad climbing means the line is picking up noise.


## the time was fake

first screen said 23:59:45 05/01/80 and rolled over to 06/01/80. that is GPS
time zero - 6 january 1980. the module fills the time fields from the first
second on and just counts up from the epoch until it hears a satellite. my
time_valid only meant "the fields were not empty", so it was true on garbage.

fixed in gps.c: time_valid only when the year is 26 or later and below 80. the
two digit year makes 1980 look bigger than 2026, hence both ends.


## sats 0 means less than i thought

the number in GGA is satellites used in the fix. it stays 0 right up to the fix,
so it cannot tell "antenna hears nothing" from "not enough yet". GSV has the
in-view count but i do not parse it. free proxy: bytes per second. with nothing
heard every GSV is the empty $GPGSV,1,1,00*79 and the total is 261. as
satellites appear GSV grows and the number climbs. about 650 with a fix.


## twelve minutes of sky, zero satellites

open sky on the balcony, 261 bytes a second for 12 minutes. the module is
-149 dBm sensitive and the sky hands you about -130; nothing "a bit off" loses
19 dB. either an open somewhere or the antenna was not an antenna.

multimeter, board off:

  antenna port to RF_IN          0.2 ohm   the trace
  antenna port to ground         0.4 ohm   the antenna's own element; a solder
                                           bridge at the module would read 0.2
  antenna ground pad to ground   0.0

board on: V_BCKP present. nothing open, nothing shorted, backup domain fed.
electrically perfect, so it had to be the RF part.


## the keepout i never made

a chip antenna is not the antenna. it is a small ceramic resonator that drives
currents into the ground plane, and the plane does the radiating. it wants
ground beside it and none under it. copper 0.2 mm below it (F.Cu to In1)
mirrors the field and cancels it, and pulls the resonance far from 1575 MHz.
easily 20-30 dB. that is the whole story.

the datasheet never says it. the table says "ground plane 50 x 100 mm", which
is the counterpoise next to the antenna, and the only place the clearance
exists is the drawing: hatched = copper, blank = nothing, and the antenna sits
in a blank 7.8 x 12 mm box in the corner. two of us read that sheet and neither
saw it. the good manufacturers write it as a sentence: keep-out, no copper on
any layer.

my board has solid pour under the antenna on all four layers. the docs said a
keepout was there. it was the plan, never the copper. check the copper.

if a chip antenna ever goes on a board of mine again:

  corner, long side along the edge, 2.3 mm in
  7.8 x 12 mm rule area on all copper layers - no pour, no vias, no tracks
  module right next to it, feed a few mm, 50 ohm for the stackup
  pi footprint on the feed, 0 ohm fitted
  hide F.Cu and look at every inner layer - DRC does not catch this

or a patch antenna, which wants ground under it and forgives.


## how i fixed it, on this board

took the chip off, soldered it back on two pieces of THT resistor leg, so it
sits about 1 cm above its own pads with air under it. that is it. from the
balcony: fix, 9 satellites, hdop 1.2, quality 2 (EGNOS corrections). it holds
until it does not - that corner is fragile now.

if it breaks, the clean fix is a U.FL receptacle (hirose U.FL-R-SMT or i-pex
20279) on the antenna pads and a passive 25 mm patch on a cable with an MHF
plug - quectel YFGA025E3BM or DM, taoglas CGGBP254.07.0100A. passive only: the
board puts no DC on RF_IN, and an active antenna has an LNA powered through the
cable that blocks the signal when unpowered. the quectel AM is the active one,
the letter matters. for a rev B, the pocket above, or a patch.


## what the first fix taught me

  position   20-50 m off, drifting 30 m while still. hdop 1.2 says the geometry
             is fine, so that is multipath off the building. dash under a
             windscreen should give 3-5 m
  hold       the position freezes for long stretches then steps. the module
             pins it when it thinks it is stationary
  speed      0.0 with blips of 1.5-2 km/h standing still, about 0.5 m/s of
             noise. the odometer gate wants to sit around 3-5 km/h
  altitude   above mean sea level, not above the street. bucharest is roughly
             70-90 m, so 16 m is about 70 low. vertical is the weak axis and a
             balcony sees half the sky. check it under open sky before
             elevation.c trusts it
  time       UTC. +3 h here in summer, a display concern. also about 3 s off
             early on; the module uses a built-in leap second default until it
             downloads the real one, up to 12 min after a cold start. recheck
             after a long fix
  the gap    one second with no bytes and age 1.8 s, right when the time went
             real. the module re-aligned its second to gps time. harmless, the
             ring did not notice


## the backup cell

the ML2032 went in on 16 sep at 3.07 V and never held anything. by 19 sep i
could not read a voltage across it at all, and every start was cold - off for
10 s after a fix, the clock took ~12 s and the fix over a minute.

swapped in a CR2032 at 3.3 V: off 10 s, clock back instantly, fix in ~10 s.
so the V_BCKP path and the module are fine, the ML2032 is the problem.

careful: a CR2032 is not rechargeable and the board trickle-charges BT1. it
cannot stay in with the charge path connected. oct: no cell in for now, a new
ML2032 next, CR2032 with D3 out as the fallback - never R6, see back on the
bench.


## the baud rate that does nothing

screen /dev/cu.usbmodem 115200. that number is ignored. CDC carries a baud field
so a usb-to-rs232 adapter can pass it on to the real uart on the far side; my
board is the far side, so it reads it and does nothing with it.

what i actually have is usb full speed, about 1 MB/s of payload. the attitude
stream at 5 lines a second is 310 bytes/s.

so the wire was never the limit. the limits are my eyes and the 10 ms flush
timeout in usb_serial - both about how many newlines a second, not how many
bytes. that is why the fix was printing less often, not printing shorter lines.


## the filter rate and the print rate are not the same thing

sample -> compute -> decide whether to say anything. the first two run at
whatever the physics needs, the third at whatever i can read. tying them together
forces a choice between a slow filter and an unreadable console.

ATT_PERIOD_MS is the maths, ATT_PRINT_DIV is the console, and changing the second
cannot touch the first. when INT1 moves the loop to 104 Hz only the first one
disappears.


## a blocking read inside a loop

env_read sits in a busy wait about 143 ms while the heater plate comes up and the
bme680 converts. in a superloop that means nothing else runs - not the attitude
tick, not gps_poll draining a ring against a module sending 900 bytes a second.

with BENCH_SENSORS on it went like this once a second: two normal 100 ms ticks,
then one tick with dt 243 ms. the filter survives that because dt is measured,
but anything i did to the board inside that window is simply gone. so i turned it
off for the step rather than test through a hole.

the real fix is a non-blocking read - kick the measurement off, come back for it
later. that belongs with the task loop, not here.


## measuring dt instead of assuming it

tempting to pass 0.1f and be done. it is not 0.1 - the tick fires on the first
pass at or after 100 ms and the loop body varies, so it is 100, sometimes 101,
sometimes more.

an assumed dt turns every millisecond of jitter into an integration error that
never comes back. measuring costs one subtraction.

the subtraction has to be unsigned. HAL_GetTick counts milliseconds in a uint32
and rolls over to zero after 49.7 days; now - last in unsigned arithmetic wraps
the same way the counter did and comes out right, while now > last + period would
break. that is why every timeout in this codebase is a subtraction.

dtmax on the console is the audit for it. read 100 on every line for 200 seconds.


## the settling curve is the proof

best thing in the step 4 logs. front edge propped up, printed at 5 Hz:

  15.71  15.43  15.74  15.41  15.11
  14.82  14.57  14.34  14.14  13.96
  ...
  12.46  12.43  12.40  12.36  12.36   ->   12.22

two separate things going on.

the overshoot to 15.7 is the gyro. while i was moving the board the gate was open
so the angle came from the gyro alone, and at 10 Hz i integrate one instantaneous
rate over a 100 ms slice as if it held for the whole slice. during fast motion it
does not, so it overshoots. INT1 fixes that.

the decay back is the accelerometer pulling. alpha is 2/2.1 = 0.952, so each step
keeps 95.2% of the gyro answer and takes 4.8% of the accel one - an exponential
with time constant tau. 63% in 2 s, 95% in 6 s.

check it against the log: 3.49 degrees to remove, and six seconds later 3.25 of
it was gone. 93%. so tau is 2 s measured, not assumed any more.

and this is the pass criterion, not the static number. a sign fight never settles
- the two halves pull against each other so it creeps or hunts. mine overshot,
decayed cleanly from both directions and came back to zero after being waved
about. that is the thing to look for.


## the ruler has error too

the reference was asin(h/L) - prop one edge up by h, the board is the hypotenuse
of length L. nothing in it comes from the accelerometer, which is the whole
point; a test that shares an assumption with the code cannot catch that
assumption.

asin and not atan: h is the opposite side and L is the board itself, the
hypotenuse, not the horizontal projection. atan would have given 11.93 instead of
12.18 - a quarter of a degree, a quarter of my whole budget.

and the reference is not exact either:

  d(angle)/dh = 1 / (L x cos angle)

  pitch, L 61.6 mm     0.95 deg per mm    +/-0.5 mm is +/-0.48 deg
  roll,  L 95.79 mm    0.60 deg per mm    +/-0.5 mm is +/-0.30 deg

roll came out 0.29 off. that is not a 0.29 error, it is inside what the ruler can
resolve; the honest sentence is "agrees to the resolution of the measurement".

it also says how to design the next one - pick a big angle. the error is fixed in
millimetres, so it shrinks in relative terms as the angle grows.


## ruling out the firmware without touching the board

the usb-c edge read -8.90 where the geometry said -12.18. before blaming anything
i checked whether the code could even do that.

  pitch_acc = atan2(ax, hypot(ay, az))

flip the sign of ax and the answer flips exactly - the function is odd. no
setting makes it squash one direction and not the other. the only thing that
could skew the pair is a zero offset: true +A and -A would read +A-d and -A-d.
solve for d from my two numbers and it needs -1.66 degrees. but d is exactly what
a flat board reads, and flat read +0.01.

contradiction, so there is no offset, so the two edges really did reach different
heights. -8.90 back-solves to 9.53 mm instead of 13, and that edge is the one
with a rigid plug body hanging off it.

the shape of this is worth keeping: find a property the code has by construction,
then find the one observation that closes the loophole. cheaper than another
experiment and more certain.


## step 4 numbers

board is 61.6 x 95.79 mm, usb-c on a 95.79 edge - so fore-aft is 61.6 and
left-right is 95.79. propped 13 mm:

  front edge up     expected +12.18    read +12.22
  left edge up      expected  +7.80    read  +8.09
  flat              expected   0.00    read  +0.01 pitch, -0.15 roll

dtmax 100 ms on every line over 200 s, err 0, drops frozen at 2980 the whole run
- so 5 Hz plus the per-second block fits with room.

the gps year gate worked here too: time -- until about 190 s, then real UTC on
the right date while still showing fix no and sats 0. time arrives before
position because one satellite is enough to set the clock, four are needed to
solve for where you are.


## the sensor tells me instead of me asking

until step 5 the loop asked - every 100 ms it looked at the clock and decided it
was time to read. that is polling. it works, but the loop and the sensor are two
clocks that know nothing about each other, so i either read too often or miss
most of the samples. at 100 ms against 104 Hz i threw away 9 out of 10.

an interrupt turns it round. the sensor raises a wire when it has something, the
cpu stops what it is doing, runs a small function, and carries on.

the chain, every link of which had to exist:

  new sample                  LSM6DSO pulls INT1 high
  PB10                        sees a rising edge
  EXTI line 10                latches a pending bit
  NVIC                        sees EXTI15_10 pending, and enabled
  EXTI15_10_IRQHandler        runs
  HAL_GPIO_EXTI_IRQHandler    clears the pending bit
  HAL_GPIO_EXTI_Callback      my code

the pin was already GPIO_EXTI10 and already GPIO_MODE_IT_RISING from an earlier
cubemx pass, so the NVIC enable was the one missing link - the whole cubemx job
was one checkbox. on the sensor side INT1_CTRL bit 0 has to be set or the pin
never moves at all.

the pin defaults happened to match the board already: H_LACTIVE 0 is active high
and PP_OD 0 is push-pull, which is what rising-edge with no pull expects. so
INT1_CTRL was the only register i wrote.


## the latch, and the trap in it

COUNTER_BDR_REG1 bit 7 is dataready_pulsed and defaults to 0. the datasheet
calls that "data-ready latched mode (returns to 0 only after an interface
reading)".

so the line is not a pulse. it goes high and stays high until i read the output
registers. which means one edge per sample as long as i keep reading - and:

  if i ever stop reading, the line stays high, no new rising edge happens, and
  interrupts stop forever.

worth remembering because it looks exactly like a dead pin or a bad joint, and
it is neither. the fix would be "read the data", not "check the hardware".

the upside is it cannot get out of step. with BDU set (imu_init already does)
the registers hold until both halves are read, so i can never get a torn sample
or the same one twice. that is why i kept latched instead of pulsed - pulsed
gives a cleaner raw count but is not what i want to ship.


## why the handler does almost nothing

the callback is three lines: check the pin, bump a counter. on purpose.

while an interrupt runs, others at the same or lower priority wait. USART1 takes
an interrupt per received byte to feed the gps ring. if i did the i2c read inside
the handler instead - 14 bytes at 400 kHz, about 350 us of blocking HAL call - i
would be blocking that path 105 times a second. a 9600 baud byte arrives every
1.04 ms, so that is a third of the gap between gps bytes spent inside an
unrelated handler. that is how nmea characters start disappearing for no visible
reason.

same reason there is no printf in there - it formats a string, walks a buffer and
can sit up to 10 ms waiting on usb.

the rule: a handler records that something happened, the main loop decides what
it means.

priority while i was at it. USART1 sits at preemption 0, the highest; i put
EXTI15_10 at 1. on cortex-m lower number means higher priority, and a higher one
can interrupt a lower one that is already running. same number means neither
interrupts the other, they queue - so with both at 0 a gps byte would wait for my
handler to finish. the margin was never the problem, a few hundred ns against a
1.04 ms byte, but one dropdown made it impossible instead of just unlikely.


## edges and reads cannot drift apart

i print two counters, edges and reads, and they came out equal every second.
that is structural, not luck:

  edge -> line high -> loop reads -> line low -> next sample -> next edge

an edge needs a read before it, a read needs an edge before it. they alternate,
so edges minus reads is 0 or 1 at any moment. reads can never run ahead of
edges, and a slow loop shows up as both numbers dropping together rather than as
a gap between them.

which is why dtmax is the thing that catches a missed sample, not the counters.

it also settled the 105 question. the ODR is called 104 Hz and i measured 105.
a dropped sample would show as about 19 ms in dtmax; dtmax never went above 10.
so it is the chip's internal oscillator running about 1% fast, not me losing
slots. the datasheet on disk gives no ODR tolerance to compare that against.

  105 Hz -> 9.52 ms period
  HAL_GetTick has 1 ms resolution, so that reads as 9 / 10 alternating, max 10

which is exactly what was on screen.


## volatile

int1_edges is static volatile uint32_t. volatile tells the compiler this can
change behind its back - the handler writes it, the main loop reads it, and
nothing in the main loop's own code explains why it would ever change. without
it the optimiser is allowed to read it once, keep it in a register and loop
forever on a stale copy.

it is a 32-bit aligned variable on a 32-bit machine so the read is atomic -
no half-updated value can be seen, and i do not need to disable interrupts
around it.


## step 5 numbers

  int1  edges/s 105  reads/s 105  dtmax 10 ms  err 0

same on every second across 66 s, drops frozen, and it held while i moved the
board hard enough to swing the angles 150 degrees. dtmax never went over 10, so
not one sample was missed out of about 6900.

what did not change: tau is still 2 s. alpha went from 2/2.1 = 0.952 at 100 ms
to 2/2.0095 = 0.9953 at 9.5 ms - each step takes a much smaller bite of the
accelerometer, but there are ten times as many steps, so the time constant is
the same. flat pitch and roll reading the same as step 4 is the right answer,
not a sign nothing happened.

what did change is the aliasing. step 4's flick overshot 3.5 degrees because i
integrated one gyro sample as if its rate held for the whole 100 ms. same
approximation now, but over 9.5 ms slices, so the error shrinks with the slice.

two things i noticed and left alone:

  roll drifted 0.33 deg over the first 16 s, at a slowing rate. gyro X is the
  suspect - roll rate is mostly gyro[0], and X is the one axis step 2 left with
  0.2 dps of doubt. test is to leave it powered 20 min on something rigid and
  see if it flattens. inside the error budget either way.

  gps age walked from 341 ms at boot up to 975 at 24 min, then wrapped to 72.
  that is my one-second print and the module's one-second output sliding past
  each other, about 400 ppm apart. not the loop - it catches the ms boundary.
  could be the 12 MHz crystal, systick, or the module's own oscillator with no
  fix to discipline it; that last one is most likely and an open-sky fix would
  settle it. 400 ppm is 35 s a day - nothing yet, matters for log timestamps.

  pitch went past -90 while i was waving it about. atan2 cannot do that, so it
  is the gyro half integrating free - nothing clamps pitch the way wrap180
  bounds roll. next to 90 degrees of pitch the euler angles are singular and
  roll stops meaning anything, which is the quaternion limit i wrote about
  earlier. only shows up past a rollover.


## the debug probe, finally

got an ST-Link V2 clone after step 5. three wires to J1: SWDIO pin 1, SWCLK pin
2, GND pin 4. pin 3 is +3.3V and i leave it alone - the board is powered from
usb-c through its own LDO, and the clone's 3.3V pin is an output, so connecting
it puts two supplies on one rail.

no NRST on the header, so it is a hot-plug attach and a software reset. "connect
under reset" is not available and does not need to be.

usb-c stays plugged in the whole time. it is still the power and still the
console - and in the car it is the only cable, so the console has to keep
working on its own regardless.

what changed: flashing is one button in clion now. no BOOT0 jumper, no power
cycle, and screen only drops because the reset re-enumerates usb.

the BOOT0 jumper is off the board now (19 sep) - the probe is the only way in.
DFU is still in the chip, so if the probe or the header ever dies i refit the
jumper and it comes back.


## what the probe actually shows

three things printf cannot do:

  breakpoints        stop mid-loop and read every local
  live watches       read a variable while the cpu runs, no halt, no code change
  peripherals        read EXTI, GPIO, I2C registers by name, live

the EXTI check was the satisfying one. step 5 i proved the interrupt worked by
counting edges; now i can just read it:

  IMR    0x00000400    bit 10 unmasked, nothing else
  RTSR   0x00000400    rising edge only
  FTSR   0x00000000    no falling edge, or i would count twice
  PR     0x00000000    nothing pending, the handler keeps up

and the axis map, which took a whole bench session in step 2, in one breakpoint:

  raw.accel_y_g  -0.001708    ->  accel[1]  +0.001708
  raw.accel_z_g  -0.991250    ->  accel[2]  +0.991250
  raw.gyro_y_dps +0.455       ->  gyro[1]   -0.455

diag(+1,-1,-1), on both sensors, exactly. also caught raw.gyro_x_dps reading
0.385 five seconds after boot against the 0.6096 stored in the header - that is
step 2's thermal drift, cold vs warm, seen directly for the first time.


## debugger things that bit

live watches only work on file-scope or static symbols. int1_edges is static in
imu.c so it has a fixed address and works; att_reads is a local in main() and
lives on the stack, so there is no address to read and gdb says no symbol. for
locals use Threads & Variables at a breakpoint instead.

never put a conditional breakpoint on a hot path. gdb evaluates the condition by
halting, reading over swd, and resuming - every single hit. on a 105 Hz line the
target ends up running at a few percent of speed and the condition never comes
true. i sat there thinking it had not triggered. a plain breakpoint fires once,
which is all i wanted anyway.

only 6 breakpoints, and they are all hardware ones - software breakpoints work by
writing a trap instruction into the code, and the code is in flash.

after a breakpoint, ovr climbs and the next dtmax is huge. that is the halted cpu
not servicing USART1, and the real gap across the halt. both expected, neither
survives a power cycle.

no SWO pin on a 4-pin header, so no ITM printf and no SWO console. leave it
disabled in the profile.


## the first flash of the display and card block

16 sep. steps 6, 7 and 8 all went on the board in one session. the panel
works, the card works, the loop holds all of it at once. two things came out
different from what i had written here and both are corrected in place below:
the panel cannot be read at all, and the card is not the worst stall in the
loop - the usb console is.


## the display has no framebuffer

320 x 240 pixels at 2 bytes each is 150 KB. the chip has 128 KB of RAM. so
there is no picture in memory, ever.

instead gfx.c owns one band, 320 x 16 pixels, 10 KB. to repaint something the
screen says which rectangle changed, gfx cuts it into bands that fit the
buffer, and for each band asks the screen to draw itself into it, then sends
it to the panel. every primitive clips itself to the band, so drawing the
whole screen into a 16 row slice costs almost nothing outside the slice.

why one band per loop pass: the IMU sample gap is 9.5 ms and the loop has to
be back for the next sample. a band is 2 ms on the wire at 42 MBit/s, a whole
frame is 15 of them, so the display never holds the loop for more than 2 ms.
dirty rectangles keep it small anyway - a moving dot is two little squares,
two bands, and a number that did not change costs nothing.


## pixels on the wire

RGB565 - 5 bits red, 6 green, 5 blue, 16 bits a pixel. the panel wants the
high byte first and the M4 stores the low byte first, so every pixel is byte
swapped as it goes into the band; the buffer then goes out untouched.

18 bit would be 3 bytes a pixel for colours nobody can tell apart on a
2.8 inch panel. 16 it is.


## reading from the panel

writes run at 42 MBit/s. the datasheet says the write cycle is 100 ns
minimum, which is 10 MHz - so i am four times over, like every driver for
these modules. if pixels come out wrong the first thing to try is prescaler 4.

reads are another story: the read cycle is 150 ns minimum, 6.67 MHz. so every
read drops SPI1 to 5.25 MHz and puts it back after. at 42 MHz a read is just
rubbish.

the trap i wrote all this for: the 24 and 32 bit reads (04h, 09h, D3h) start
with one dummy clock, not a dummy byte. one dummy bit then 24 bits of ID, so
four bytes come back as 00 49 A0 FF and the ID is that shifted right by
seven. the 8 bit reads, 0Ah and 0Ch, have no dummy at all - p38 draws both
cases. lcd_probe prints the raw bytes and decodes the long one both ways.

the gate was two reads, not one. D3h for identity, and the pixel format
register before and after writing it, 06 then 05 - a write that goes in and
comes back proves the path without needing D3h, which the datasheet lets a
vendor blank out.

none of it works on this module. every read comes back FF - D3h, power mode,
pixel format, all four. not a blanked register; nothing drives the line at
all, and PA6 carries a pull-up so an undriven MISO reads exactly FF.

i checked the mcu side in the debugger before believing it. SPI1 CR1 344 -
master, mode 0, 8 bit, enabled; PA5 PA6 PA7 all AF5; CS high, DC and RST
where the driver left them. four clean transfers at 5.25 MHz, err 0, nothing
came back.

so the probe is advisory now. it still runs and still prints, because those
FFs are real evidence about this module, but lcd_init runs whatever it says
and its own read-back only happens where reads work. the colour bars are the
proof instead, and they came up first try.

the lesson: a read-back gate is worth having, but it must not be the thing
that decides whether to go on. writes and reads are separate paths and a
module can have one without the other.


## display init order

all from the datasheet:

  reset pin low 10 us or more, then 120 ms
  sleep out (11h), then 120 ms
  pixel format 55h
  madctl - orientation
  clear the whole frame memory to black
  display on (29h)
  backlight, last

the clear before display on matters: frame memory is random at power-on, and
with the backlight on that is a screen of static. so the panel stays dark
until a black frame is in it, then the boot screen goes up and the backlight
fades in over 300 ms.


## madctl

one register, six bits, says how memory maps onto the glass. MV swaps rows
and columns, that is landscape. MX and MY mirror. BGR swaps red and blue -
these panels are wired that way and need the bit set.

which combination is right is not in the datasheet, it depends on how the
glass is glued to the module. so the test pattern decides: five bars in the
order red green blue white grey, and a line of text. wrong order, flip BGR.
upside down, use the other constant. mirrored text, MX or MY.

settled: MV and BGR alone came out upside down, so it is MX MY MV BGR - E8.
BGR was right from the start; the bars read red green blue white grey first
try and the dot renders blue.

the bars are gone from the firmware now. they had one job, settling these
bits, and they did it once; boot is a black screen with the build date on it.

the orientation is not only cosmetic. the bottom of the screen is now the
usb-c edge, so the top is the edge opposite it, which is the front of the
car. screen.c moves the dot toward the top under braking, and braking throws
you forward, so the dot now points the way you are thrown. the other constant
pointed it backwards and looked perfectly fine.


## bitmap fonts

the panel cannot draw text, it only takes pixels. so a font is a table of
little pictures, one per character, 1 bit per pixel, packed in rows.

i did not draw them. tools/gen_fonts.py takes DejaVu Sans Bold - free to
embed - and rasterises it with FreeType through matplotlib, which the
python.org 3.12 python on this mac happens to have. three sizes, 12, 18 and
26 px cells. every glyph in a font has the same height and shares a baseline
so text lines up; widths are proportional. about 6.5 KB of flash.

one thing that bit: the lookup is code minus the first code, so a font's
characters must be one unbroken run. my first large font was 0-9 then . and
- and the dot vanished - it sits before 0 in ASCII. the generator asserts it
now.

and the consequence i forgot about later: the large font starts at character
45, so it holds - . and 0-9 and no letters at all. i wrote DASHBOARD in it
for the boot screen and got an empty screen. any text uses small or medium.


## the screen on the mac

gfx.c and screen.c have no HAL in them. tests/render_screen.c swaps the panel
for a framebuffer in RAM, renders the boot screen and screen 1 with made-up
numbers and writes them out as images. that is how the layout was checked
before a panel existed. it also counts bands: a full paint is 15, a dot move
plus a clock tick is 2, no change is 0.


## the screen, second pass

after looking at it on the balcony i took things off it. session number gone,
moving time gone, hdop and fix quality gone - none of them told me anything
while driving. what is left got more room:

  top       wall clock in local time, time since power-up, GPS and SD flags
  left      SPEED big, MAX, DIST
  centre    the gauge, unchanged
  right     temperature, humidity, pressure, gas
  bottom    satellites and position small, ALTITUDE on its own, bigger

two things i had to fix while moving it:

the left column dropped from four rows to three, which put MAX exactly level
with the gauge's left peak number - 132 and 0.6 rendered as 1320.6. the left
values now stop 10 px short of where the gauge numbers start.

the altitude on the panel was the raw GPS number. at hdop 2.2 that wanders by
tens of metres - i saw 114 m and 90 m at the same spot. it now shows the
fused one out of elevation.c, which is GPS steadied by the barometer, and it
sits still.

the live magnitude moved up by one line so its bottom is level with the top
of the side peak numbers. at rest the dot sits dead centre and the number was
landing on top of it.


## gps speed when nothing is moving

the panel said SPEED 2 and MAX 8 with the board sitting on a table. not a
bug in the maths - that is what the module reported.

speed noise tracks the fix. on the good balcony run, 9 satellites and hdop
1.2, it was 1.5-2 km/h. on the bad one, 6 satellites, hdop 2.5, no EGNOS, it
reached 8.

the real mistake was mine: only the odometer was gated, at 3 km/h. the
speedometer, the maximum and the acceleration estimate all read the raw
number, so 8 km/h of noise walked straight into vmax and - being over the
3 km/h gate - into the odometer too. 200 m of distance and 112 seconds of
moving time from a board that never moved.

now one floor, TRIP_SPEED_MIN_KMH, applied where the fix enters trip.c. below
it the speed is zero for everything downstream. 10 km/h for now, which is
above the worst noise i have seen; a real drive settles it.

the lesson: if a number is noisy, gate it once at the source. gating one
consumer and not the others is worse than not gating at all, because the
numbers stop agreeing with each other.


## the sd card talks spi

an sd card has two personalities. the fast one is the 4 wire sd bus, which
needs an SDIO peripheral. the slow one is plain SPI, which every card has to
support, and that is what the module's slot is wired for.

the card boots in sd mode. CMD0 with chip select held low is what switches it
to spi; it stays there until power is cut. the handshake after, figure 7-2 in
the spec:

  74 clocks with CS high     wake it up
  CMD0                       reset, expect 01 = idle
  CMD8 with 1AA              2.00 card, and does it take 3.3 V; it echoes
                             1AA back if yes
  ACMD41, repeated           start up; keep asking until the idle bit clears
  CMD58                      read the OCR; bit 30 says whether addresses are
                             blocks (big cards) or bytes (small ones)

all of it at 100-400 kHz, the spec says so for init - so SPI2 runs at 164 kHz
until the handshake is done and 21 MBit/s after. both switches are in
sd_spi.c; cubemx keeps its 21 MBit/s.

two numbers the simplified spec does not give: how many idle bytes to wait
for a response (8) and how long ACMD41 may keep saying idle (1 s). both
marked as my choice in the source.


## blocks and tokens

the card only moves 512 byte blocks. a read is CMD17, then R1 back, then a
wait, then a start token FE, then 512 bytes, then 2 bytes of CRC. a write is
CMD24, R1, then i send FE, the block and the CRC, and the card answers one
byte: 05 taken, 0B bad CRC, 0D write error. then it holds MISO low while it
programs the block - up to 250 ms by the spec, usually a few ms.

every command carries a 7 bit CRC and every block a 16 bit one. the card can
be told to ignore them and most code does, but i turn checking on (CMD59) and
check every block myself. 21 MBit/s through two rows of pin sockets is
exactly where a bit flips quietly, and without CRC the card would never say.
the polynomials come from the spec and the test checks them against its
worked examples - CMD0 giving 95 is the one everybody knows.


## fatfs

a card is just numbered sectors. a file system is the bookkeeping on top:
which sectors belong to which file, what it is called, how big it is. FatFs
is the standard small implementation of FAT, the one every PC understands, so
the card reads on the mac.

cubemx generates it. i fill in five functions in user_diskio.c - init,
status, read sectors, write sectors, sync - and each one just calls sd_spi.
FatFs does the rest.

long file names started off. 8.3 means eight characters and three of
extension, and SESSIONS.JSON needs four, so i turned LFN on in cubemx -
static working buffer on the BSS, MAX_LFN 32. it costs 3.4 KB of flash and 72
bytes of RAM and it wants a real code page, not plain ASCII; 850 was already
set so nothing else had to change.

f_sync is the one that matters. writing to a file goes into a buffer; f_sync
pushes the buffer and the directory entry to the card. until it runs, a power
cut loses everything since the last one. so it runs after every record write,
every 10 s.


## one record, four sectors

the first version was one line of terse JSON per drive, 512 bytes, keys like
g_brk and km. it worked and i could not read it. so it is now a proper
document, pretty-printed, nested, every key a whole word - the card gets
pulled out and read on a laptop and that matters more than the bytes.

the file is one JSON object:

  512 bytes     prologue: { "sessions": [   then spaces
  2048 bytes    drive 1
  2048 bytes    drive 2
  ...

every block is a fixed 2048 bytes, four sectors, and the object inside uses
about 1750 of them. the last 16 bytes of a block are the separator: a comma
if another block follows, or the closing ] } if it is the last one.

the fixed size is the whole safety story, same as before:

- a block is a whole number of sectors, so the 10 s rewrite touches four
  sectors and a power cut can tear at most this drive's own record
- the file only grows once per drive, at boot, with the car parked; the FAT
  and the directory are never written while driving
- the block count is the file size minus the prologue, over 2048 - no
  scanning

## making it one valid document

a stream of JSON objects one after another is not valid JSON, and jq will not
read it. the trick is that the closing brackets live inside whichever block
is last. when a new drive claims a slot, the block before it has its ] }
rewritten into a comma, and the new block closes the document.

that is two writes and the order matters. i do the comma first. if power dies
between them, the last block ends in a comma and the file is broken - but the
file never grew, so the next boot claims the same slot, writes the comma
again over a comma, and closes the document. it repairs itself. the other
order would leave two closers in the middle and stay broken.

both writes happen at boot with the car parked, so this is not in the way of
anything.

## one drive, one record - and the bug that broke it

records 9 and 10 in the file were the same drive. every peak identical to the
last digit, same start time, and the numbers that grow had grown - that is
one trip written twice, not two trips.

session_log_init claims a slot: reads the file size, takes the next one,
bumps the number. the loop calls it again every 5 s whenever the card is not
mounted. a single failed write unmounts the card, the retry ran init, and
init happily claimed a *second* slot for the same drive.

fixed: the slot is claimed once per power cycle and remembered. a remount
takes the same slot back. one drive is one record again.

## the faults section

every counter a failure would have shown up in now goes in the record:
whether each init returned ok as a word, how many IMU and BME680 reads
failed, bad GPS checksums, ring overruns, panel errors, card CRC errors,
timeouts, rejects, log write errors. plus the longest gap between two IMU
samples, the slowest record write, and the date the GPS sent if a fix ever
came with a date the year check threw out.

dropped console characters came out on 19 sep - in the car it is always the
boot printout, 566, and the block was full. worst case is now 2023 of 2032
bytes, so the next field means taking one out.

the point is that the console shouts all of this once a second at a terminal
that is not attached when the board is in the car. the card keeps it with the
drive it belongs to. counters only, no message text - if something needs
digging into, that is what the ST-Link is for.

## local time

the GPS gives UTC and the firmware keeps UTC everywhere, on purpose - one
clock, no conversions hiding in the middle. UTC only becomes local where a
human reads it: the clock on the panel, and start_local in the record.

one constant, VEHICLE_UTC_OFFSET_MIN, currently 180 for summer. i edit it
twice a year. no DST rule in firmware - that is a calendar to maintain for a
number i can change in one line.

adding three hours can cross midnight, a month end and a new year, so the
record's conversion carries the date properly, leap years included. the panel
only shows hours and minutes so it just wraps at 1440.

## it works

eight boots of the old format: eight records, 4096 bytes on the mac, every
line exactly 512 including its newline, every one of those eight ended by
pulling the usb cable with no warning, and not one record was torn. the new
format has not been on the board yet.


## the loop

still a plain superloop, no rtos. one pass:

  gps_poll        every pass, the ring holds ~2 s
  imu tick        on the INT1 edge, ~105 a second: read, map, attitude, trip
  new fix         once per RMC: trip_update_gps, and the push estimate goes
                  back into the attitude filter
  env             trigger, come back 143 ms later, read - every 3 s
  screen          20 Hz: gather the numbers, mark what changed
  one band        at most one gfx_flush_band per pass
  log             every 10 s: rewrite the record, sync
  console         every second, only if something is listening

the budget is the 9.5 ms between IMU samples. a pass with a band in it is
about 3 ms. the pass time comes from the DWT cycle counter - a free running
counter at 168 MHz inside the core - and the worst pass each second is on the
console as pass max.

i expected the card to be the thing that breaks the budget - up to 250 ms by
the spec while it programs a block. measured: 12 ms worst over 50 writes,
card busy 6 ms. that is one lost IMU sample, not twenty-five. the only 233 ms
i ever saw was the first write into a fresh file, which is the card
allocating, once.

what actually breaks the budget is the console. about once a minute a pass
runs 90 ms instead of 3, always the pass carrying the console print, and that
second runs 22,000 fewer passes. not the card - writes did not move across
it. usb_serial_flush waits up to 10 ms a line for the host to take the
previous one, a block is nine lines, and drops did not increment, so no line
hit the full timeout; the mac just stopped reading for a moment. costs about
8 IMU samples.

and it cannot happen in the car. with no host enumerated usb_serial_ready is
false and the console drops instantly instead of waiting. so the worst stall
on the bench is the one stall that does not exist where it matters.


## the env read in two halves

env_read used to sit in a busy wait for 143 ms. now env_start kicks the
measurement off and says how long it takes, and env_read is called after
that; called early, the driver says no new data and the loop tries again next
pass. same sensor, same settings, nothing blocks. it is also the shape BSEC
wants later - it says when to trigger and when to read.


## the console is its own file

every printf moved into console.c. main.c prints one warning and nothing
else. the per-second block is the evidence for every step at once: INT1
edges against reads, dtmax, the gps ladder, the panel byte count, the card
write times. gated on usb_serial_ready, so in the car it is one call that
returns.


## i2c dies after a debugger reset

both sensors came back ERR_BUS one boot, out of nowhere, nothing touched.

it is the bus stuck, not the chips. J1 has no reset pin, so every clion
attach is a software reset of the mcu alone. if that lands while the LSM6DSO
is clocking a byte out, the chip goes on holding SDA low waiting for clocks
that never come, and everything after fails - both devices, because one held
line kills the whole bus.

a full power cycle fixes it; the slaves lose power and let go. the debugger
showed it afterwards too: GPIOB IDR low byte FF, both lines idle high, I2C1
SR2 busy clear.

the proper fix is nine clock pulses on SCL with SDA released, bit-banged
before I2C1 is initialised. not written yet. for now: unplug everything, five
seconds, plug back in - and do it before a flash rather than after a puzzling
failure.


## what the bench cannot test

two records in the file say dur_s 0 with null weather. those are the two
i2c-locked boots. trip has no clock of its own; the duration adds up from the
dt that imu_tick hands trip_update_motion, so with the IMU dead the session
clock never moves. the record is honest - it measures how long the IMU was
alive, not how long the board was powered.

and every g peak in all eight records is 0.00, including the runs where i
slid the board about and watched the dot move. the peaks are gated on speed
>= 5 km/h and the lean angles on the distance gate, and speed comes from the
gps, which never left zero indoors. so trip.c's peaks, lean angles, distance
and moving time have 38 host checks and no board evidence at all. nothing to
fix - it needs a real drive with a fix - but worth knowing which numbers have
actually been seen.


## the first drive

19 sep, session 5. 18 min, 1.6 km, 65 km/h top, car charger only. the first
ten minutes the board sat loose, then i taped it by the vents 25 deg nose-up.
hard right turn read -0.39 g, braking -0.34, signs right. no references, so
nothing is checked yet - the ring on the screen and the record are the same
number, them agreeing proves nothing.

what it found:

- coasting ran before the first fix. only a valid fix resets the time since
  the last one, so 2 s after boot trip.c thought the gps had dropped and
  started guessing speed from the accelerometer. picking the board up was
  enough for fake speed, distance and lean. fixed: no coasting until a real
  gps speed has turned up
- the lean numbers (67 deg, -45 deg) are that plus the 25 deg mount. lean
  only means something with the board flat
- gps bytes lost on every boot. the uart started before the panel init and
  the card mount, which take longer than the 256 byte ring lasts at 9600
  baud (~265 ms). moved gps_init to the end of init
- 265 bytes lost in session 5, ~250 of them mid-drive - the loop stopped for
  half a second somewhere. the record could not say where, so now it keeps
  the longest imu gap, the slowest card write and the BME680 read errors
- the gps had a fix for minutes and never gave a date that passed the year
  check, so no start time. the check was right to refuse it, but nothing kept
  what the module sent. now the record does
- -0.0 on the ring. minus zero is a real float value and printf shows the
  sign. fabsf instead of a minus

altitude is still wrong on a fresh fix: -65 m, +30 m, -70 m on three first
fixes. it is not the cabin pressure - the barometer only adds changes, the
absolute height comes from the first gps reading. that seed error walks out
over a few minutes and gets counted as climb or descent: 92 m of descent in
session 5 when the pressure only moved 1.3 hPa, about 11 m.

humidity went to 60% with fresh-air AC, 44% on recirculation. that is colder
air, not wetter: 60% at 19 C is a dew point of ~11 C, 44% at 30 C is ~16 C.
RH only compares at the same temperature.


## back on the bench, 28 sep - 3 oct

flashed the shake-down fixes, three sessions. 6 outside with clear sky for 38
min, 7 and 8 inside. all the fixes held: ring overruns 0 on every boot, no
speed or lean on a still board, 0.0 on the gauge, start time there.

climb was the new problem. 170 m in session 6, 234 m in session 7, board never
moved. after a cold start the gps height starts way off - about -280 and -170
m here - and walks to the real one over minutes. the fused altitude followed
it and every metre of that walk got counted as climb. the pressure moved 0.4
hPa at most.

fix:
- climb and descent from the barometer only. it only knows changes, but
  changes is all climb needs. a weather front is ~8 m an hour, nothing next to
  234 m
- the start is the average of the first 10 fixes, ~30 s. averaged as gps minus
  barometer, so driving uphill while it averages does not drag it down
- a settled flag - gps and the fused number agree for 60 s. the altitude stays
  grey on the panel until then

session 6 was its own thing: clear sky, fix for 38 min, still ~180 m low and
the position ~180 m off. the barometer said both sessions were at the same
height, so it was the gps, not the board moving. most likely the lifted
antenna. session 7 ended up within ~20 m.

pressure read 1013 while a weather app and my watch said 1023. the app gives
sea level pressure; the BME680 gives the real pressure where it sits. at ~72 m
that is ~8.6 hPa less, so 1022 vs 1023 - fine. the panel keeps the local one.

the card: slowest write 245 ms, longest loop gap 253 ms. the sd spec allows
250 ms per block, so the card is fine - but the 256 byte gps ring only lasts
~265 ms at 9600 baud. 20 ms of margin. ring is 2048 bytes now, ~2.1 s - a
record is 5 blocks with the directory, 5 x 250 = 1.25 s worst case.

bt1: the ml2032 charged to 3.3 V on a bench supply, 0 V in the holder. dead.
no cell in for now, so every start is cold. next a new ml2032; CR2032 is the
fallback. the circuit is 3.3V -> D3 -> V_BCKP -> R6 -> cell. D3 is the charge
path; R6 is the cell's only way to V_BCKP. so for a CR2032 D3 comes out, not
R6 - pull R6 and the backup is gone. with D3 out the cell feeds 8 uA through
1 k, 8 mV lost. and a clock before the fix does not mean a cell is in - the
module gets the time from the first satellite it hears.

printed the bottom tray and re-did the flat calibration in it - the old one was
the board sitting on its solder tails. numbers and the car steps in
vehicle_info.md.


## left for later

INT1 is done - the loop is paced by the sensor now, 105 edges a second and no
sample missed. the decimation and the aliasing from the ODR section are gone.

the BME680 module is done for temperature, pressure, humidity and raw gas
resistance. the air quality numbers still need BSEC.

the display, the SD card and the main loop are done on the board. BSEC is
still untouched.

bring-up so far: TIM2 blinking an LED proved power, flashing, the 168 MHz clock
tree and the timers. then the usb console - the mac sees a serial port, printf
works, floats print. then the i2c bring-up: both sensors answer, both init calls
return OK, readings stream once a second. then the bench calibration: the axis
map is measured and both constants are in vehicle_axes.h. then the gps: the
firmware side passed on the first flash, the antenna did not, and it fixes now
with the chip lifted off the board. then step 4: pitch and roll live from the
fused filter, matching a ruler to 0.04 degrees, and tau confirmed at 2 s. then
step 5: the sensor's own data-ready line drives the loop, 105 edges a second,
zero missed. then steps 6 to 8, written blind against the datasheets and all
three passed in one session - the panel, the card, and the whole loop running
together for eight minutes with nothing missed.

next: a new ml2032 (or D3 out and a CR2032), a drive with references (trip meter, speedo, a
roundabout, a signed grade, known altitudes), then BSEC, tuning, the rtos, the
top case. and the gps needs open sky: time to first fix, altitude, and whether
the 3 s clock offset and the age walk survive a proper fix.

the lateral fix is flashed - every sample now, not once a second. the rest
reading in the header is the car one now, 43 deg nose up on the dash, taken
both ways round on a slope (vehicle_info.md). the flat one is in a comment
above it. the gyro offset does not care how the board sits, so the tray value
stays.
