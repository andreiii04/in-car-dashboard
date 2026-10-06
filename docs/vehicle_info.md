# vehicle info

everything the firmware knows about the car. it all lives in one file,
App/Inc/vehicle_axes.h, so moving to a different car is that file and a
rebuild. nothing else in App/ has a car-specific number in it.

three groups, kept apart because they get filled in at different times and in
different ways:

  axis map       which sensor axis points which way in the car
  calibration    resting gravity and the gyro zero offset, measured on the board
  car specs      wheelbase, track, cog height - handbook or tape measure


## the axis map

the firmware works in car axes: X forward, Y left, Z up. that is ISO 8855, what
the car industry uses, and attitude.c / trip.c / elevation.c all assume it. the
LSM6DSO has its own X/Y/Z, fixed by how the chip sits on the board, so the map
is the swap-and-flip between the two - no maths, just which sensor number goes
where and whether it needs a minus.

how i filled it in on the bench, board flat on the table:

  1. picked a forward - the edge opposite the USB-C connector
  2. flat board reads -0.990 on sensor Z. an accelerometer reads +1 g on the
     axis pointing UP, so sensor +Z points at the table -> car Z = -sensor Z
  3. lifted the front edge. sensor X went to +0.71, so sensor +X points
     forward -> car X = +sensor X
  4. lowered the right edge. sensor Y went to -0.61, so sensor +Y points
     right. car Y is LEFT -> car Y = -sensor Y

so:

  car X = + sensor X
  car Y = - sensor Y
  car Z = - sensor Z


## why two of the three flip

the IMU is soldered to the underside of the board - a 180 degree turn about the
fore-aft axis. forward survives that turn; up ends up pointing at the table and
left ends up pointing right. one motion, two axes reversed.

flipping only Z would be a mirror, and no rigid rotation makes a mirror. the
check is the determinant: (+1)(-1)(-1) = +1 means rotation. -1 would mean i got
it wrong.

none of this is about how the board gets mounted in the car. it is only about
which side of the PCB the chip sits on.


## checking it

ran the three real tilt readings back through vehicle_map() on the mac:

  flat              car +0.000 +0.005 +0.990    pitch  +0.0   roll  +0.3
  nose up           car +0.710 +0.050 +0.680    pitch +46.2   roll  +4.2
  right side down   car -0.027 +0.610 +0.780    pitch  -1.6   roll +38.0

attitude.h says pitch + is nose up, roll + is right side down. both come out
right. that is the real test - drop the Y minus and roll reads -38, and the
display leans the wrong way while everything still looks plausible.

the map stayed the same once the board went into the tray and onto the dash.
it can only express quarter turns and flips, so the mounting angle is not in
here at all - that lives in the rest reading below.


## the calibration

two numbers, both measured with the board still, both stored after the axis map
has been applied.

VEHICLE_REST_ACCEL_X/Y/Z is the accelerometer at rest. attitude_set_mount turns
it into the rotation that cancels the mount tilt. three readings, not a
nine-number matrix, because three readings are something i can measure. do it
once and hardcode it - boot happens when the ignition turns on, which might be
on a hill, and that hill would get baked in as "level" for the whole drive.

VEHICLE_GYRO_BIAS_X/Y/Z is what the gyro reads standing still, deg/s. the
datasheet allows +/-1 dps. averaging 500 samples takes most of it out; what is
left is temperature drift, about 0.010 dps per degree C.


## what i measured on the bench

board flat on a wooden table, no case, warm. 500 samples a run, four runs over
20 minutes:

  rest accel   -0.00002   +0.00530   +0.98941  g
  gyro bias    +0.6096    -0.4463    +0.2017   dps

accel agreed to 0.0005 g between runs. gyro Y and Z moved 0.002 and 0.006 dps.
all solid.

gyro X never settled - +0.42 cold, +0.61 at 21 minutes, still climbing. so it
carries about 0.2 dps of doubt, which at tau 2 s is 0.4 degrees of standing
error. the car's own 40 degree temperature swing causes 0.8 degrees regardless,
so chasing it further on a table buys nothing. stopped there.

turned out the rest reading was the board sitting on its THT solder tails, not
lying parallel to the table. next section.


## re-measured in the tray, 3 oct

printed the bottom tray. the board drops in snug, no screws, USB-C in. on the
same table it read pitch +0.96, roll -1.28 - with the old numbers that should
have been 0, 0.

two causes:

- the tails. accel alone said +0.84 / -0.65 relative to the old rest
- the gyro bias. X was 0.61, now 0.45; Y -0.45, now -0.51. a wrong bias
  leaves bias x tau as a standing error: 0.16 x 2 s = 0.33 deg of roll,
  0.06 x 2 = 0.12 of pitch. exactly what was left over

took it again in the tray, ~18 min warm, three runs of 500:

  rest accel   +0.01448   -0.01117   +0.99091  g
  gyro bias    +0.4460    -0.5069    +0.1804   dps

runs agreed to 0.0001 g and 0.004 dps. both went in.

this is the flat set. it is also written in a comment above the defines in
vehicle_axes.h - paste it back when the board comes out of the car and has to
read flat on a table again. on the dash without it, the board reads ~43 deg
nose down.


## two things that bit

rigid surface. the same board reads 0.87 degrees of tilt on a mouse pad and
0.43 on a wooden table - the foam squashes under the THT solder tails.
calibrate on something hard.

warm-up is longer than i thought. i guessed five minutes; it was still drifting
at twenty. the BME680 temperature is no guide either - it sits in its own
self-heated pocket and saturates in the first minute, while the IMU die warms on
a slower curve. the LSM6DSO has its own temperature register if this ever needs
settling properly.


## what is bench-only and what is real

the bench rest reading was only ever interim. flat on a table there is no mount
tilt, so it mostly measures the accelerometer's own zero error - the datasheet
allows 20 mg, worth 1.15 degrees of permanent tilt on its own. so the absolute
tilt it implies is not trustworthy; the difference between two surfaces is,
because the zero error is the same in both and cancels.

so it got taken again in the car, board in the tray on the dash - see the car
set below. 43 degrees nose up reads 0.683 / 0.032 / 0.723 instead of
about 0 / 0 / 1. big change, not a fault. the axis map did not move with it.

the gyro offset does not care how the board sits, so the bench value is the
real one. it is stored after the map, so if the map changes later the same three
numbers just re-map - nothing to re-measure.

the pitch and roll test later checked both against something that is not the
accelerometer - a ruler and asin(h/L). pitch came out 12.22 against 12.18
expected, roll 8.09 against 7.80, and the reference itself carries 0.30. so the
map and both constants are right as a set, not just individually plausible. that test only measures tilt
relative to the rest pose, which is why it passed with the board on its tails.


## calibrating in the car

1. board in the tray, mounted, front edge (opposite USB-C) facing forward -
   the axis map assumes that; a tilt is fine, a turn is not
2. car on level ground, engine off, nobody moving in it
3. BENCH_CAL 1 in bench_cal.h, flash, laptop on the usb console
4. wait 5 min, take the last 2-3 calibration blocks
5. average the three VEHICLE_REST_ACCEL numbers into vehicle_axes.h. ignore
   the gyro numbers it prints - 5 min is not warm, the bench bias stays
6. BENCH_CAL back to 0, flash

5 min is fine for the accel - the bench runs, cold to warm, agreed to
0.0005 g, about 0.03 deg. the gyro is what needs the long soak.

ground not level: do it once, turn the car round on the same spot, do it
again, average the two. the slope flips sign, the mount does not.

the cal build stops the loop ~5 s every 20 s while it averages - screen
freezes, gps bytes get lost. normal for that build, ignore its record.


## the car set, 4 oct

new dash position, front edge forward. parked on a slight slope, so did both
ways round on the same spot:

  front up the slope   +0.71401  +0.02540  +0.69531   pitch 45.74  roll 2.09
  turned round         +0.65235  +0.03786  +0.75008   pitch 40.98  roll 2.89
  average              +0.68318  +0.03163  +0.72270   pitch 43.36  roll 2.51

2 runs one way, 3 the other, all within 0.0004 g. the slope was 2.4 deg, not
the 10 i guessed - half the gap between the two pitches. mount is 43.4 deg
nose up, 2.5 deg roll.

gyro X read 0.13-0.26 in the car against 0.446 on the bench. kept the bench
one - X never settles, it moves with temperature. ~0.4 deg of standing roll
error worst case.


## the car specs

one car: an audi a4 b6 avant 1.8T 190hp quattro, 2004, 6-speed manual, built
nov 2002 to dec 2004.

straight off its own data sheet:

  track              1528 mm front, 1526 mm rear
  wheelbase          2650 mm
  turning circle     11.1 m
  length             4548 mm
  width              1772 mm
  height             1428 mm
  kerb weight        1530 kg
  gross weight       2080 kg

i average the two tracks to 1.527 m - they are 2 mm apart. turning circle is the
diameter, so the radius is half of it, 5.55 m.

on mass: 1530 kg is the empty car, 1650 is with a driver and a passenger, and
that is what actually rolls, so 1650 goes in the header. nothing calculates with
it. the two people raise the cog by about 5 mm, under 1% on the rollover limit -
smaller than the error on the cog estimate itself, so i leave the estimate alone.

two numbers no handbook prints:

  cog height          0.53 m   estimate; typical for a low passenger car, well
                               below an SUV's 0.65
  cog to rear axle    1.54 m   estimate; the car carries roughly 58% on the front
                               axle, so the cog sits at that fraction of the
                               wheelbase measured from the rear: 0.58 x 2.65

used in the maths:

  VEHICLE_TRACK_M         1.527 m
  VEHICLE_COG_HEIGHT_M    0.53 m
  VEHICLE_COG_TO_REAR_M   1.54 m
  VEHICLE_TURN_RADIUS_M   5.55 m
  VEHICLE_WHEELBASE_M     2.65 m

logged but not calculated with:

  VEHICLE_MASS_KG         1650 kg
  VEHICLE_LENGTH_M        4.548 m
  VEHICLE_WIDTH_M         1.772 m
  VEHICLE_HEIGHT_M        1.428 m


## how much sideways before it tips

  rollover limit = track / (2 x cog height)
                 = 1.527 / (2 x 0.53)
                 = 1.44 g

called the static stability factor. a normal car lands near 1.4, a tall SUV near
1.1 - lower tips sooner. an a4 avant is a low estate, so 1.44 is right.

standing-still figure. it ignores the suspension leaning and assumes the tyres
grip hard enough to get there, which on tarmac they usually do not. a reference
to compare cornering against, not a prediction.

trip.c uses it for lateral_load_worst - hardest cornering of the trip as a
fraction of the limit. 1.0 would be right at it.


## the side slope it would tip over on

  critical roll = atan(1.44) = 55 degrees

same geometry written as an angle: the sideways slope where the cog passes over
the downhill wheels, standing still. trip.c uses it for roll_load_worst. this is
the one that means something off-road.


## the climb it would tip backwards on

  critical pitch = atan(cog to rear / cog height)
                 = atan(1.54 / 0.53)
                 = 71 degrees

vehicle_critical_pitch_deg() works it out. nothing uses it yet - it belongs with
the off-road display.


## the turn rate sanity check

before the gyro's turn rate gets used for sideways acceleration it is clamped to
what the car can physically do. two limits, the tighter one wins:

  steering limit   speed / turning radius       slow: the wheels only turn so far
  tipping limit    rollover g x 9.81 / speed    fast: harder than this is a roof

   10 km/h    steering  28.7 dps    tipping  291.4 dps    ->   28.7
   50 km/h    steering 143.4 dps    tipping   58.3 dps    ->   58.3
  100 km/h    steering 286.8 dps    tipping   29.1 dps    ->   29.1

at walking pace the steering is what stops you; from about 32 km/h up it is the
tipping limit, tightening as speed rises. that is right - the same turn rate is
far more violent at speed.

a glitch filter, not a model of the car. one corrupt gyro sample would otherwise
shove a huge false correction into the attitude filter.


## the lever arm, and why i left it alone

the sensor is not at the centre of gravity - roughly 1.5 m forward of it and
0.8 m above, on the dash. a sensor away from the centre of rotation feels extra
acceleration the cog never does:

  spinning        yaw rate squared x distance
                  30 dps at 1.5 m  ->  0.04 g

  changing spin   how fast the rate changes x distance
                  100 dps/s of pitching at 1.5 m  ->  0.27 g

the second is big enough to matter off-road. not corrected: working out how fast
the rate changes means differentiating the gyro, which makes the noise much
worse, and doing it properly needs filtering tuned against real drives that do
not exist yet. so pitching over rough ground reads slightly stronger than the
cog really feels - a known thing, not a mystery later.
