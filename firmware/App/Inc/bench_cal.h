//
// bench calibration - public interface
//
// - the step 2 averaging block, moved out of main.c: 500 still samples,
//   printed as the two constants ready to paste into vehicle_axes.h
// - off by default; set BENCH_CAL to 1 to bring it back, and bench_cal_tick
//   then runs the average every 20 s. with it off the tick compiles to nothing
// - board flat on something rigid, warm, untouched - see docs/notes.md
//

#ifndef BENCH_CAL_H
#define BENCH_CAL_H

#include <stdbool.h>

#ifndef BENCH_CAL
#define BENCH_CAL 0
#endif

// call from the main loop; does nothing unless BENCH_CAL is 1 and 20 s have
// passed since the last run. imu_ok gates it on the sensor having come up
void bench_cal_tick(bool imu_ok);

#endif // BENCH_CAL_H
