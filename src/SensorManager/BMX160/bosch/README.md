# Bosch BMI160/BMM150 Sensor APIs

These files are vendored without functional changes from Bosch Sensortec's
official SensorAPI repositories:

- `bmi160.c`, `bmi160.h`, `bmi160_defs.h`:
  <https://github.com/boschsensortec/BMI160_SensorAPI>, commit
  `252ac2859ac1d2010915b7a7cfcbf501d7adfb40`
- `bmm150.c`, `bmm150.h`, `bmm150_defs.h`:
  <https://github.com/boschsensortec/BMM150_SensorAPI>, commit
  `0dce0617873cda1f6d51f6b7b961fdc2641e0c7c`

Both APIs are distributed under the BSD 3-Clause license. The corresponding
license texts are stored in `BMI160_LICENSE` and `BMM150_LICENSE`.

Project-specific integration, FIFO configuration, bus locking, conversion and
instrumentation live in the adjacent `BMX160_Bosch.cpp` wrapper.
