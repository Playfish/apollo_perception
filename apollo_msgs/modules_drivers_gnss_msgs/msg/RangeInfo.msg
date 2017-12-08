Constellation constellation  #[default = GPS]

int32 prn   # Satellite PRN number.

  # Carrier frequency of the signal used in this range measurement. Note that
  # GLONASS uses FDMA, so different satellites transmit signals on different
  # frequencies.
int32 carrier_frequency   # In Hz. protobuf is fixed32

float64 pseudorange #[default = nan] In meters.

float32 pseudorange_std_dev  #[default = nan]   In meters.

float64 carrier_phase  #[default = nan] In cycles.

float32 carrier_phase_std_dev  #[default = nan] In cycles.

float32 doppler  #[default = nan] In Hz.

float32 doppler_std_dev  #[default = nan] In Hz.

  # Carrier-to-noise ratio (signal # strength) in dB-Hz.
int32 cno  #[default = -1]

  # Carrier phase lock time in seconds.
int32 lock_time  #[default = -1]
