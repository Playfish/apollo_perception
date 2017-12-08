#This message defines one band observation of a certain satellite
#observation on a certain frequency band

GnssBandID band_id #[defaultBAND_UNKNOWN]

float64 frequency_value
PseudoType pseudo_type #[defaultCODE_UNKNOWN]

#unit in meter
float64 pseudo_range

#unit in cycle
float64 carrier_phase

#Indicator of losing tracking of the signal
uint32 loss_lock_index

#unit in /s
float64 doppler

#Signal strength: signal noise ratio or carrier noise ratio
float32 snr
