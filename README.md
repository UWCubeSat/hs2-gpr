# hs2-gpr
Standalone driver code for the Ground Penetrating Radar Instrument for HS2.

## Current Status

The radar hardware has been tested and functions as expected (see coaxial cable test jupyter notebook in this repo). We are currently bringing up the DDS for end-to-end SAR simulations.

## TODO

1. Generate a hann windowed chirp and capture it on a DSO
2. Feed the hann windowed chirp into the Gerekos simulator and calculate a rangeline
3. Synthesize that rangeline on the DDS and capture it on a DSO - show that it matches the Gerekos rangeline
4. Feed that rangeline into the radar, and save the digital de-chirped data. Show that it matches the "ideal" de-chirped signal
5. Synthesize several rangelines, and combine them to form a radargram