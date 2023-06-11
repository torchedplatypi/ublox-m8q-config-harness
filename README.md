# What is this

This is a harness to configure a ublox NEO-M8Q-01A chip, but is likely compatible with a wide variety of ublox receivers.

It is loaded with a set of defaults that I found useful for a real time automotive use case.

Currently it covers the following configurations:
* serial baud rate
* GPS measurement rate (i.e. time b/w location reports)
* Dynamic mode (ublox provides a list of useful presets, see ublox documentation)
* Disabling messages by ID/prefix (keeps serial bus clear of everything except the location data I wanted)

It is also hopefully close to copy/paste extensible for any other chip configurations you may want to make,
using the ublox proprietary UBX protocol.

The general principle is to create and fill a buffer of chars per a given configuration message outlined
in the ublox documentation. I then send it over I2C to the receiver, and look for an ACK message in the format
also outlined in the ublox manual.

I am publishing these mostly because working examples make the ublox docs WAY easier to understand.

## ubxtool

I also *highly* recommend the `ubxtool` utility that is included in the `gpsd`.

My code runs on raspberry pi, for which I couldn't find a prepackaged `gpsd`, so I compiled from source.
`gpsd` documentation is fairly comprehensive.
You will also need a python environment with `PySerial` installed to use ubxtool, once you have a working
`gpsd` installation.

This utility makes it *far* easier to look at the actual contents of what any given config message buffer should
look like. Using verbosity flag `-v 3` will print out the actual buffer contents in hex, which you can then
use as a direct copy paste to build the message buffer in C code.
