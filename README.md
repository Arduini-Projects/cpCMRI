CMRI Protocol handling for Node implementations

Code status:  Compiles, basic mock testing performed, NOT TESTED on actual hardware


This library implements a CMRI serial protocol node implemented in an Arduino style system board,
with optional I2C expanders used to increase the number of I/O points availble.

it depends on the I2Cexpander library at https://github.com/plocher/I2Cexpander

 The heavy lifting is done behind the scenes with the following class libraries:
   * cpCMRI:  Implements all the protocol handling fiddly bits to work with CMRInet, also abstracts the reading and writing of bits to ports/pins and devices
   * I2Cexpander:  abstracts the initialization, reading and writing details of 8- and 16-bit I2C expanders

