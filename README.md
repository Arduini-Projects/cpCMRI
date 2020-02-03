CMRI Protocol handling for Node implementations

Code status:  Tested on original cpNode and on ProMini with JMRI

This library implements a CMRI serial protocol node implemented in an Arduino style system board,
with optional I2C expanders used to increase the number of I/O points availble.

it depends on the I2Cexpander library at https://github.com/plocher/I2Cexpander 
and the elapsedMillis library at https://github.com/pfeerick/elapsedMillis/wiki

 The heavy lifting is done behind the scenes with the following class libraries:
   * cpCMRI:  Implements all the protocol handling fiddly bits to work with CMRInet, also abstracts the reading and writing of bits to ports/pins and devices
   * I2Cexpander:  abstracts the initialization, reading and writing details of 8- and 16-bit I2C expanders

Differences from other codebases, including Chuck's MRCS cpNode kernel1.5

 *  Chuck's MRCS Kernel treated the onboard BBLeo pins as BOTH inputs and Outputs on cards 0 and 1, which meant that all IOX bits started on card 2.  Chuck's code was compiled specially for whatever I/O divisions the user desired, and ignored "outputs to input bits" as well as "inputs from output bits" based on compile time #defines.
     * cpCMRI doesn't do this - card "0" is the first 8-bits of inputs/outputs defined in the configuration, irregardless of which device(s) they are serviced by.
     * This means that existing JMRI configurations probably won't work as-is with this new firmware, as the card numbers may change.
 * cpCMRI is slightly larger in both codespace and in RAM memory usage.
 * cpCMRI has been tested on both the BBLeo and a DFRobot ProMini on a cpNode baseboard, and it is expected to "just work" on Unos, Megas, ESP8266 (Wemos D1 Mini) and ESP32 (dev kit board) as well.
  * cpCMRI lets you intermix input and output pin directions within the entire bit and port space of the devices you are using, depending on the capabilities of the hardware itself.  This differs from the SUSIC and SMINI cards that expose inputs and outputs as contiguous 8-bit "cards".
     * CMRI is mostly a BYTE oriented protocol, and most CMRI software and hardware operates on 8-bit quantities.  Most examples talk about input and output card numbers with an assumption that each card is 8-bits.   This relationship is easy to understand if you define inputs and outputs in groups of 8.
     * You *could*  create a node with 3x inputs and 13x outputs connected to an IOX-16 set up as 9 inputs and 7 outputs; things will still work. The downside is that there will be bits in those bytes that can't be used (input bits 13, 14,15 & 16,  output bits 21,22, 23 & 24).   As long as you are aware of this, and can perform the mental gynmnastics to make it work with your VB code or JMRI,  no harm, no foul...
  * cpCMRI uses an I2C expander library that supports many more I2C expander devices, and all of them can be used as part of a CMRI Node.
  * cpCMRI supports the use of sketch variables (bool and byte) to be the source and destination of CMRInet RX and TX packet content.  This allows the Node designer to trigger animations or provide computed results to the Host computer.

For the MRCS BBLeo cpNode, the logical to physical mapping is:
<table>
	<tr><th>BBLeo pin 	</th><th>MRCS Header.pin 		</th><th>CMRI byte.bit	</th></td>
	<tr><td>D4          		</td><td>J1.1                 		</td><td>J1.1 </td></tr>
	<tr><td>D5          		</td><td>J1.2                 		</td><td>J1.2 </td></tr>
	<tr><td>D6          		</td><td>J1.3                 		</td><td>J1.3 </td></tr>
	<tr><td>D7          		</td><td>J1.4                 		</td><td>J1.4 </td></tr>
	<tr><td>D8          		</td><td>J1.5                			</td><td>J 1.5 </td></tr>
	<tr><td>D9          		</td><td>J1.6                			</td><td>J 1.6 </td></tr>
	<tr><td>D10         		</td><td>J1.7                 		</td><td>J1.7 </td></tr>
	<tr><td>D11         		</td><td>J1.8                 		</td><td>J1.8 </td></tr>
	<tr><td>D12         		</td><td>J2.1                			</td><td>J 2.1 </td></tr>
	<tr><td>D13         		</td><td>J2.2                			</td><td>J 2.2 </td></tr>
	<tr><td>A0          		</td><td>J2.3                			</td><td>J 2.3 </td></tr>
	<tr><td>A1          		</td><td>J2.4                 		</td><td>J2.4 </td></tr>
	<tr><td>A2          		</td><td>J2.5                 		</td><td>J2.5 </td></tr>
	<tr><td>A3          		</td><td>J2.6                 		</td><td>J2.6 </td></tr>
	<tr><td> A4          		</td><td>J2.7                			</td><td>J 2.7 </td></tr>
	<tr><td>A5          		</td><td>J2.8                 		</td><td>J2.8 </td></tr>
</table>

A sketch consists of two logical parts:

  1. per-Node customization (Node address, I/O configuration, etc)
  2. boilerplate code that doesn't need to be changed.

The per-Node personalization is done by editing the compile-time initialization of a cpIOMap data structure (aka table).  Here's the configuration for a generic cpNode with a single IOX-16 expansion card, configured as 8-in / 24-out.
``` C
#define CMRINET_NODE_ID      1
#define CMRINET_SPEED      19200  // make sure this matches the CMRInet speed set in JMRI

cpIOMap node_configuration[] = {
    // device type     port/I2Caddr  I/O        '+' ' ' = input pullup or HiZ, '0' / '1' = initial value output
  { I2Cexpander::BUILTIN,       4,   "O",                "1"},
  { I2Cexpander::BUILTIN,       5,   "O",                "1"},
  { I2Cexpander::BUILTIN,       6,   "O",                "1"},
  { I2Cexpander::BUILTIN,       7,   "O",                "1"},
  { I2Cexpander::BUILTIN,       8,   "O",                "1"},
  { I2Cexpander::BUILTIN,       9,   "O",                "1"},
  { I2Cexpander::BUILTIN,      10,   "O",                "1"},
  { I2Cexpander::BUILTIN,      11,   "O",                "1"},

  { I2Cexpander::BUILTIN,      12,   "O",                "1"},
  { I2Cexpander::BUILTIN,      13,   "O",                "1"},
  { I2Cexpander::BUILTIN,      A0,   "O",                "1"},
  { I2Cexpander::BUILTIN,      A1,   "O",                "1"},
  { I2Cexpander::BUILTIN,      A2,   "O",                "1"},
  { I2Cexpander::BUILTIN,      A3,   "O",                "1"},
  { I2Cexpander::BUILTIN,      A4,   "O",                "1"},
  { I2Cexpander::BUILTIN,      A5,   "O",                "1"},

  // Add any I2C expanders here
  { I2Cexpander::MCP23017,     0x20, "OOOOOOOOIIIIIIII", "11111111++++++++"},
  _END_OF_IOMAP_LIST_
};
```


The above table is used to define and initialize the physically connected pins/devices; it is also used to automatically pack and unpack input and output bits in response to POLL and TRANSMIT packets.

  1. The first column defines the type of device, either a BUILTIN pin on the processor,  or an I2C expander of a particular type, as defined in the I2Cexpander library.    Additional I2C expanders can easily be added to that library as needed.    By default, it supports the 16-bit MCP23017 chip used in the MRCS IOX16 and IOX32 boards,    9555's and MCP731x's, as well as 8-bit expanders in the 8574 & 8574A families.    
 
  2. The 2nd column is either the pin name, or the expander device type.

  3.  Column 3 is a string that defines the direction of the pin(s) exposed by that device.    This is an ordered list of 'I' and 'O' characters that specify an input or output bit.   The length of this string MUST be the same as the number of bits managed by that device,   either 1 (for MCU BUILTIN pins) or 8 or 16 (for I2C expanders.    Upper case ('I', 'O') imply normal polarity, while lower case ('i','o') cause the underlying  code to invert the polarity on pin read/write.
 
  4. The last column defines special handling - do inputs need pullups (if supported?), should outputs be initialized?
      *   For Inputs, "+" means turn on pullups for this bit/pin, ' ' (space) means no pullups
      *   For Outputs, '1' and '0 are used as the initial values written to the pin

The order of bits/bytes in the table are directly tied to the CMRI IB and OB contents found in a TX or RX packet.

There are example sketches in this library for most of the configurations that were supported by Chuck's code.
Start with one of them, edit it to match your layout needs, and save it with a name that will help you associate it with the particular CMRI Node it will be loaded on.

You don't need to modify the boilerplate code, and probably shouldn't - but there is no real harm in playing with it.  Take a look at the cpCMRI Example: cpCMRI-animation; it implements an animated warehouse that can be controlled by CMRI, with all of the actual I/O to the layout being controlled by custom code in the sketch;  the CMRInet I/O is simply used as remote commands and animation state feedback...


 
