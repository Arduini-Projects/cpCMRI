/**
 * cpNode - Control Point CMRI Node
 * =================================
 */

#include <cpCMRI.h>
#include <I2Cexpander.h>

//==============================================
//====   BEGIN CONFIGURATION PARAMETERS     ====
//==============================================

#define CMRI_NODE_ID               1    // can be [0..64]  change this - must be unique for each node...
#define CMRI_SPEED             19200    // make sure this matches the speed set in JMRI
#define CMRI_NODE_DESCRIPTION  "BBLeo BASE_NODE_12OUT4IN + I2C"

#define DEBUG 1                         // enable Serial printing of debug messages
// #define DEBUG 0                      // Turn off if you use Serial for CMRInet!

ioMap *setupMap() {
    // Declare the I2C expanders you will be using, providing the device type and I2C address
    I2Cexpander *mcp23017_20 = new I2Cexpander(I2Cexpander::MCP23017, 0x20);

    // There are 2x ways to interpret CMRI Body content (the IB and OB arrays):
    // HOST_CENTRIC: keep the IB and OB completely independent from each other,
    //               with disjoint pack and unpack mechanisms,
    //               just like the BASIC code and JLC hardware currently does.
    //               From the HOST's perspective, there are inputs and outputs,
    //               and how the NODE interprets them is completely up to the NODE.
    //               In simple terms, IB(1) and OB(1) refer to different I/O bits
    // or
    //
    // NODE_CENTRIC: intermix the content and interpretation of IB and OB by
    //               effectively turning them into a single logical structure with a different
    //               pack/unpack mechanism.
    //               From the NODE's perspective, the inputs and outputs are intertwined,
    //               and the HOST sees the entire mixed list, and must take steps to not
    //               output on input pins or input on output pins.
    //               In simple terms, IB(1) and OB(1) refer to the SAME I/O bit, but only one is valid and can be used.


    ioMap *iomap = new ioMap();  // ioMap::HOST_CENTRIC (default) or ioMap::NODE_CENTRIC


    //    The following table is used to define and initialize the physically connected
    //    pins/devices; it is also used to automatically pack and unpack input and output bits
    //    in response to POLL and TRANSMIT packets.
    //
    //    The first column defines the DIRECTION of the I/O pin, either INPUT or OUTPUT.
    //
    //    Next comes the type of device, either a BUILTIN pin on the processor,
    //    or an I2C expander of a particular type, as defined in the I2Cexpander library.
    //    Additional I2C expanders can easily be added to that library as needed.
    //    By default, it supports the 16-bit MCP23017 chip used in the MRCS IOX16 and IOX32 boards,
    //    PCA9555's and MCP731x's, as well as 8-bit expanders in the PCF8574 and PCF8574A families.
    //
    //    The 3rd column is either the pin name or number.
    //
    //    Column 4 holds pin attributes and behaviors, logically combined with "|":
    //    If an input, should it be pulled up? INPUT_PULLUP
    //    If an output, should it be initialized high or low? OUTPUT_HIGH and OUTPUT_LOW
    //    Should the value be inverted?  INVERT
    //
    //    Finally, each line has a comment that describes how it is used and what it is connected to.
    //    This lets the sketch itself be part of the documentation of how the layout is wired.
    //
    //    The order of bits/bytes in this table are directly tied to the CMRI IB and OB contents
    //    found in a TX or RX packet.  The first line would be CS0001 in JMRI, the second, CS0002, etc.
    //    The choice of HOST_CENTRIC or NODE_CENTRIC will determine whether the input on pin A2 is
    //    referenced as CS0001 (HOST_CENTRIC) or CS0013 (NODE_CENTRIC)...


    iomap->add(OUTPUT,  BUILTIN,         4,   OUTPUT_LOW   | INVERT ); //  Trackside Light track 1
    iomap->add(OUTPUT,  BUILTIN,         5,   OUTPUT_LOW   | INVERT ); //  Trackside Light track 2
    iomap->add(OUTPUT,  BUILTIN,         6,   OUTPUT_LOW   | INVERT ); //  UNUSED
    iomap->add(OUTPUT,  BUILTIN,         7,   OUTPUT_LOW   | INVERT ); //  UNUSED
    iomap->add(OUTPUT,  BUILTIN,         8,   OUTPUT_LOW   | INVERT ); //  LED Maintainer Call
    iomap->add(OUTPUT,  BUILTIN,         9,   OUTPUT_HIGH           ); //  LED Yardmaster Call
    iomap->add(OUTPUT,  BUILTIN,        10,   OUTPUT_HIGH           ); //  Audio Trigger - Train Entering Station
    iomap->add(OUTPUT,  BUILTIN,        11,   OUTPUT_HIGH           ); //  Audio Trigger - Train Leaving Station

    iomap->add(INPUT,   BUILTIN,        12,   OUTPUT_LOW            ); //  Something...
    iomap->add(INPUT,   BUILTIN,        13,   OUTPUT_LOW            ); //  More something
    iomap->add(INPUT,   BUILTIN,        A0,   OUTPUT_LOW            ); //  something else
    iomap->add(INPUT,   BUILTIN,        A1,   OUTPUT_LOW            ); //  last something
    iomap->add(INPUT,   BUILTIN,        A2,   INPUT_PULLUP | INVERT ); //  Detector Yard Track 1 entry
    iomap->add(INPUT,   BUILTIN,        A3,   INPUT_PULLUP | INVERT ); //  Detector Yard Track 1 exit
    iomap->add(INPUT,   BUILTIN,        A4,   INPUT_PULLUP | INVERT ); //  Detector Yard Track 2 entry
    iomap->add(INPUT,   BUILTIN,        A5,   INPUT_PULLUP | INVERT ); //  Detector Yard Track 2 exit


    iomap->add(OUTPUT,  mcp23017_20,     0,   OUTPUT_HIGH           ); // Yard SW1 - inbound throat, D=Y1
    iomap->add(INPUT,   mcp23017_20,     1,   INPUT_PULLUP | INVERT ); //            OS Occupied?
    iomap->add(INPUT,   mcp23017_20,     2,   INPUT_PULLUP | INVERT ); //            SW Normal?
    iomap->add(INPUT,   mcp23017_20,     3,   INPUT_PULLUP | INVERT ); //            SW Reverse?

    iomap->add(OUTPUT,  mcp23017_20,     4,   OUTPUT_HIGH           ); // Yard SW3 - inbound throat, D=Y2
    iomap->add(INPUT,   mcp23017_20,     5,   INPUT_PULLUP | INVERT ); //            OS Occupied?
    iomap->add(INPUT,   mcp23017_20,     6,   INPUT_PULLUP | INVERT ); //            SW Normal?
    iomap->add(INPUT,   mcp23017_20,     7,   INPUT_PULLUP | INVERT ); //            SW Reverse?

    iomap->add(OUTPUT,  mcp23017_20,     8,   OUTPUT_HIGH           ); // Yard SW5 - inbound throat, D=Y3 N=Bypass
    iomap->add(INPUT,   mcp23017_20,     9,   INPUT_PULLUP | INVERT ); //            OS Occupied?
    iomap->add(INPUT,   mcp23017_20,    10,   INPUT_PULLUP | INVERT ); //            SW Normal?
    iomap->add(INPUT,   mcp23017_20,    11,   INPUT_PULLUP | INVERT ); //            SW Reverse?

    iomap->add(OUTPUT,  mcp23017_20,    12,   OUTPUT_HIGH  | INVERT ); //  Unused
    iomap->add(OUTPUT,  mcp23017_20,    13,   OUTPUT_HIGH  | INVERT ); //  Unused
    iomap->add(OUTPUT,  mcp23017_20,    14,   OUTPUT_HIGH           ); //  Unused
    iomap->add(OUTPUT,  mcp23017_20,    15,   OUTPUT_HIGH           ); //  Unused
    iomap->initialize();

    return iomap;
}

//==============================================
//====    END CONFIGURATION PARAMETERS      ====
//==============================================

#define TRACE() if (DEBUG)

ioMap *iomap;
CMRI_Node *node;

/**
 * These routines are called automatically when the protocol_handler() routine gets either a
 * POLL or a TX packet.
 *
 * POLL calls out to this routine to gather input values and put them into the provided packet
 */
void gatherInputs(CMRI_Packet &p) {
    iomap->pack(p.content(), p.length());
    // TRACE() { Serial.print("POLL:==>\nRX: <== "); Serial.println(CMRI_Node::packetToString(p)); }
}

/**
 * When a TX packet is received, this routine needs to distribute the output bits to the
 * pins and devices that need them.
 */
void distributeOutputs(CMRI_Packet &p) {
    // TRACE() { Serial.print("TX: ==> "); Serial.println(CMRI_Node::packetToString(p));  }
    iomap->unpack(p.content(), p.length());
}

void errorHandler(CMRI_Packet &p) {
    TRACE() { Serial.print("ERROR: ==> "); Serial.println(CMRI_Node::packetToString(p));  }
}

void setup() {
    TRACE() {
        Serial.begin(115200);
        while (!Serial) {
          ; // wait for serial port to connect. Needed for native USB on LEO
        }
        Serial.print("CMRI Node - ");
        Serial.println(CMRI_NODE_DESCRIPTION);
    }

    Serial1.begin(CMRI_SPEED, SERIAL_8N2);

    iomap = setupMap();

    node = new CMRI_Node(CMRI_NODE_ID, Serial1);
    node->set_num_input_bits(iomap->numInputs());   // how many Input bits?
    node->set_num_output_bits(iomap->numOutputs()); // how many output bits?
    node->setInputHandler(gatherInputs);
    node->setOutputHandler(distributeOutputs);
    node->setErrorHandler(errorHandler);

    TRACE() {
        Serial.println("Configured for:");
        Serial.print("    Address:   "); Serial.println(CMRI_NODE_ID, DEC);
        Serial.print("    Baud Rate: "); Serial.println(CMRI_SPEED);
        Serial.print("    Inputs:    "); Serial.println(node->get_num_input_bits());
        Serial.print("    Outputs:   "); Serial.println(node->get_num_output_bits());
        Serial.print("    IB and OB: "); Serial.println(iomap->isHostCentric() ? "Independent" : "Interleaved");
    }
}

void loop() {
    node->protocol_handler();
}
