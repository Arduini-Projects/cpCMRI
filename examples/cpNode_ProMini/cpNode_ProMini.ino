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
#define CMRI_NODE_DESCRIPTION  "ProMini"

ioMap *setupMap() {

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
    //    found in a TX or RX packet.  The first line would be CT0001 in JMRI, the second, CT0002, etc.
    //
    //    The choice of HOST_CENTRIC or NODE_CENTRIC will determine the exact numbering for mixed
    //    INPUTS and OUTPUTS...

    // MRCS ProMini daughterboard for cpNode
    //
    // Differences:
    // The ProMini only has 1x serial port (Serial1 does not exist), so you can not use debug print statements
    // AND do CMRInet packet I/O at the same time.
    //
    // The I2C pins are on different pins (A4/A5 instead of D2/D3), which means the I/O mapping of bits to pins changes.
    // (use D2 and D3, don't use A4 and A5)
    //
    // Pin mapping
    // ProMini pin    BBLeo pin   MRCS Header.pin   CMRI byte.bit
    // ===========    =========   ===============   =============
    //     D2             D4          J1.1                 1.1
    //     D3             D5          J1.2                 1.2
    //     D4             D6          J1.3                 1.3
    //     D5             D7          J1.4                 1.4
    //     D6             D8          J1.5                 1.5
    //     D7             D9          J1.6                 1.6
    //     D8             D10         J1.7                 1.7
    //     D9             D11         J1.8                 1.8
    //
    //     D10            D12         J2.1                 2.1
    //     D11            D13         J2.2                 2.2
    //     D12            A0          J2.3                 2.3
    //     D13            A1          J2.4                 2.4
    //     A0             A2          J2.5                 2.5
    //     A1             A3          J2.6                 2.6
    //     A2             A4          J2.7                 2.7
    //     A3             A5          J2.8                 2.8

    iomap->add(INPUT,  BUILTIN,         2,   INPUT); //
    iomap->add(INPUT,  BUILTIN,         3,   INPUT); //
    iomap->add(INPUT,  BUILTIN,         4,   INPUT); //
    iomap->add(INPUT,  BUILTIN,         5,   INPUT); //
    iomap->add(INPUT,  BUILTIN,         6,   INPUT); //
    iomap->add(INPUT,  BUILTIN,         7,   INPUT); //
    iomap->add(INPUT,  BUILTIN,         8,   INPUT); //
    iomap->add(INPUT,  BUILTIN,         9,   INPUT); //

    iomap->add(INPUT,  BUILTIN,        10,   INPUT); //
    iomap->add(INPUT,  BUILTIN,        11,   INPUT); //
    iomap->add(INPUT,  BUILTIN,        12,   INPUT); //
    iomap->add(INPUT,  BUILTIN,        13,   INPUT); //
    iomap->add(INPUT,  BUILTIN,        A0,   INPUT); //
    iomap->add(INPUT,  BUILTIN,        A1,   INPUT); //
    iomap->add(INPUT,  BUILTIN,        A2,   INPUT); //
    iomap->add(INPUT,  BUILTIN,        A3,   INPUT); //

    iomap->initialize();

    return iomap;
}

//==============================================
//====    END CONFIGURATION PARAMETERS      ====
//==============================================

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
}

/**
 * When a TX packet is received, this routine needs to distribute the output bits to the
 * pins and devices that need them.
 */
void distributeOutputs(CMRI_Packet &p) {
    iomap->unpack(p.content(), p.length());
}

void errorHandler(CMRI_Packet &p) {
}

void setup() {
    Serial.begin(CMRI_SPEED, SERIAL_8N2);

    iomap = setupMap();

    node = new CMRI_Node(CMRI_NODE_ID, Serial);
    node->set_num_input_bits(iomap->numInputs());   // how many Input bits?
    node->set_num_output_bits(iomap->numOutputs()); // how many output bits?
    node->setInputHandler(gatherInputs);
    node->setOutputHandler(distributeOutputs);
    node->setErrorHandler(errorHandler);
}

void loop() {
    node->protocol_handler();
}
