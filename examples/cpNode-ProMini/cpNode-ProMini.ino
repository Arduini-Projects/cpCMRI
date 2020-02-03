/**
 * cpNode - Control Point CMRI Node
 * =================================
 * This sketch supports the MRCS Arduino Pro Mini.
 */

#include <cpCMRI.h>
#include <I2Cexpander.h>

//==============================================
//====   BEGIN CONFIGURATION PARAMETERS     ====
//==============================================

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


#define CMRINET_NODE_ID        1  // can be [0..64]  change this - must be unique for each node...
#define CMRINET_SPEED      19200  // make sure this matches the speed set in JMRI


cpIOMap node_configuration[] = {
  { I2Cexpander::BUILTIN,       2,   "O",                "1"},
  { I2Cexpander::BUILTIN,       3,   "O",                "1"},
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

  { I2Cexpander::MCP23017,     0x20, "OOOOOOOOIIIIIIII", "11111111++++++++"}, // lower case means invert state
  _END_OF_IOMAP_LIST_
};

//==============================================
//====     END CONFIGURATION PARAMETERS     ====
//==============================================

CMRI_Node *node;

/**
 * These routines are called automatically when the protocol_handler() routine gets either a 
 * POLL or a TX packet.
 * 
 * POLL calls out to this routine to gather input values and put them into the provided packet 
 */
void gatherInputs(CMRI_Packet &p) {
      cpIOMap::collectIOMapInputs(node_configuration, p.content());
}

/**
 * When a TX packet is received, this routine needs to distribute the output bits to the 
 * pins and devices that need them.
 */
void distributeOutputs(CMRI_Packet &p) {
      cpIOMap::distributeIOMapOutputs(node_configuration, p.content());
}

void errorHandler(CMRI_Packet &p) {
      ;
}

void setup() {
    Serial.begin(CMRINET_SPEED, SERIAL_8N2);

    cpIOMap::setupIOMap(node_configuration);

    node = new CMRI_Node(CMRINET_NODE_ID, Serial);
    node->set_num_input_bits(cpIOMap::countIOMapInputs(node_configuration));  // how many Input bits?
    node->set_num_output_bits(cpIOMap::countIOMapOutputs(node_configuration)); // how many output bits?
    node->setInputHandler(gatherInputs);
    node->setOutputHandler(distributeOutputs);
    node->setErrorHandler(errorHandler);
}

void loop() {
    node->protocol_handler();
}