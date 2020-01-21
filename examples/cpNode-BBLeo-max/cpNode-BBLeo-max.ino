/**
 * cpNode - Control Point CMRI Node
 * =================================
 * This sketch supports the maximum-configured MRCS cpNode + IOX system with 144 I/O points.
 * 
 * The heavy lifting is done behind the scenes with the following class libraries:
 *     cpCMRI:  Implements all the protocol handling fiddly bits to work with CMRInet
 *              includes
 *                    CMRI_Packet: The details of a CMRInet packet structure.
 *                    cpIOMap: Abstracts the reading and writing of bits to ports/pins and devices
 *     I2Cexpander:  abstracts the initialization, reading and writing details of 8- and 16-bit I2C expanders
 */

#include <cpCMRI.h>
#include <I2Cexpander.h>


//==============================================
//====    NODE CONFIGURATION PARAMETERS     ====
//==============================================

#define CMRINET_NODE_ID       3
#define CMRINET_SPEED      9600  // make sure this matches your speed set in JMRI

cpIOMap node_configuration[] = {
    // device                 pin or                              '1'/'0' = initialized output ' ' = dontcare
    // type                    addr  I/O               initilize   '+'    = input pullup, ' ' = input HiZ
  { I2Cexpander::BUILTIN,      11,   "I",                "+"},			// individual pins on the processor
  { I2Cexpander::BUILTIN,      10,   "I",                " "},			//' ' = regular pinMode(INPUT)
  { I2Cexpander::BUILTIN,       9,   "I",                "+"},			// + = enable pullups (INPUT_PULLUP)
  { I2Cexpander::BUILTIN,       8,   "i",                "+"},			// i = invert (0 becomes 1, 1 becomes 0)
  { I2Cexpander::BUILTIN,       7,   "i",                " "},
  { I2Cexpander::BUILTIN,       6,   "1",                " "},
  { I2Cexpander::BUILTIN,       5,   "I",                " "},
  { I2Cexpander::BUILTIN,       4,   "I",                " "},

  { I2Cexpander::BUILTIN,      13,   "O",                "1"},
  { I2Cexpander::BUILTIN,      12,   "o",                "0"},
  { I2Cexpander::BUILTIN,      A0,   "O",                "0"},
  { I2Cexpander::BUILTIN,      A1,   "O",                "0"},
  { I2Cexpander::BUILTIN,      A2,   "o",                "1"},
  { I2Cexpander::BUILTIN,      A3,   "o",                "1"},
  { I2Cexpander::BUILTIN,      A4,   "o",                "0"},
  { I2Cexpander::BUILTIN,      A5,   "o",                "1"},

  { I2Cexpander::MCP23017,     0x20, "OOOOOOOOIIIIIIII", "00000000++++++++"},	// MCP23017 expanders add 16 bits each
  { I2Cexpander::MCP23017,     0x21, "OOOOOOOOOOOOOOOO", "0000000000000000"},
  { I2Cexpander::MCP23017,     0x22, "IIIIIIIIIIIIIIII", "++++++++++++++++"},	// some expanders can turn pullups on and off...
  { I2Cexpander::MCP23017,     0x23, "IIIIIIIIOOOOOOOO", "++++++++11111111"},
  { I2Cexpander::MCP23017,     0x24, "IOIOIOIOIOIOIOIO", "+0+0+0+0+0+0+0+0"},   // bits don't need to be contiguous
  { I2Cexpander::MCP23017,     0x25, "OOOOOOOOIIIIIIII", "00000000++++++++"},
  { I2Cexpander::MCP23017,     0x26, "OOOOOOOOIIIIIIII", "00000000++++++++"},
  { I2Cexpander::MCP23017,     0x27, "ooooooooiiiiiiii", "11111111++++++++"},	// lower case means invert state
  _END_OF_IOMAP_LIST_
};

CMRI_Node *node;

/**
 * These routines are called automatically when the protocol_handler() routine gets either a 
 * POLL or a TX packet.
 * 
 * POLL calls out to a routine that gathers input values and puts them into the provided packet 
 */
void gatherInputs(CMRI_Packet &p) {
      cpIOMap::collectIOMapInputs(node_configuration, p.content());
      Serial.print("POLL:==>\nRX: <== "); Serial.println(CMRI_Node::packetToString(p));
}

/**
 * When a TX packet is received, this routine needs to distribute the output bits to the 
 * pins and devices that need them.
 */
void distributeOutputs(CMRI_Packet &p) {
      Serial.print("TX: ==> "); Serial.println(CMRI_Node::packetToString(p));
      cpIOMap::distributeIOMapOutputs(node_configuration, p.content());
}

void setup() {
    Serial.begin(115200);
    Serial.println("CMRI Node - cpNode + 8x IOX-16 example");

    Serial1.begin(CMRINET_SPEED, SERIAL_8N2);

    cpIOMap::setupIOMap(node_configuration);

    node = new CMRI_Node(CMRINET_NODE_ID, Serial1);
    node->set_num_input_bits(cpIOMap::countIOMapInputs(node_configuration));  // how many Input bits?
    node->set_num_output_bits(cpIOMap::countIOMapOutputs(node_configuration)); // how many output bits?
    node->setInputHandler(gatherInputs);
    node->setOutputHandler(distributeOutputs);
}

void loop() {
    node->protocol_handler();
}


