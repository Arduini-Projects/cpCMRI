/**
 * cpNode - Control Point CMRI Node
 * =================================
 * This sketch supports the maximum-configured MRCS cpNode + IOX system with 144 I/O points.
 */

#include <cpCMRI.h>
#include <I2Cexpander.h>
#define DEBUG 1 // enable Serial printing of debug messages
// #define DEBUG 0
#define TRACE() if (DEBUG)

//==============================================
//====   BEGIN CONFIGURATION PARAMETERS     ====
//==============================================

#define CMRINET_NODE_ID        1  // can be [0..64]  change this - must be unique for each node...
#define CMRINET_SPEED      19200  // make sure this matches the speed set in JMRI

cpIOMap node_configuration[] = {
    // device type              PIN  I/O      '+' ' ' = input pullup or HiZ, '0' / '1' = initial value output
  { I2Cexpander::BUILTIN,       4,   "I",                "+"},
  { I2Cexpander::BUILTIN,       5,   "I",                "+"},
  { I2Cexpander::BUILTIN,       6,   "I",                "+"},
  { I2Cexpander::BUILTIN,       7,   "I",                "+"},
  { I2Cexpander::BUILTIN,       8,   "I",                "+"},
  { I2Cexpander::BUILTIN,       9,   "I",                "+"},
  { I2Cexpander::BUILTIN,      10,   "I",                "+"},
  { I2Cexpander::BUILTIN,      11,   "I",                "+"},
  
  { I2Cexpander::BUILTIN,      12,   "O",                "1"},
  { I2Cexpander::BUILTIN,      13,   "O",                "1"},
  { I2Cexpander::BUILTIN,      A0,   "O",                "1"},
  { I2Cexpander::BUILTIN,      A1,   "O",                "1"},
  { I2Cexpander::BUILTIN,      A2,   "O",                "1"},
  { I2Cexpander::BUILTIN,      A3,   "O",                "1"},
  { I2Cexpander::BUILTIN,      A4,   "O",                "1"},
  { I2Cexpander::BUILTIN,      A5,   "O",                "1"},

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

//==============================================
//====    END CONFIGURATION PARAMETERS      ====
//==============================================


CMRI_Node *node;

/**
 * These routines are called automatically when the protocol_handler() routine gets either a 
 * POLL or a TX packet.
 * 
 * POLL calls out to a routine that gathers input values and puts them into the provided packet 
 */
void gatherInputs(CMRI_Packet &p) {
      cpIOMap::collectIOMapInputs(node_configuration, p.content());
      TRACE() { Serial.print("POLL:==>\nRX: <== "); Serial.println(CMRI_Node::packetToString(p)); }
}

/**
 * When a TX packet is received, this routine needs to distribute the output bits to the 
 * pins and devices that need them.
 */
void distributeOutputs(CMRI_Packet &p) {
      TRACE() { Serial.print("TX: ==> "); Serial.println(CMRI_Node::packetToString(p));  }
      cpIOMap::distributeIOMapOutputs(node_configuration, p.content());
}

void errorHandler(CMRI_Packet &p) {
      TRACE() { Serial.print("ERROR: ==> "); Serial.println(CMRI_Node::packetToString(p));  }
}


void setup() {
    Serial1.begin(CMRINET_SPEED, SERIAL_8N2);

    cpIOMap::setupIOMap(node_configuration);

    node = new CMRI_Node(CMRINET_NODE_ID, Serial1);
    node->set_num_input_bits(cpIOMap::countIOMapInputs(node_configuration));  // how many Input bits?
    node->set_num_output_bits(cpIOMap::countIOMapOutputs(node_configuration)); // how many output bits?
    node->setInputHandler(gatherInputs);
    node->setOutputHandler(distributeOutputs);
    node->setErrorHandler(errorHandler);

    TRACE() {
        Serial.begin(115200);
        Serial.println("CMRI Node - cpNode + 8x IOX-16 example");
        Serial.println("Configured for:");
        Serial.print("    "); Serial.print(CMRINET_SPEED);  Serial.println(" Baud");
        Serial.print("    "); Serial.print(node->get_num_input_bits());  Serial.println(" Inputs");
        Serial.print("    "); Serial.print(node->get_num_output_bits()); Serial.println(" Outputs");
    }
}

void loop() {
    node->protocol_handler();
}


