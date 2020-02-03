/**
 * cpNode - Control Point CMRI Node
 * =================================
 * This sketch supports a minimal cpNode WEMOS with 1x IOX expander and NO BUILTIN pins
 */

#include <cpCMRI.h>
#include <I2Cexpander.h>

//==============================================
//====    NODE CONFIGURATION PARAMETERS     ====
//==============================================
#define CMRINET_NODE_ID        1  // can be [0..64]  change this - must be unique for each node...
#define CMRINET_SPEED      19200  // make sure this matches the speed set in JMRI


cpIOMap node_configuration[] = {
    // device type          I2Caddr  I/O              '+' ' ' = input pullup or HiZ, '0' / '1' = initial value output
    { I2Cexpander::MCP23017,     0x20, "OOOOOOOOOOOOOOOO", "1111111100000000"},  // lower case means invert state

  _END_OF_IOMAP_LIST_
};

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

void setup() {
    Wire.begin();
    Serial.begin(CMRINET_SPEED, SERIAL_8N2);

    cpIOMap::setupIOMap(node_configuration);

    node = new CMRI_Node(CMRINET_NODE_ID, Serial);
    node->set_num_input_bits(cpIOMap::countIOMapInputs(node_configuration));  // how many Input bits?
    node->set_num_output_bits(cpIOMap::countIOMapOutputs(node_configuration)); // how many output bits?
    node->setInputHandler(gatherInputs);
    node->setOutputHandler(distributeOutputs);
}

void loop() {
    node->protocol_handler();
}