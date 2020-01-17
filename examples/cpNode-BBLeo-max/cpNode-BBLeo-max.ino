/**
 * cpNode - Control Point CMRI Node
 * =================================
 * This sketch is the code template for a CMRI serial protocol 
 * node implemented in an Arduino style system board, 
 * with optional I2C expanders used to increase the 
 * number of I/O points availble.  
 * 
 * The heavy lifting is done behind the scenes with the following class libraries:
 *     cpCMRI:  Implements all the protocol handling fiddly bits to work with CMRInet
 *     cpIOMap: Abstracts the reading and writing of bits to ports/pins and devices
 *     I2Cexpander:  abstracts the initialization, reading and writing details of 8- and 16-bit I2C expanders
 * 
 * The following sketch is an example of what a COMPLETE CMRI Node implementation might look like.
 * I have implemented and mock-tested all of the cpCMRI library, and most of the cpIPMap one, save the collect() and distribute() methods.
 * The I2Cexpander library is something I have been using for almost a decade now, and has proven useful in many other sketches.
 * 
 * Once I get the last bits working, I'll put all this up on github and recruit some pre-alpha testers.
 */

#include <cpCMRI.h>
#include <I2Cexpander.h>


//==============================================
//====    NODE CONFIGURATION PARAMETERS     ====
//==============================================

#define CMRINET_NODE_ID      20
#define CMRINET_SPEED      9600  // make sure this matches your speed set in JMRI

cpIOMap node_configuration[] = {
    // device                 pin or                              '1'/'0' = initialized output ' ' = dontcare
    // type                    addr  I/O               initilize   '+'    = input pullup, ' ' = input HiZ
  { I2Cexpander::BUILTIN,      11,   "I",                "+"},
  { I2Cexpander::BUILTIN,      10,   "I",                "+"},
  { I2Cexpander::BUILTIN,       9,   "I",                "+"},
  { I2Cexpander::BUILTIN,       8,   "i",                "+"},
  { I2Cexpander::BUILTIN,       7,   "i",                " "},
  { I2Cexpander::BUILTIN,       6,   "1",                " "},
  { I2Cexpander::BUILTIN,       5,   "I",                " "},
  { I2Cexpander::BUILTIN,       4,   "I",                " "},
  { I2Cexpander::BUILTIN,      13,   "O",                "1"},
  { I2Cexpander::BUILTIN,      12,   "o",                "0"},
  { I2Cexpander::MCP28017,     0x22, "OOOOOOOOIIIIIIII", "00000000++++++++"},
  { I2Cexpander::MCP28017,     0x22, "OOOOOOOOIIIIIIII", "00000000++++++++"},
  { I2Cexpander::MCP28017,     0x22, "OOOOOOOOIIIIIIII", "00000000++++++++"},
  { I2Cexpander::MCP28017,     0x22, "OOOOOOOOIIIIIIII", "00000000++++++++"},
  { I2Cexpander::MCP28017,     0x22, "OOOOOOOOIIIIIIII", "00000000++++++++"},
  { I2Cexpander::MCP28017,     0x22, "OOOOOOOOIIIIIIII", "00000000++++++++"},
  { I2Cexpander::MCP28017,     0x22, "OOOOOOOOIIIIIIII", "00000000++++++++"},
  { I2Cexpander::MCP28017,     0x22, "OOOOOOOOIIIIIIII", "00000000++++++++"},
  _END_OF_IOMAP_LIST_
};

CMRI_Node *node;
CMRI_Packet p;                        // for CMRInet send / receive...
int txdelay = 0;                      // delay before sending packets...
int input_bits = 0;
int output_bits = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("CMRI Node - example 1");

    Serial1.begin(CMRINET_SPEED, SERIAL_8N2);

    node = new CMRI_Node(CMRINET_NODE_ID, Serial1);
    input_bits  = cpIOMap::countIOMapInputs(node_configuration);  // how many Input bits?
    output_bits = cpIOMap::countIOMapOutputs(node_configuration); // how many output bits?
    
    cpIOMap::setupIOMap(node_configuration);
}

void loop() {
    CMRI_Packet::Type type;

    type = node->get_packet(p);
    switch (type) {
      case CMRI_Packet::ERROR:    // we got a corrupted or invalid packet...
          break;
      case CMRI_Packet::NOOP:     // do nothing, nothing to see here, no bytes are moving over the CMRInet wires...
          break;
      case CMRI_Packet::INIT:     // initialize things...
          if (p.address() == CMRINET_NODE_ID) {   // is it for us?
              byte *body = p.content();
              txdelay = body[1] * 256 + body[2];
          }
          break;
      case CMRI_Packet::POLL:     // HOST wants our INPUT bits...
          if (p.address() == CMRINET_NODE_ID) {   // is it for us?
              // make the RX packet
              cpIOMap::collectIOMapInputs(node_configuration, p.content()); 
              p.set_type(CMRI_Packet::RX);
              p.set_address(CMRINET_NODE_ID);
              p.set_length( (input_bits + 7) / 8);
              delayMicroseconds(txdelay * 10);
              // send it out
              node->put_packet(p);
          }
          break;
      case CMRI_Packet::RX:       // ignore POLL responses from everyone...
          break;
      case CMRI_Packet::TX:       // HOST wants us to update our OUTPUTS
          if (p.address() == CMRINET_NODE_ID) {   // is it for us?
              cpIOMap::distributeIOMapOutputs(node_configuration, p.content()); 
          }
          break;
    }
}
