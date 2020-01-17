//
// Created by John Plocher on 1/9/20.
//

#ifndef CPCMRI_H
#define CPCMRI_H


#define CPCMRI_VERSION 0.0.1 // version of this library
#include <Arduino.h>
#include <elapsedMillis.h>
#include <I2Cexpander.h>


/**
 * A CMRI Packet
 * as a Stream subclass, it can be used as a mock Serial source for testing
 */

class CMRI_Packet {
public:

   enum Type {
        INIT    = 'I',     ///< An init packet
        TX      = 'T',     ///< transmit - bits to use as outputs
        POLL    = 'P',     ///< Host wants to know our input bit status.  Send a Response with input data as a reply
        RX      = 'R',     ///< receive  - bits from inputs
        NOOP    = 'N',     ///< do nothing
        ERROR   = 'E',     ///< packet decoding error
        UNKNOWN = 'U'      ///< Uninitialized
    };
    enum NodeType {
        NODE_TYPE_X = 'X',  ///< 32-bit SUSIC
        NODE_TYPE_N = 'N',  ///< 24-bit SUSIC or 24-bit USIC
        NODE_TYPE_M = 'M',  ///< SMini
        NODE_TYPE_C = 'C',  ///< cpNode type Nodes
    };
    enum ControlChars {
        STX  = 0x02,        ///< packet start byte
        ETX  = 0x03,        ///< packet end byte
        DLE  = 0x10,        ///< packet data link escape byte
        SYN  = 0xFF,        ///< packet sync bytes

        BODY_MAX  = 256,    ///< max packet length in bytes (64 i/o cards @ 32 bits each + packet type and address bytes)
        OVERHEAD  = 6       ///< SYNs, Header and other framing
    };

    CMRI_Packet(void)
        : _type(UNKNOWN)
        , _address(0)
        , _bodylen(0) { }

    int   length()   { return _bodylen; }
    void  set_length(int len) { _bodylen = len; }

    Type  type()     { return _type;    }
    void  set_type(Type t)    { _type = t; }

    int   address()  { return _address; }
    void  set_address(int addr) { _address = addr; }

    byte *content()  { return _body;    }

    /**
     * make an INIT packet for a cpNode
     * @param a             The destination Node's address (0..64)
     * @param delay_time    In 10's of uS
     */
    void initCPNode(int a, int delay_time) {
        byte body[4];
        body[0] = 'C';
        body[1] = (delay_time / 256);
        body[2] = (delay_time % 256);
        body[3] = 0;
        set(INIT, a, 4, body);
    }

    /**
     * make a POLL packet
     * @param a             The queried Node's address (0..64)
     */
    void pollNode(int a) {
        set(POLL, a, 0, NULL);
    }

    /**
     * make a TRANSMIT packet
     * @param a             The destination Node's address (0..64)
     * @param len           The number of bytes in the body array
     * @param body          The bits to be transmitted, as an array of 8-bit bytes
     */
    void transmitToNode(int a, int len, byte* body) {
        set(TX, a, len, body);
    }

    /**
     * make a RECEIVE packet
     * @param a             My (the sending Node's) address (0..64)
     * @param len           The number of bytes in the body array
     * @param body          The bits to be transmitted, as an array of 8-bit bytes
     */
    void transmitFromNode(int a, int len, byte* body) {
        set(RX, a, len, body);
    }

    void set(char t, int a, int l, byte* b) {
        _type = (Type)t;
        _address = a;
        _bodylen = l;
        for (int idx = 0; idx < _bodylen; idx++) {
            _body[idx] = *b++;
        }
    }

private:
    Type _type;
    int  _address;
    int  _bodylen;
    byte _body[BODY_MAX];
};



class CMRI_Node {
public:
    /**
     * Handle the serial RS485/422 communication between HOST and NODE for a CMRI NODE
     * get and put CMRI_Packets using the _serial link
     *
     * @param Node_address      0..64
     * @param serial_class      A Serial instance that is connected to the RS485/422 haedware
     */
    CMRI_Node(int NodeAddress = 0, Stream& serial_class = Serial)
        : _address(NodeAddress)
        , _delay(0)
        , _rx_length(0)
        , _tx_length(0)
        , _serial(serial_class) { }
    /**
     * Convenience routine to get Node Configuration information
     * @return      the number of OUTPUT bits managed by this Node
     */
    int  get_num_output_bits(void)                    { return _tx_length; }

    /**
     * Convenience routine to store Node Configuration information
     * @param outputbits the number of OUTPUT bits managed by this Node
     */
    void set_num_output_bits(unsigned int outputbits) { _tx_length = outputbits; }

    /**
     * Convenience routine to get Node Configuration information
     * @return      the number of INPUT bits managed by this Node
     */
    int  get_num_input_bits(void)                     { return _rx_length; }

    /**
     * Convenience routine to store Node Configuration information
     * @param inputbits the number of INPUT bits managed by this Node
     */
    void set_num_input_bits(unsigned int inputbits)   { _rx_length = inputbits; }

    /**
     * Convenience routine to get Node Configuration information
     * @return      the Node address (0..64)
     */
    int  get_node_address(void)                       { return _address; }
    /**
     * Convenience routine to store Node Configuration information
     * @param address the Node address (0..64)
     */
    void set_node_address(unsigned int address)       { _address = address; }

    /**
     * Convenience function to pretty print the various CMRI protocol control bytes
     * @param b
     * @return
     */
    static String b2s(byte b) {
      String s("");
      switch (b) {
          case CMRI_Packet::SYN:  s = "<SYN>"; break;
          case CMRI_Packet::STX:  s = "<STX>"; break;
          case CMRI_Packet::ETX:  s = "<ETX>"; break;
          case CMRI_Packet::DLE:  s = "<DLE>"; break;
          default:   
              if ((b >= ' ') && (b <= '~')) { 
                s = s + "'" +   (char)b        + "'";
              } else { 
                s = s + "{0x" + String(b, HEX) + "}"; 
              }
              break;
        }
        return s;
    }

    /**
     * Read and parse a correctly structured CMRI packet from the serial link
     * @param packet    A place to store the incoming information
     * @return          The Type of the packet (I, P, T, R, N(oop), E(rror) or U(nknown))
     */
    CMRI_Packet::Type get_packet(CMRI_Packet &packet);
    /**
     * Send out a packet over the serial link
     * @param packet The packet to send
     */
    void              put_packet(CMRI_Packet &packet);

    /**
     * wrapper around _serial.read() with a timeout that returns -1
     * @return byte read, or -1 if timeout
     */
    int readByte(void);

private:
    enum ParseState { SYNC, HEADER, BODY };
    int    _address;
    int   _delay;
    int   _rx_length;
    int   _tx_length;

    // Variables used by parser...
    int _ptype, _paddr; // packet type and address
    byte  _pbody[CMRI_Packet::BODY_MAX + 1];  // TODO: +1 isn't needed, but might be useful for overflow protection

    Stream& _serial;
};



/**
 * Port / device I/O initialization
 * Abstraction that moves the details of reading/writing specific I/O bits in an 8 or 16-bit
 * I2C expander type device, with individual bits in a port/device being either inputs
 * or outputs.
 *
 * Depends on the I2Cexpander library and its device handlers.
 */
struct cpIOMap {
    enum IOType { BUILTIN, I2C};

    I2Cexpander::ExpanderType device;   ///< Device type (BUILTIN or I2C device type)
    int pin_address;        ///< pin (for builtin) or I2C address (for I2C)
    const char *direction;  ///< I - input, O = output, lower case = invert
    /**
     * Behavior of the pin: pullups or initialization values
     * INPUT:  '+' =  pullup, ' ' =  float, '-' = pulldown (if supported)
     * OUTPUT: '1' = high, '0' = low, ' ' = not specified
     */
    const char *initialize;
    // the following are all initialized to 0/NULL...
    uint32_t mask;      ///< bitwise form of initialization I/O configuration
    uint32_t value;     ///< outputs initialized as 0 or 1?
    uint32_t invert;    ///< invert i/o bit(s)?
    I2Cexpander *expander;

    void setup(void);       ///< initialize the system based on the initialized state
    int countIO(char io);   ///< return the number of bits configured as either I or O
    int countInputs(void)  { return countIO('I'); }
    int countOutputs(void) { return countIO('O'); }

    static int  countIOMapInputs(cpIOMap *iomap);   ///< operates on the array of cpIOMaps to count inputs
    static int  countIOMapOutputs(cpIOMap *iomap);  ///< operates on the array of cpIOMaps to count outputs
    static void setupIOMap(cpIOMap *iomap);         ///< configure ports, pins and devices used by the array

    static bool getBit(int bitnum);                 ///< read the IOMap's "bitnum" input bit
    static void setBit(int bitnum, bool val);       ///< write the IOMap's "bitnum" output bit

    static void collectIOMapInputs(cpIOMap *iomap, byte *body);     ///< fill body with input bits
    static void distributeIOMapOutputs(cpIOMap *iomap, byte *body); ///< take body bits and output them
};

#define _END_OF_IOMAP_LIST_ { I2Cexpander::IGNORE, 0, NULL, NULL}, // Marker entry at end of IOMap list

#endif // CPCMRI_H
