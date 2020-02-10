//
// Created by John Plocher on 1/9/20.
//

#ifndef CPCMRI_H
#define CPCMRI_H


#define CPCMRI_VERSION 0.0.2 // version of this library
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

    void clear(void) {
        for (_bodylen = 0; _bodylen < (sizeof(_body) / sizeof(byte)); _bodylen++) {
            _body[_bodylen] = 0;
        }
        _bodylen = 0;
    }
    void append(char c) { if (_bodylen < (sizeof(_body) / sizeof(byte))) { _body[_bodylen++] = c;} }

    void set(char t, int a, int l, byte* b) {
        clear();    // nothing in body ...
        _type = (Type)t;
        _address = a;
        for (int idx = 0; idx < l; idx++) {
            append(*b++);
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
        , initHandler(NULL)
        , inputHandler(NULL)
        , outputHandler(NULL)
        , errorHandler(NULL)
        , _serial(serial_class) { }

    void setInitHandler(void(*newInitHandler) (CMRI_Packet &p)) {
        initHandler = newInitHandler;
    }
    void setErrorHandler(void(*newErrorHandler) (CMRI_Packet &p)) {
        errorHandler = newErrorHandler;
    }
    void setInputHandler(void(*newInputHandler) (CMRI_Packet &p)) {
        inputHandler = newInputHandler;
    }
    void setOutputHandler(void (*newOutputHandler) (CMRI_Packet &p)) {
        outputHandler = newOutputHandler;
    }

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
     * Convenience routine to get transmit delay
     * @return      the tx delay, units of 10uS
     */
    unsigned long  get_tx_delay(void)                 { return _tx_delay; }

    /**
     * Convenience routine to set the transmpt packet delay
     * @param txdelay the tx delay, units of 10uS
     */
    void set_tx_delay(unsigned int txdelay)           { _tx_delay = txdelay; }

    /**
     * Print out a human readable form of the byte using CMRI terminology
     * @param b byte to "print"
     * @param printASCII Should it print out 'A' or 0x41
     * @return the printable version of the byte
     */
    static String b2s(byte b, bool printASCII= true) {
      String s("");
      if (!printASCII) return String("{0x" + String(b, HEX) + "}");
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
     * Convert a CMRI packet to a printable string for debugging
     * @param p CMRI Packet to show
     * @return the human readable representation of the packet
     */
    static String packetToString(CMRI_Packet &p) {
        String s("");
        s = s + "Packet(type=" + String((char)p.type())
              + ", addr="      + String((byte)p.address())
              + ", bodylen="   + String(p.length())
              + ", body["  ;
        if (p.length() > 0) {
            byte *content = p.content();
            for (int idx = 0; idx < p.length(); idx++) {
                s = s + CMRI_Node::b2s(content[idx], false);
            }
        }
        s = s + "]);";
        return s;
    }
    /**
     * Read and parse a correctly structured CMRI packet from the serial link
     * @param packet    A place to store the incoming information
     * @return          The Type of the packet (I, P, T, R, N(oop), E(rror) or U(nknown))
     */
    CMRI_Packet::Type protocol_handler(void);

    /**
     * Send out a packet over the serial link
     * @param packet The packet to send
     */
    void              send_packet(CMRI_Packet &packet);

    /**
     * wrapper around _serial.read() with a timeout that returns -1
     * @return byte read, or -1 if timeout
     */
    int readByte(void);

private:
    enum ParseState { SYNC, HEADER, BODY };
    int    _address;
    int   _delay;
    uint16_t   _rx_length;
    uint16_t   _tx_length;
    unsigned long _tx_delay;

    // callbacks
    void (*initHandler)   (CMRI_Packet &p);
    void (*inputHandler)  (CMRI_Packet &p);
    void (*outputHandler) (CMRI_Packet &p);
    void (*errorHandler)  (CMRI_Packet &p);

    // Variables used by parser...
    CMRI_Packet _packet;
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

#define MAX_EXPANDERS        8

#define BUILTIN (NULL)
#define MEM1        0x0100  ///< device-> a bool variable (1 bit)
#define MEM8        0x0200  ///< device-> a byte variable (8 bits)
#define MEM16       0x0400  ///< device-> an uint16_t variable (16 bits)

#define INVERT      0x1000
#define OUTPUT_LOW  0x2000
#define OUTPUT_HIGH 0x4000

class ioEntry {
  public:
    ioEntry(int iodir, void *device, int iopin, unsigned int attributes);
    bool read(void);
    void write(bool val) ;

    byte iodirection;
    I2Cexpander *expander;
    byte pin;
    unsigned int flags;
    ioEntry *next;
};

class ioMap {
    public:
    enum MapType { HOST_CENTRIC, NODE_CENTRIC };

    ioMap(  MapType t = HOST_CENTRIC );
    ioMap *add(int iodir, void *device, int pin, unsigned int attributes);
    ioMap *initialize(void);

    int numInputs();
    int numOutputs();

    bool isHostCentric() { return (_type == HOST_CENTRIC); }

    void unpack(byte *OB, int maxlen);
    void pack(byte *IB, int maxlen);

    private:
    MapType _type;
    ioEntry *_root;
    ioEntry *_tail;

    I2Cexpander *seen  [MAX_EXPANDERS];
    unsigned int config[MAX_EXPANDERS];
    int          numSeen = 0;
};

#endif // CPCMRI_H
