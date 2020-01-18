/**
 * cpNode - Control Point CMRI Node
 * =================================
 * This sketch builds and uses a mocking framework to test
 * the cpCMRI protocol framework.
 *
 * With debugging on, it uses more than 2K of RAM
 * Testing performed on an ESP32
 *
 * Since the cpCMRI instance reads and writes to a "serial Stream" instance,
 * all we need to do is build something that looks like a Stream class,
 * fill it with the packets to "send", connect it and "run" the parser,
 * and we'll end up with a bunch of captured response packets.
 */

#include <String.h>
#include <cpCMRI.h>
#include <I2Cexpander.h>

class mockSerial : public Stream {
public:
    mockSerial(void)
    : _output_buffer_len(0)
    , _read_position(0)
    , _write_position(0)
    {}

    /**
     * Take a string description of a packet and stuff the corresponding
     * real packet into the "to be read from" buffer
     * @param str
     */
    void preload_data(const char *str) {
        /*
         * Format:
         * <c=type> <#=address> '[' <#> [',' ...] ']'
         * I3[0x42,0xFF]
         */
        const char *s = str;
        byte buffer[128];
        int bpos = 0;

        for (int i = 0; i < strlen(s); ++i) {
            char *nexts;
            buffer[bpos++] = CMRI_Packet::SYN;
            buffer[bpos++] = CMRI_Packet::SYN;
            buffer[bpos++] = CMRI_Packet::STX;
            byte ptype = *s++;
            unsigned long int paddr = strtoul(s, &nexts, 0) + 'A';
            buffer[bpos++] = (byte)paddr;
            buffer[bpos++] = ptype;

            s = nexts;
            if (*s++ != '[') { return; }
            while (*s != ']') {
                unsigned long int val = strtoul(s, &nexts, 0);
                if (  (val == CMRI_Packet::DLE)
                   || (val == CMRI_Packet::SYN)
                   || (val == CMRI_Packet::STX)
                   || (val == CMRI_Packet::ETX) ) {
                    buffer[bpos++] = CMRI_Packet::DLE;
                }
                buffer[bpos++] = val;
                s = nexts;
                if (*s == ',') { s++;}  // skip commas
                else if ( *s != ']') { return; }
            }
            buffer[bpos++] = CMRI_Packet::ETX;
        }
        for (int i = 0; i < bpos; i++) {
            pushback(buffer[i]);
        }
    }

    // Stream methods
    virtual void begin(int baud, int config=SERIAL_8N1) { /* NOOP */ }
    virtual void end(void)                              { /* NOOP */ }
    virtual void flush()                                { /* NOOP */ };
    virtual int available() {
        int count = _output_buffer_len - _read_position;
        /* Serial.print("... available=");
         * Serial.print(count);
         * Serial.print(" ");
         * Serial.println(toString());
         * */
        return count;
    }
    virtual int read() {
        return _read_position < _output_buffer_len ? _output_buffer[_read_position++] : -1;
    }
    virtual int peek() {
        return _read_position < _output_buffer_len ? _output_buffer[_read_position]   : -1;
    }
    virtual int availableForWrite() {
        return (sizeof(_input_buffer) / sizeof(byte) ) - _write_position;
    }
    size_t pushback(uint8_t c) {
        if (_output_buffer_len < sizeof(_output_buffer) / sizeof(byte)) {
            _output_buffer[_output_buffer_len++] = c;
            return 1;
        }
    }
    virtual size_t write(uint8_t c) {
        // Serial.print("stream write(0x");Serial.print((byte)c, HEX); Serial.print(", pos="); Serial.print(_write_position, DEC); Serial.print(") ");
        if (_write_position < sizeof(_input_buffer) / sizeof(byte)) {
            _input_buffer[_write_position++] = c;
            return 1;
        }
        return 0;
    }

    byte _output_buffer[256];   // reads come from here
    byte _input_buffer[256];    // writes go here
    int _output_buffer_len;     // how much can be read?
    int _read_position;         // how much have we read?
    int _write_position;        // how much has been written?


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
     * @param p
     * @return
     */
    static String toString(CMRI_Packet &p) {
        String s("");
        s = s + "Packet(type=" + String((char)p.type())
              + ", addr="      + String((byte)p.address())
              + ", bodylen="   + String(p.length())
              + ", body["  ;
        if (p.length() > 0) {
            byte *content = p.content();
            for (int idx = 0; idx < p.length(); idx++) {
                s = s + b2s(content[idx], false);
            }
        }
        s = s + "]);";
        return s;
    }
};

//==============================================
//====    NODE CONFIGURATION PARAMETERS     ====
//==============================================

#define CMRINET_NODE_ID      3
#define CMRINET_SPEED      9600  // make sure this matches your speed set in JMRI

cpIOMap node_configuration[] = {
    // device                 pin or                              '1'/'0' = initialized output ' ' = dontcare
    // type                    addr  I/O               initilize   '+'    = input pullup, ' ' = input HiZ
  { I2Cexpander::BUILTIN,      4,   "I",                "+"},
  { I2Cexpander::BUILTIN,      5,   "i",                "+"},
  { I2Cexpander::BUILTIN,     32,   "i",                "+"},
  { I2Cexpander::BUILTIN,     33,   "i",                "+"},
  { I2Cexpander::BUILTIN,     18,   "i",                " "},
  { I2Cexpander::BUILTIN,     19,   "i",                " "},
  { I2Cexpander::BUILTIN,     23,   "i",                " "},
  { I2Cexpander::BUILTIN,     25,   "I",                "+"},
  { I2Cexpander::PCF8574,      0,   "OOOOoooo",         "11111111"},
  { I2Cexpander::PCF8574A,     0,   "OOOOoooo",         "00000000"},

  _END_OF_IOMAP_LIST_
};

CMRI_Node *node;
CMRI_Packet p;                        // for CMRInet send / receive...
int txdelay = 0;                      // delay before sending packets...
int input_bits = 0;
int output_bits = 0;
mockSerial SerialMock;

void setup() {

    Wire.begin();
    Serial.begin(115200);
    Serial.println("CMRI Node - example 1");

    SerialMock.begin(CMRINET_SPEED, SERIAL_8N2);
    SerialMock.preload_data("I1[66,0,0,0]");
    SerialMock.preload_data("I2[66,0,0,0]");
    SerialMock.preload_data("I3[66,0,0,0]");
    SerialMock.preload_data("P3[]");
    SerialMock.preload_data("T3[0xFF,0x00]");
    SerialMock.preload_data("T3[0x0F,0xF0]");
    SerialMock.preload_data("T3[0xF0,0x0F]");
    SerialMock.preload_data("T3[0x00,0xFF]");



    node = new CMRI_Node(CMRINET_NODE_ID, SerialMock);
    
    cpIOMap::setupIOMap(node_configuration);
    node->set_num_input_bits(cpIOMap::countIOMapInputs(node_configuration));  // how many Input bits?
    node->set_num_output_bits(cpIOMap::countIOMapOutputs(node_configuration)); // how many output bits?
}

void loop() {
    CMRI_Packet::Type type;

    type = node->get_packet(p);
    switch (type) {
      case CMRI_Packet::ERROR:    // we got a corrupted or invalid packet...
          Serial.println("Parser returned Error");
          for(;;) {}    // hang...
          break;
      case CMRI_Packet::NOOP:     // do nothing, nothing to see here, no bytes are moving over the CMRInet wires...
          break;
      case CMRI_Packet::INIT:     // initialize things...
          Serial.print("INIT: "); Serial.println(mockSerial::toString(p));
          if (p.address() == CMRINET_NODE_ID) {   // is it for us?
              byte *body = p.content();
              txdelay = body[1] * 256 + body[2];
              node->set_tx_delay(txdelay);
              Serial.print("INIT: Set delay="); Serial.print(txdelay); Serial.println(" * 10 uS");
          }
          break;
      case CMRI_Packet::POLL:     // HOST wants our INPUT bits...
          Serial.print("POLL: "); Serial.println(mockSerial::toString(p));
          if (p.address() == CMRINET_NODE_ID) {   // is it for us?
              Serial.println("POLL: Generating response...");
              // make the RX packet
              cpIOMap::collectIOMapInputs(node_configuration, p.content());
              p.set_type(CMRI_Packet::RX);
              p.set_address(CMRINET_NODE_ID);
              p.set_length( (node->get_num_input_bits() + 7) / 8);  // in bytes, rounded up
              Serial.print("\t\t\t\t\tRX: "); Serial.println(mockSerial::toString(p));
              // send it out
              node->put_packet(p);
          }
          break;
      case CMRI_Packet::RX:       // ignore POLL responses from everyone...
          Serial.print("RX (ignored): "); Serial.println(mockSerial::toString(p));
          break;
      case CMRI_Packet::TX:       // HOST wants us to update our OUTPUTS
          Serial.print("TX: "); Serial.println(mockSerial::toString(p));
          if (p.address() == CMRINET_NODE_ID) {   // is it for us?
              cpIOMap::distributeIOMapOutputs(node_configuration, p.content());
          }
          break;
    }

    delay(1000);
}
