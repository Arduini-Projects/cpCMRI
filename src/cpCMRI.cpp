#include <Arduino.h>
#include <cpCMRI.h>
#include <I2Cexpander.h>


#define CMRI_DEBUG_PROTOCOL   0x0001
#define CMRI_DEBUG_SERIAL     0x0002
#define CMRI_DEBUG_IOMAP      0x0004
#define CMRI_DEBUG_IO         0x0008
#define CMRI_DEBUG_TIMING     0x0010

#define DEBUG_INFO            0x0100
#define DEBUG_FREERAM         0x0200
#define DEBUG_IOMAP           0x0400
#define DEBUG_IOMAP_ADD       0x0800
#define DEBUG_IOMAP_INIT      0x1000
#define DEBUG_IOMAP_PACK      0x2000
#define DEBUG_IOMAP_UNPACK    0x4000
#define DEBUG_IOENTRY         0x8000

// Enable Serial.print() calls for debugging
#define CMRI_DEBUG (0)
// #define CMRI_DEBUG (CMRI_DEBUG_PROTOCOL | CMRI_DEBUG_SERIAL | CMRI_DEBUG_IOMAP | CMRI_DEBUG_IO)
// #define CMRI_DEBUG (CMRI_DEBUG_PROTOCOL | CMRI_DEBUG_IO)
// #define CMRI_DEBUG CMRI_DEBUG_IOMAP
// #define CMRI_DEBUG CMRI_DEBUG_TIMING | CMRI_DEBUG_IOMAP

#define TRACE(level) if (CMRI_DEBUG & (level))  // { then execute block }

/**
 * Read a byte from the serial stream, if available.  Timeout if no activity
 *
 * At 8N1, 9600 baud, the bit time is about 104 microseconds which makes each
 * character sent take 1.04 milliseconds.  At 19200, bit time is 52.083 microseconds
 * and 0.5 milliseconds per character.
 *
 * Timeout is set to 1x character timings @9600 baud, or 2 @ 19,200.
 * If there isn't another byte waiting to be read by that time, get back to
 * the sketch's main loop()
 *
 * @return (0..0xFF valid incoming byte, -1 on timeout)
 */
int CMRI_Node::readByte(void) {
    elapsedMillis timeout = 0;
    while (true) {
        if (_serial.available() > 0) {
          return byte(_serial.read());
        }
        if (timeout > 3) {
            TRACE(CMRI_DEBUG_SERIAL) { Serial.println("CMRI_Node::readByte() TIMEOUT"); }
            return -1;  // after 10 character times @ 9600 baud (10mS), give up
        }
    }
}


/**
 * return ERROR on timeout without cluttering code
 */
#define GET_BYTE_WITH_ERRORCHECK() { c = CMRI_Node::readByte();  if (c == -1) { return CMRI_Packet::ERROR; }}

/**
 * Read and parse a correctly structured CMRI packet from the serial link
 * while rejecting incorrectly formed and incomplete packets.
 *
 * Return NOOP if no input is available or if a read() timeout happens.
 * Processing overhead is less than 1mS for a full packet on an ESP32, 1uS for "nothing available"...
 *
 * @param packet    Caller provides the packet instance to store the incoming data
 * @return          The Type of the packet (I, P, T, R, N(oop), E(rror) or U(nknown))
 */
CMRI_Packet::Type CMRI_Node::protocol_handler(void) {
    unsigned long time_begin, time_end;
    TRACE(CMRI_DEBUG_TIMING) { time_begin = micros(); }
    if (_serial.available() <= 0) {  // fast return if nothing to read()
        TRACE(CMRI_DEBUG_TIMING) { time_end = micros();
            if ((time_end - time_begin)> 25) {
                Serial.print("Microseconds for nothing: ");
                Serial.println(time_end - time_begin);
            }
        }
        return CMRI_Packet::NOOP;
    }
    TRACE(CMRI_DEBUG_TIMING) { Serial.print("Serial queue:"); Serial.println(_serial.available()); }


    int idx = 0;      // incoming packet's BODY length
    int loops = 0;    // in SYNC state, count of sequential "SYN" chars seen
    int c;            // protocol byte being processed

    CMRI_Node::ParseState state = CMRI_Node::SYNC;
    while (1) {
        switch (state) {
            case CMRI_Node::SYNC:               // Sync with packet byte stream...
                TRACE(CMRI_DEBUG_PROTOCOL) { Serial.println("state: CMRI_Node::SYNC"); }
                loops = 0;
                do {
                    GET_BYTE_WITH_ERRORCHECK()
                    loops++;                // count the SYNs we commit...
                } while (c == CMRI_Packet::SYN);

                if (loops < 3) {
                    break;                      // (we fell out early if we didn't see two SYNs and a presumed STX...)
                }
                state = CMRI_Node::HEADER;
                break;

            case CMRI_Node::HEADER:             // ==== Packet Header ====
                TRACE(CMRI_DEBUG_PROTOCOL) { Serial.println("state: CMRI_Node::HEADER"); }
                if (c != CMRI_Packet::STX) {    // .. followed by a STX
                    TRACE(CMRI_DEBUG_PROTOCOL) {
                        Serial.print("ERROR: parse packet HEADER, no STX, instead got ");
                        Serial.print(b2s(c));
                    }
                    return CMRI_Packet::ERROR;    // ERROR
                }
                GET_BYTE_WITH_ERRORCHECK()
                CMRI_Node::_paddr = byte(c) - 'A';

                GET_BYTE_WITH_ERRORCHECK()
                CMRI_Node::_ptype = byte(c);

                state = CMRI_Node::BODY;
                break;

            case CMRI_Node::BODY:               // ==== Packet Body ====
                TRACE(CMRI_DEBUG_PROTOCOL) { Serial.println("state: CMRI_Node::BODY"); }
                GET_BYTE_WITH_ERRORCHECK()

                if (c == CMRI_Packet::ETX) {    // ETX terminates a packet
                    _packet.set(CMRI_Node::_ptype, CMRI_Node::_paddr, idx, CMRI_Node::_pbody);
                    switch (_packet.type()) {
                        case CMRI_Packet::INIT:
                            if (_packet.address() == _address) { // to me
                                if (initHandler) {
                                    (*initHandler)(_packet);
                                } else {
                                    byte *body = _packet.content();
                                    set_tx_delay(body[1] * 256 + body[2]);
                                }
                            }
                            break;
                        case CMRI_Packet::POLL:
                            if (_packet.address() == _address) { // to me
                                if (inputHandler) {
                                    _packet.clear();
                                    _packet.set_type(CMRI_Packet::RX);
                                    _packet.set_address(_address);
                                    _packet.set_length( (get_num_input_bits() + 7) / 8);  // in bytes, rounded up
                                    (*inputHandler)(_packet);  // fill in _body...
                                } else {
                                    _packet.set(CMRI_Packet::RX, _address, 0, NULL); // fake placeholder...
                                }
                                send_packet(_packet);
                            }
                            break;
                        case CMRI_Packet::TX:
                            if (_packet.address() == _address) { // to me
                                if (outputHandler) {
                                    (*outputHandler)(_packet);
                                }
                            }
                            break;
                        case CMRI_Packet::ERROR:
                            if (errorHandler) {
                                (*errorHandler)(_packet);
                            }
                            break;
                        default:
                            break;
                    }
                    TRACE(CMRI_DEBUG_TIMING) {
                        time_end = micros();
                        Serial.print("Microseconds for complete packet: ");
                        Serial.println(time_end - time_begin);
                    }
                    return _packet.type();
                }
                if (idx >= CMRI_Packet::BODY_MAX) {
                    TRACE(CMRI_DEBUG_PROTOCOL) {
                        Serial.print("ERROR: parse packet BODY overflow: more than ");
                        Serial.print(CMRI_Packet::BODY_MAX);
                        Serial.println(" bytes without ETX");
                    }
                    return CMRI_Packet::ERROR;  // ERROR if buffer would overflow
                }
                if (c == CMRI_Packet::DLE) {    // DLE escapes the next character
                    GET_BYTE_WITH_ERRORCHECK()
                }
                CMRI_Node::_pbody[idx++] = byte(c);      // record the contents
                break;
        }
    }
}

/**
 * Convenience function to send out an already-constructed packet
 * @param packet The packet to send
 */
void CMRI_Node::send_packet(CMRI_Packet &packet) {
    if (_tx_delay) {
        delayMicroseconds(_tx_delay * 10);
    }
    _serial.write(CMRI_Packet::SYN);
    _serial.write(CMRI_Packet::SYN);
    _serial.write(CMRI_Packet::STX);
    _serial.write('A' + packet.address());
    _serial.write(packet.type());
    byte *body = packet.content();

    for (int idy = 0; idy < packet.length(); idy++) {
        _serial.write(body[idy]);
    }
    _serial.write(CMRI_Packet::ETX);
}


/**
 * Simple linked list of IO entities
 */

ioEntry::ioEntry(int iodir, void *device, int iopin, unsigned int attributes) :
    iodirection(iodir),
    expander((I2Cexpander *)device),
    pin(iopin),
    flags(attributes),
    next(NULL) {
      // ensure pin is always in range for mem variables
      if (pin < 0) pin = 0;
      if ((flags & MEM1)  && (pin >  0))  pin =  0;
      if ((flags & MEM8)  && (pin >  7))  pin =  7;
      if ((flags & MEM16) && (pin > 15))  pin = 15;
}


bool ioEntry::read(void) {
    bool val;

    if (expander == BUILTIN) {
        val = digitalRead(pin);
    } else if (flags & (MEM1 | MEM8 | MEM16)) {
        val = bitRead(*(bool *)(expander), pin);
    } else {
        val = bitRead(expander->current(), pin);
    }

    TRACE(DEBUG_IOMAP_UNPACK) {
        Serial.print("ioEntry::write(");
        Serial.print(val, DEC);
    }
    if (flags & INVERT) {
        val = !val;
         TRACE(DEBUG_IOMAP_UNPACK) {
            Serial.print(" INVERT=> ");
            Serial.print(val, DEC);
        }
    }
    TRACE(DEBUG_IOMAP_UNPACK) {
        Serial.print(")");
    }
    return val;
}

void ioEntry::write(bool val) {
    TRACE(DEBUG_IOMAP_UNPACK) {
        Serial.print("ioEntry::write(");
        Serial.print(val, DEC);

    }
    if (flags & INVERT) {
        val = !val;
        TRACE(DEBUG_IOMAP_UNPACK) {
            Serial.print(" INVERT=> ");
            Serial.print(val, DEC);
        }
    }
    if (expander == BUILTIN) {
        TRACE(DEBUG_IOMAP_UNPACK) { Serial.print(" BUILTIN "); }
        digitalWrite(pin, val);
    } else if (flags & (MEM1 | MEM8 | MEM16)) {
        TRACE(DEBUG_IOMAP_UNPACK) { Serial.print(" MEMORY "); }
        bitWrite(*(bool *)(expander), pin, val);
    } else {    // I2Cexpander
        TRACE(DEBUG_IOMAP_UNPACK) { Serial.print(" I2C "); }
        bitWrite(expander->next, pin, val);
    }
    TRACE(DEBUG_IOMAP_UNPACK) {
        Serial.println(")");
    }
}

/**
 *
 */

ioMap::ioMap(MapType t) {
  _type = t;
  _root = NULL;
  _tail = NULL;

  TRACE(DEBUG_IOMAP) {
    Serial.print("ioMap::ioMap(");
    Serial.print( (t == HOST_CENTRIC) ? "HOST_CENTRIC" : (t == NODE_CENTRIC) ? "NODE_CENTRIC" : "UNKNOWN");
    Serial.println(");");
  }
}

ioMap *ioMap::add(int iodir, void *device, int pin, unsigned int attributes) {
  TRACE(DEBUG_IOMAP_ADD) {
    Serial.print("ioMap::add(");
    Serial.print( (iodir == INPUT) ? "INPUT" : (iodir == OUTPUT) ? "OUTPUT" : "UNKNOWN");
    Serial.print(", ");
    if (device == NULL) {
      Serial.print("BUILTIN");
    } else {
      Serial.print("<"); Serial.print((unsigned int)device, HEX); Serial.print(">");
    }
    Serial.print(", ");
    Serial.print(pin, DEC);
    Serial.print(", flags:0x");
    Serial.print((unsigned int)attributes, HEX);
    Serial.print(": ");
    const char *sep = "";
    if (attributes & INPUT_PULLUP) {
      Serial.print(sep); Serial.print("INPUT_PULLUP"); sep="|";
    }
    if (attributes & OUTPUT_HIGH) {
      Serial.print(sep); Serial.print("OUTPUT_HIGH"); sep="|";
    }
    if (attributes & OUTPUT_LOW) {
      Serial.print(sep); Serial.print("OUTPUT_LOW"); sep="|";
    }
    if (attributes & MEM1) {
      Serial.print(sep); Serial.print("MEM1"); sep="|";
    }
    if (attributes & MEM8) {
      Serial.print(sep); Serial.print("MEM8"); sep="|";
    }
    if (attributes & MEM16) {
      Serial.print(sep); Serial.print("MEM16"); sep="|";
    }
    if (attributes & INVERT) {
      Serial.print(sep); Serial.print("INVERT"); sep="|";
    }    Serial.print("); ");
    Serial.print(" _root = <");
    Serial.print((unsigned int)_root, HEX);
    Serial.print("> ");
    Serial.print(" _tail = <");
    Serial.print((unsigned int)_tail, HEX);
    Serial.println(">");
  }
  ioEntry *io = new ioEntry(iodir, device, pin, attributes);
  if (_root == NULL) {
    _root = io;
    _tail = _root;
  } else {
    _tail->next = io;
    _tail = io;
  }
  return this;
}

ioMap *ioMap::initialize(void) {
  TRACE(DEBUG_IOMAP_INIT) {
    Serial.println("ioMap::initialize()");
  }
  int count = 0;
  byte initial[MAX_EXPANDERS];

  auto getExpanderNum = [&](I2Cexpander *e) {
    for (int idx = 0; idx < numSeen; idx++) {
      if (seen[idx] == e) return idx;
    } // didn't find it...
    if (numSeen < MAX_EXPANDERS) {
      seen[numSeen] = e;
      config[numSeen] = 0;
      initial[numSeen] = 0;
      return numSeen++;
    }
    return -1;
  };

  // collect all I2C expanders and their bit directions...
  for (ioEntry *io = _root; io; io = io->next) {
    TRACE(DEBUG_IOMAP_INIT) {
      Serial.print("    ioEntry:");
      if (count < 10) Serial.print(" ");
      Serial.print(count++, DEC);
      Serial.print(" ");
    }

    bool val = 0;
    if (io->flags | OUTPUT_HIGH) {
      val = 1;
    } else if (io->flags | OUTPUT_LOW) {
      val = 0;
    }
    if (io->flags | INVERT) {
      val = !val;
    }

    bool conf = ((io->iodirection == INPUT) ? 1 : 0);

    byte attr = INPUT;
    if (io->flags | INPUT_PULLUP) {
      attr = INPUT_PULLUP;
    }

     TRACE(DEBUG_IOMAP_INIT) {
      Serial.print("pin: ");
      if (io->pin < 10) Serial.print(" ");
      Serial.print(io->pin);
      Serial.print(", direction: ");
      Serial.print((io->iodirection == INPUT) ? "IN " : "OUT");
      Serial.print(", initialize: ");
      Serial.print((io->flags & OUTPUT_HIGH) ? 1 : 0);
      Serial.print(", invert: ");
      Serial.print((io->flags & INVERT) ? 1 : 0);
    }

    if (io->expander == BUILTIN) {
      TRACE(DEBUG_IOMAP_INIT) {
        Serial.println(" Builtin");
      }

      if (io->iodirection == INPUT) {
        pinMode(io->pin, attr);
      } else if (io->iodirection == OUTPUT) {
        pinMode(io->pin, OUTPUT);
        digitalWrite(io->pin, val);
      }
    } else if (io->flags & (MEM1 | MEM8 | MEM16)) {
      TRACE(DEBUG_IOMAP_INIT) {
        Serial.print(" Memory: ");
        if (io->flags & MEM1)  { Serial.print("MEM1"); }
        if (io->flags & MEM8)  { Serial.print("MEM8"); }
        if (io->flags & MEM16) { Serial.print("MEM16");}
        Serial.println();
      }
      if (io->iodirection == OUTPUT) {
        io->write(val);
      }
    } else {      // I2C expanders...
      int idx = getExpanderNum(io->expander);
      TRACE(DEBUG_IOMAP_INIT) {
        Serial.print(" Expander: ");
        Serial.print(idx);
        if (idx < 0) Serial.print(" ERROR");
        Serial.println();
      }
      if (idx < 0) return this;  // error

      config[idx]  |= conf << io->pin;
      initial[idx] |= val  << io->pin;
    }
  }


  // After loop completes, initialize all the expanders...
  for (int idx = 0; idx < numSeen; idx++) {
    TRACE(DEBUG_IOMAP_INIT) {
        Serial.print(" Writing Expander: ");
        Serial.print(idx);
        Serial.print(": config=0x");
        Serial.print(config[idx], HEX);
        Serial.print(", write=0x");
        Serial.print(initial[idx], HEX);
        Serial.println();
    }
    seen[idx]->init(config[idx]);
    seen[idx]->write(initial[idx]);
  }
}


void ioMap::unpack(byte *OB, int maxlen) {
  TRACE(DEBUG_IOMAP_UNPACK) {
    Serial.print("ioMap::unpack(");
    Serial.print("OB[");
    const char *sep = "0x";
    for (int x = 0; x < maxlen; x++) {
      Serial.print(sep);
      Serial.print(OB[x], HEX);
      sep = ", 0x";
    }
    Serial.print("]:");
    Serial.print(maxlen, DEC);
    Serial.print(") ");
    Serial.print(_type == HOST_CENTRIC ? "HOST_CENTRIC"
                  :_type == NODE_CENTRIC ? "NODE_CENTRIC"
                  :                        "UNKNOWN" );
    Serial.println("");
  }

  // walk the bits in OB and the ordered list in _root, assigning output bits to each output...
  ioEntry *io = _root;
  int count = 0;
  for (int _bit = 0; (_bit < (maxlen * 8)) && io; _bit++) {    // two alternatives:
    // 1) HOST_CENTRIC: each bit in OB is an output bit, find each ioEntry of type OUTPUT and assign, or
    // 2) NODE_CENTRIC: each bit in OB is associated with an ioEntry, but only type==OUTPUTs get used/assigned.
    // the MapType _type determines which heurstic to use

    if (_type == HOST_CENTRIC) {
      while (io && (io->iodirection != OUTPUT)) {
        count++;
        io=io->next;
      }
      if (io == NULL) {
        TRACE(DEBUG_IOMAP_UNPACK) {
          Serial.println("    Break: out of ioEntries");
        }
        break; // at end of ioEntries...
      }
    } // else SPARSE

    if (io && (io->iodirection == OUTPUT)) {
      bool val = bitRead(OB[_bit / 8], (_bit % 8));

      TRACE(DEBUG_IOMAP_UNPACK) {
        Serial.print("    ioEntry:");
        Serial.print(count);
        Serial.print(" bit: ");
        Serial.print(_bit, DEC);
        Serial.print(", val:");
        Serial.print(val);
        Serial.print("  ");
      }
      io->write(val);
    }
    if (io == NULL) {
      TRACE(DEBUG_IOMAP_UNPACK) {
          Serial.println("    Break: out of ioEntries");
      }
      break; // at end of ioEntries...
    }
    count++;
    io=io->next;
  }


  // after loop completes, write out all the expanders...
  for (int idx = 0; idx < numSeen; idx++) {
    seen[idx]->write();
  }
}

void ioMap::pack(byte *IB, int maxlen) {
  TRACE(DEBUG_IOMAP_PACK) {
    Serial.print("ioMap::pack(");
    Serial.print(_type == HOST_CENTRIC ? "HOST_CENTRIC"
                  :_type == NODE_CENTRIC ? "NODE_CENTRIC"
                  :                        "UNKNOWN" );
    Serial.println(")");
  }
  // pre-read all the expanders...
  for (int idx = 0; idx < numSeen; idx++) {
    seen[idx]->read();
  }
  // walk the bits in IB and the ordered list in _root, reading bits from each input...
  ioEntry *io = _root;
  int count = 0;
  for (int _bit = 0; (_bit < (maxlen * 8)) && io; _bit++) {
    // two alternatives:
    // 1) HOST_CENTRIC: each bit in IB is an input bit, find each ioEntry of type INPUT and read it, or
    // 2) NODE_CENTRIC: each bit in IB is associated with an ioEntry, but only type==INPUTs get used/read.
    // the MapType _type determines which heurstic to use

    if (_type == HOST_CENTRIC) {
      while (io && (io->iodirection != INPUT)) {
        count++;
        io=io->next;
      }
      if (io == NULL) {
        TRACE(DEBUG_IOMAP_PACK) {
          Serial.println("    Break: out of ioEntries");
        }
        break; // at end of ioEntries...
      }
    } // else SPARSE


    // read the bits
    if (io && (io->iodirection == INPUT)) {
      bool val = io->read();
      TRACE(DEBUG_IOMAP_PACK) {
        Serial.print("    ioEntry:");
        Serial.print(count);
        Serial.print(" bit: ");
        Serial.print(_bit, DEC);
        Serial.print(", val:");
        Serial.println(val);
      }
      bitWrite(IB[_bit / 8], (_bit % 8), val);
    }

    count++;
    if (io == NULL) {
      TRACE(DEBUG_IOMAP_UNPACK) {
          Serial.println("    Break: out of ioEntries");
      }
      break; // at end of ioEntries...
    }
    io=io->next;
  }
}


int ioMap::numInputs() {
    ioEntry *io = _root;
    int count = 0;
    if (_type == NODE_CENTRIC) { // count = number of i/o bits total, ignoring I or O direction...
        for (io = _root; io; io = io->next) { count++; }
        return count;
    } else {
        for (io = _root; io; io = io->next) {
            if (io->iodirection == INPUT) { count++; }
        }
        return count;
    }
}

int ioMap::numOutputs() {
    ioEntry *io = _root;
    int count = 0;
    if (_type == NODE_CENTRIC) { // count = number of i/o bits total, ignoring I or O direction...
        for (io = _root; io; io = io->next) { count++; }
        return count;
    } else {
        for (io = _root; io; io = io->next) {
            if (io->iodirection == OUTPUT) { count++; }
        }
        return count;
    }
}