#include <Arduino.h>
#include <cpCMRI.h>
#include <I2Cexpander.h>


#define CMRI_DEBUG_PROTOCOL 0x01
#define CMRI_DEBUG_SERIAL   0x02
#define CMRI_DEBUG_IOMAP    0x04
#define CMRI_DEBUG_IO       0x08
#define CMRI_DEBUG_TIMING   0x10

// Enable Serial.print() calls for debugging
#define CMRI_DEBUG 0
//#define CMRI_DEBUG (CMRI_DEBUG_PROTOCOL | CMRI_DEBUG_SERIAL | CMRI_DEBUG_IOMAP | CMRI_DEBUG_IO)
//#define CMRI_DEBUG (CMRI_DEBUG_PROTOCOL | CMRI_DEBUG_IO)
//#define CMRI_DEBUG CMRI_DEBUG_IO
//#define CMRI_DEBUG CMRI_DEBUG_TIMING

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
        if (timeout > 1) {
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
            if ((time_end - time_begin)> 1) {
                Serial.print("Microseconds for nothing: ");
                Serial.println(time_end - time_begin);
            }
        }
        return CMRI_Packet::NOOP;
    }

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


int cpIOMap::countIO(char io) {  ///< return the number of bits configured as either I or O
    int count = 0;
    const char *I = "Ii1";  // characters that signify Inputs ..
    const char *O = "Oo0";  // .. and Outputs
    const char *aliases;
    if (io == 'I') aliases = I;
    else if (io == 'O') aliases = O;
    else {
        TRACE(CMRI_DEBUG_IOMAP) {
            Serial.print("cpIOMap::countIO('");
            Serial.print((char) io);
            Serial.print("') INVALID argument");
        }
        return 0;
    }

    if (device == I2Cexpander::IGNORE) return 0;
    for (int idy = 0; direction[idy]; idy++) {
        if (strchr(aliases, direction[idy]) != NULL) count++;
    }
    return count;
}

int cpIOMap::setup(unsigned int &bitcounter) {
    int mode;
    bool need_invert = false;
    bool err = false;

    switch (device) {
        case I2Cexpander::IGNORE: break;
        case I2Cexpander::BUILTIN:
        case I2Cexpander::BIT:
            range_start = range_end = bitcounter++;
            switch (direction[0]) {
                case 'i': need_invert = true;   // FALLTHRU
                case 'I':
                case '1':
                    mask = 1;
                    invertin = need_invert;
                    switch (initialize[0]) {
                        case '+':
                            mode = INPUT_PULLUP;
                            break;
                        case ' ':
                            mode = INPUT;
                            break;
                        case '-':
                        default:
                            mode = INPUT;
                            break;
                    }
                    if (device == I2Cexpander::BUILTIN) {
                        pinMode((int) pin_address, mode);
                        lastin = digitalRead((int) pin_address) ^ invertin;
                    } else {
                        lastin = *((bool *)pin_address) ^ invertin;
                    }
                    break;

                case 'o': need_invert = true;   // FALLTHRU
                case 'O':
                case '0':
                    mask = 0;
                    mode = OUTPUT;
                    invertout = need_invert;
                    switch (initialize[0]) {
                        case '1':
                        case 'I':
                        case 'i':
                            value = 1;
                            break;
                        case '0':
                        case 'O':
                        case 'o':
                            value = 0;
                            break;
                        case ' ':           // no initialization specified
                            value = 0;
                            break;
                        default:
                            value = 0;
                            break;
                    }
                    if (device == I2Cexpander::BUILTIN) {
                        pinMode((int) pin_address, mode);
                        digitalWrite((int)pin_address, (value ^ invertout));
                    } else {
                        *((bool *)pin_address) = value ^ invertout;
                    }
                    lastout = value;
                    break;

                default:    // not handled...
                    TRACE(CMRI_DEBUG_IOMAP) {
                        Serial.print("cpIOMap: Unknown configuration direction '");
                        Serial.print(char(direction[0]));
                        Serial.println("'");
                    }
                    break;
            }
            break;

        default:  // some sort of I2C expander or BYTE memory variable...
            invertin = invertout = 0;
            for (int x = 0; direction[x] && initialize[x]; x++) {
                need_invert = false;
                switch (direction[x]) {
                    case 'i': need_invert = true;   // FALLTHRU
                    case 'I':
                    case '1':
                        mask   = ((mask   << 1) | 1);
                        value  = ((value  << 1) | 0);
                        invertin = ((invertin << 1) | (need_invert ? 1 : 0));
                        break;

                    case 'o': need_invert = true;   // FALLTHRU
                    case 'O':
                    case '0':
                        mask  = ((mask << 1) | 0);
                        switch (initialize[x]) {
                            case '1':
                            case 'I':
                            case 'i':
                                value = ((value << 1) | 1);
                                break;
                            default:
                                value = ((value << 1) | 0);
                                break;
                        }
                        invertout = ((invertout << 1) | (need_invert ? 1 : 0));
                        break;

                    default:
                        break;
                }
            }
            mask = mask;
            lastout = value;
            int maxsize;
            if (device == I2Cexpander::BYTE) {
                byte *b = (byte *)pin_address;

                lastin = *b ^ invertin;  // grab any input bits...
                *b =  ((lastout ^ mask) |   // only outputs: ensure all input bits are zero
                       (~mask ^ lastin)) ^   // only input bits: ensure all output bits are zero
                       invertout;
                maxsize = 8;
                range_start = bitcounter;
                range_end = bitcounter + maxsize;
            } else {
                expander = new I2Cexpander();
                expander->init((size_t)pin_address, device, mask, true);
                expander->write(value ^ invertout); // initial values
                lastin = expander->read() ^ invertin;
                maxsize = expander->getSize();
                range_start = bitcounter;
                range_end = bitcounter + expander->getSize();
            }
            if (strlen(direction) != maxsize) {
                TRACE(CMRI_DEBUG_IOMAP) {
                    Serial.print("Error: size mismatch: direction string has ");
                    Serial.print(strlen(direction));
                    Serial.println(" bits defined.");
                }
                err = true;
            }
            if (strlen(initialize) != maxsize) {
                TRACE(CMRI_DEBUG_IOMAP) {
                    Serial.print("Error: size mismatch: initialization string has ");
                    Serial.print(strlen(initialize));
                    Serial.println(" bits defined.");
                }
                err = true;
            }
            if (err) {
                TRACE(CMRI_DEBUG_IOMAP) {
                    Serial.print("Error: size mismatch: I/O expander supports ");
                    Serial.print(maxsize);
                    Serial.println(" bits.");
                }
                return -1;      // error
            }
            break;
    }
    return device;
}

// Helper functions
// static class members that work on the array of cpIOMaps...

/**
 * Walk thru the config array and count the number of Input bits so we know how many bits to send to HOST
 * @param iomap Array of IOMap port or device configuration and initialization specifications
 */
int cpIOMap::countIOMapInputs(cpIOMap *iomap) {
    int retval = 0;
    for (int idx = 0; ; idx++) {
        cpIOMap *m = &(iomap[idx]);
        if (m->device == I2Cexpander::IGNORE) break;
            retval += m->countInputs();
    }
    return retval;
}


/**
 * Walk thru the config tree and count the number of Output bits so we know how many bits to read from HOST
 * @param iomap Array of IOMap port or device configuration and initialization specifications
 */
int cpIOMap::countIOMapOutputs(cpIOMap *iomap) {
    int retval = 0;
    for (int idx = 0; ; idx++) {
        cpIOMap *m = &(iomap[idx]);
        if (m->device == I2Cexpander::IGNORE) break;
            retval += m->countOutputs();
    }
    return retval;
}

/**
 * Walk thru the config tree and initialize the physical ports and devices
 * @param iomap Array of IOMap port or device configuration and initialization specifications
 */
void cpIOMap::setupIOMap(cpIOMap *iomap) {
    unsigned int bit_count = 0;
    unsigned long time_begin, time_end;
    TRACE(CMRI_DEBUG_TIMING) { time_begin = micros(); }
    for (int idx = 0;; idx++) {
        cpIOMap *m = &(iomap[idx]);
        if (m->device == I2Cexpander::IGNORE) break;

        TRACE(CMRI_DEBUG_IOMAP) {
            const char *prefix = "";
            int base = DEC;
            if (m->device == I2Cexpander::BUILTIN) {
                base = DEC;
                prefix = "";
            } else {
                base = HEX;
                prefix = "0x";
            }
            Serial.print("calling m->setup(");
            Serial.print(m->device);
            Serial.print(", ");
            Serial.print(prefix);
            Serial.print((unsigned int) m->pin_address, base);
            Serial.print(", \"");
            Serial.print(m->direction);
            Serial.print("\", \"");
            Serial.print(m->initialize);
            Serial.println("\")\t");
        }
        int cc = m->setup(bit_count);
    }
    TRACE(CMRI_DEBUG_TIMING) {
        time_end = micros();
        Serial.print("Microseconds to setup IOMaps: ");
        Serial.println(time_end - time_begin);
    }
}

/**
 * fill body with input bits referenced in the cpIOMap array
 * @param iomap
 * @param body
 */
void cpIOMap::collectIOMapInputs(cpIOMap *iomap, byte *body)     { processIOMapIO(iomap, body, 'I'); }

/**
 * take body bits and output them to the ports in the cpIOMap array
 * @param iomap
 * @param body
 */
void cpIOMap::distributeIOMapOutputs(cpIOMap *iomap, byte *body) { processIOMapIO(iomap, body, 'O'); }

/**
 * do the grunt work of reading and writing all the bits in the IOMap list
 * @param iomap
 * @param body
 * @param dir   'I' to read from hardware into body or 'O' to write from body to hardware...
 */
void cpIOMap::processIOMapIO(cpIOMap *iomap, byte *body, char dir) {
    int body_bit_idx = 0;
    bool need_invert = false;
    unsigned long time_begin, time_end;
    TRACE(CMRI_DEBUG_TIMING) { time_begin = micros(); }

    TRACE(CMRI_DEBUG_IOMAP) {
        Serial.print("\nprocessIOMapIO(map, body, '");
        Serial.print(dir);
        Serial.println(")");
    }

    for (int idx = 0; ; idx++) {  // each IOMap entry...
        cpIOMap *map = &(iomap[idx]);
        if (map->device == I2Cexpander::IGNORE) {
            TRACE(CMRI_DEBUG_TIMING) {
                time_end = micros();
                Serial.print("Microseconds to process I/O: ");
                Serial.println(time_end - time_begin);
            }
            return;
        }
        int lastbit = 1;
        if (map->device == I2Cexpander::BYTE) {
            if (dir == 'I') {
                    map->lastin = *((byte *)(map->pin_address)) ^ map->invertin;   // grab any input bits...
                }
                if (dir == 'O') {
                    map->lastout = 0;
                }
                lastbit = 8;
        } else if (map->device > I2Cexpander::BYTE) {  // must be I2C...
            if (map->expander != NULL) {
                if (dir == 'I') {
                    map->lastin = map->expander->read() ^ map->invertin;
                }
                if (dir == 'O') {
                    map->lastout = 0;
                }
                lastbit = map->expander->getSize();
            } else {
                TRACE(CMRI_DEBUG_IOMAP) {
                    Serial.println("ERROR: expander pointer is NULL!");
                }
            }
        }

        TRACE(CMRI_DEBUG_IOMAP) {
            Serial.print("    Map: dev=");
            Serial.print(map->device);
            Serial.print(" bits=");
            Serial.print(lastbit);
            Serial.print(": ");
        }
        for (int bit = 0; bit < lastbit; bit++) {
            bool m = bitRead(map->mask, bit);     // mask bit
            char val = '-';
            if (dir == 'I' && m == 1) {   // input needed
                bool v = map->getBit(bit);
                val = v ? '1' : '0';
                bitWrite((body[(body_bit_idx / 8)]), (body_bit_idx % 8), v);
                body_bit_idx++;
            }
            if (dir == 'O' && m == 0) {   // otherwise, output
                bool v = bitRead((body[(body_bit_idx / 8)]), (body_bit_idx % 8));
                val = v ? '1' : '0';
                map->setBit(bit, v);
                body_bit_idx++;
            }
            TRACE(CMRI_DEBUG_IOMAP) {
                Serial.print(val);
                Serial.print(" ");
            }
        }
        TRACE(CMRI_DEBUG_IOMAP) {
            Serial.println();
        }
        if (map->device == I2Cexpander::BYTE) {
            if (dir == 'O') {
                byte *b = (byte *)(map->pin_address);
                byte v= (
                                (map->lastout ^ map->mask) |   // only outputs: ensure all input bits are zero
                                (~map->mask ^ map->lastin)     // only input bits: ensure all output bits are zero
                        );
                *b = v ^ map->invertout;
            }
        } else if (map->device != I2Cexpander::BUILTIN) {  // must be I2C...
            if (map->expander != NULL) {
                if (dir == 'O') {
                    map->expander->write(map->lastout ^ map->invertout);
                }
            }
        }
    }
    // NOTREACHED
}

/**
 * Find the requested bit (reading expander state as needed) and return its value.
 * @param iomap     The list of maps
 * @param bitnum    The bit number (0..numbits) that should be read
 * @return          The vlue of the bit
 */
bool cpIOMap::getBit(cpIOMap *iomap, int bitnum) {
    TRACE(CMRI_DEBUG_IO) {
        Serial.print("cpIOMap::getBit(");
        Serial.print(bitnum, DEC);
        Serial.println(")");
    }
    for (int idx = 0;; idx++) {
        // for each IOMap entry...
        cpIOMap *map = &(iomap[idx]);
        if (map->device == I2Cexpander::IGNORE) {
            return 0;  // at end of list, didn't find handler
        }
        if (map->in_range(bitnum)) {
            if (map->device == I2Cexpander::BYTE) {
                map->lastin = *((byte *)(map->pin_address)) ^ map->invertin;   // grab any input bits...
            } else if ((map->device > I2Cexpander::BYTE)   // a I2C device...
                && (map->mask)) {                       // with at lease 1 input bit
                    map->lastin = map->expander->read() ^ map->invertin;
            }
            return map->getBit(bitnum - map->range_start);
        }
    }
}

/**
 * Get the state of an input bit.
 * Caller MUST ensure that I2C expanders are read before this routine is called,
 * as it only looks at the cached last read value...
 * @param bitnum
 * @param v
 */
bool cpIOMap::getBit(int bit) {
    if (device == I2Cexpander::IGNORE) return 0;
    bool m = bitRead(mask, bit);     // mask bit
    bool v;
    if (m == 1) {   // input needed
        if (device == I2Cexpander::BIT) {
            v = *((bool *)pin_address);
        } else if (device == I2Cexpander::BUILTIN) {
            v = digitalRead((int)pin_address);
        } else {
            v = bitRead(lastin, bit);
        }
        return v ^ bitRead(invertin, bit);
    } else {         // what was last output?
        v = bitRead(lastout, bit);   // last output bit
        return v ^ bitRead(invertout, bit);
    }
}

/**
 * find the pin/device that corresponds to the desired bit, and set it as requested
 * if the bit in question is part of an expander, write it out after changing it state.
 */
void cpIOMap::setBit(cpIOMap *iomap, int bitnum, bool val) {
    TRACE(CMRI_DEBUG_IO) {
        Serial.print("cpIOMap::setBit(");
        Serial.print(bitnum, DEC);
        Serial.println(")");
    }
    for (int idx = 0;; idx++) {
        // for each IOMap entry...
        cpIOMap *map = &(iomap[idx]);
        if (map->device == I2Cexpander::IGNORE) {
            return;  // at end of list, didn't find handler
        }
        if (map->in_range(bitnum)) {
            map->setBit(bitnum - map->range_start, val);
            if (map->device == I2Cexpander::BYTE) {
                *((byte *)(map->pin_address)) =
                        (
                                (map->lastout ^ map->mask) |   // only outputs: ensure all input bits are zero
                                (~map->mask ^ map->lastin)     // only input bits: ensure all output bits are zero
                        ) ^ map->invertout;
            } else if (map->device != I2Cexpander::BUILTIN) {  // a I2C device...
                map->expander->write(map->lastout ^ map->invertout);
            }
            return;
        }
    }
}

/**
 * Update the output state with a specific bit.
 * Caller MUST ensure that I2C expanders are written after bits are changed...
 * @param bitnum
 * @param v
 */
void cpIOMap::setBit(int bit, bool v) {
    if (device == I2Cexpander::IGNORE) return;
    bool m = bitRead(mask, bit);     // mask bit

    if (m == 0) {   // output needed
        if (device == I2Cexpander::BIT) {
            lastout = v;
            *((bool *)pin_address) = v ^ invertout;
        } else if (device == I2Cexpander::BUILTIN) {
            lastout = v;
            bool i = bitRead(invertout, bit);
            digitalWrite((int)pin_address, v ^ i);
        } else {
            v = bitWrite(lastout, bit, v);
        }
        return;
    }
}