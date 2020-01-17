#include <Arduino.h>
#include <cpCMRI.h>
#include <I2Cexpander.h>

/**
 * Read a byte from the serial stream, if available.  Timeout if no activity
 *
 * At 8N1, 9600 baud, the bit time is about 104 microseconds which makes each
 * character sent take 1.04 milliseconds.  At 19200, bit time is 52.083 microseconds
 * and 0.5 milliseconds per character.
 *
 * Timeout is conservatively set to 10x character timings @9600 baud, or
 * 20 @ 19,200.  If there isn't another byte waiting to be read by that time,
 * it is safe to assume that the HOST has finished whatever packet it had been
 * sending.
 *
 * It could probably be shortened to 2 or 3 character times if desired.
 *
 * @return (0..0xFF valid incoming byte, -1 on timeout)
 */
int CMRI_Node::readByte(void) {
    elapsedMillis timeout = 0;
    while (true) {
        if (_serial.available() > 0) {
          return byte(_serial.read());
        }
        if (timeout > 10) return -1;  // after 10 character times @ 9600 baud (10mS), give up
    }
}


/**
 * return ERROR on timeout without cluttering code
 */
#define GET_BYTE_WITH_ERRORCHECK() { c = CMRI_Node::readByte();  if (c == -1) return CMRI_Packet::ERROR; }

/**
 * Read and parse a correctly structured CMRI packet from the serial link
 * while rejecting incorrectly formed and incomplete packets.
 *
 * Return NOOP if no input is available or if a read() timeout happens.
 *
 * @param packet    Caller provides the packet instance to store the incoming data
 * @return          The Type of the packet (I, P, T, R, N(oop), E(rror) or U(nknown))
 */
CMRI_Packet::Type CMRI_Node::get_packet(CMRI_Packet &packet) {
      if (_serial.available() <= 0) {  // fast return if nothing to read()
          return CMRI_Packet::NOOP;
      }

      int idx = 0;      // incoming packet's BODY length
      int loops = 0;    // in SYNC state, count of sequential "SYN" chars seen
      int c;            // protocol byte being processed

      CMRI_Node::ParseState state = CMRI_Node::SYNC;
      while (1) {
          switch (state) {
              case CMRI_Node::SYNC:               // Sync with packet byte stream...
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
                  if (c != CMRI_Packet::STX) {    // .. followed by a STX
                      Serial.print("ERROR: parse packet HEADER, no STX, instead got ");
                      Serial.print(b2s(c));
                      return CMRI_Packet::ERROR;    // ERROR
                  }
                  GET_BYTE_WITH_ERRORCHECK()
                  CMRI_Node::_paddr    = byte(c) - 'A';

                  GET_BYTE_WITH_ERRORCHECK()
                  CMRI_Node::_ptype    = byte(c);

                  state    = CMRI_Node::BODY;
                  break;

              case CMRI_Node::BODY:               // ==== Packet Body ====
                  GET_BYTE_WITH_ERRORCHECK()

                  if (c == CMRI_Packet::ETX) {    // ETX terminates a packet
                      packet.set(CMRI_Node::_ptype, CMRI_Node::_paddr, idx, CMRI_Node::_pbody);
                      return packet.type();
                  }
                  if (idx >= CMRI_Packet::BODY_MAX) {
                      Serial.print("ERROR: parse packet BODY overflow: more than ");
                      Serial.print(CMRI_Packet::BODY_MAX);
                      Serial.println(" bytes without ETX");
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
void CMRI_Node::put_packet(CMRI_Packet &packet) {
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
    const char *I = "Ii1";
    const char *O = "Oo0";
    const char *aliases;
    if (io == 'I') aliases = I;
    else if (io == 'O') aliases = O;
    else return 0;

    if (device == I2Cexpander::IGNORE) return 0;
    for (int idy = 0; direction[idy]; idy++) {
        if (strchr(aliases, direction[idy]) != NULL) count++;
    }
    return count;
}

void cpIOMap::setup(void) {
    int mode;
    bool need_invert = false;

    switch (device) {
        case I2Cexpander::IGNORE: break;
        case I2Cexpander::BUILTIN:
            switch (direction[0]) {
                case 'i': need_invert = true;   // FALLTHRU
                case 'I':
                case '1':
                    mask = 1;
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
                    pinMode(pin_address, mode);
                    break;

                case 'o': need_invert = true;   // FALLTHRU
                case 'O':
                case '0':
                    mask = 0;
                    mode = OUTPUT;
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
                    invert = need_invert;
                    pinMode(pin_address, mode);
                    digitalWrite(pin_address, (value ^ invert));
                    break;

                default:    // not handled...
                    Serial.print("cpIOMap: Unknown configuration direction '");
                    Serial.print(char(direction[0]));
                    Serial.println("'");
                    break;
            }
            break;

        default:  // some sort of I2C expander...
            for (int x = 0; direction[x] && initialize[x]; x++) {
                need_invert = false;
                switch (direction[x]) {
                    case 'i': need_invert = true;   // FALLTHRU
                    case 'I':
                    case '1':
                        mask   = ((mask   << 1) | 1);
                        value  = ((value  << 1) | 0);
                        invert = ((invert << 1) | (need_invert ? 1 : 0));
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
                        invert = ((invert << 1) | (need_invert ? 1 : 0));
                        break;

                    default:
                        break;
                }
            }
            mask = mask;

            expander = new I2Cexpander();
            expander->init(pin_address, device, mask, true);
            expander->write(value ^ invert); // initial values
            break;
    }
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
 * Walk thru hte config tree and initialize the physical ports and devices
 * @param iomap Array of IOMap port or device configuration and initialization specifications
 */
void cpIOMap::setupIOMap(cpIOMap *iomap) {
    const char *prefix = "";
    int base = DEC;
    for (int idx = 0; ; idx++) {
        cpIOMap *m = &(iomap[idx]);
        if (m->device == I2Cexpander::IGNORE) break;
        if (m->device == I2Cexpander::BUILTIN) {
            base=DEC;
            prefix="";
        } else {
            base=HEX;
            prefix="0x";
        }
        Serial.print("calling m->setup(");
        Serial.print(m->device);
        Serial.print(", ");
        Serial.print(prefix);
        Serial.print(m->pin_address, base);
        Serial.print(", \"");
        Serial.print(m->direction);
        Serial.print("\", \"");
        Serial.print(m->initialize);
        Serial.print("\")\t");
        m->setup();
    }
}

/**
 * fill body with input bits referenced in the cpIOMap array
 * @param iomap
 * @param body
 */
void cpIOMap::collectIOMapInputs(cpIOMap *iomap, byte *body) {
    int body_bit_idx = 0;
    bool need_invert = false;

    for (int idx = 0; ; idx++) {  // each IOMap entry...
        cpIOMap *map = &(iomap[idx]);
        if (map->device == I2Cexpander::IGNORE) return;
        if (map->device == I2Cexpander::BUILTIN) {
            int bit = 0;
            bool m = bitRead(map->mask, bit);     // mask bit
            bool i = bitRead(map->invert, bit);   // invert bit
            if (m == 1) {   // input needed
                bool v = digitalRead(map->pin_address);
                if (i) { v = (v ^ i); }
                    bitWrite((body[(body_bit_idx / 8)]), (body_bit_idx % 8), v);
                    body_bit_idx++;
            }
        } else { // must be an I2C device...
            if (map->expander != NULL) {
                uint32_t val = map->expander->read();
                bool v;
                for (int bit = 0; bit < map->expander->getSize(); bit++) {
                    bool m = bitRead(map->mask, bit);     // mask bit
                    bool i = bitRead(map->invert, bit);   // invert bit
                    if (m == 1) {   // input needed
                        bool v = bitRead(val, bit);
                        if (i) { v = (v ^ i); }
                        bitWrite((body[(body_bit_idx / 8)]), (body_bit_idx % 8), v);
                        body_bit_idx++;
                    }
                }
            }
        }
    }
}

/**
 * take body bits and output them to the ports in the cpIOMap array
 * @param iomap
 * @param body
 */
void cpIOMap::distributeIOMapOutputs(cpIOMap *iomap, byte *body) {
    int body_bit_idx = 0;
    bool need_invert = false;
    for (int idx = 0; ; idx++) {

        // for each IOMap entry...
        cpIOMap *map = &(iomap[idx]);

        if (map->device == I2Cexpander::IGNORE) return;  // at end of list
        if (map->device == I2Cexpander::BUILTIN) {       // a digital pin...
            int bit = 0;
            bool m = bitRead(map->mask, bit);     // mask bit
            bool i = bitRead(map->invert, bit);   // invert bit
            if (m == 0) {   // output needed
                bool v = bitRead((body[(body_bit_idx / 8)]), (body_bit_idx % 8));
                if (i) { v = (v ^ i); }
                digitalWrite(map->pin_address, v);
                body_bit_idx++;
            }
        } else {
            if (map->expander != NULL) { // an I2C expander device...
                uint32_t outputs = map->expander->next;
                for (int bit = 0; bit < map->expander->getSize(); bit++) {
                    bool m = bitRead(map->mask, bit);     // mask bit
                    bool i = bitRead(map->invert, bit);   // invert bit
                    if (m == 0) {   // output needed
                        bool v = bitRead((body[(body_bit_idx / 8)]), (body_bit_idx % 8));
                        if (i) { v = (v ^ i); }
                        bitWrite((body[(body_bit_idx / 8)]), (body_bit_idx % 8), v);
                        body_bit_idx++;
                    }
                }
                map->expander->write(map->expander->next);
            }
        }

    }
}


