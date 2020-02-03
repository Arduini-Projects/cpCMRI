/**
 * cpNode - Control Point CMRI Node
 * =================================
 * This sketch chows how to use virtual pins and devices that
 * are not associated with physical pins on your processor.
 * 
 * This would be useful of you were building an automation with servos and flashing lights, and
 * you wanted to control it remotely via CMRI.
 *
 * This example sets up 16 bits as a control interface (8-in, 8-out) and does animation-like 
 * things in the background while waiting for CMRInet commands...
 */

/*
 *	Design overview
 *
 *	We are animating a warehouse, with 3 lit rooms and a servo-operated door
 *
 *	The CMRI HOST can tell us
 *		OPEN or CLOSE the door
 *		Turn on the room lights for
 *		    room 1
 *		    room 2
 *		    room 3
 *
 *	When the HOST POLL's us, we can inform it of the state of the building
 *
 *	The warehouse has some odd details in how it works:
 *		The door is heavy and only opens and closes slowly
 *		The lighting in room 1 works as expected, on and off
 *		Room 2, however, has a blinking light that can be off or blinking
 *		Room 3 can only be on when room 2 is on, but room3 doesn't blink
 */
 

#include <cpCMRI.h>
#include <I2Cexpander.h>
#include <elapsedMillis.h>
#include <Servo.h>
#define DEBUG 1 // enable Serial printing of debug messages
// #define DEBUG 0
#define TRACE() if (DEBUG)

//==============================================
//====   BEGIN CONFIGURATION PARAMETERS     ====
//==============================================

#define CMRINET_NODE_ID        1  // can be [0..64]  change this - must be unique for each node...
#define CMRINET_SPEED      19200  // make sure this matches the speed set in JMRI


//==============================================
//====    END CONFIGURATION PARAMETERS      ====
//==============================================

//////// Animation implementation details follow ...

/** 
 * Bits set/cleared by the CMRI HOST 
 */

bool door_state;         // 0 = closed, 1 = open 
bool light_room1;        // 0 == off
bool light_room2;        // 0 == off
bool light_room3;        // 0 == off
bool unused_bit5;
bool unused_bit6;
bool unused_bit7;
bool unused_bit8;

/**
 * STATUS values
 */
const int DOOR_OPEN      = 0x01; 
const int DOOR_AJAR      = 0x02;   // while door is moving, it is neither open or closed...
const int DOOR_CLOSED    = 0x04;
const int ROOM1_OCCUPIED = 0x10;
const int ROOM2_OCCUPIED = 0x20;
const int ROOM3_OCCUPIED = 0x40;

/**
 * Bits we set/clear as feedback to the HOST
 */
byte status;

/**
 * PINS that we will use to control the animation
 */

const int servo_PIN = 5;
const int room1_LED_PIN = 6;
const int room2_LED_PIN = 7;
const int room3_LED_PIN = 8;

elapsedMillis timer_move_servo;
#define DELAY_MOVE_SERVO  15
const int door_open_position = 10;
const int door_closed_position = 120;
bool last_door_state;
int door_current_position;
int door_desired_position;
Servo myservo;  // create servo object to control a servo

elapsedMillis timer_blinker;
#define DELAY_BLINKER  60
bool blinkstate = 0;


cpIOMap node_configuration[] = {
    // device                 pin or                              '1'/'0' = initialized output ' ' = dontcare
    // type                    addr  I/O               initilize   '+'    = input pullup, ' ' = input HiZ
  { I2Cexpander::BIT,   (uintptr_t)&door_state,        "O",         "0"},
  { I2Cexpander::BIT,   (uintptr_t)&light_room1,       "O",         "0"},
  { I2Cexpander::BIT,   (uintptr_t)&light_room2,       "O",         "0"},
  { I2Cexpander::BIT,   (uintptr_t)&light_room3,       "O",         "0"},
  { I2Cexpander::BIT,   (uintptr_t)&unused_bit5,       "O",         "0"},  // it is good to have multiples of 8 bits for I's and O's
  { I2Cexpander::BIT,   (uintptr_t)&unused_bit6,       "O",         "0"},
  { I2Cexpander::BIT,   (uintptr_t)&unused_bit7,       "O",         "0"},
  { I2Cexpander::BIT,   (uintptr_t)&unused_bit8,       "O",         "0"},
  
  { I2Cexpander::BYTE,  (uintptr_t)&status,            "IIIIIIII",  "        "},

_END_OF_IOMAP_LIST_
};


CMRI_Node *node;


void gatherInputs(CMRI_Packet &p) {
      cpIOMap::collectIOMapInputs(node_configuration, p.content());
      TRACE() { Serial.print("POLL:==>\nRX: <== "); Serial.println(CMRI_Node::packetToString(p));}
}

void distributeOutputs(CMRI_Packet &p) {
      TRACE() { Serial.print("TX: ==> "); Serial.println(CMRI_Node::packetToString(p)); }
      cpIOMap::distributeIOMapOutputs(node_configuration, p.content());
}

void setup() {
    Wire.begin();
    Serial1.begin(CMRINET_SPEED, SERIAL_8N2);

    cpIOMap::setupIOMap(node_configuration);

    node = new CMRI_Node(CMRINET_NODE_ID, Serial1);
    node->set_num_input_bits(cpIOMap::countIOMapInputs(node_configuration));  // how many Input bits?
    node->set_num_output_bits(cpIOMap::countIOMapOutputs(node_configuration)); // how many output bits?
    node->setInputHandler(gatherInputs);
    node->setOutputHandler(distributeOutputs);

    pinMode(servo_PIN,     OUTPUT);
    pinMode(room1_LED_PIN, OUTPUT);
    pinMode(room2_LED_PIN, OUTPUT);
    pinMode(room3_LED_PIN, OUTPUT);

    last_door_state = door_state;
    door_current_position = door_desired_position = door_state ? door_open_position : door_closed_position;
    myservo.attach(servo_PIN); 
    myservo.write(door_desired_position);

    TRACE() {
        Serial.begin(115200);
        Serial.println("CMRI Node - Animation example");
        Serial.println("Configured for:");
        Serial.print("    "); Serial.print(CMRINET_SPEED);  Serial.println(" Baud");
        Serial.print("    "); Serial.print(node->get_num_input_bits());  Serial.println(" Inputs");
        Serial.print("    "); Serial.print(node->get_num_output_bits()); Serial.println(" Outputs");
    }
}


void loop() {
    // handle any communications from the HOST
    node->protocol_handler();

    // handle any state changes caused by getting a TX packet...
    // (most of the time, nothing will have changed and there isn't anything to do
    // but blink the lights and run the servo...)
    
    // room1 turns on and off immediately when HOST sends new state
    digitalWrite(room1_LED_PIN, light_room1); 
    
    // Room 2 has a blinking light that only blinks if the HOST turns it on
    if (timer_blinker > DELAY_BLINKER) {      
        blinkstate = (blinkstate ? 0 : 1);
        digitalWrite(room2_LED_PIN, blinkstate & light_room2);
        timer_blinker = 0;
    }
    
    // room 3 can only be on if room 2 is also on, but room3 doesn't blink...
    digitalWrite(room3_LED_PIN, light_room2 && light_room3 ? 1 : 0);  


    if (last_door_state != door_state) { // HOST wants to move the door to the other position...
        door_desired_position = door_state ? door_open_position : door_closed_position;
        last_door_state = door_state;
    }
    
    if (timer_move_servo > DELAY_MOVE_SERVO) {  // move servo 1 degree at a time to simulate a slow door opening/closing
        if (door_current_position < door_desired_position) {
            myservo.write(++door_current_position);
        } else if (door_current_position > door_desired_position){
            myservo.write(--door_current_position);
        } // else the door is fully open or closed, so we have nothing to do
        timer_move_servo = 0;
    }

    // collect the animations's status for when a POLL command is received.
    status = 0;
    if (door_desired_position == door_current_position) {
        if (door_current_position == door_open_position) {
	        status |= DOOR_OPEN;
        } else if (door_current_position == door_closed_position) {
            status |= DOOR_CLOSED;
        } else {
            status |= DOOR_AJAR;
        }
    } else {
        status |= DOOR_AJAR;
    }
    if (light_room1) {
    	status |= ROOM1_OCCUPIED;
    }
    if (light_room2) {
    	status |= ROOM2_OCCUPIED;
    }
    if (light_room3) {
    	status |= ROOM3_OCCUPIED;
    }
}

