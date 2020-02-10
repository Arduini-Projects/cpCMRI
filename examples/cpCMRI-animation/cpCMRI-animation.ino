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

//==============================================
//====   BEGIN CONFIGURATION PARAMETERS     ====
//==============================================
#define CMRI_NODE_ID               1    // can be [0..64]  change this - must be unique for each node...
#define CMRI_SPEED             19200    // make sure this matches the speed set in JMRI
#define CMRI_NODE_DESCRIPTION  "BBLeo Animation Automation Example"

#define DEBUG 1                         // enable Serial printing of debug messages
// #define DEBUG 0                      // Turn off if you use Serial for CMRInet!


//////// Animation implementation details follow ...

/**
 * Bits set/cleared by the CMRI HOST
 */

bool door_state;         // 0 = closed, 1 = open
bool light_room1;        // 0 == off
bool light_room2;        // 0 == off
bool light_room3;        // 0 == off

/**
 * STATUS feedback values
 */
bool door_open;
bool door_ajar;
bool door_closed;
bool room1_occupied;
bool room2_occupied;
bool room3_occupied;


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

// CMRI communication

ioMap *setupMap() {
    // Declare the I2C expanders you will be using, providing the device type and I2C address
    I2Cexpander *mcp23017_20 = new I2Cexpander(I2Cexpander::MCP23017, 0x20);

    // There are 2x ways to interpret CMRI Body content (the IB and OB arrays):
    // HOST_CENTRIC: keep the IB and OB completely independent from each other,
    //               with disjoint pack and unpack mechanisms,
    //               just like the BASIC code and JLC hardware currently does.
    //               From the HOST's perspective, there are inputs and outputs,
    //               and how the NODE interprets them is completely up to the NODE.
    //               In simple terms, IB(1) and OB(1) refer to different I/O bits
    // or
    //
    // NODE_CENTRIC: intermix the content and interpretation of IB and OB by
    //               effectively turning them into a single logical structure with a different
    //               pack/unpack mechanism.
    //               From the NODE's perspective, the inputs and outputs are intertwined,
    //               and the HOST sees the entire mixed list, and must take steps to not
    //               output on input pins or input on output pins.
    //               In simple terms, IB(1) and OB(1) refer to the SAME I/O bit, but only one is valid and can be used.


    ioMap *iomap = new ioMap();  // ioMap::HOST_CENTRIC (default) or ioMap::NODE_CENTRIC


    //    The following table is used to define and initialize the physically connected
    //    pins/devices; it is also used to automatically pack and unpack input and output bits
    //    in response to POLL and TRANSMIT packets.
    //
    //    The first column defines the DIRECTION of the I/O pin, either INPUT or OUTPUT.
    //
    //    Column 2 is the address of a bool variable
    //
    //    The 3rd column is "bit 0", the only value in a bool.
    //
    //    Column 4 holds pin attributes and behaviors, logically combined with "|":
    //    MEM1 is the attribute for a bool variable.
    //
    //    Finally, each line has a comment that describes how it is used and what it is connected to.
    //    This lets the sketch itself be part of the documentation of how the layout is wired.


    iomap->add(OUTPUT,  &door_state,     0,   OUTPUT_LOW   | MEM1 ); //  door Open 1 or closed 0
    iomap->add(OUTPUT,  &light_room1,    0,   OUTPUT_LOW   | MEM1 ); //  Light in room 1
    iomap->add(OUTPUT,  &light_room2,    0,   OUTPUT_LOW   | MEM1 ); //  Light in room 2
    iomap->add(OUTPUT,  &light_room3,    0,   OUTPUT_LOW   | MEM1 ); //  Light in room 3
    iomap->add(OUTPUT,  &door_open,      0,   INPUT        | MEM1 ); //  door is all the way open
    iomap->add(OUTPUT,  &door_ajar,      0,   INPUT        | MEM1 ); //  door is in the b=middle, not open or closed
    iomap->add(OUTPUT,  &door_closed,    0,   INPUT        | MEM1 ); //  door is all the way closed
    iomap->add(OUTPUT,  &room1_occupied, 0,   INPUT        | MEM1 ); //  Room 1's light is on
    iomap->add(OUTPUT,  &room2_occupied, 0,   INPUT        | MEM1 ); //  Room 2's light
    iomap->add(OUTPUT,  &room3_occupied, 0,   INPUT        | MEM1 ); //  Room 3's light

    iomap->initialize();

    return iomap;
}
//==============================================
//====    END CONFIGURATION PARAMETERS      ====
//==============================================


ioMap *iomap;
CMRI_Node *node;

#define TRACE() if (DEBUG)

/**
 * These routines are called automatically when the protocol_handler() routine gets either a
 * POLL or a TX packet.
 *
 * POLL calls out to this routine to gather input values and put them into the provided packet
 */
void gatherInputs(CMRI_Packet &p) {
    iomap->pack(p.content(), p.length());
    // TRACE() { Serial.print("POLL:==>\nRX: <== "); Serial.println(CMRI_Node::packetToString(p)); }
}

/**
 * When a TX packet is received, this routine needs to distribute the output bits to the
 * pins and devices that need them.
 */
void distributeOutputs(CMRI_Packet &p) {
    // TRACE() { Serial.print("TX: ==> "); Serial.println(CMRI_Node::packetToString(p));  }
    iomap->unpack(p.content(), p.length());
}

void errorHandler(CMRI_Packet &p) {
    TRACE() { Serial.print("ERROR: ==> "); Serial.println(CMRI_Node::packetToString(p));  }
}

void setup() {
    TRACE() {
        Serial.begin(115200);
        while (!Serial) {
          ; // wait for serial port to connect. Needed for native USB on LEO
        }
        Serial.print("CMRI Node - ");
        Serial.println(CMRI_NODE_DESCRIPTION);
    }

    Serial1.begin(CMRI_SPEED, SERIAL_8N2);

    iomap = setupMap();

    node = new CMRI_Node(CMRI_NODE_ID, Serial1);
    node->set_num_input_bits(iomap->numInputs());   // how many Input bits?
    node->set_num_output_bits(iomap->numOutputs()); // how many output bits?
    node->setInputHandler(gatherInputs);
    node->setOutputHandler(distributeOutputs);
    node->setErrorHandler(errorHandler);

    TRACE() {
        Serial.println("Configured for:");
        Serial.print("    Address:   "); Serial.println(CMRI_NODE_ID, DEC);
        Serial.print("    Baud Rate: "); Serial.println(CMRI_SPEED);
        Serial.print("    Inputs:    "); Serial.println(node->get_num_input_bits());
        Serial.print("    Outputs:   "); Serial.println(node->get_num_output_bits());
        Serial.print("    IB and OB: "); Serial.println(iomap->isHostCentric() ? "Independent" : "Interleaved");
    }

    pinMode(servo_PIN,     OUTPUT);
    pinMode(room1_LED_PIN, OUTPUT);
    pinMode(room2_LED_PIN, OUTPUT);
    pinMode(room3_LED_PIN, OUTPUT);

    last_door_state = door_state;
    door_current_position = door_desired_position = door_state ? door_open_position : door_closed_position;
    myservo.attach(servo_PIN);
    myservo.write(door_desired_position);

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
    // set everything "off" and then turn on only the bits that matter...
    door_open      = false;
    door_closed    = false;
    door_ajar      = false;
    room1_occupied = false;
    room2_occupied = false;
    room3_occupied = false;

    if (door_desired_position == door_current_position) {
        if (door_current_position == door_open_position) {
	        door_open = true;
        } else if (door_current_position == door_closed_position) {
            door_closed = true;
        } else {
            door_ajar = true;
        }
    } else {
        door_ajar = true;
    }
    if (light_room1) {
    	room1_occupied = true;
    }
    if (light_room2) {
    	room2_occupied = true;
    }
    if (light_room3) {
    	room3_occupied = true;
    }
}

