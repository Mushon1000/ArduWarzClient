#include "Events.h"
#include "EventQueue.h"
#include "EventDispatcher.h"
#include <inttypes.h>
#include "Servo.h"
#include <Arduino.h>
#include "DigiFi.h"
#include "MyTypes.h"
using namespace MyTypes;




/***************************************************************************************************/
/*	Global parameters
/***************************************************************************************************/
char arduWarzServer[] = "192.168.2.76"; // The server's IP address
const int ARDUWARZ_PACKET_SIZE = 64; // The UDP data header to read in bytes
String mDebugMessage = ""; // A string to concatinate debug message in order to send as packet to server
uint8_t packetBuffer[ARDUWARZ_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
uint8_t RecvBuffer[ARDUWARZ_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
uint32_t packetCount = 0; //Total incoming packets
DigiFi client; //digix wifi client
int timestamp; //last state of millis
int lapseTimestamp; //for calc if duration passed
unsigned long last_seq_num = 0;
int delayFactor = 1; //this is the number delay will be multiplied in when getting duration in unit command
int debugLevel = 1; //0 - no debug, 1 - serial only, 2 - packet only, 4 - both
int PWMPWR = 255; // The power to drive motors, between 0-255, when only about 100 and above actually move thems



struct OperationPacket { //A struct to pass to functions as a result of commands
	unsigned int Operation; //change to uint16_t Operation
	unsigned int Duration;
};


// the event queue
EventQueue q;

// the event dispatcher
EventDispatcher disp(&q);

/***************************************************************************************************/
/*	Global parameters
/***************************************************************************************************/



/* keeping this code for future use
// use this analog channel
#define AN_CHAN 0

// generate an event when the analog
// channel value changes this much
// increase value for noisy sources
#define AN_DELTA 5

// analog event handler
void analogHandler(int event, int param) {
	Serial.print("Analog value: ");
	Serial.println(param);
}


// this function generates an EV_TIME event
// each 1000 ms
void timeManager() {
	static unsigned long prevMillis = 0;
	unsigned long currMillis;

	currMillis = millis();
	if (currMillis - prevMillis >= 1000) {
		prevMillis = currMillis;
		q.enqueueEvent(Events::EV_TIME, 0, 0);    // param is not used here
	}
}


 //this function generates an EV_ANALOG event
 //whenever the analog channel AN_CHAN changes
void analogManager() {
	static int prevValue = 0;
	int currValue;

	currValue = analogRead(AN_CHAN);

	if (abs(currValue - prevValue) >= AN_DELTA) {
		prevValue = currValue;
		q.enqueueEvent(Events::EV_ANALOG0, currValue, currValue);    // use param to pass analog value to event handler
	}
}

*/




int lastDir = 0;

/***************************************************************************************************/
/*	Pins layout declaration
/***************************************************************************************************/
// Motor 1
int dir1PinA = 2;
int dir2PinA = 3;
int speedPinA = 6; // Needs to be a PWM pin to be able to control motor speed

// Motor 2
int dir1PinB = 4;
int dir2PinB = 5;
int speedPinB = 7; // Needs to be a PWM pin to be able to control motor speed

// Servo 1
int servoPin = 11;
Servo myservo;
int pos = 0;
int servoMoveSpeed = 5; //this is actually the delay in ms between each pos change

// led to be turned on or off
#define LED_PIN 13
/***************************************************************************************************/
/*	Pins layout declaration
/***************************************************************************************************/


// program setup
void(*resetFunc) (void) = 0; //declare reset function at address 0 - use for doing a remote SW reset (problem is that if board is stuck this reset cannot be called...
void Setup_SW_Reset(int opcode, int dummy, int duration)
{
	Serial.println("SW reset has been called, but disabled");
	//resetFunc(); //call reset
}




/***************************************************************************************************/
/*	Arduino setup function
/***************************************************************************************************/
void setup() {
	delay(2000);
	Serial.begin(9600); //Serial with the arduino serial line
	FlushInput(); //clear the serial line by reading it
	Serial.println("Serial started, Server IP: ");
	Serial.println(arduWarzServer);
    delay(1000);
 
	ledblinkRED(200); //blink to debug with led - could be removed
    // start the connection to wifi module:
   client.begin(115200);
   client.setDebug(false); 
  //wait for wifi module to be ready
  while (client.ready() != 1)
  {
	if (debugLevel == 1 || debugLevel == 4) Serial.println("Connecting to network..."); //debug level was added since there were phenomenon when serial line was busy with debug messages
    delay(1000);
	ledblinkRED(10); //blink to debug with led - could be removed
  } 

  ledblinkBLUE(50); //blink to debug with led - could be removed

  client.setMode(UDP); //must come before connect
  client.reset();
  client.connect(arduWarzServer,9001); //connect to UDP server - not sure why a connection is needed but it wont work without it...
  //client.setNetParams("UDP","CLIENT",9002,"192.168.1.77"); //this causes only 1 time loop for some reason //setNetParams should work but seems to have a bug
  if (debugLevel == 1 || debugLevel == 4) Serial.println("Setting up UDP connection");
  

	//disp.addEventListener(Events::EV_RECPACKET3, fun3);
	last_seq_num = 0;

	/***************************************************************************************************/
	/*	function disaptcher attachments
	/*	this is the way to bound commmand event code to function
	/***************************************************************************************************/
	disp.addEventListener(102, go_forward_timed);
	disp.addEventListener(103, go_reverse_timed);
	disp.addEventListener(106, go_left_timed);
	disp.addEventListener(107, go_right_timed);
	disp.addEventListener(110, stop_go_forward); //stop go forward doesn't use any parameters...
	disp.addEventListener(220, Servo_1_forward_steps);
	disp.addEventListener(221, Servo_1_backward_steps);

	disp.addEventListener(255, Setup_SW_Reset);
	disp.addEventListener(256, Setup_Motors_Power);
	/***************************************************************************************************/
	/*	function disaptcher attachments
	/***************************************************************************************************/



	/***************************************************************************************************/
	/*	Pins layout mode
	/***************************************************************************************************/
	pinMode(dir1PinA, OUTPUT);
	pinMode(dir2PinA, OUTPUT);
	pinMode(speedPinA, OUTPUT);
	pinMode(dir1PinB, OUTPUT);
	pinMode(dir2PinB, OUTPUT);
	pinMode(speedPinB, OUTPUT);
	myservo.attach(servoPin);
	pinMode(servoPin, OUTPUT);
	myservo.write(pos);
	/***************************************************************************************************/
	/*	Pins layout mode
	/***************************************************************************************************/

}


/***************************************************************************************************/
/*	Program loop - this will happen infinitley
/***************************************************************************************************/
void loop() {
	recvPacketNew();
	disp.run();
	delay(10);
	}
/***************************************************************************************************/
/*	Program loop - this will happen infinitley
/***************************************************************************************************/



/***************************************************************************************************/
/*	Debug functions (used only for debugging)
/***************************************************************************************************/
void ledblinkRED(int Delay)
{
	digitalWrite(13, HIGH);
	delay(Delay);
	digitalWrite(13, LOW);
	delay(Delay);
}
void ledblinkBLUE(int Delay)
{
	digitalWrite(12, HIGH);
	delay(Delay);
	digitalWrite(12, LOW);
	delay(Delay);
}
void FlushInput()
{
	while (Serial.available() > 0) Serial.read();
}
/***************************************************************************************************/
/*	Debug functions (used only for debugging)
/***************************************************************************************************/


/***************************************************************************************************/
/*	Packet handling
/***************************************************************************************************/

/*
This function should handle ack responses to the server upon packets arrival
It is not finished for the moment
*/
unsigned long sendAck(unsigned long last_seq_num) //TODO add unit id and cunstruct the correct ack packet
{
	// set all bytes in the buffer to 0
	// memset(packetBuffer, 0, ARDUWARZ_PACKET_SIZE);
	packetBuffer[0] = (int)((last_seq_num >> 24) & 0xFF);
	packetBuffer[1] = (int)((last_seq_num >> 16) & 0xFF);
	packetBuffer[2] = (int)((last_seq_num >> 8) & 0XFF);
	packetBuffer[3] = (int)((last_seq_num & 0XFF));
	//packetBuffer[0] = 0x31;   // LI, Version, Mode

	if (debugLevel == 2 || debugLevel == 4){
		for (int i = 0; i < mDebugMessage.length(); i++)
		{
			packetBuffer[4 + i] = (uint8_t)mDebugMessage[i];
		}
	}
client.write(packetBuffer, ARDUWARZ_PACKET_SIZE);
	
	
	if (debugLevel == 2 || debugLevel == 4) mDebugMessage = "";
}

/*
This function doesn't work, and I consider removing the crc from the protocol
*/
boolean calc_crc32(uint32_t crc, const char *buf, size_t len) //checksum calc from http://rosettacode.org/wiki/CRC-32#C
{
	static uint32_t table[256];
	uint32_t orig_crc;
	static int have_table = 0;
	uint32_t rem;
	uint8_t octet;
	int i, j;
	const char *p, *q;

	/* This check is not thread safe; there is no mutex. */
	if (have_table == 0) {
		/* Calculate CRC table. */
		for (i = 0; i < 256; i++) {
			rem = i;  /* remainder from polynomial division */
			for (j = 0; j < 8; j++) {
				if (rem & 1) {
					rem >>= 1;
					rem ^= 0xedb88320;
				}
				else
					rem >>= 1;
			}
			table[i] = rem;
		}
		have_table = 1;
	}

	crc = ~crc;
	q = buf + len;
	for (p = buf; p < q; p++) {
		octet = *p;  /* Cast to unsigned octet. */
		crc = (crc >> 8) ^ table[(crc & 0xff) ^ octet];
	}
	if (orig_crc == ~crc)
	{
		return true;
	}
	else 
	{
		return false;
	}
}


/*
This function is the receiving packets function that handles parsing of the packet, calling the ack function, and the correct command action functions
*/
struct OperationPacket recvPacketNew(){ //http://www.tutorialspoint.com/cprogramming/c_quick_guide.htm
	unsigned int readSize		= 23;
	byte protocol_id			= 23;	//0x17
	byte curr_msg_type[2]		= { 0, 0 };
	byte unit_id[4]				= {0, 0, 0, 1 };
	byte seq_num[8]				= { 0, 0, 0, 0, 0, 0, 0, 0 };
	unsigned long seq_num_long	= 0;
	// - moved to global params unsigned long last_seq_num	= 0;
	boolean use_checksum		= false;
	byte checksum[4]			= { 0, 0, 0, 0 };
	boolean checksum_valid		= false;

	struct OperationPacket parameters;

	parameters.Duration = 0;
	parameters.Operation = 0;

	if (client.available()) {

		client.read(RecvBuffer, readSize); // read the packet into the buffer
		
		if (debugLevel == 1 || debugLevel == 4){
			String RecvBuff = "";
			for (int i = 0; i < readSize; i++)
			{
				RecvBuff += "[";
				RecvBuff += RecvBuffer[i];
				RecvBuff += "]";
			}
			Serial.println(RecvBuff);
		}
		if (RecvBuffer[0] != protocol_id) return parameters; //this is not arduino wars protocol id - quit the function;
		curr_msg_type[0] = RecvBuffer[1];
		curr_msg_type[1] = RecvBuffer[2]; //get message type
		if (curr_msg_type[0] != 0 && curr_msg_type[1] != 1) //message type is not global
		{
			if ((unit_id[0] != RecvBuffer[3] && unit_id[1] != RecvBuffer[4] && unit_id[2] != RecvBuffer[5] && unit_id[3] != RecvBuffer[6])) //unit id is not our unit id
			{
				return parameters; //this message is not for us - quit the function;
			}
		}
		for (int i = 0; i<4; i++) {
			seq_num[i] = RecvBuffer[7 + i]; //fill seq_num
		}
		seq_num_long = (((long)seq_num[0]) << 24 | ((long)seq_num[1]) << 16 | ((long)seq_num[2]) << 8 | (long)seq_num[3]);
		/*Serial.println((long)seq_num[0]);
		Serial.println((long)seq_num[1]);
		Serial.println((long)seq_num[2]);
		Serial.println((long)seq_num[3]);
		Serial.println(seq_num_long);*/
		if (use_checksum == true)
		{
			for (int i = 0; i<4; i++) {
				checksum[i] = RecvBuffer[11 + i]; //fill checksum
			}
			uint32_t checksum32 = (uint32_t)checksum[0] << 24 | //convert the checksum to uint32_t
				(uint32_t)checksum[1] << 16 |
				(uint32_t)checksum[2] << 8 |
				(uint32_t)checksum[3];

			checksum_valid = calc_crc32(checksum32, (char*)RecvBuffer, readSize); //TODO checksum validation is not complete...
			if (checksum_valid != true)
			{
				return parameters; //checksum is not correct - quit the function;
			}
		}
		//if server did not received ack for a sequence, it will not send the next packet;
		//this will reduce memory by not saving earlier messages received on client;
		//if the ack was not received within a second from time server sent the sequenced packet, it will resend it - this can cause delay on actions, but since actions takes time in seconds, the risk is low;
		//if from some reason the client received the same sequence he already had as last, it will send ack for it, but will not process it again.
		//if the server will not receive the client's ack, it will send the packet again, up to a count of x - if counted to x - assuming the client is stuck\dead.
		//server sequence must star at 1.
		if (last_seq_num == seq_num_long)
		{
			if (debugLevel == 2 || debugLevel == 4) mDebugMessage += "finished processing the packet since we've processed it before\n";
			if (debugLevel == 1 || debugLevel == 4) Serial.println("finished processing the packet since we've processed it before");
			sendAck(last_seq_num);
			return parameters; //finished processing the packet since we've processed it before 
		}
		else if (last_seq_num + 1 == seq_num_long)
		{
			if (debugLevel == 2 || debugLevel == 4) mDebugMessage += "new packet sequence - processing...\n";
			if (debugLevel == 1 || debugLevel == 4) Serial.println("new packet sequence - processing...");
			last_seq_num = seq_num_long;
			sendAck(last_seq_num);
		}
		else
		{
			if (debugLevel == 2 || debugLevel == 4) mDebugMessage += "Somehow missed a sequence - what to do???\n";
			if (debugLevel == 1 || debugLevel == 4) Serial.println("Somehow missed a sequence - what to do???");
			sendAck(last_seq_num);
			//? ? ? we lost track of sequence - very unlikely since the server should not send a new sequence before it received and ack
		}


		ArduWarsMsgTypes msg_type = (ArduWarsMsgTypes)(((int)curr_msg_type[0] << 8) | curr_msg_type[1]);

		/*
		Serial.println(curr_msg_type[0]);
		Serial.println(curr_msg_type[1]);
		Serial.println(msg_type);*/

		switch (msg_type){
		case GLOBALALERT:
			//call message type GLOBALALERT handling function
			if (debugLevel == 1 || debugLevel == 4) Serial.println("msg_type: GLOBALALERT");
			break;
		case GLOBALINFORM:
			//call message type GLOBALINFORM handling function
			if (debugLevel == 1 || debugLevel == 4) Serial.println("msg_type: GLOBALINFORM");
			break;
		case UNITALERT:
			//call message type UNITALERT handling function
			if (debugLevel == 1 || debugLevel == 4) Serial.println("msg_type: UNITALERT");
			break;
		case UNITQUERY:
			//call message type UNITQUERY handling function
			if (debugLevel == 1 || debugLevel == 4) Serial.println("msg_type: UNITQUERY");
			break;
		case UNITINFORM:
			//call message type UNITINFORM handling function
			if (debugLevel == 1 || debugLevel == 4) Serial.println("msg_type: UNITINFORM");
			break;
		case UNITCOMMAND:
			//call message type UNITCOMMAND handling function
			if (debugLevel == 2 || debugLevel == 4) mDebugMessage += "msg_type: UNITCOMMAND\n";
			if (debugLevel == 1 || debugLevel == 4) Serial.println("msg_type: UNITCOMMAND");
			MessageHandlingUNITCOMMAND(15);
			break;
		case COORDSUPDATE:
			//call message type COORDSUPDATE handling function
			if (debugLevel == 1 || debugLevel == 4) Serial.println("msg_type: COORDSUPDATE");
			break;
		case COORDSQUERY:
			//call message type COORDSQUERY handling function
			if (debugLevel == 1 || debugLevel == 4) Serial.println("msg_type: COORDSQUERY");
			break;


		};
		//TODO - check if client.flush() is needed
		//client.flush();
		return parameters;
	}
	else
	{
		//ledblinkRED(500);
	}
}

/*
This function handles only UNITCOMMAND type of packets
*/
void MessageHandlingUNITCOMMAND(int msgDataOffset){
	int commandsCount = RecvBuffer[msgDataOffset];
	struct OperationPacket parameters;

	parameters.Duration = 0;
	parameters.Operation = 0;

	for (int i = 0; i < commandsCount; i++)
	{
		unsigned int opCode = ((int)RecvBuffer[msgDataOffset + 1 + (i * 8)] << 8) | RecvBuffer[msgDataOffset + 2 + (i * 8)]; //msgDataOffset + 1 for count byte + i
		unsigned int duration = (((long)RecvBuffer[msgDataOffset + 4 + (i * 8)]) << 24 | ((long)RecvBuffer[msgDataOffset + 5 + (i * 8)]) << 16 | ((long)RecvBuffer[msgDataOffset + 6 + (i * 8)]) << 8 | (long)RecvBuffer[msgDataOffset + 7 + (i * 8)]);

		//Serial.print("opCode: ");
		//Serial.println(opCode);
		//Serial.print("duration: ");
		//Serial.println(duration);
		q.enqueueEvent(opCode, opCode, duration);
		//sendAck(1);
	}

}
/***************************************************************************************************/
/*	Packet handling
/***************************************************************************************************/



/***************************************************************************************************/
/*	Robot movement functions
/	need to add rotate function that enable turning while moving forward\backwards
/	need to change implementation of all "timed" functions to not use delay since this is the major
/	block in the correct operation of the robot
/***************************************************************************************************/


void Setup_Motors_Power(int opcode, int dummy, int duration)
{
	PWMPWR = duration;
	if (debugLevel == 2 || debugLevel == 4) mDebugMessage += "Setup_Motors_Power: \n";
	if (debugLevel == 2 || debugLevel == 4) mDebugMessage += duration;
	if (debugLevel == 1 || debugLevel == 4) Serial.println("Setup_Motors_Power: "); // Prints out �Motor 1 Forward� on the serial monitor
	if (debugLevel == 1 || debugLevel == 4) Serial.println(duration); // Creates a blank line printed on the serial monitor
	if (debugLevel == 1 || debugLevel == 4) Serial.println("   "); // Creates a blank line printed on the serial monitor
}

/* basic multi-chassis operations*/
void Motor_1_Forward(){
	analogWrite(speedPinA, PWMPWR);//Sets speed variable via PWM 
	digitalWrite(dir1PinA, LOW);
	digitalWrite(dir2PinA, HIGH);
	if (debugLevel == 2 || debugLevel == 4) mDebugMessage += "Motor 1 Forward\n";
	if (debugLevel == 1 || debugLevel == 4) Serial.println("Motor 1 Forward"); // Prints out �Motor 1 Forward� on the serial monitor
	if (debugLevel == 1 || debugLevel == 4) Serial.println("   "); // Creates a blank line printed on the serial monitor
}

void Motor_1_Stop(){
	analogWrite(speedPinA, 0);
	digitalWrite(dir1PinA, LOW);
	digitalWrite(dir2PinA, HIGH);
	if (debugLevel == 2 || debugLevel == 4) mDebugMessage += "Motor 1 Stop\n";
	if (debugLevel == 1 || debugLevel == 4) Serial.println("Motor 1 Stop");
	if (debugLevel == 1 || debugLevel == 4) Serial.println("   ");
}

void Motor_1_Backward(){
	analogWrite(speedPinA, PWMPWR);
	digitalWrite(dir1PinA, HIGH);
	digitalWrite(dir2PinA, LOW);
	if (debugLevel == 2 || debugLevel == 4) mDebugMessage += "Motor 1 Reverse\n";
	if (debugLevel == 1 || debugLevel == 4) Serial.println("Motor 1 Reverse");
	if (debugLevel == 1 || debugLevel == 4) Serial.println("   ");
}

void Motor_2_Forward(){
	analogWrite(speedPinB, PWMPWR);//Sets speed variable via PWM 
	digitalWrite(dir1PinB, LOW);
	digitalWrite(dir2PinB, HIGH);
	if (debugLevel == 2 || debugLevel == 4) mDebugMessage += "Motor 2 Forward\n";
	if (debugLevel == 1 || debugLevel == 4) Serial.println("Motor 2 Forward"); // Prints out �Motor 1 Forward� on the serial monitor
	if (debugLevel == 1 || debugLevel == 4) Serial.println("   "); // Creates a blank line printed on the serial monitor
}

void Motor_2_Stop(){
	analogWrite(speedPinB, 0);
	digitalWrite(dir1PinB, LOW);
	digitalWrite(dir2PinB, HIGH);
	if (debugLevel == 2 || debugLevel == 4) mDebugMessage += "Motor 2 Stop\n";
	if (debugLevel == 1 || debugLevel == 4) Serial.println("Motor 2 Stop");
	if (debugLevel == 1 || debugLevel == 4) Serial.println("   ");
}

void Motor_2_Backward(){
	analogWrite(speedPinB, PWMPWR);
	digitalWrite(dir1PinB, HIGH);
	digitalWrite(dir2PinB, LOW);
	if (debugLevel == 2 || debugLevel == 4) mDebugMessage += "Motor 2 Reverse\n";
	if (debugLevel == 1 || debugLevel == 4) Serial.println("Motor 2 Reverse");
	if (debugLevel == 1 || debugLevel == 4) Serial.println("   ");
}

void Servo_1_forward_steps(int opcode, int dummy, int duration)
{
	for (int i = 0; i < duration; i++)
	{
		Servo_1_forward_step();
	}
	if (debugLevel == 2 || debugLevel == 4) mDebugMessage += "Servo_1_forward_steps\n";
	if (debugLevel == 1 || debugLevel == 4) Serial.println("Servo_1_forward_steps");
	if (debugLevel == 1 || debugLevel == 4) Serial.println("   ");
}

void Servo_1_backward_steps(int opcode, int dummy, int duration)
{
	for (int i = 0; i < duration; i++)
	{
		Servo_1_backward_step();
	}
	if (debugLevel == 2 || debugLevel == 4) mDebugMessage += "Servo_1_backward_steps\n";
	if (debugLevel == 1 || debugLevel == 4) Serial.println("Servo_1_backward_steps");
	if (debugLevel == 1 || debugLevel == 4) Serial.println("   ");
}

void Servo_1_forward_step()
{
	if (pos < 181)
	{
		pos += 1;
		myservo.write(pos);
		delay(servoMoveSpeed);
		if (debugLevel == 2 || debugLevel == 4) mDebugMessage += "Servo_1_forward_step\n";
		if (debugLevel == 1 || debugLevel == 4) Serial.println("Servo_1_forward_step");
		if (debugLevel == 1 || debugLevel == 4) Serial.println("   ");
	}
}

void Servo_1_backward_step()
{
	if (pos > 0)
	{
		pos -= 1;
		myservo.write(pos);
		delay(servoMoveSpeed);
		if (debugLevel == 2 || debugLevel == 4) mDebugMessage += "Servo_1_backward_step\n";
		if (debugLevel == 1 || debugLevel == 4) Serial.println("Servo_1_backward_step");
		if (debugLevel == 1 || debugLevel == 4) Serial.println("   ");
	}
}

void go_forward() {
	//myServo1.writeMicroseconds(LSforward1);
	//myServo2.writeMicroseconds(RSforward1);
	Motor_1_Forward();
	Motor_2_Forward();

}

void go_forward_timed(int opcode, int dummy, int duration){
	go_forward();
	delay(duration * delayFactor);
	stop_go_forward(opcode, dummy, dummy); //wrong params
}

void go_reverse() {
	Motor_1_Backward();
	Motor_2_Backward();
}

void go_reverse_timed(int opcode, int dummy, int duration){
	go_reverse();
	delay(duration * delayFactor);
	stop_go_forward(duration, dummy, dummy); //wrong params
}

void stop_go_forward(int opcode, int dummy, int duration) {
	Motor_1_Stop();
	Motor_2_Stop();
}

void turn_left() {
	Motor_1_Backward();
	Motor_2_Forward();
}

void go_left_timed(int opcode, int dummy, int duration){
	turn_left();
	delay(duration * delayFactor);
	stop_go_forward(duration, dummy, dummy); //wrong params
}

void turn_right() {
	Motor_1_Forward();
	Motor_2_Backward();
}

void go_right_timed(int opcode, int dummy, int duration){
	turn_right();
	delay(duration * delayFactor);
	stop_go_forward(duration, dummy, dummy); //wrong params
}


/***************************************************************************************************/
/*	Robot movement functions
/***************************************************************************************************/


