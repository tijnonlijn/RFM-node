
/* **************************************************************
* Arduino sketch
* Author: Martijn Quaedvlieg / Jan de Laet (april 2018)
* Basic sketch is generated with Generate script by Jan de Laet  http://lcd-web.nl/ttngenerator/
* Credits to Kostas Kokora for the OBD / Bluetooth logic
* Uses LMIC libary by Thomas Telkamp and Matthijs Kooijman (https://github.com/matthijskooijman/arduino-lmic)
* Pin mappings based on the 'cheapest possible node' https://www.thethingsnetwork.org/labs/story/build-the-cheapest-possible-node-yourself
* *************************************************************/
#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SoftwareSerial.h>

// define the activation method ABP or OTAA
#define ACT_METHOD_ABP

// show debug statements; comment next line to disable debug statements
#define DEBUG

/* **************************************************************
* OBD specific declarations
* *************************************************************/

#define RxD 2      //!          //Arduino pin connected to Tx of HC-05
#define TxD 3      //!          //Arduino pin connected to Rx of HC-05
#define PIO11 9   //!          //Arduino pin connected to PI011 of HC-05 (enter AT Mode with HIGH)
#define HC_VCC 8     //!			// Arduino pin connected to the HC-05 VCC (needed to reset the AT mode)
#define BT_CMD_RETRIES 5     //Number of retries for each Bluetooth AT command in case of not responde with OK
#define OBD_CMD_RETRIES 3    //Number of retries for each OBD command in case of not receive prompt '>' char
#define RPM_CMD_RETRIES 5    //Number of retries for RPM command

int addr = 0;                  //EEPROM address for storing Shift Light RPM
unsigned int rpm, rpm_to_disp;//Variables for RPM
boolean bt_error_flag;       //Variable for bluetooth connection error
boolean obd_error_flag;      //Variable for OBD connection error
boolean rpm_error_flag;      //Variable for RPM error
boolean rpm_retries;         //Variable for RPM cmd retries

SoftwareSerial blueToothSerial(RxD, TxD);

/* **************************************************************
* keys for device
* *************************************************************/

//OBD_BLUETOOTH_GW
static const uint8_t PROGMEM NWKSKEY[16] = { 0xF6, 0xF0, 0xA1, 0x76, 0x48, 0x3F, 0x09, 0x16, 0x49, 0x7B, 0x1F, 0x57, 0x28, 0x2C, 0x2E, 0xD4 };
static const uint8_t PROGMEM APPSKEY[16] = { 0xFC, 0x47, 0x16, 0xEA, 0x99, 0x1C, 0x96, 0x15, 0xDB, 0x53, 0x51, 0x12, 0x71, 0x7D, 0xB6, 0x5A };
static const uint32_t DEVADDR = 0x260116BB;


// Declare the job control structures
static osjob_t sendjob;

// These callbacks are only used in over-the-air activation, so they are
// left empty when ABP (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
#ifdef ACT_METHOD_ABP
void os_getArtEui(u1_t* buf) { }
void os_getDevEui(u1_t* buf) { }
void os_getDevKey(u1_t* buf) { }
#else
void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }
#endif

/* **************************************************************
* Pin mapping
* *************************************************************/

// Doug LaRue
const lmic_pinmap lmic_pins = {
	.nss = 10, // Connected to pin D10
	.rxtx = 0, // For placeholder only, Do not connected on RFM92/RFM95
	.rst = 0, // Needed on RFM92/RFM95? (probably not)
	.dio = { 4, 5, 7 }, // Specify pin numbers for DIO0, 1, 2
						// connected to D4, D5, D7 
};

//Cheapest possible node
/*const lmic_pinmap lmic_pins = {
	.nss = 6,
	.rxtx = LMIC_UNUSED_PIN,
	.rst = 5,
	.dio = { 2, 3, LMIC_UNUSED_PIN },
};*/



// TTNmapper settings

// data to send
static uint8_t dataTX[4];   // MQ 20180401: Changed 1 -> 4  

/* **************************************************************
* user settings
* *************************************************************/
unsigned long starttime;
unsigned long cycle_length = 15 * 1 * 1000UL; // cycle * mins_or_secs * 1000;
											  
/* **************************************************************
 * setup
* *************************************************************/
void setup() {

	


	// Wait (max 10 seconds) for the Serial Monitor
	while ((!Serial) && (millis() < 10000)) {}

	//Set baud rate
	Serial.begin(9600);
	
	Serial.println("TTN OBD bluetooth gateway v0.07 - Start");
	pinMode(RxD, INPUT);
	pinMode(TxD, OUTPUT);
	pinMode(PIO11, OUTPUT);
	pinMode(HC_VCC, OUTPUT);

	rpm_retries = 0;

	//start Bluetooth Connection
	init_BT_conn();

	//in case of Bluetoth connection error
	if (bt_error_flag) {
		bt_err_flash();   // currently empty 
	}


	//OBDII initialization
	init_obd();

	//in case of OBDII connection error   
	if (obd_error_flag) {
		obd_err_flash(); // currently empty 
		Serial.print("Error flag (OBD):  ");
		Serial.println(obd_error_flag);
	}
	else
	{
		read_rpm();
	}


	init_node();


	starttime = millis();
}



//-----------------------------------------------------//
//---------------flashes fast red light----------------//
//-------in case of Bluetooth connection error---------//
//------loops for ever, need to restart Arduino--------//
void bt_err_flash() {

	/*	while (1) {
	digitalWrite(ledpin_red, HIGH);
	delay(100);
	digitalWrite(ledpin_red, LOW);
	delay(100);
	}
	*/
}

//-----------------------------------------------------//
//---------------flashes slow red light----------------//
//---------in case of OBDII connection error-----------//
//------loops for ever, need to restart Arduino--------//
void obd_err_flash() {
	/*
	while (1) {
	digitalWrite(ledpin_red, HIGH);
	delay(1000);
	digitalWrite(ledpin_red, LOW);
	delay(1000);
	}
	*/
}


//-----------------------------------------------------//
//----------Retrieve RPM value from OBD----------------//
//---------convert it to readable number---------------//
void read_rpm() {
	boolean prompt, valid;
	char recvChar;
	char bufin[15];
	int i;

	Serial.println("Retrieving RPM");
	delay(100);

	if (!(obd_error_flag)) {                                   //if no OBD connection error

		valid = false;
		prompt = false;
		blueToothSerial.write("010C1");                        //send to OBD PID command 010C is for RPM, the last 1 is for ELM to wait just for 1 respond (see ELM datasheet)
		blueToothSerial.write("\r\n");                           //send to OBD cariage return char
		while (blueToothSerial.available() <= 0);              //wait while no data from ELM
		i = 0;
		while ((blueToothSerial.available()>0) && (!prompt)) {  //if there is data from ELM and prompt is false
			recvChar = blueToothSerial.read();                   //read from ELM

			Serial.print(recvChar);

			if ((i<15) && (!(recvChar == 32))) {                     //the normal respond to previus command is 010C1/r41 0C ?? ??>, so count 15 chars and ignore char 32 which is space
				bufin[i] = recvChar;                                 //put received char in bufin array
				i = i + 1;                                             //increase i
			}
			if (recvChar == 62) prompt = true;                       //if received char is 62 which is '>' then prompt is true, which means that ELM response is finished 
		}


		if ((bufin[0] == '4') && (bufin[1] == '1') && (bufin[2] == '0') && (bufin[3] == 'C')) { //if first four chars after our command is 410C
			valid = true;                                                                  //then we have a correct RPM response
		}
		else {
			valid = false;                                                                 //else we dont
		}


		if (valid) {                                                                    //in case of correct RPM response
			rpm_retries = 0;                                                               //reset to 0 retries
			rpm_error_flag = false;                                                        //set rpm error flag to false

																						   //start calculation of real RPM value
																						   //RPM is coming from OBD in two 8bit(bytes) hex numbers for example A=0B and B=6C
																						   //the equation is ((A * 256) + B) / 4, so 0B=11 and 6C=108
																						   //so rpm=((11 * 256) + 108) / 4 = 731 a normal idle car engine rpm
			rpm = 0;

			for (i = 4;i<8;i++) {                              //in that 4 chars of bufin array which is the RPM value
				if ((bufin[i] >= 'A') && (bufin[i] <= 'F')) {        //if char is between 'A' and 'F'
					bufin[i] -= 55;                                 //'A' is int 65 minus 55 gives 10 which is int value for hex A
				}

				if ((bufin[i] >= '0') && (bufin[i] <= '9')) {        //if char is between '0' and '9'
					bufin[i] -= 48;                                 //'0' is int 48 minus 48 gives 0 same as hex
				}

				rpm = (rpm << 4) | (bufin[i] & 0xf);              //shift left rpm 4 bits and add the 4 bits of new char

			}
			rpm = rpm >> 2;                                     //finaly shift right rpm 2 bits, rpm=rpm/4
		}

	}
	if (!valid) {                                              //in case of incorrect RPM response
		rpm_error_flag = true;                                    //set rpm error flag to true
		rpm_retries += 1;                                         //add 1 retry
		rpm = 0;                                                  //set rpm to 0
																  //Serial.println("RPM_ERROR");
		if (rpm_retries >= RPM_CMD_RETRIES) obd_error_flag = true;  //if retries reached RPM_CMD_RETRIES limit then set obd error flag to true
	}
	
	Serial.println("");
	Serial.print("RPM retrieved: ");
	Serial.println(rpm);
	delay(100);
}

//----------------------------------------------------------//
//---------------------Send OBD Command---------------------//
//--------------------for initialitation--------------------//

void send_OBD_cmd(char *obd_cmd) {
	char recvChar;
	boolean prompt;
	int retries;

	Serial.print("Executing obd cmd> ");
	Serial.println(obd_cmd);

	if (!(obd_error_flag)) {                                        //if no OBD connection error

		Serial.println("Ok");
		prompt = false;
		retries = 0;
		while ((!prompt) && (retries<OBD_CMD_RETRIES)) {                //while no prompt and not reached OBD cmd retries
																		//Serial.print(".");
			blueToothSerial.write(obd_cmd);                             //send OBD cmd
			blueToothSerial.write("\r\n");                                //send cariage return

			while (blueToothSerial.available() <= 0);                   //wait while no data from ELM

			while ((blueToothSerial.available()>0) && (!prompt)) {       //while there is data and not prompt
				recvChar = blueToothSerial.read();                        //read from elm
				if (recvChar == 62) prompt = true;                            //if received char is '>' then prompt is true
			}
			retries = retries + 1;                                          //increase retries
			delay(2000);
		}
		if (retries >= OBD_CMD_RETRIES) {                               // if OBD cmd retries reached
			obd_error_flag = true;                                        // obd error flag is true
			Serial.println("obd error flag set to true");
		}
	}
}

//----------------------------------------------------------//
//----------------initialitation of OBDII-------------------//
void init_obd() {

//	Serial.println("obd init start");
	obd_error_flag = false;     // obd error flag is false

	send_OBD_cmd("ATZ");      //send to OBD ATZ, reset
	delay(1000);
	send_OBD_cmd("ATSP0");    //send ATSP0, protocol auto

	send_OBD_cmd("0100");     //send 0100, retrieve available pid's 00-19
	delay(100);
	send_OBD_cmd("0120");     //send 0120, retrieve available pid's 20-39
	delay(100);
	send_OBD_cmd("0140");     //send 0140, retrieve available pid's 40-??
	delay(100);
	send_OBD_cmd("010C1");    //send 010C1, RPM cmd
	delay(100);
	Serial.println("obd init end");
	delay(100);
}

//----------------------------------------------------------//
//-----------start of bluetooth connection------------------//
void init_BT_conn()
{

	bt_error_flag = false;                    //set bluetooth error flag to false

	enterATMode();                          //enter HC-05 AT mode
	delay(500);



	sendATCommand("RESET");                  //send to HC-05 RESET
	delay(1000);
	sendATCommand("ORGL");                   //send ORGL, reset to original properties
	sendATCommand("ROLE=1");                 //send ROLE=1, set role to master
	sendATCommand("CMODE=0");                //send CMODE=0, set connection mode to specific address
	sendATCommand("BIND=001D,A5,68988A");    //send BIND=??, bind HC-05 to OBD bluetooth address
	sendATCommand("INIT");                   //send INIT, cant connect without this cmd 
	delay(1000);
	sendATCommand("PAIR=001D,A5,68988A,20"); //send PAIR, pair with OBD address
	delay(1000);
	sendATCommand("LINK=001D,A5,68988A");    //send LINK, link with OBD address
	delay(1000);

	enterComMode();                          //enter HC-05 comunication mode
	delay(500);
}


//----------------------------------------------------------//
//--------Enter HC-05 bluetooth module command mode---------//
//-------------set HC-05 mode pin to LOW--------------------//
void enterComMode()
{
	//Serial.println("Leaving AT Mode");
	//Serial.println("Initializing com mode");
	blueToothSerial.flush();
	delay(500);
	digitalWrite(PIO11, LOW);

	delay(500);

	digitalWrite(HC_VCC, HIGH);

	delay(500);

	blueToothSerial.begin(38400); //default communication baud rate of HC-05 is 38400
}

//----------------------------------------------------------//
//----------Enter HC-05 bluetooth moduel AT mode------------//
//-------------set HC-05 mode pin to HIGH--------------------//
void enterATMode()
{
	Serial.println("Initializing AT mode");

	blueToothSerial.flush();
	delay(500);

	// The code below is for HC-05 modules with an 'EN' pin. They lack the reset pin and needs to be reset using the code below
	// HC-05 VCC to low, then EN pin to high, and finally HC-05 VCC back to high. The module remains in AT mode until the EN pin is set to LOW again
	digitalWrite(HC_VCC, LOW);
	digitalWrite(PIO11, LOW);
	delay(500);
	Serial.println("Shutting down HC-05");

	digitalWrite(PIO11, HIGH);
//	Serial.println("Set EN pin to HIGH");

	delay(500);
	digitalWrite(HC_VCC, HIGH);
//	Serial.println("Set HC VCC pin to HIGH");
	delay(500);
	blueToothSerial.begin(38400);//HC-05 AT mode baud rate is 38400

}

//----------------------------------------------------------//

void sendATCommand(char *command)
{

	Serial.print("Executing AT command> ");
	Serial.println(command);
	char recvChar;
	char str[2];   // MQ 20180401: should be increased. Sufficient for 'ok', but too small for actual errors. Ignored for now
	int i, retries;
	boolean OK_flag;

	if (!(bt_error_flag)) {                                  //if no bluetooth connection error
		retries = 0;
		OK_flag = false;

		while ((retries < BT_CMD_RETRIES) && (!(OK_flag))) {     //while not OK and bluetooth cmd retries not reached

			blueToothSerial.write("AT");                       //sent AT cmd to HC-05
			if (strlen(command) > 1) {
				blueToothSerial.write("+");
				blueToothSerial.write(command);
			}

			blueToothSerial.write("\r\n");

			while (blueToothSerial.available() <= 0);              //wait while no data

			i = 0;
			while (blueToothSerial.available() > 0) {               // while data is available
				recvChar = blueToothSerial.read();                 //read data from HC-05
				if (i < 2) {
					str[i] = recvChar;                               //put received char to str
					i = i + 1;
				}
			}
			retries = retries + 1;                                  //increase retries 
			if ((str[0] == 'O') && (str[1] == 'K')) {
				OK_flag = true;   //if response is OK then OK-flag set to true
				Serial.println("OK");
			}
			else
			{
				Serial.print("nok: ");
				Serial.println(str);
			}
			delay(1000);

		}


		if (retries >= BT_CMD_RETRIES) {                        //if bluetooth retries reached
			bt_error_flag = true;                                 //set bluetooth error flag to true
			Serial.println("Bluetooth error - timeout occured");
		}
	}


}







/* **************************************************************
* loop
* *************************************************************/
void loop() {

	// check if need to send
	Serial.print("*");
	delay("1000");
	if ((millis() - starttime) > cycle_length) 
	{ 
		Serial.println("Entering loop");
		delay(100);
		read_rpm();
		build_data(); 
		do_send(); 
		starttime = millis(); 
	}

}


/* **************************************************************
* build data to transmit in dataTX
*
* Suggested payload function for this data
*
* if (bytes[0] >= 0x20) {
*   str = '';
*   for (var i = 0; i < bytes.length; i += 1) str += String.fromCharCode(bytes[i]);
*   return { payload: str };
* }
*
* *************************************************************/
void build_data() {

	dataTX[0] = 0x01;     //first byte is send as 01 to recognise this is a distance in our dashboard // tmp workaround since the dashboard does not know rpm
	dataTX[1] = rpm >> 8;
	dataTX[2] = rpm & 0xFF;
}

/* **************************************************************
* radio code, typical would be init_node(), do_send(), etc
* *************************************************************/
/* **************************************************************
* init the Node
* *************************************************************/
void init_node() {
#ifdef VCC_ENABLE
	// For Pinoccio Scout boards
	pinMode(VCC_ENABLE, OUTPUT);
	digitalWrite(VCC_ENABLE, HIGH);
	delay(1000);
#endif

	// LMIC init
	os_init();
	// Reset the MAC state. Session and pending data transfers will be discarded.
	LMIC_reset();

#ifdef ACT_METHOD_ABP
	// Set static session parameters. Instead of dynamically establishing a session
	// by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
	// On AVR, these values are stored in flash and only copied to RAM
	// once. Copy them to a temporary buffer here, LMIC_setSession will
	// copy them into a buffer of its own again.
	uint8_t appskey[sizeof(APPSKEY)];
	uint8_t nwkskey[sizeof(NWKSKEY)];
	memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
	memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
	LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);
#else
	// If not running an AVR with PROGMEM, just use the arrays directly
	LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
	// Set up the channels used by the Things Network, which corresponds
	// to the defaults of most gateways. Without this, only three base
	// channels from the LoRaWAN specification are used, which certainly
	// works, so it is good for debugging, but can overload those
	// frequencies, so be sure to configure the full frequency range of
	// your network here (unless your network autoconfigures them).
	// Setting up channels should happen after LMIC_setSession, as that
	// configures the minimal channel set.
	// NA-US channels 0-71 are configured automatically
	LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);      // g-band
	LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
	LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);      // g-band
	LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);      // g-band
	LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);      // g-band
	LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);      // g-band
	LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);      // g-band
	LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);      // g-band
	LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);      // g2-band
																					// TTN defines an additional channel at 869.525Mhz using SF9 for class B
																					// devices' ping slots. LMIC does not have an easy way to define set this
																					// frequency and support for class B is spotty and untested, so this
																					// frequency is not configured here.
#elif defined(CFG_us915)
	// NA-US channels 0-71 are configured automatically
	// but only one group of 8 should (a subband) should be active
	// TTN recommends the second sub band, 1 in a zero based count.
	// https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
	LMIC_selectSubBand(1);
#endif

	// Disable link check validation
	LMIC_setLinkCheckMode(0);

	// TTN uses SF9 for its RX2 window.
	LMIC.dn2Dr = DR_SF9;

	// Set data rate and transmit power (note: txpow seems to be ignored by the library)
	LMIC_setDrTxpow(DR_SF7, 14);
#endif

#ifdef ACT_METHOD_OTAA
	// got this fix from forum: https://www.thethingsnetwork.org/forum/t/over-the-air-activation-otaa-with-lmic/1921/36
	LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
#endif

}

/* **************************************************************
* send the message
* *************************************************************/
void do_send() {

	Serial.print(millis());
	Serial.print(F(" Sending.. "));

	send_message(&sendjob);

	// wait for send to complete
	Serial.print(millis());
	Serial.print(F(" Waiting.. "));

	while ((LMIC.opmode & OP_JOINING) or (LMIC.opmode & OP_TXRXPEND)) { os_runloop_once(); }
	Serial.print(millis());
	Serial.println(F(" TX_COMPLETE"));
}

/* *****************************************************************************
* send_message
* ****************************************************************************/
void send_message(osjob_t* j) {
	// Check if there is not a current TX/RX job running
	if (LMIC.opmode & OP_TXRXPEND) {
		Serial.println(F("OP_TXRXPEND, not sending"));
	}
	else {
		// Prepare upstream data transmission at the next possible time.
		LMIC_setTxData2(1, dataTX, sizeof(dataTX), 0);
		Serial.println(F("Packet queued"));
	}
}

/*******************************************************************************/
void onEvent(ev_t ev) {
	switch (ev) {
	case EV_SCAN_TIMEOUT:
		Serial.println(F("EV_SCAN_TIMEOUT"));
		break;
	case EV_BEACON_FOUND:
		Serial.println(F("EV_BEACON_FOUND"));
		break;
	case EV_BEACON_MISSED:
		Serial.println(F("EV_BEACON_MISSED"));
		break;
	case EV_BEACON_TRACKED:
		Serial.println(F("EV_BEACON_TRACKED"));
		break;
	case EV_JOINING:
		Serial.println(F("EV_JOINING"));
		break;
	case EV_JOINED:
		Serial.println(F("EV_JOINED"));
		// Disable link check validation (automatically enabled
		// during join, but not supported by TTN at this time).
		LMIC_setLinkCheckMode(0);
		break;
	case EV_RFU1:
		Serial.println(F("EV_RFU1"));
		break;
	case EV_JOIN_FAILED:
		Serial.println(F("EV_JOIN_FAILED"));
		break;
	case EV_REJOIN_FAILED:
		Serial.println(F("EV_REJOIN_FAILED"));
		break;
	case EV_TXCOMPLETE:
		Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
		if (LMIC.dataLen) {
			// data received in rx slot after tx
			Serial.print(F("Data Received: "));
			Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
			Serial.println();
		}
		// schedule next transmission
		// os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), send_message);
		break;
	case EV_LOST_TSYNC:
		Serial.println(F("EV_LOST_TSYNC"));
		break;
	case EV_RESET:
		Serial.println(F("EV_RESET"));
		break;
	case EV_RXCOMPLETE:
		// data received in ping slot
		Serial.println(F("EV_RXCOMPLETE"));
		break;
	case EV_LINK_DEAD:
		Serial.println(F("EV_LINK_DEAD"));
		break;
	case EV_LINK_ALIVE:
		Serial.println(F("EV_LINK_ALIVE"));
		break;
	default:
		Serial.println(F("Unknown event"));
		break;
	}

}


