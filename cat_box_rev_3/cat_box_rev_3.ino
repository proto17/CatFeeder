#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>

//#define DEBUG 1

// Tags for each cat
#define TAGS_CHARLIE 0x9a, 0xA8, 0x13, 0xC5
#define TAGS_PEPPERMINT 0xba, 0x30, 0xd4, 0xb5

// Tag for the cat that is able to open the box
const byte correctCat[] = {TAGS_PEPPERMINT};

// Digital IO pin that drives the servo (must be PWM capable)
const byte servoOutput = 6;

// Angle of the servo when the door is open
// Pepper needs 15
// Charlie needs 160
const byte servoOpenValue = 15;

// Angle of the servo when the door is closed
// Pepper needs 140
// Charlie needs 0
const byte servoClosedValue = 140;

// There is no readers.length to tell how many readers there are
// so set the number here.
const byte numberOfReaders = 2;

// Create an array of MFRC522 objects that will control each of the
// RFID readers.  Using the numberOfReaders variable here to ensure
// that the numberOfReaders variable contains the correct value.  If
// the value of numberOfReaders and the number of elements in the array
// do not match, then the compiler will error out.
MFRC522 readers[numberOfReaders] = {MFRC522(10, 9), MFRC522(3,2)};

// Controls the servo.  Will be initialized in setup().
Servo servo;

// Holds on to the current gain setting.  Used to know which gain setting
// to go to next.
MFRC522::PCD_RxGain gain = MFRC522::RxGain_min;

// How many milliseconds the door can stay open without there being
// a valid RFID tag read.
long doorTimeout = 15L * 1000L;


// Setup method for Arduino
void setup() {

	// Only initialize the serial connection if debug is enabled
	#ifdef DEBUG
	Serial.begin(9600);
	while(!Serial);
	#endif
	
	// Start the SPI controller (needed to control the MFRC522s)
	SPI.begin();

	// Go over each reader, turn it on, set the gain, and turn
	// the antenna off.
	// TODO: This can probably be removed now that each reader is 
	//       actually powered down after an attempt to read.
	for(byte a = 0; a < numberOfReaders; a++){
		readers[a].PCD_Init();
		readers[a].PCD_SetAntennaGain(gain);
		readers[a].PCD_AntennaOff();
	}

	// Attach the servo that controls the door
	servo.attach(servoOutput);
	// Close the door (this is the default state)
	servo.write(servoClosedValue);

	// Show information about the readers if debug mode is on.
	// Useful for being able to tell if all the readers are properly
	// connected.
	#ifdef DEBUG
	ShowReaderDetails();
	#endif
}

// Function that checks the UID read by the reader against the 
// value stored in correctCat.
// Returns true if the UID matches the value of correctCat
// Returns false otherwise
bool isCorrectCat(MFRC522::Uid * uid){
	if(uid->size < 4){
		return false;
	}
	
	// Print the UID of the tag out if debug is on
	#ifdef DEBUG
	print("UID: ");
	for(unsigned char a = 0; a < uid->size; a++){
		printHex(uid->uidByte[a]);
	}
	println("");
	#endif

	// Loop over each byte of the UID and compare it to
	// the value of correctCat.  The first time a non matching
	// byte is found, return false (fast fail)
	for(unsigned char a = 0; a < uid->size; a++){
		if(correctCat[a] != uid->uidByte[a]){
			return false;
		}
	}
	
	// All of the bytes matched, so return true
	return true;
}

// Keeps track of whether or not the correct tag was found
bool found = false;

// Keeps track of the last time (in milliseconds) that the correct
// tag was read
long lastFound = 0l;

// Keeps track of whether or not the door is currently open
bool doorIsOpen = false;

// Just used to hold on to the current value of millis() so that
// the function does not need to be called a lot
long currentMillis = 0L;

// Used to tell if it's time to change the gain
unsigned short iterationNumber = 0;

// Main method of the program.
//
// Constantly tries to find the correct tag.  When the correct
// tag is found, the door is opened.  The door will remain open
// so long as the correct tag can still be read, or after doorTimeout
// is reached.
//
// If the tag cannot be found, then when iterationNumber reaches
// a certain value (in the loop() method) then the gain is changed.
// This is done because when max gain is set and the tag is very close
// to, or touching, the reader, it will not be read.  So, to still 
// allow the door to open when the tag is an inch or two away as well
// as when the tag is against the reader, the gain is changed every
// so often.
//
// Additionally, since the readers are right next to one another, they
// cannot be on at the same time.  Just turning the antenna off is not
// enough to prevent the readers from interfering with one another.
// The currently working solution is to only power on the reader that
// is being polled.  The others will be in a powerdown state.  
// In order to come back from power on, each reader needs to be initialized
// as well as having the correct gain set.
//
// In order to power the readers down, the MFRC522 library had to be modified.
// The only modification was the addition of a new public method in MFRC522.cpp:
//   byte getPowerDownPin() const { return _resetPowerDownPin; }
//
// The _resetPowerDownPin is not accessible by default and is needed in order
// to power the reader down
void loop() {
	// Always assume the correct tag has not been found
	found = false;

	// Loop through each of the readers
	for(unsigned char a = 0; a < numberOfReaders; a++){
		// Power the reader back on
		digitalWrite(readers[a].getPowerDownPin(), HIGH);
		// Initialize the reader
		readers[a].PCD_Init();
		// Set the gain of the reader
		readers[a].PCD_SetAntennaGain(gain);
		// Turn the reader's antenna on
		// TODO: This might not be required
		readers[a].PCD_AntennaOn();

		// Check to see if a tag could be read (this returns true if ANY tag is read)
		if(readers[a].PICC_IsNewCardPresent()){
			// Some debugging output
			print("Found a new card at reader #");
			println(a);

			// Check to see if the UID of the tag could be read.  This isn't always
			// possible.  The tag could have been pulled away too quickly.
			if(readers[a].PICC_ReadCardSerial()){

				// Print out some more debugging info
				print("Read the serial number on reader #");
				println(a);
				
				// Check to see if the UID read is the correct one
				if(isCorrectCat(&readers[a].uid)){

					// Print some debugging info
					print("Found the correct cat on reader #");
					println(a);

					// The correct tag was read so set found to true
					found = true;

					// TODO: There should probably be a 'break' here since there is no reason
					//       to continue checking the other readers...
				}else{

					// This was not the correct tag, so print that info for debugging
					print("Did not find the correct cat on reader #");
					println(a);
				}
			}else{

				// The UID could not be read, so print that info for debugging
				print("Could not read the card serial number on reader #");
				println(a);
			}

			// There is no reason to print the fact that no card could be read.
			// It just spams the serial port and slows the program down.
		}

		// Turn the antenna off
		// TODO: This might not be required since the reader will be powered down
		readers[a].PCD_AntennaOff();

		// Power down the reader
		digitalWrite(readers[a].getPowerDownPin(), LOW);
	}
	
	// The millis() function will eventually roll over back to 0.  It takes a while,
	// but it will happen and it does need to be handled.  So, any time that the value
	// of millis() is less than the last value of millis() (which is stored in
	// currentMillis) then a roll over happened.  In that case, update the value of
	// currentMillis, print a debugging message, and bail out.
	if(millis() < currentMillis){
		currentMillis = millis();
		println("Millis rolled over!");
		return;
	}

	// Grab the current millis() value now so that it does not need to be called 
	// twice later on.
	currentMillis = millis();

	// Check to see if found was false (no valid tag was read)
	if(found == false){
		// Check to see if the current iteration number is at the max value.  Notice
		// that a pre-increment is used here.  A post increment can be used too.
		//
		// This is where the gain is changed.  When the iterationNumber reaches a 
		// certain max value, then the gain is adjusted.  Simply adjusting the gain
		// on every single pass can cause the program to slow down.  But, when using
		// multiple readers, this might need to happen every pass in order for the
		// gain changes to be fast enough to pick up a tag that is either too close
		// or too far away.
		if(++iterationNumber == 1){
			// Reset the iterationNumber value
			iterationNumber = 0;

			// Output a debugging message to show that the gain is about to change
			print("Switching gain from ");
			printHex(gain);

			// Determine what the current gain setting is, and change it to either 
			// one level higher (can read from farther away) or reset it back to the
			// minimum (18dB) if the current value is the max (48dB).
			switch(gain){
				case MFRC522::RxGain_18dB:
					gain = MFRC522::RxGain_23dB;
					break;
				case MFRC522::RxGain_23dB:
					gain = MFRC522::RxGain_33dB;
					break;
				case MFRC522::RxGain_33dB:
					gain = MFRC522::RxGain_38dB;
					break;
				case MFRC522::RxGain_38dB:
					gain = MFRC522::RxGain_43dB;
					break;
				case MFRC522::RxGain_43dB:
					gain = MFRC522::RxGain_48dB;
					break;
				case MFRC522::RxGain_48dB:
					gain = MFRC522::RxGain_18dB;
					break;
			}

			// Print some debugging info to show what the new gain setting is
			print(" to ");
			printHex(gain);
			println("");
			
			// TODO: This might not be needed anymore since all the readers are off
			//       at this point.
			for(byte a = 0; a < numberOfReaders; a++){
				readers[a].PCD_SetAntennaGain(gain);
				readers[a].PCD_AntennaOff();
			}
		}

		// No need to check millis if the door is already closed and there was no
		// valid tag read.
		if(doorIsOpen == false){
			return;
		}
		
		// Since the door is open and no valid tag was read, check to see if the
		// amount of time since a valid tag was read has exceeded the value of 
		// doorTimeout.
		if(currentMillis - lastFound > doorTimeout){
			// The timeout has been exceeded, so close the door.

			// Print some debugging info
			println("Closing the door");
			
			// Move the servo to the closed position
			servo.write(servoClosedValue);

			// Set the open flag to false so that the program knows the door is currently closed.
			doorIsOpen = false;
		}
	}else{

		// Tag was found, so update lastFound and open the door if needed
		lastFound = currentMillis;

		// Reset the iteration number so that the gain does not suddenly change.
		// This is important because since a valid tag was read, the reader needs
		// to stay on the gain in order to read the card the most often.
		iterationNumber = 0;

		// The correct tag was read, now check to see if the door is currently
		// closed.  If so, open it and set the flag.
		if(doorIsOpen == false){
			doorIsOpen = true;
			println("Opening the door");
			servo.write(servoOpenValue);
		}
	}
}

void println(const char * str){
	#ifdef DEBUG
	Serial.println(str);
	#endif
}

void println(int a){
	#ifdef DEBUG
	Serial.println(a);
	#endif
}

void print(const char * str){
	#ifdef DEBUG
	Serial.print(str);
	#endif
}

void print(int a){
	#ifdef DEBUG
	Serial.print(a);
	#endif
}

void printHex(int a){
	#ifdef DEBUG
	Serial.print(a, HEX);
	#endif
}


void ShowReaderDetails() {
	// Get the MFRC522 software version
	for(byte a = 0; a < numberOfReaders; a++){
		byte v = readers[a].PCD_ReadRegister(readers[a].VersionReg);
		Serial.print(F("MFRC522 Software Version: 0x"));
		Serial.print(v, HEX);
		if (v == 0x91)
			Serial.print(F(" = v1.0"));
		else if (v == 0x92)
			Serial.print(F(" = v2.0"));
		else
			Serial.print(F(" (unknown)"));
		Serial.println("");
		// When 0x00 or 0xFF is returned, communication probably failed
		if ((v == 0x00) || (v == 0xFF)) {
			Serial.println(F("WARNING: Communication failure, is the MFRC522 properly connected?"));
		}
	}
} 
