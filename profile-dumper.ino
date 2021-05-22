#include <Arduino.h>
#include <SdFat.h>
#include <SPI.h>

/* This emulates the ProFile controller card; it reads all of the
 * sectors and dumps them to a flash image. It also grabs block
 * 0xFFFFFF (the drive info block). */

#define IMAGENAME "profile10.img"
#define TAGNAME "profile10.tag"
int32_t volumeSize = 0x004C00; // assumption. The ProFile10 was this size.
int16_t blockSize = 512; // assumption that there's no tag data.

#define BSYIN 25
#define TPARITYIN 26

#define PH0OUT 31
#define RRWOUT 27
#define PSTRBOUT 28
#define CMDOUT 29
#define CRESOUT 30
#define PCHKOUT 32

#define DIR 10
#define OE 11
#define DEBUGPIN 34

SdFat sd;
FsFile actualFile;
FsFile tagFile;

// This is the master "Apple" side.
//
// The DIR pin is wired to the 74LVC245s, and HIGH is A-to-B ("A" is
// the ProFile side in this circuit).
//
// The RRW signal is LOW when the Apple is writing, which
// matches... could wire these up together and save an instruction or
// three...
#define setDirectionAppleToProfile() { digitalWrite(DIR, LOW); digitalWrite(RRWOUT, LOW); }
#define setDirectionProfileToApple() { digitalWrite(DIR, HIGH); digitalWrite(RRWOUT, HIGH); }

void setup()
{
  pinMode(DEBUGPIN, OUTPUT);
  digitalWrite(DEBUGPIN, LOW);

  // Disable the data bus and set it up to read from the ProFile
  pinMode(OE, OUTPUT);
  digitalWrite(OE, HIGH);
  for (int i=2; i<=9; i++) {
    pinMode(i, OUTPUT);
  }
  pinMode(DIR, OUTPUT);
  digitalWrite(DIR, HIGH);

  // Set up control lines
  pinMode(BSYIN, INPUT);
  pinMode(TPARITYIN, INPUT);
  
  pinMode(PCHKOUT, INPUT_PULLUP); // Used to detect whether or not
				  // it's grounded, indicating the
				  // ProFile is connected
  
  pinMode(PH0OUT, OUTPUT);
  digitalWrite(PH0OUT, LOW); // I'm hoping the drive doesn't use this
			     // signal line...
  pinMode(RRWOUT, OUTPUT); // State will be updated below in busIdle()
  pinMode(PSTRBOUT, OUTPUT);
  digitalWrite(PSTRBOUT, HIGH);
  pinMode(CMDOUT, OUTPUT);
  digitalWrite(CMDOUT, HIGH);
  pinMode(CRESOUT, OUTPUT);
  digitalWrite(CRESOUT, HIGH);

  busIdle();

  Serial.begin(115200);
  delay(1000);
  Serial.println("Opening file...");

  sd.begin(SdioConfig(FIFO_SDIO));
  actualFile.open(IMAGENAME, O_CREAT|O_RDWR);
  if (actualFile) {
    Serial.println("File is open");
  } else {
    Serial.println("File open error");
  }
  tagFile.open(TAGNAME, O_CREAT|O_RDWR);
  if (tagFile) {
    Serial.println("Tag file is open");
  } else {
    Serial.println("Tag file open error");
  }

  // Engage the bus drivers
  digitalWrite(OE, LOW);

#if 0
  // Force a reset
  Serial.println("Resetting");
  digitalWrite(CRESOUT, LOW);
  delayMicroseconds(8);
  digitalWrite(CRESOUT, HIGH);
#endif
  
  Serial.println("Setup complete");
}

// This deliberately doesn't do pinMode; it's expected that writes
// will use grabBus(), writeBus(), releaseBus()
uint8_t readBusWithStrobe()
{
  digitalWrite(PSTRBOUT, LOW);
  uint8_t ret = 0;
  // pin 2 is the high bit
  for (int i=2; i<=9; i++) {
    ret <<= 1;
    if (digitalRead(i)) ret |= 1;
  }
  digitalWrite(PSTRBOUT, HIGH);
  return ret;
}

void writeBus(uint8_t v)
{
  for (int i=2; i<=9; i++) {
    digitalWrite(i, (v & 0x80) ? HIGH : LOW);
    v <<= 1;
  }
}

uint8_t writeBusWithStrobe(uint8_t v)
{
  writeBus(v);
  digitalWrite(PSTRBOUT, LOW);
  delayMicroseconds(16.2);
  digitalWrite(PSTRBOUT, HIGH);
}

void busIdle()
{
  digitalWrite(CRESOUT, HIGH);
  digitalWrite(CMDOUT, HIGH);
  digitalWrite(PSTRBOUT, HIGH);
  setDirectionProfileToApple();
  for (int i=2; i<=9; i++) {
    pinMode(i, INPUT);
    digitalWrite(i, LOW); // pull-up off
  }
}

void grabBus()
{
  setDirectionAppleToProfile();
  for (int i=2; i<9; i++) {
    pinMode(i, OUTPUT);
  }
}

void releaseBus()
{
  setDirectionProfileToApple();
  for (int i=9; i>=2; i--) {
    pinMode(i, INPUT);
    //    digitalWrite(i, LOW); // make sure pullups are off
  }
}

/*
 * On the CMD raise transition, ProFile sends a byte stating its intention.
 * If it has nothing else it was doing, it sends 01
 * If it's reading a block, 02
 * receive write data, 03
 * receive write/verify, 04
 * perform write or w/v, 06
 */
uint8_t hsmsg[6]; // handshake message
uint8_t statusMessage[4] = { 0, 0, 0, 0 };
uint32_t currentBlockNumber = 0;
uint8_t currentBlockData[532]; // 532 to allow for 20-byte tag reads

uint8_t spareTable[32];

bool readBlock(uint32_t blockNumber)
{
  busIdle();

  // [1]
  Serial.println("[1]");
  digitalWrite(CMDOUT, LOW);
  // [2] and [3]
  Serial.println("[2][3]");
  while (digitalRead(BSYIN)) ;
  Serial.println("[4]");
  uint8_t v = readBusWithStrobe();
  if (v != 0x01) { // [4]
    Serial.print("0x ");
    Serial.println(v, HEX);
    return false;
  }

  grabBus();
  Serial.println("[5]");
  writeBus(0x55); // [5]
  Serial.println("[6]");
  digitalWrite(CMDOUT, HIGH); // [6]

  // [7]
  Serial.println("[7]");
  while (!digitalRead(BSYIN)) ;

  // [8]
  Serial.println("[8]");
  writeBusWithStrobe(0x00); // read command
  // Block number (high byte first)
  Serial.print("Read block 0x");
  Serial.println(blockNumber, HEX);
  writeBusWithStrobe( (blockNumber >> 16) & 0xFF );
  writeBusWithStrobe( (blockNumber >>  8) & 0xFF );
  writeBusWithStrobe( (blockNumber      ) & 0xFF );
  // retry count
  writeBusWithStrobe(0x64); // how many times to re-read if CRC error happens
  // spares count
  writeBusWithStrobe(0x14); // limit when block should be considered bad and replaced by a spare

  // [9]
  Serial.println("[9]");
  releaseBus();
  digitalWrite(CMDOUT, LOW);

  // [10] [11]
  Serial.println("[10][11]");
  while (digitalRead(BSYIN));

  // [12]
  Serial.println("[12]");
  if (readBusWithStrobe() != 0x02) {
    busIdle();
    return false;
  }

  grabBus();
  Serial.println("[13]");
  writeBus(0x55); // [13]

  // [14]
  Serial.println("[14]");
  digitalWrite(CMDOUT, HIGH);

  Serial.println("[15]");
  while (!digitalRead(BSYIN)); // [15]
  Serial.println("[16]");
  releaseBus(); // [16]
  delayMicroseconds(16);

  Serial.print("[17]");
  uint8_t statusMessage[4];
  for (int i=0; i<4; i++) {
    statusMessage[i] = readBusWithStrobe(); // [17]
    delayMicroseconds(8);
  }
  for (int i=0; i<4; i++) {
    Serial.print(" ");
    Serial.print(statusMessage[i], HEX);
  }
  Serial.println();
  /* Status bit meanings:
   * [0]: 7 = 1 if ProFile didn't read 0x55
   *      6 = 1 if write aborted b/c too many bits sent or spares full
   *      5 = 1 if data flushed from RAM, needs retry
   *      4 = 1 if SEEK_ERROR
   *      3 = 1 if CRC error
   *      2 = 1 if TIMEOUT_ERROR (can't find sector, needs low level format)
   *      1 not used
   *      0 = 1 if operation unsuccessful
   *
   * [1]: 7 = 1 if SEEK_ERROR
   *      6 = 1 if spared sector table overflow (>32 spared)
   *      5 not used
   *      4 = 1 if bad block table overflow (> 100 bad blocks in table)
   *      3 = 1 if ProFile unable to read status sector
   *      2 = 1 if sparing occurred
   *      1 = 1 if seek to wrong track occurred
   *      0 not used
   * 
   * [2]: 7 = 1 if ProFile has been reset
   *      6 = 1 if block number invalid
   *      5 = 1 if block ID at end of sector mismatch
   *      4 not used
   *      3 not used
   *      2 = 1 if ProFile was reset
   *      1 = 1 if ProFile gave a bad response
   *      0 = 1 if parity error
   *
   * [3]: Bits 7-0: number of errors encountered when rereading a block after
   *                a read error
   */
  if (statusMessage[0] ||
      (statusMessage[1] & 0xFD) ||
      (statusMessage[2] & 0x7B)) {
    Serial.println("Status bits indicate error");
    return false;
  }

  Serial.println("[data]");
  for (int i=0; i<256; i++) {
    currentBlockData[i] = readBusWithStrobe();
    delayMicroseconds(10);
  }
  // Not sure how long the delay is between the two; guessing 100uS
  delayMicroseconds(100);
  for (int i=0; i<256; i++) {
    currentBlockData[256+i] = readBusWithStrobe();
    delayMicroseconds(10);
  }

  if (blockSize == 532) {
    // Read the 20 tag bytes too.
    delayMicroseconds(100); // again, a guess
    for (int i=0; i<20; i++) {
      currentBlockData[512+i] = readBusWithStrobe();
      delayMicroseconds(20); // another guess
    }
  }

  for (int i=0; i<blockSize; i++) {
    if ((i % 32) == 0) Serial.println();
    Serial.print(currentBlockData[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  busIdle();

  return true;
}

void flush()
{
  actualFile.close();
  actualFile.open(IMAGENAME, O_RDWR);
  tagFile.close();
  tagFile.open(TAGNAME, O_RDWR);
  Serial.println("Flushed");
}

void CopyDrive()
{
  Serial.println("Preparing to copy drive...");
  // Start by reading the status block to find out the block size
  // (in bytes) and the drive size (in blocks)
  blockSize = 512; // start with an assumption that blocks are only 512 bytes
  if (!readBlock(0xFFFFFF)) {
    Serial.println("Failed to read status block; aborting");
    return;
  }
  // Save a copy of the metadata block for later inspection
  FsFile metadataFile;
  metadataFile.open("metadata.dat", O_CREAT|O_RDWR);
  metadataFile.write(currentBlockData, 512);
  metadataFile.close();
  Serial.println("wrote block 0xFFFFFF to metadata.dat");

  // Pull out the actual drive size and block size
  volumeSize = currentBlockData[0x14];
  volumeSize <<= 8;
  volumeSize |= currentBlockData[0x13];
  volumeSize <<= 8;
  volumeSize |= currentBlockData[0x12];

  blockSize = currentBlockData[0x15];
  blockSize <<= 8;
  blockSize |= currentBlockData[0x16];

  Serial.print("Block size: 0x");
  Serial.print(blockSize, HEX);
  Serial.print("; block count: 0x");
  Serial.println(volumeSize, HEX);

  if (blockSize != 512 && blockSize != 532) {
    Serial.println("Don't know how to deal with this block size; aborting");
    return;
  }
  
  for (int i=0; i<volumeSize; i++) {
    if (readBlock(i)) {
      // Write it to the flash file @ the right position
      actualFile.seek(i * 512);
      actualFile.write(currentBlockData, 512);
      if (blockSize == 532) {
	tagFile.seek(i * 20);
	tagFile.write(&currentBlockData[512], 20);
      }
    } else {
      Serial.print("Failed to read block 0x");
      Serial.println(i, HEX);
      return;
    }
  }
  Serial.println("Flushing...");
  flush();
  Serial.println("Drive copy complete.");
}

void loop()
{
  while (!Serial.available());
  
  uint8_t r = Serial.read();
  if (r == 'r') {
    Serial.println("read");
    if (readBlock(0xFFFFFF)) {
    FsFile metadataFile;
      metadataFile.open("metadata.dat", O_CREAT|O_RDWR);
      metadataFile.write(currentBlockData, 512);
      metadataFile.close();
      Serial.println("wrote block 0xFFFFFF to metadata.dat");
    } else {
      Serial.println("Failed to read block 0xFFFFFF");
    }
  } else if (r == 't') { // testing, read one block
    blockSize = 532;
    static uint32_t bc = 0;
    readBlock(bc++);
  } else if (r == 'c') {
    CopyDrive();
  } else if (r == 'R') {
    Serial.println("reset");
    digitalWrite(CRESOUT, LOW);
    delay(100);
    digitalWrite(CRESOUT, HIGH);
    Serial.println("reset complete");
  } else if (r == 'f') {
    Serial.println("Flushing flash...");
    flush();
  }
}
