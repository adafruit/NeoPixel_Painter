// Adafruit NeoPixel Light Painter sketch:
// Reads 24-bit BMP image from SD card, plays back on 4 NeoPixel
// strips arranged in a continuous line.

// Requires a momentary pushbutton connected between pin A1 and GND
// to trigger playback.  An analog pot (e.g. 10K) connected to A0
// sets the brightness at startup and the playback speed each time
// the trigger is tapped.  BRIGHTNESS IS SET ONLY AT STARTUP; can't
// adjust this in realtime, not fast enough.  To change brightness,
// set dial & tap reset.  Adjustment takes place during conversion.
// Then set dial for playback speed.

// This is a 'plain vanilla' example, with no UI or anything -- it
// always reads a fixed file at startup (paint.bmp, in root folder),
// outputs paint.tmp (warning: doesn't ask, just overwrites it),
// then plays back from this file each time a button is tapped (or
// loops repeatedly if held).  Core pieces are here for developing
// more advanced cases: applications could add a UI (e.g. 16x2 LCD
// shield), process multiple files for animation, or use a rotary
// encoder for absolute positioning.  None of that is here though,
// you will need to get clever and rip up some of this code.

// Implementation is closely tailored to the Arduino Uno or similar
// boards; this WILL NOT WORK with 'soft' SPI on the Arduino Mega,
// and the use of AVR-specific registers means this won't work on
// the Due.  On a good day it could perhaps be adapted to Teensy 2.0
// or other AVR Arduino-type boards that support SD cards and can
// meet the pin requirements.  Easiest by far just to use an Uno.

#include <SD.h>
#include <avr/pgmspace.h>

// CONFIGURABLE STUFF ----------------------------------------------

#define N_LEDS      144 // MUST be multiple of 4, max value of 168
#define CARD_SELECT  10 // SD card select pin
#define SPEED        A0 // Speed-setting dial
#define BRIGHTNESS   A0 // Brightness-setting dial
#define TRIGGER      A1 // Playback trigger pin

const uint8_t PROGMEM ledPin[4] = { 4, 5, 6, 7 };
// Display is split into four equal segments, each connected to a
// different pin (listed in above array).  The 4 pins MUST be on the
// same PORT register.  They don't necessarily need to be adjacent,
// but the requirements there are quite complex to explain -- it's
// easiest just to use 4 contiguous bits on the port (e.g. on
// Arduino Uno, pins 4-7 correspond to PORTD bits 4-7).  Segments
// are placed left-to-right, with the first two segments reversed
// (first pixel at right) in order to minimize wire lengths (Arduino
// is positioned in middle of four segments).
// <---Strip 1---< <---Strip 2---< >---Strip 3---> >---Strip 4--->
//         Pin 4 ^         Pin 5 ^ ^ Pin 6         ^ Pin 7
// If the image is backwards, you don't need to re-wire anything,
// just flip the bar over.


// NON-CONFIGURABLE STUFF ------------------------------------------

#define OVERHEAD 150 // Extra microseconds for loop processing, etc.

const uint8_t gamma[] PROGMEM = { // Brightness ramp for LEDs
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };

volatile uint8_t  ledBuf[N_LEDS * 3], // Data for NeoPixels
                 *ledPort;            // PORT register for all LEDs
uint8_t           ledPortMask,        // PORT bitmask for all LEDs
                  ledMaskA[4],        // Bitmask for each pin, LED bits 7, 5, 3, 1
                  ledMaskB[4];        // Bitmask for each pin, LED bits 6, 4, 2, 0
Sd2Card           card;               // SD card global instance (only one)
SdVolume          volume;             // Filesystem global instance (only one)
SdFile            root;               // Root directory (only one)
uint16_t          maxLPS;             // Max playback lines/sec
uint32_t          firstBlock,         // First block # in working (temp) file
                  nBlocks;            // Number of blocks in file


// INITIALIZATION --------------------------------------------------

void setup() {
  uint8_t i, p;

  Serial.begin(57600);

  for(ledPortMask = i = 0; i<4; i++) {     // NeoPixel pin setup:
    p = pgm_read_byte(&ledPin[i]);         // Arduino pin number
    pinMode(p, OUTPUT);                    // Set as output
    digitalWrite(p, LOW);                  // Output low
    ledMaskA[i]  = digitalPinToBitMask(p); // Bitmask for this pin
    ledMaskB[i]  = (ledMaskA[i] << 4) | (ledMaskA[i] >> 4);
    ledPortMask |= ledMaskA[i];            // Cumulative mask, all pins
  }
  ledPort = portOutputRegister(digitalPinToPort(p));

  Serial.print(F("Initializing SD card..."));
  pinMode(CARD_SELECT, OUTPUT);
  if(!card.init(SPI_FULL_SPEED, CARD_SELECT)) {
    error(F("failed. Things to check:\n"
            "* is a card is inserted?\n"
            "* Is your wiring correct?\n"
            "* did you edit CARD_SELECT to match the SD shield or module?"));
  }
  Serial.println(F("OK"));

  if(!volume.init(card)) {
    error(F("Could not find FAT16/FAT32 partition.\n"
            "Make sure the card is formatted."));
  }
  Serial.println(F("Volume type is FAT"));
  root.openRoot(volume);

  // This is rigged for the test application: always convert 'paint.bmp'
  // at startup, then play back each time the trigger button is pressed
  // (or loop repeatedly if button is held down).
  bmpConvert(root, "paint.bmp", "paint.tmp", analogRead(BRIGHTNESS) / 4);

  // Prepare for reading from file; determine first block, block count,
  // make a read pass through the file to estimate block read time (+10%
  // margin) and max playback lines/sec.
  uint32_t fileSize = 0L;
  if(!(firstBlock = contigFile(root, "paint.tmp", &fileSize))) {
    error(F("Could not open tempfile for input"));
  }
  nBlocks = fileSize / 512;
  maxLPS  = (uint16_t)(1000000L /
    (((benchmark(firstBlock, nBlocks) * 11) / 10) +
     ((N_LEDS * 30L) / 4) + OVERHEAD));
  if(maxLPS > 400) maxLPS = 400; // NeoPixel PWM rate is ~400 Hz

  digitalWrite(TRIGGER, HIGH);   // Enable pullup on trigger button

  // Set up Timer1 for 64:1 prescale (250 KHz clock source),
  // fast PWM mode, no PWM out.
  TCCR1A = _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
  // IMPORTANT: Timer1 interrupt is not used; instead, overflow bit is
  // tightly polled.  Provides same accuracy for line interval timing.
  // Very infrequently a block read may inexplicably take longer than
  // normal...using an interrupt, the code would clobber itself.
  // By polling instead, worst case is a slight glitch where the
  // corresponding line will be a little thicker than normal.

  // Disable Timer0 interrupt for smoother playback
  TIMSK0 = 0;
}

// Fatal error handler; doesn't return, doesn't run loop()
static void error(const __FlashStringHelper *ptr) {
    Serial.println(ptr);
    for(;;);
}


void loop() {
  uint32_t block    = 0;     // Current block # within file
  boolean  stopFlag = false; // If set, stop playback loop

  // Stage first block (but don't display yet --
  // the loop below does that only when Timer1 overflows).
  card.readData(firstBlock, 0, sizeof(ledBuf), (uint8_t *)ledBuf);

  while(digitalRead(TRIGGER) == HIGH);   // Wait for trigger button

  uint32_t linesPerSec = map(analogRead(SPEED), 0, 1023, 10, maxLPS);
  Serial.println(linesPerSec);

  OCR1A = (F_CPU / 64) / linesPerSec;    // Timer1 interval

  for(;;) {
    while(!(TIFR1 & _BV(TOV1)));         // Wait for Timer1 overflow
    TIFR1 |= _BV(TOV1);                  // Clear overflow bit

    show((uint8_t *)ledBuf);             // Display current line
    if(stopFlag) break;                  // Break when done

    if(++block >= nBlocks) {             // Past last block?
      if(digitalRead(TRIGGER) == HIGH) { // Trigger released?
        // Clear ledBuf to turn off LEDs on next overflow,
        // set flag to stop playback after that.
        // memset() was causing inexplicable crash,
        // so clearing ledBuf manually here:
        for(int i=0; i<sizeof(ledBuf); i++) ledBuf[i] = 0;
        stopFlag = true;
        continue;
      }                                  // Else trigger still held
      block = 0;                         // Loop back to start
    }
    card.readData(firstBlock + block, 0, sizeof(ledBuf), (uint8_t *)ledBuf);
  }
}


// BMP->NEOPIXEL FILE CONVERSION -----------------------------------

#define NEO_GREEN 0 // NeoPixel color component order
#define NEO_RED   1
#define NEO_BLUE  2

// Convert file from 24-bit Windows BMP format to the 4-way parallel
// NeoPixel datastream.  Conversion is bottom-to-top (see notes
// below)...for horizontal light painting, the image is NOT rotated
// here (the per-pixel file seeking this requires takes FOREVER on
// the Arduino).  Instead, such images should be rotated
// counterclockwise on computer prior to transferring to SD card.
// As currently written, both the input and output files need to be
// in the same directory.
// Brightness is set during conversion; there aren't enough cycles
// to do this in realtime during playback.  To change brightness,
// re-convert image with new brightness setting.
boolean bmpConvert(SdFile &path, char *inFile, char *outFile, uint8_t brightness) {

  SdFile   bmpFile;              // Windows BMP file for input
  boolean  goodBmp   = false;    // True on valid BMP header parse
  int      bmpWidth, bmpHeight;  // W, H in pixels
  uint32_t bmpImageoffset,       // Start of image data in BMP file
           startTime = millis();

  Serial.print(F("Loading image '"));
  Serial.print(inFile);
  Serial.print(F("'..."));
  if(NULL == bmpFile.open(path, inFile, O_RDONLY)) {
    Serial.println(F("not found"));
    return false;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) {     // BMP signature
    (void)read32(bmpFile);            // Ignore file size
    (void)read32(bmpFile);            // Ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    (void)read32(bmpFile);            // Ignore DIB header
    bmpWidth       = read32(bmpFile); // Image dimensions
    bmpHeight      = read32(bmpFile);
    if((read16(bmpFile) ==  1) &&     // Planes - must be 1
       (read16(bmpFile) == 24) &&     // Bits per pixel - must be 24
       (read32(bmpFile) ==  0)) {     // Compression - must be 0 (uncompressed)

      goodBmp = true;                 // Supported BMP format -- proceed!

      Serial.write('\n');
      Serial.print(bmpWidth);
      Serial.write('x');
      Serial.print(bmpHeight);
      Serial.println(F(" pixels"));

      // NeoPixel working file is always 512 bytes (one SD block) per row:
      uint32_t firstBlock,
               convertedSize = 512L * bmpHeight;

      if(firstBlock = contigFile(path, outFile, &convertedSize)) { // File created?

        uint8_t bmpBuf[(N_LEDS / 4) * 3],             // 1/4 scanline from input file
                pixel[3];                             // Current pixel bring procesed
        uint32_t rowSize = ((bmpWidth * 3) + 3) & ~3; // BMP rows padded to 4 bytes

        boolean flip = false;
        if(bmpHeight < 0) {        // If bmpHeight is negative,
          bmpHeight  = -bmpHeight; // image is in top-down order.
          flip       = true;       // Not canon, but has been observed in the wild.
        }

        int bmpStartColumn, ledStartColumn, columns;
        if(bmpWidth >= N_LEDS) { // BMP matches LED bar width, or crop image
          bmpStartColumn = (bmpWidth - N_LEDS) / 2;
          ledStartColumn = 0;
          columns        = N_LEDS;
        } else {                 // Letterbox narrow image within LED bar
          bmpStartColumn = 0;
          ledStartColumn = (N_LEDS - bmpWidth) / 2;
          columns        = bmpWidth;
        }

        brightness++; // Rollover OK, fun with math!

        Serial.print(F("Converting..."));
        for(int row=0; row<bmpHeight; row++) { // For each image row...
          Serial.write('.');
  
          // Image is converted from bottom to top.  This is on purpose!
          // The ground (physical ground, not the electrical kind) provides
          // a uniform point of reference for multi-frame vertical painting...
          // could then use something like a leaf switch to trigger playback,
          // lifting the light bar like making giant soap bubbles.
          // Seek to first pixel to load for this row...
          bmpFile.seekSet(bmpImageoffset + (bmpStartColumn * 3) + (rowSize * (flip ?
            (bmpHeight - 1 - row) : // Image is stored top-to-bottom
            row)));                 // Image stored bottom-to-top (normal BMP)
  
          // Clear ledBuf on each row so conversion can just use OR ops (no masking)
          memset((void *)ledBuf, 0, N_LEDS * 3);

          uint8_t seg              = ledStartColumn / (N_LEDS / 4);         // First segment
          int     ledColumn        = ledStartColumn - (seg * (N_LEDS / 4)), // First LED within segment
                  columnsRemaining = columns;

          while(columnsRemaining) {                              // For each segment...
            uint8_t columnsToProcess = (N_LEDS / 4) - ledColumn; // Columns within segment
            if(columnsToProcess > columnsRemaining)
               columnsToProcess = columnsRemaining;              //  Clip right (narrow BMP)

            if(!bmpFile.read(bmpBuf, columnsToProcess * 3))      // Load segment
              Serial.println(F("Read error"));

            columnsRemaining -= columnsToProcess;

            for(uint8_t *bmpPtr = bmpBuf; columnsToProcess--; ledColumn++) { // For each column...
              // Determine starting address (in ledBuf) for this pixel
              uint8_t *ledPtr = (uint8_t *)&ledBuf[12 * ((seg < 2) ?
                ((N_LEDS / 4) - 1 - ledColumn) : // First half of display - strips are reversed
                                    ledColumn)]; // Second half of display - strips are forward

              // Reorder BMP BGR to NeoPixel GRB
              if(brightness) {
                pixel[NEO_BLUE]  = pgm_read_byte(&gamma[(*bmpPtr++ * brightness) >> 8]);
                pixel[NEO_GREEN] = pgm_read_byte(&gamma[(*bmpPtr++ * brightness) >> 8]);
                pixel[NEO_RED]   = pgm_read_byte(&gamma[(*bmpPtr++ * brightness) >> 8]);
              } else {
                pixel[NEO_BLUE]  = pgm_read_byte(&gamma[*bmpPtr++]);
                pixel[NEO_GREEN] = pgm_read_byte(&gamma[*bmpPtr++]);
                pixel[NEO_RED]   = pgm_read_byte(&gamma[*bmpPtr++]);
              }

              // Process 3 color bytes, turning sideways, MSB first
              for(uint8_t color=0; color<3; color++) {
                uint8_t p = pixel[color], // G, R, B
                        b = 0x80;
                do {
                  if(p & b) *ledPtr |= ledMaskA[seg]; // Bits 7, 5, 3, 1
                  b >>= 1;
                  if(p & b) *ledPtr |= ledMaskB[seg]; // Bits 6, 4, 2, 0
                  b >>= 1;
                  ledPtr++;
                } while(b);
              } // Next color byte
            } // Next column
            ledColumn = 0; // Reset; always 0 on subsequent columns
            seg++;
          } // Next segment

          // Dirty rotten pool: the SD card block size is always 512 bytes.
          // ledBuf will always be slightly smaller than this, yet we write
          // a full 512 bytes starting from this address (in order to save
          // RAM...in very short supply here...ledBuf is not padded to the
          // full 512 bytes).  This means random residue from the heap (or
          // whatever follows ledBuf in memory) is written to the file at
          // the end of every block.  This is okay for our application
          // because the other code that reads from this file is also using
          // block reads, and only uses the first N_LEDS * 3 bytes from
          // each block; the garbage is ignored.  This would be an
          // inexcusable transgression in a 'normal' application (sensitive
          // information in RAM could get written to the card) and would
          // totally land you an 'F' in CompSci, so don't look at this as a
          // good programming model, it's just a dirty MCU hack to save RAM
          // for a trifle application.  Tomfoolery hereby acknowledged.

          // Using regular block writes; tried multi-block sequence write
          // but it wouldn't play nice.  No biggie, still plenty responsive.
          if(!card.writeBlock(firstBlock + row, (uint8_t *)ledBuf))
            Serial.println(F("Write error"));

        } // Next row
        Serial.println(F("OK"));

      } else { // Error opening contigFile for output
        Serial.print(F("Error creating working file '"));
        Serial.print(outFile);
        Serial.println("'");
      }

      Serial.print(F("Converted in "));
      Serial.print(millis() - startTime);
      Serial.println(" ms");
    } // end goodBmp
  } // end bmp signature

  bmpFile.close();
  if(!goodBmp) Serial.println(F("BMP format not recognized."));
  return goodBmp;
}


// FILE UTILITIES --------------------------------------------------

// Word (16-bit) and long (32-bit) read functions used by the BMP
// reading function above.  These are endian-dependent and nonportable;
// avr-gcc implementation of words & longs happens to have the same
// endian-ness as Intel.

static uint16_t read16(SdFile &f) {
  uint16_t result = 0;
  f.read(&result, sizeof(result));
  return result;
}

static uint32_t read32(SdFile &f) {
  uint32_t result = 0L;
  f.read(&result, sizeof(result));
  return result;
}

// Create or access a contiguous (un-fragmented) file.  Pointer to
// size (uint32_t) MUST be passed -- if this contains 0, existing
// file is opened for reading and actual file size is then stored
// here.  If a non-zero value is provided, a file of this size will
// be created (deleting any existing file first).  Returns the
// index of the first SD block of the file (or 0 on error).  The
// SdFile handle is NOT returned; this is intended for raw block
// read/write ops.
static uint32_t contigFile(SdFile &path, char *filename, uint32_t *bytes) {

  if(NULL == bytes) {
    Serial.println(F("contigFile: bad parameter"));
    return 0;
  }

  SdFile file;

  if(*bytes) { // Create file?
    // Serial.println(F("Removing old file"));
    (void)file.remove(path, filename);
    Serial.print(F("Creating contiguous file..."));
    if(!file.createContiguous(path, filename, *bytes)) {
      Serial.println(F("error"));
      return 0;
    }
    Serial.println(F("OK"));
  } else {
    if(!file.open(path, filename, O_RDONLY)) {
      Serial.println(F("Error opening contiguous file"));
      return 0;
    }
    *bytes = file.fileSize();
  }

  uint32_t c = file.firstCluster(),
           b = volume.dataStartBlock() + ((c - 2) << volume.clusterSizeShift());

  file.close();

  return b; // First block of file
}

// Estimate maximum block-read time for card (microseconds)
static uint32_t benchmark(uint32_t block, uint32_t n) {
  uint32_t t, maxTime = 0L;

  do {
    t = micros();
    card.readData(block++, 0, sizeof(ledBuf), (uint8_t *)ledBuf);
    if((t = (micros() - t)) > maxTime) maxTime = t;
  } while(--n);

  return maxTime;
}

// NEOPIXEL HANDLER ------------------------------------------------

// The normal NeoPixel library isn't used by this project...
// long strips proved a bigger bottleneck than SD card reading!
// Instead, this code runs four Arduino pins in parallel, each
// controlling one NeoPixel strip (1/4 of the total display).
// No, this approach will not be implemented in the standard
// NeoPixel library...it's only a benefit here because we're using
// pre-processed data directly from the SD card.  Pixel plotting
// in this format is considerably more complicated than with the
// normal NeoPixel code, which would erase any performance benefit
// in general-purpose use.  So don't ask.

static void show(uint8_t *ptr) {

  noInterrupts();

  uint16_t i  = N_LEDS * 3;
  uint8_t  hi = *ledPort |  ledPortMask,
           lo = *ledPort & ~ledPortMask,
           b, next;

    // 20 inst. clocks per bit: HHHHHxxxxxxxxLLLLLLL
    // ST instructions:         ^    ^       ^       (T=0,5,13+20,25,33)

  asm volatile (
   "NeoX4_%=:"                "\n\t" // Clk  Pseudocode
    "ld   %[b]    , %a[ptr]+" "\n\t" // 2    b     = *ptr++        (T =  0, 40)
    "st   %a[port], %[hi]"    "\n\t" // 2   *port  = hi            (T =  2)
    "mov  %[next] , %[b]"     "\n\t" // 1    next  = b             (T =  3)
    "and  %[next] , %[mask]"  "\n\t" // 1    next &= mask          (T =  4)
    "or   %[next] , %[lo]"    "\n\t" // 1    next |= lo            (T =  5)
    "st   %a[port], %[next]"  "\n\t" // 2   *port  = next          (T =  7)
    "rjmp .+0"                "\n\t" // 2    nop nop               (T =  9)
    "rjmp .+0"                "\n\t" // 2    nop nop               (T = 11)
    "rjmp .+0"                "\n\t" // 2    nop nop               (T = 13)
    "st   %a[port], %[lo]"    "\n\t" // 2   *port  = lo            (T = 15)
    "swap %[b]"               "\n\t" // 1    b     = (b<<4)|(b>>4) (T = 16)
    "and  %[b]    , %[mask]"  "\n\t" // 1    b    &= mask          (T = 17)
    "or   %[b]    , %[lo]"    "\n\t" // 1    b    |= lo            (T = 18)
    "rjmp .+0"                "\n\t" // 2    nop nop               (T = 20)
    "st   %a[port], %[hi]"    "\n\t" // 2   *port  = hi            (T = 22)
    "rjmp .+0"                "\n\t" // 2    nop nop               (T = 24)
    "nop"                     "\n\t" // 1    nop                   (T = 25)
    "st   %a[port], %[b]"     "\n\t" // 2   *port  = b             (T = 27)
    "rjmp .+0"                "\n\t" // 2    nop nop               (T = 29)
    "rjmp .+0"                "\n\t" // 2    nop nop               (T = 31)
    "sbiw %[i]    , 1"        "\n\t" // 2    i--                   (T = 33)
    "st   %a[port], %[lo]"    "\n\t" // 2   *port  = lo            (T = 35)
    "nop"                     "\n\t" // 1    nop                   (T = 36)
    "brne NeoX4_%="           "\n"   // 2    if(i) goto NeoX4      (T = 38)
    : [port]  "+e" (ledPort),
      [b]     "+r" (b),
      [next]  "+r" (next),
      [i] "+w" (i)
    : [ptr]   "e" (ptr),
      [hi]    "r" (hi),
      [lo]    "r" (lo),
      [mask]  "r" (ledPortMask));

  interrupts();

  // There's no explicit 50 uS delay here as with most NeoPixel code;
  // SD card block read provides ample time for latch!
}

