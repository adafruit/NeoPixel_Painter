// ADAFRUIT NEOPIXEL LIGHT PAINTER SKETCH: Reads 24-bit BMP image from
// SD card, plays back on NeoPixel strip for long-exposure photography.

// Requires SdFat and NeoPixel libraries for Arduino:
// http://code.google.com/p/sdfatlib/
// https://github.com/adafruit/Adafruit_NeoPixel

// As written, uses a momentary pushbutton connected between pin A1 and
// GND to trigger playback.  An analog potentiometer connected to A0 sets
// the brightness at startup and the playback speed each time the trigger
// is tapped.  BRIGHTNESS IS SET ONLY AT STARTUP; can't adjust this in
// realtime, not fast enough.  To change brightness, set dial and tap reset.
// Then set dial for playback speed.

// This is a 'plain vanilla' example with no UI or anything -- it always
// reads a fixed file at startup (frame000.bmp in root folder), outputs
// frame000.tmp (warning: doesn't ask, just overwrites it), then plays back
// from this file each time button is tapped (repeating in loop if held).
// More advanced applications could add a UI (e.g. 16x2 LCD shield) or
// process multiple files for animation.  NONE OF THAT IS HERE THOUGH,
// you will need to get clever and rip up some of this code for such.

// This is well-tailored to the Arduino Uno or similar boards.  It may work
// with the Arduino Leonardo *IF* your SD shield or breakout board uses the
// 6-pin ICSP header for SPI rather than pins 11-13.  This WILL NOT WORK
// with 'soft' SPI on the Arduino Mega (too slow).  Also, even with 'hard'
// SPI, this DOES NOT RUN ANY FASTER on the Mega -- a common misconception.

// Written by Phil Burgess / Paint Your Dragon for Adafruit Industries.
// MIT license.

// Adafruit invests time and resources providing this open source code,
// please support Adafruit and open-source hardware by purchasing
// products from Adafruit!

#include <SdFat.h>
#include <Adafruit_NeoPixel.h>
#include <avr/pgmspace.h>
#include "./gamma.h"

// CONFIGURABLE STUFF --------------------------------------------------------

#define N_LEDS       144 // Max value of 170 (fits one SD card block)
#define CARD_SELECT   10 // SD card select pin (some shields use #4, not 10)
#define LED_PIN        6 // NeoPixels connect here
#define SPEED         A0 // Speed-setting dial
#define BRIGHTNESS    A0 // Brightness-setting dial
#define TRIGGER       A1 // Playback trigger pin
#define CURRENT_MAX 3500 // Max current from power supply (mA)
// The software does its best to limit the LED brightness to a level that's
// manageable by the power supply.  144 NeoPixels at full brightness can
// draw about 10 Amps(!), while the UBEC (buck converter) sold by Adafruit
// can provide up to 3A continuous, or up to 5A for VERY BRIEF intervals.
// The default CURRENT_MAX setting is a little above the continuous value,
// figuring light painting tends to run for short periods and/or that not
// every scanline will demand equal current.  For extremely long or bright
// images or longer strips, this may exceed the UBEC's capabilities, in
// which case it shuts down (will need to disconnect battery).  If you
// encounter this situation, set CURRENT_MAX to 3000.  Alternately, two
// UBECs can be used in parallel -- each connected to 1/2 of the NeoPixel
// strip (connect the grounds together, but NOT the +5V outputs -- the 5V
// side of the strip must then be cut and each half powered by a separate
// UBEC), CURRENT_MAX can then be bumped to 6000 mA.

// Define ENCODERSTEPS to use rotary encoder rather than timer to advance
// each line.  The encoder MUST be on the T1 pin...this is digital pin 5
// on the Arduino Uno...check datasheet/pinout ref for other board types.
//#define ENCODERSTEPS 10 // # of steps needed to advance 1 line


// NON-CONFIGURABLE STUFF ----------------------------------------------------

#define OVERHEAD 150 // Extra microseconds for loop processing, etc.

// The strip is always declared w/171 pixels, regardless of the number
// actually used.  This allows the LED buffer to do double duty as the
// SD read buffer, saving RAM and a data copying step.
Adafruit_NeoPixel strip = Adafruit_NeoPixel(171, LED_PIN);
uint8_t          *ledBuf;     // -> raw NeoPixel data
uint16_t          maxLPS;     // Max playback lines/sec
uint32_t          firstBlock, // First block # in temp working file
                  nBlocks;    // Number of blocks in file
Sd2Card           card;       // SD card global instance (only one)
SdVolume          volume;     // Filesystem global instance (only one)
SdFile            root;       // Root directory (only one)

// INITIALIZATION ------------------------------------------------------------

void setup() {
  uint8_t  b, startupTrigger;
  uint32_t lastBlock;
  SdFile   tmp;

  digitalWrite(TRIGGER, HIGH);           // Enable pullup on trigger button
  startupTrigger = digitalRead(TRIGGER); // Poll startup trigger ASAP
  Serial.begin(57600);
  strip.begin();
  strip.show();                          // Clear LEDs
  ledBuf = strip.getPixels();            // Get pointer to NeoPixel buffer
#ifdef ENCODERSTEPS
  digitalWrite(5, HIGH);                 // Enable pullup on encoder pin
#endif

  Serial.print(F("Initializing SD card..."));
  if(!card.init(SPI_FULL_SPEED, CARD_SELECT)) {
    error(F("failed. Things to check:\n"
            "* is a card is inserted?\n"
            "* Is your wiring correct?\n"
            "* did you edit CARD_SELECT to match the SD shield or module?"));
  }
  Serial.println(F("OK"));

  if(!volume.init(&card)) {
    error(F("Could not find FAT16/FAT32 partition.\n"
            "Make sure the card is formatted."));
  }
  root.openRoot(&volume);

  // This simple application always reads the file 'frame000.bmp' in the
  // root directory; there's no file selection mechanism or UI.  Clearly,
  // there are plans to do multi-frame rendering, but there is currently
  // no mechanism implemented for this.

  // If button is held at startup, the processing step is skipped, just
  // goes right to playback of the prior converted file (if present).
  if(startupTrigger == HIGH) { // No button press
    // Two passes are made over the input image.  First pass estimates
    // the max brightness level that the power supply can sustain...
    b = 255;                                    // Start with max brightness
    bmpProcess(root, "frame000.bmp", NULL, &b); // b modified to 'safe' max
    // If one were to use multiple images for animation, best to make a pass
    // through all the frames first, determining the minimum safe max among
    // all of them, then adjust every image to a uniform brightness level.

    // Read dial, setting brightness between 1 (almost but not quite off)
    // and the previously-estimated safe max.
    b = map(analogRead(BRIGHTNESS), 0, 1023, 1, b);
  
    // Second pass now applies brightness adjustment while converting
    // the image from BMP to a raw representation of NeoPixel data
    // (this outputs the file 'frame000.tmp' -- any existing file by
    // that name will simply be clobbered, IT DOES NOT ASK).
    bmpProcess(root, "frame000.bmp", "frame000.tmp", &b);
  } else {
    // Get existing contiguous tempfile info
    if(!tmp.open(&root, "frame000.tmp", O_RDONLY)) {
      error(F("Could not open NeoPixel tempfile for input"));
    }
    if(!tmp.contiguousRange(&firstBlock, &lastBlock)) {
      error(F("NeoPixel tempfile is not contiguous"));
    }
    // Number of blocks needs to be calculated from file size, not the
    // range values.  The contiguous file creation and range functions
    // work on cluster boundaries, not necessarily the actual file size.
    nBlocks = tmp.fileSize() / 512;

    tmp.close(); // File handle is no longer accessed, just block reads
    while(digitalRead(TRIGGER) == LOW); // Wait for button release
  }

#ifdef ENCODERSTEPS
  // To use a rotary encoder rather than timer, connect one output
  // of encoder to T1 pin (digital pin 5 on Arduino Uno).  A small
  // capacitor across the encoder pins may help for debouncing.
  TCCR1A = _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS12) | _BV(CS11);
#else
  // Prepare for playback from file; make a full read pass through the
  // file to estimate block read time (+5% margin) and max playback
  // lines/sec.  Not all SD cards perform the same.  This makes sure a
  // reasonable speed limit is used.
  maxLPS  = (uint16_t)(1000000L /                   // 1 uSec /
    (((benchmark(firstBlock, nBlocks) * 21) / 20) + // time+5% +
     (N_LEDS * 30L) + OVERHEAD));                   // 30 uSec/pixel
  if(maxLPS > 400) maxLPS = 400; // NeoPixel PWM rate is ~400 Hz
  Serial.print(F("Max lines/sec: "));
  Serial.println(maxLPS);

  // Set up Timer1 for 64:1 prescale (250 KHz clock source),
  // fast PWM mode, no PWM out.
  TCCR1A = _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
  // Timer1 interrupt is not used; instead, overflow bit is tightly
  // polled.  Very infrequently a block read may inexplicably take longer
  // than normal...using an interrupt, the code would clobber itself.
  // By polling instead, worst case is just a minor glitch where the
  // corresponding row will be a little thicker than normal.
#endif

  // Timer0 interrupt is disabled for smoother playback.
  // This means delay(), millis(), etc. won't work after this.
  TIMSK0 = 0;
}

// Startup error handler; doesn't return, doesn't run loop(), just stops.
static void error(const __FlashStringHelper *ptr) {
    Serial.println(ptr); // Show message
    for(;;);             // and hang
}

// PLAYBACK LOOP -------------------------------------------------------------

void loop() {
  uint32_t block    = 0;     // Current block # within file
  boolean  stopFlag = false; // If set, stop playback loop

  // Stage first block, but don't display yet -- the loop below
  // does that only when Timer1 overflows.
  card.readStart(firstBlock);
  card.readData(ledBuf);

  while(digitalRead(TRIGGER) == HIGH);   // Wait for trigger button

#ifdef ENCODERSTEPS
  // Set up for rotary encoder
  TCNT1 = 0;
  OCR1A = ENCODERSTEPS;
#else
  // Set up timer based on dial input
  uint32_t linesPerSec = map(analogRead(SPEED), 0, 1023, 10, maxLPS);
  // Serial.println(linesPerSec);
  OCR1A = (F_CPU / 64) / linesPerSec;    // Timer1 interval
#endif

  for(;;) {
    while(!(TIFR1 & _BV(TOV1)));         // Wait for Timer1 overflow
    TIFR1 |= _BV(TOV1);                  // Clear overflow bit

    strip.show();                        // Display current line
    if(stopFlag) break;                  // Break when done

    if(++block >= nBlocks) {             // Past last block?
      card.readStop();
      if(digitalRead(TRIGGER) == HIGH) { // Trigger released?
        memset(ledBuf, 0, N_LEDS * 3);   // LEDs off on next pass
        stopFlag = true;                 // Stop playback on next pass
        continue;
      }                                  // Else trigger still held
      block = 0;                         // Loop back to start
      card.readStart(firstBlock);        // Re-init multi-block read
    }
    card.readData(ledBuf);               // Load next pixel row
  }
}

// BMP->NEOPIXEL FILE CONVERSION ---------------------------------------------

#define BMP_BLUE  0 // BMP and NeoPixels have R/G/B color
#define BMP_GREEN 1 // components in different orders.
#define BMP_RED   2 // (BMP = BGR, Neo = GRB)
#define NEO_GREEN 0
#define NEO_RED   1
#define NEO_BLUE  2

// Convert file from 24-bit Windows BMP format to raw NeoPixel datastream.
// Conversion is bottom-to-top (see notes below)...for horizontal light
// painting, the image is NOT rotated here (the per-pixel file seeking this
// requires takes FOREVER on the Arduino).  Instead, such images should be
// rotated counterclockwise (in Photoshop or other editor) prior to moving
// to SD card.  As currently written, both the input and output files need
// to be in the same directory.  Brightness is set during conversion; there
// aren't enough cycles to do this in realtime during playback.  To change
// brightness, re-process image file using new brightness value.
boolean bmpProcess(
  SdFile  &path,
  char    *inName,
  char    *outName,
  uint8_t *brightness) {

  SdFile    inFile,              // Windows BMP file for input
            outFile;             // NeoPixel temp file for output
  boolean   ok        = false,   // 'true' on valid BMP & output file
            flip      = false;   // 'true' if image stored top-to-bottom
  int       bmpWidth,            // BMP width in pixels
            bmpHeight,           // BMP height in pixels
            bmpStartCol,         // First BMP column to process (crop/center)
            columns,             // Number of columns to process (crop/center)
            row,                 // Current image row (Y)
            column;              // and column (X)
  uint8_t  *ditherRow,           // 16-element dither array for current row
            pixel[3],            // For reordering color data, BGR to GRB
            b = 0,               // 1 + *brightness
            d,                   // Dither value for row/column
            color,               // Color component index (R/G/B)
            raw,                 // 'Raw' R/G/B color value
            corr,                // Gamma-corrected R/G/B
           *ledPtr,              // Pointer into ledBuf (output)
           *ledStartPtr;         // First LED column to process (crop/center)
  uint16_t  b16;                 // 16-bit dup of b
  uint32_t  bmpImageoffset,      // Start of image data in BMP file
            lineMax   = 0L,      // Cumulative brightness of brightest line
            rowSize,             // BMP row size (bytes) w/32-bit alignment
            sum,                 // Sum of pixels in row
            startTime = millis();

  if(brightness)           b = 1 + *brightness; // Wraps around, fun with maths
  else if(NULL == outName) return false; // MUST pass brightness for power est.

  Serial.print(F("Reading file '"));
  Serial.print(inName);
  Serial.print(F("'..."));
  if(!inFile.open(&path, inName, O_RDONLY)) {
    Serial.println(F("error"));
    return false;
  }

  if(inFile.read(ledBuf, 34)             &&    // Borrow ledBuf to load header
    (*(uint16_t *)&ledBuf[ 0] == 0x4D42) &&    // BMP signature
    (*(uint16_t *)&ledBuf[26] == 1)      &&    // Planes: must be 1
    (*(uint16_t *)&ledBuf[28] == 24)     &&    // Bits per pixel: must be 24
    (*(uint32_t *)&ledBuf[30] == 0)) {         // Compression: must be 0 (none)
    // Supported BMP format -- proceed!
    bmpImageoffset = *(uint32_t *)&ledBuf[10]; // Start of image data
    bmpWidth       = *(uint32_t *)&ledBuf[18]; // Image dimensions
    bmpHeight      = *(uint32_t *)&ledBuf[22];
    // That's some nonportable, endian-dependent code there.

    Serial.print(bmpWidth);
    Serial.write('x');
    Serial.print(bmpHeight);
    Serial.println(F(" pixels"));

    if(outName) { // Doing conversion?  Need outFile.
      // Delete existing outFile file (if any)
      (void)SdFile::remove(&path, outName);
      Serial.print(F("Creating contiguous file..."));
      // NeoPixel working file is always 512 bytes (one SD block) per row
      if(outFile.createContiguous(&path, outName, 512L * bmpHeight)) {
        uint32_t lastBlock;
        outFile.contiguousRange(&firstBlock, &lastBlock);
        // Once we have the first block index, the file handle
        // is no longer needed -- raw block writes are used.
        outFile.close();
        nBlocks = bmpHeight; // See note in setup() re: block calcs
        ok      = true;      // outFile is good; proceed
        Serial.println(F("OK"));
      } else {
        Serial.println(F("error"));
      }
    } else ok = true; // outFile not needed; proceed

    if(ok) { // Valid BMP and contig file (if needed) are ready
      Serial.print(F("Processing..."));

      rowSize = ((bmpWidth * 3) + 3) & ~3; // 32-bit line boundary
      b16     = (int)b;

      if(bmpHeight < 0) {       // If bmpHeight is negative,
        bmpHeight = -bmpHeight; // image is in top-down order.
        flip      = true;       // Rare, but happens.
      }

      if(bmpWidth >= N_LEDS) { // BMP matches LED bar width, or crop image
        bmpStartCol = (bmpWidth - N_LEDS) / 2;
        ledStartPtr = ledBuf;
        columns     = N_LEDS;
      } else {                 // Center narrow image within LED bar
        bmpStartCol = 0;
        ledStartPtr = &ledBuf[((N_LEDS - bmpWidth) / 2) * 3];
        columns     = bmpWidth;
        memset(ledBuf, 0, N_LEDS * 3); // Clear left/right pixels
      }

      for(row=0; row<bmpHeight; row++) { // For each row in image...
        Serial.write('.');
        // Image is converted from bottom to top.  This is on purpose!
        // The ground (physical ground, not the electrical kind) provides
        // a uniform point of reference for multi-frame vertical painting...
        // could then use something like a leaf switch to trigger playback,
        // lifting the light bar like making giant soap bubbles.

        // Seek to first pixel to load for this row...
        inFile.seekSet(
          bmpImageoffset + (bmpStartCol * 3) + (rowSize * (flip ?
          (bmpHeight - 1 - row) : // Image is stored top-to-bottom
          row)));                 // Image stored bottom-to-top
        if(!inFile.read(ledStartPtr, columns * 3))  // Load row
          Serial.println(F("Read error"));

        sum       = 0L;
        ditherRow = (uint8_t *)&dither[row & 0x0F]; // Dither values for row
        ledPtr    = ledStartPtr;
        for(column=0; column<columns; column++) {   // For each column...
          if(b) { // Scale brightness, reorder R/G/B
            pixel[NEO_BLUE]  = (ledPtr[BMP_BLUE]  * b16) >> 8;
            pixel[NEO_GREEN] = (ledPtr[BMP_GREEN] * b16) >> 8;
            pixel[NEO_RED]   = (ledPtr[BMP_RED]   * b16) >> 8;
          } else { // Full brightness, reorder R/G/B
            pixel[NEO_BLUE]  = ledPtr[BMP_BLUE];
            pixel[NEO_GREEN] = ledPtr[BMP_GREEN];
            pixel[NEO_RED]   = ledPtr[BMP_RED];
          }

          d = pgm_read_byte(&ditherRow[column & 0x0F]); // Dither probability
          for(color=0; color<3; color++) {              // 3 color bytes...
            raw  = pixel[color];                        // 'Raw' G/R/B
            corr = pgm_read_byte(&gamma[raw]);          // Gamma-corrected
            if(pgm_read_byte(&bump[raw]) > d) corr++;   // Dither up?
            *ledPtr++ = corr;                           // Store back in ledBuf
            sum      += corr;                           // Total brightness
          } // Next color byte
        } // Next column

        if(outName) {
          if(!card.writeBlock(firstBlock + row, (uint8_t *)ledBuf))
            Serial.println(F("Write error"));
        }
        if(sum > lineMax) lineMax = sum;

      } // Next row
      Serial.println(F("OK"));

      if(brightness) {
        lineMax = (lineMax * 20) / 255; // Est current @ ~20 mA/LED
        if(lineMax > CURRENT_MAX) {
          // Estimate suitable brightness based on CURRENT_MAX
          *brightness = (*brightness * (uint32_t)CURRENT_MAX) / lineMax;
        } // Else no recommended change
      }

      Serial.print(F("Processed in "));
      Serial.print(millis() - startTime);
      Serial.println(F(" ms"));

    } // end 'ok' check
  } else { // end BMP header check
    Serial.println(F("BMP format not recognized."));
  }

  inFile.close();
  return ok; // 'false' on various file open/create errors
}

// MISC UTILITY FUNCTIONS ----------------------------------------------------

// Estimate maximum block-read time for card (microseconds)
static uint32_t benchmark(uint32_t block, uint32_t n) {
  uint32_t t, maxTime = 0L;

  card.readStart(block);
  do {
    t = micros();
    card.readData(ledBuf);
    if((t = (micros() - t)) > maxTime) maxTime = t;
  } while(--n);
  card.readStop();

  return maxTime;
}
