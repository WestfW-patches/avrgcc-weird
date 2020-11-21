#define FUNC_READ 1
#define FUNC_WRITE 1

#define UART 0
#define LED_START_FLASHES 3

#define OPTIBOOT_MAJVER 8
#define OPTIBOOT_MINVER 1

unsigned const int __attribute__((section(".version"))) 
optiboot_version = 256*(OPTIBOOT_MAJVER) + OPTIBOOT_MINVER;


#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

/*
 * optiboot uses several "address" variables that are sometimes byte pointers,
 * sometimes word pointers. sometimes 16bit quantities, and sometimes built
 * up from 8bit input characters.  avr-gcc is not great at optimizing the
 * assembly of larger words from bytes, but we can use the usual union to
 * do this manually.  Expanding it a little, we can also get rid of casts.
 */
typedef union {
	uint8_t  *bptr;
	uint16_t *wptr;
	uint16_t word;
	uint8_t bytes[2];
} addr16_t;

/*
 * Note that we use a replacement of "boot.h"
 * <avr/boot.h> uses sts instructions, but this version uses out instructions
 * This saves cycles and program memory, if possible.
 * boot_opt.h pulls in the standard boot.h for the odd target (?)
 */
#include "boot_opt.h"


// We don't use <avr/wdt.h> as those routines have interrupt overhead we don't need.

/*
 * pin_defs.h
 * This contains most of the rather ugly defines that implement our
 * ability to use UART=n and LED=D3, and some avr family bit name differences.
 */
#include "pin_defs.h"

/*
 * stk500.h contains the constant definitions for the stk500v1 comm protocol
 */
#include "stk500.h"

/* set the UART baud rate defaults */
#ifndef BAUD_RATE
#define BAUD_RATE   115200L // Highest rate Avrdude win32 will support
#endif

/* Normal U2X usage */
#define BAUD_SETTING (( (F_CPU + BAUD_RATE * 4L) / ((BAUD_RATE * 8L))) - 1 )
#define BAUD_ACTUAL (F_CPU/(8 * ((BAUD_SETTING)+1)))

/* Watchdog settings */
#define WATCHDOG_OFF    (0)
#define WATCHDOG_16MS   (_BV(WDE))
#define WATCHDOG_1S     (_BV(WDP2) | _BV(WDP1) | _BV(WDE))

/*
 * Watchdog timeout translations from human readable to config vals
 */
# define WDTPERIOD WATCHDOG_1S  // 1 second default

/*
 * We can never load flash with more than 1 page at a time, so we can save
 * some code space on parts with smaller pagesize by using a smaller int.
 */
typedef uint8_t pagelen_t;
#define GETLENGTH(len) (void) getch() /* skip high byte */; len = getch()


/* Function Prototypes
 * The main() function is in init9, which removes the interrupt vector table
 * we don't need. It is also 'OS_main', which means the compiler does not
 * generate any entry or exit code itself (but unlike 'naked', it doesn't
 * supress some compile-time options we want.)
 */

void pre_main(void) __attribute__ ((naked)) __attribute__ ((section (".init8")));
int main(void) __attribute__ ((OS_main)) __attribute__ ((section (".init9"))) __attribute__((used));

void __attribute__((noinline)) __attribute__((leaf)) putch(char);
uint8_t __attribute__((noinline)) __attribute__((leaf)) getch(void) ;
void __attribute__((noinline)) verifySpace();
void __attribute__((noinline)) watchdogConfig(uint8_t x);

static void getNch(uint8_t);
#if LED_START_FLASHES > 0
static inline void flash_led(uint8_t);
#endif
static inline void watchdogReset();
static inline void writebuffer(int8_t memtype, addr16_t mybuff,
			       addr16_t address, pagelen_t len);
static inline void read_mem(uint8_t memtype,
			    addr16_t, pagelen_t len);


/* C zero initialises all global variables. However, that requires */
/* These definitions are NOT zero initialised, but that doesn't matter */
/* This allows us to drop the zero init code, saving us memory */
static addr16_t buff = {(uint8_t *)(RAMSTART)};

/* everything that needs to run VERY early */
void pre_main(void) {
  // Allow convenient way of calling do_spm function - jump table,
  //   so entry to this function will always be here, indepedent of compilation,
  //   features etc
  asm volatile (
    "	rjmp	1f\n"
    "	rjmp	do_spm\n"
    "1:\n"
  );
}


/* main program starts here */
int main(void) {
  uint8_t ch;
  asm("9:\n");
  /*
   * Making these local and in registers prevents the need for initializing
   * them, and also saves space because code no longer stores to memory.
   * (initializing address keeps the compiler happy, but isn't really
   *  necessary, and uses 4 bytes of flash.)
   */
  register addr16_t address;
  register pagelen_t  length;

  // This code makes the following assumptions:
  //  No interrupts will execute
  //  SP points to RAMEND
  //  r1 contains zero
  //
  // If not, uncomment the following instructions:
  // cli();
  asm volatile ("clr __zero_reg__");

  /*
   * Protect as much from MCUSR as possible for application
   * and still skip bootloader if not necessary
   * 
   * Code by MarkG55
   * see discusion in https://github.com/Optiboot/optiboot/issues/97
   */
  ch = MCUSR;
  // Skip all logic and run bootloader if MCUSR is cleared (application request)
  if (ch != 0) {
      /*
       * To run the boot loader, External Reset Flag must be set.
       * If not, we could make shortcut and jump directly to application code.
       * Also WDRF set with EXTRF is a result of Optiboot timeout, so we
       * shouldn't run bootloader in loop :-) That's why:
       *  1. application is running if WDRF is cleared
       *  2. we clear WDRF if it's set with EXTRF to avoid loops
       * One problematic scenario: broken application code sets watchdog timer 
       * without clearing MCUSR before and triggers it quickly. But it's
       * recoverable by power-on with pushed reset button.
       */
      if ((ch & (_BV(WDRF) | _BV(EXTRF))) != _BV(EXTRF)) { 
	  if (ch & _BV(EXTRF)) {
	      /*
	       * Clear WDRF because it was most probably set by wdr in bootloader.
	       * It's also needed to avoid loop by broken application which could
	       * prevent entering bootloader.
	       * '&' operation is skipped to spare few bytes as bits in MCUSR
	       * can only be cleared.
	       */
	      MCUSR = ~(_BV(WDRF));
	  }
	  /* 
	   * save the reset flags in the designated register
	   * This can be saved in a main program by putting code in .init0 (which
	   * executes before normal c init code) to save R2 to a global variable.
	   */
	  __asm__ __volatile__ ("mov r2, %0\n" :: "r" (ch));

	  // switch off watchdog
	  watchdogConfig(WATCHDOG_OFF);
	  // Note that appstart_vec is defined so that this works with either
	  // real or virtual boot partitions.
	   __asm__ __volatile__ (
	    // Jump to 'save' or RST vector
	    // use rjmp to go around end of flash to address 0
	    // it uses fact that optiboot_version constant is 2 bytes before end of flash
	    "rjmp optiboot_version+2\n"
	  );
      }
  }

  // Set up Timer 1 for timeout counter
  TCCR1B = _BV(CS12) | _BV(CS10); // div 1024

  UART_SRA = _BV(U2X0); //Double speed mode USART0
  UART_SRB = _BV(RXEN0) | _BV(TXEN0);
  UART_SRC = _BV(UCSZ00) | _BV(UCSZ01);
  UART_SRL = (uint8_t)BAUD_SETTING;

  // Set up watchdog to trigger after desired timeout
  watchdogConfig(WDTPERIOD);

  /* Flash onboard LED to signal entering of bootloader */
  flash_led(LED_START_FLASHES * 2);

  /* Forever loop: exits by causing WDT reset */
  for (;;) {
    /* get character from UART */
    ch = getch();

//#define TICK(name) asm(".global " #name "\n .equ " #name ", .-9b\n9:\n" :::)
#define TICK(name)

    TICK(size_init);
    if(ch == STK_GET_PARAMETER) {
      unsigned char which = getch();
      verifySpace();
      /*
       * Send optiboot version as "SW version"
       * Note that the references to memory are optimized away.
       */
      if (which == STK_SW_MINOR) {
	  putch(optiboot_version & 0xFF);
      } else if (which == STK_SW_MAJOR) {
	  putch(optiboot_version >> 8);
      } else {
	/*
	 * GET PARAMETER returns a generic 0x03 reply for
         * other parameters - enough to keep Avrdude happy
	 */
	putch(0x03);
      }
      TICK(size_getparameter);
    }
    else if(ch == STK_SET_DEVICE) {
      // SET DEVICE is ignored
      getNch(20); 
      TICK(set_device_size);
   }
    else if(ch == STK_SET_DEVICE_EXT) {
      // SET DEVICE EXT is ignored
      getNch(5);
      TICK(size_setdevext);
    }
    else if(ch == STK_LOAD_ADDRESS) {
      // LOAD ADDRESS
      address.bytes[0] = getch();
      address.bytes[1] = getch();
      address.word *= 2; // Convert from word address to byte address
      verifySpace();
      TICK(size_loadaddress);
    }
    else if(ch == STK_UNIVERSAL) {
      // UNIVERSAL command is ignored
      getNch(4);
      putch(0x00);
      TICK(size_universal);
    }
    /* Write memory, length is big endian and is in bytes */
    else if(ch == STK_PROG_PAGE) {
      // PROGRAM PAGE - we support flash programming only, not EEPROM
      uint8_t desttype;
      uint8_t *bufPtr;
      pagelen_t savelength;

      GETLENGTH(length);
      savelength = length;
      desttype = getch();

      // read a page worth of contents
      bufPtr = buff.bptr;
      do *bufPtr++ = getch();
      while (--length);

      // Read command terminator, start reply
      verifySpace();

      writebuffer(desttype, buff, address, savelength);

      TICK(size_writePage);
    }
    /* Read memory block mode, length is big endian.  */
    else if(ch == STK_READ_PAGE) {
      uint8_t desttype;
      GETLENGTH(length);

      desttype = getch();

      verifySpace();

      read_mem(desttype, address, length);
      TICK(size_readPage);
    }

    /* Get device signature bytes  */
    else if(ch == STK_READ_SIGN) {
      // READ SIGN - return what Avrdude wants to hear
      verifySpace();
      putch(SIGNATURE_0);
      putch(SIGNATURE_1);
      putch(SIGNATURE_2);
      TICK(size_readSign);
    }
    else if (ch == STK_LEAVE_PROGMODE) { /* 'Q' */
      // Adaboot no-wait mod
      watchdogConfig(WATCHDOG_16MS);
      verifySpace();
    }
    else {
      // This covers the response to commands like STK_ENTER_PROGMODE
      verifySpace();
    }
    putch(STK_OK);
  }
}

void putch(char ch) {
    while (!(UART_SRA & _BV(UDRE0))) {  /* Spin */ }
    UART_UDR = ch;
}

uint8_t getch(void) {
  uint8_t ch;

  LED_PIN |= _BV(LED);

  while(!(UART_SRA & _BV(RXC0)))  {  /* Spin */ }
  if (!(UART_SRA & _BV(FE0))) {
      /*
       * A Framing Error indicates (probably) that something is talking
       * to us at the wrong bit rate.  Assume that this is because it
       * expects to be talking to the application, and DON'T reset the
       * watchdog.  This should cause the bootloader to abort and run
       * the application "soon", if it keeps happening.  (Note that we
       * don't care that an invalid char is returned...)
       */
    watchdogReset();
  }
  ch = UART_UDR;
  return ch;
}

void getNch(uint8_t count) {
  do getch(); while (--count);
  verifySpace();
}

void verifySpace() {
  if (getch() != CRC_EOP) {
    watchdogConfig(WATCHDOG_16MS);    // shorten WD timeout
    while (1)			      // and busy-loop so that WD causes
      ;				      //  a reset and app start.
  }
  putch(STK_INSYNC);
}

void flash_led(uint8_t count) {
  do {
      TCNT1 = -(F_CPU/(1024*16));
      TIFR1 = _BV(TOV1);
      while(!(TIFR1 & _BV(TOV1)));
    
      LED_PIN |= _BV(LED);
      watchdogReset();
      /*
       * While in theory, the STK500 initial commands would be buffered
       *  by the UART hardware, avrdude sends several attempts in rather
       *  quick succession, some of which will be lost and cause us to
       *  get out of sync.  So if we see any data; stop blinking.
       */
      if (UART_SRA & _BV(RXC0))
          break;
  } while (--count);
}

// Watchdog functions. These are only safe with interrupts turned off.
void watchdogReset() {
  __asm__ __volatile__ (
    "wdr\n"
  );
}

void watchdogConfig(uint8_t x) {
#ifdef WDCE //does it have a Watchdog Change Enable?
 #ifdef WDTCSR
  WDTCSR = _BV(WDCE) | _BV(WDE);
 #else
  WDTCR= _BV(WDCE) | _BV(WDE);
 #endif
#else //then it must be one of those newfangled ones that use CCP
  CCP=0xD8; //so write this magic number to CCP
#endif 

#ifdef WDTCSR
  WDTCSR = x;
#else
  WDTCR= x;
#endif
}


/*
 * void writebuffer(memtype, buffer, address, length)
 */
static inline void writebuffer(int8_t memtype, addr16_t mybuff,
			       addr16_t address, pagelen_t len)
{
    switch (memtype) {
    case 'E': // EEPROM
	/*
	 * On systems where EEPROM write is not supported, just busy-loop
	 * until the WDT expires, which will eventually cause an error on
	 * host system (which is what it should do.)
	 */
	while (1)
	    ; // Error: wait for WDT
	break;
    default:  // FLASH
	/*
	 * Default to writing to Flash program memory.  By making this
	 * the default rather than checking for the correct code, we save
	 * space on chips that don't support any other memory types.
	 */
	{
	    // Copy buffer into programming buffer
	    uint16_t addrPtr = address.word;

	    /*
	     * Start the page erase and wait for it to finish.  There
	     * used to be code to do this while receiving the data over
	     * the serial link, but the performance improvement was slight,
	     * and we needed the space back.
	     */
	    __boot_page_erase_short(address.word);
	    boot_spm_busy_wait();

	    /*
	     * Copy data from the buffer into the flash write buffer.
	     */
	    do {
		__boot_page_fill_short((uint16_t)(void*)addrPtr, *(mybuff.wptr++));
		addrPtr += 2;
	    } while (len -= 2);

	    /*
	     * Actually Write the buffer to flash (and wait for it to finish.)
	     */
	    __boot_page_write_short(address.word);
	    boot_spm_busy_wait();
#if defined(RWWSRE)
	    // Reenable read access to flash
	    __boot_rww_enable_short();
#endif
	} // default block
	break;
    } // switch
}

static inline void read_mem(uint8_t memtype, addr16_t address, pagelen_t length)
{
    uint8_t ch;

    switch (memtype) {

    default:
	do {
	    // read a Flash byte and increment the address
	    __asm__ ("lpm %0,Z+\n" : "=r" (ch), "=z" (address.bptr): "1" (address));
	    putch(ch);
	} while (--length);
	break;
    } // switch
}


/*
 * Separate function for doing spm stuff
 * It's needed for application to do SPM, as SPM instruction works only
 * from bootloader.
 *
 * How it works:
 * - do SPM
 * - wait for SPM to complete
 * - if chip have RWW/NRWW sections it does additionaly:
 *   - if command is WRITE or ERASE, AND data=0 then reenable RWW section
 *
 * In short:
 * If you play erase-fill-write, just set data to 0 in ERASE and WRITE
 * If you are brave, you have your code just below bootloader in NRWW section
 *   you could do fill-erase-write sequence with data!=0 in ERASE and
 *   data=0 in WRITE
 */

static void do_spm(uint16_t address, uint8_t command, uint16_t data)  __attribute__ ((used));
static void do_spm(uint16_t address, uint8_t command, uint16_t data) {
    // Do spm stuff
    asm volatile (
	"    movw  r0, %3\n"
    "    __wr_spmcsr %0, %1\n"
    "    spm\n"
    "    clr  r1\n"
    :
    : "i" (_SFR_MEM_ADDR(__SPM_REG)),
        "r" ((uint8_t)command),
        "z" ((uint16_t)address),
        "r" ((uint16_t)data)
    : "r0"
    );

    // wait for spm to complete
    //   it doesn't have much sense for __BOOT_PAGE_FILL,
    //   but it doesn't hurt and saves some bytes on 'if'
    boot_spm_busy_wait();
#if defined(RWWSRE)
    // this 'if' condition should be: (command == __BOOT_PAGE_WRITE || command == __BOOT_PAGE_ERASE)...
    // but it's tweaked a little assuming that in every command we are interested in here, there
    // must be also SELFPRGEN set. If we skip checking this bit, we save here 4B
    if ((command & (_BV(PGWRT)|_BV(PGERS))) && (data == 0) ) {
      // Reenable read access to flash
      __boot_rww_enable_short();
    }
#endif
}
