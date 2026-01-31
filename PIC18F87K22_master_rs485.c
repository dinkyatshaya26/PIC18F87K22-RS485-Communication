// PIC18F87K22_TX

// CONFIG1L
#pragma config RETEN = OFF      // VREG Sleep Enable bit (Disabled - Controlled by SRETEN bit)
#pragma config INTOSCSEL = HIGH // LF-INTOSC Low-power Enable bit (LF-INTOSC in High-power mode during Sleep)
#pragma config SOSCSEL = HIGH   // SOSC Power Selection and mode Configuration bits (High Power SOSC circuit selected)
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)

// CONFIG1H
#pragma config FOSC = INTIO2    // Oscillator (Internal RC oscillator)
#pragma config PLLCFG = OFF     // PLL x4 Enable bit (Disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor (Disabled)
#pragma config IESO = OFF       // Internal External Oscillator Switch Over Mode (Disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power Up Timer (Disabled)
#pragma config BOREN = OFF      // Brown Out Detect (Disabled in hardware, SBOREN disabled)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (1.8V)
#pragma config BORPWR = ZPBORMV // BORMV Power level (ZPBORMV instead of BORMV is selected)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer (WDT disabled in hardware; SWDTEN bit disabled)
#pragma config WDTPS = 1048576  // Watchdog Postscaler (1:1048576)

// CONFIG3L
#pragma config RTCOSC = SOSCREF // RTCC Clock Select (RTCC uses SOSC)
#pragma config EASHFT = ON      // External Address Shift bit (Address Shifting enabled)
#pragma config ABW = MM         // Address Bus Width Select bits (8-bit address bus)
#pragma config BW = 16          // Data Bus Width (16-bit external bus mode)
#pragma config WAIT = OFF       // External Bus Wait (Disabled)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 Mux (RC1)
#pragma config ECCPMX = PORTE   // ECCP Mux (Enhanced CCP1/3 [P1B/P1C/P3B/P3C] muxed with RE6/RE5/RE4/RE3)
#pragma config MSSPMSK = MSK7   // MSSP address masking (7 Bit address masking mode)
#pragma config MCLRE = OFF      // Master Clear Enable (MCLR Disabled, RG5 Enabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Overflow Reset (Enabled)
#pragma config BBSIZ = BB2K     // Boot Block Size (2K word Boot Block size)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protect 00800-03FFF (Disabled)
#pragma config CP1 = OFF        // Code Protect 04000-07FFF (Disabled)
#pragma config CP2 = OFF        // Code Protect 08000-0BFFF (Disabled)
#pragma config CP3 = OFF        // Code Protect 0C000-0FFFF (Disabled)
#pragma config CP4 = OFF        // Code Protect 10000-13FFF (Disabled)
#pragma config CP5 = OFF        // Code Protect 14000-17FFF (Disabled)
#pragma config CP6 = OFF        // Code Protect 18000-1BFFF (Disabled)
#pragma config CP7 = OFF        // Code Protect 1C000-1FFFF (Disabled)

// CONFIG5H
#pragma config CPB = OFF        // Code Protect Boot (Disabled)
#pragma config CPD = OFF        // Data EE Read Protect (Disabled)

// CONFIG6L
#pragma config WRT0 = OFF       // Table Write Protect 00800-03FFF (Disabled)
#pragma config WRT1 = OFF       // Table Write Protect 04000-07FFF (Disabled)
#pragma config WRT2 = OFF       // Table Write Protect 08000-0BFFF (Disabled)
#pragma config WRT3 = OFF       // Table Write Protect 0C000-0FFFF (Disabled)
#pragma config WRT4 = OFF       // Table Write Protect 10000-13FFF (Disabled)
#pragma config WRT5 = OFF       // Table Write Protect 14000-17FFF (Disabled)
#pragma config WRT6 = OFF       // Table Write Protect 18000-1BFFF (Disabled)
#pragma config WRT7 = OFF       // Table Write Protect 1C000-1FFFF (Disabled)

// CONFIG6H
#pragma config WRTC = OFF       // Config. Write Protect (Disabled)
#pragma config WRTB = OFF       // Table Write Protect Boot (Disabled)
#pragma config WRTD = OFF       // Data EE Write Protect (Disabled)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protect 00800-03FFF (Disabled)
#pragma config EBTR1 = OFF      // Table Read Protect 04000-07FFF (Disabled)
#pragma config EBTR2 = OFF      // Table Read Protect 08000-0BFFF (Disabled)
#pragma config EBTR3 = OFF      // Table Read Protect 0C000-0FFFF (Disabled)
#pragma config EBTR4 = OFF      // Table Read Protect 10000-13FFF (Disabled)
#pragma config EBTR5 = OFF      // Table Read Protect 14000-17FFF (Disabled)
#pragma config EBTR6 = OFF      // Table Read Protect 18000-1BFFF (Disabled)
#pragma config EBTR7 = OFF      // Table Read Protect 1C000-1FFFF (Disabled)

// CONFIG7H
#pragma config EBTRB = OFF      // Table Read Protect Boot (Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define _XTAL_FREQ 8000000



#define RE_DE LATDbits.LATD0  // RS-485 Direction Control

void UART_Init() {
    TRISC6 = 0;  // TX (output)
    TRISC7 = 1;  // RX (input)
    
    TXSTA1 = 0x24;     // BRGH=1, TXEN=1
    RCSTA1 = 0x90;     // SPEN=1, CREN=1
    BAUDCON1 = 0x08;   // BRG16=1
    SPBRG1 = 51;       // 9600 baud at 8MHz
}

void UART_Write(char data) {
    while (!TX1IF);       // Wait until TXREG is empty
    TXREG1 = data;
}

void UART_Write_Text(const char *text) {
    while (*text) {
        UART_Write(*text++);
        __delay_ms(20);     // Allow slave LCD to process
    }

    while (!TXSTA1bits.TRMT);  // Wait for the last byte to finish transmission
}

void main() {
    OSCCON = 0x70;    // Internal oscillator at 8 MHz

    TRISD0 = 0;       // RD0 as output (RE/DE control)
    RE_DE = 0;        // Initially receive mode

    UART_Init();

    __delay_ms(1000); // Wait for Slave to power up and init LCD

    RE_DE = 1;        // Enable transmit mode
    __delay_ms(1);    // Give MAX487 time to settle

    UART_Write_Text("HELLO");
    UART_Write_Text(" FROM  MASTER");

    __delay_ms(2);    // Just a little buffer
    RE_DE = 0;        // Return to receive mode

    while (1);
}