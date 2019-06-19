// code that is working presently

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    	#include <p33Exxxx.h>
    #if defined(__dsPIC33E__)
    #elif defined(__dsPIC33F__)
    	#include <p33Fxxxx.h>
    #endif
#endif

#include <stdio.h>        
#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */
#include <stdlib.h>
#include <string.h>
#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp              */
#include "libpic30.h"      // für __delay_ms() von Microchip
#include "i2c.h"           // see <https://people.ece.cornell.edu/land/courses/ece4760/PIC32/Microchip_stuff/32-bit-Peripheral-Library-Guide.pdf>

//#include "Aufgaben.h"      // Funktionen / Typen / Variablen von andernen Aufgaben verfügbar machen

// MACROS
// maximum size of the string while receiving from UART
#define STR_SIZE 3

#define EXP2_3 LATBbits.LATB0

#define S_TO_MS   1000
#define TCN75_SLV_ADR 1001000

// I2C Macros

#define	TEMP_C_INIT

#define SIGNED_WORD int16_t


#define	I2C_ADDRESS                     0x90

#define	I2C_START_I2C					StartI2C1					//Generate bus start condition
#define	I2C_START_IN_PROGRESS_BIT		I2C1CONbits.SEN				//Bit indicating start is still in progress
#define	I2C_RESTART_I2C                 RestartI2C1					//Generate bus restart condition
#define	I2C_RESTART_IN_PROGRESS_BIT     I2C1CONbits.RSEN			//Bit indicating re-start is still in progress
#define	I2C_STOP_I2C					StopI2C1					//Generate bus stop condition
#define	I2C_STOP_IN_PROGRESS_BIT		I2C1CONbits.PEN				//Bit indicating Stop is still in progress
#define	I2C_WRITE_BYTE(a)				MasterWriteI2C1(a)			//Write byte to I2C device
#define	I2C_TX_IN_PROGRESS_BIT			I2C1STATbits.TRSTAT			//Bit indicating transmit byte is still in progress
#define	I2C_ACK_NOT_RECEIVED_BIT		I2C1STATbits.ACKSTAT		//Bit that is high when ACK was not received
#define	I2C_READ_BYTE					MasterReadI2C1()			//Read byte from I2C device function / result byte of I2C__READ_FUNCTION_START
#define I2C_ACK                         AckI2C1						//Generate bus ACK condition
#define I2C_NOT_ACK                     NotAckI2C1					//Generate bus Not ACK condition
#define	I2C_ACK_IN_PROGRESS_BIT         I2C1CONbits.ACKEN			//Bit indicating ACK is still in progress
#define	I2C_IDLE_I2C					IdleI2C1					//Test if I2C1 module is idle (wait until it is ready for next operation)


// initialize TIMER 1

void init_ms_t1( void )
{

    T1CON = 0;          // Timer reset
    TMR1 = 0x0000;
    // PR1 is 1 ms with the current calculations

    if (SYS_FREQ < 130000000L){
        PR1 = (SYS_FREQ /2/S_TO_MS) - 1;
    }
    else{
        T1CONbits.TCKPS = 0b01; // Prescaler
        PR1 = (SYS_FREQ /2/8/S_TO_MS) - 1;
    }

    T1CONbits.TON = 1;  // Enable Timer1 and start the counter
}

// timer 2

void init_ms_t2( void )
{

    T2CON = 0;          // Timer reset
    TMR2 = 0x0000;
    // PR2 is 1 ms with the current calculations

    if (SYS_FREQ < 130000000L){
        PR2 = (SYS_FREQ /2/S_TO_MS) - 1;
    }
    else{
        T2CONbits.TCKPS = 0b01; // Prescaler
        PR2 = (SYS_FREQ /2/8/S_TO_MS) - 1;
    }

    T2CONbits.TON = 1;  // Enable Timer2 and start the counter
}

// initialize I2C function

void I2C_Init()
{
    // Configre SCA/SDA pin as open-drain (it is). This may change from device to device.
    //Refer the datasheet for more information.
	
	// this will be needed to initialize the I2C
    ODCDbits.ODCD9 = 0;
    ODCDbits.ODCD10 = 0;

    I2C1CONbits.A10M = 0;
    I2C1CONbits.SCLREL = 1;
    I2C1BRG = 300;

    I2C1ADD = 0;
    I2C1MSK = 0;

    I2C1CONbits.I2CEN = 1;
    IEC1bits.MI2C1IE = 1;
    IFS1bits.MI2C1IF = 0;
}

//Returns 1 if temperature was read sucessfully, 0 if not
//temperature:
//	Temperature reading x0.5ºC (0x0000 = 0ºC)
int read_temperature (int *temperature)
{

	//Send Start
	I2C_IDLE_I2C();					//Wait for I2C bus to be ready
	I2C_START_I2C();
	while(I2C_START_IN_PROGRESS_BIT)
		;

	//Write Slave address and with RW bit 0 for write
	I2C_IDLE_I2C();
	I2C_WRITE_BYTE(I2C_ADDRESS);
	while(I2C_TX_IN_PROGRESS_BIT)
		;
	if(I2C_ACK_NOT_RECEIVED_BIT ==1)
		goto read_temperature_fail;
	
	//Write pointer to Temperature register
	I2C_IDLE_I2C();
	I2C_WRITE_BYTE(0x00);
	while(I2C_TX_IN_PROGRESS_BIT);
	if(I2C_ACK_NOT_RECEIVED_BIT == 1)
		goto read_temperature_fail;

	//Send Restart
	I2C_IDLE_I2C();
	I2C_RESTART_I2C();
	while(I2C_RESTART_IN_PROGRESS_BIT)
		;

	//Write Slave address and with RW bit 1 for read
	I2C_IDLE_I2C();
	I2C_WRITE_BYTE(I2C_ADDRESS | 0x01);
	while(I2C_TX_IN_PROGRESS_BIT)
		;
	if(I2C_ACK_NOT_RECEIVED_BIT == 1)
		goto read_temperature_fail;

	//Get byte 0
	I2C_IDLE_I2C();
	#ifdef I2C_READ_BYTE_START
		I2C_READ_BYTE_START
	#endif
	*temperature = ((SIGNED_WORD)I2C_READ_BYTE << 1);
	if (*temperature & 0x0100)				//If value is negative then set all the high bits
		*temperature |= 0xfe00;

	//Send Ack
	I2C_IDLE_I2C();
	I2C_ACK();
	while(I2C_ACK_IN_PROGRESS_BIT)
		;

	//Get byte 1 (Least significant bit is in bit 7, rest are unused)
	I2C_IDLE_I2C();
	#ifdef I2C_READ_BYTE_START
		I2C_READ_BYTE_START
	#endif
	if (I2C_READ_BYTE & 0x80)
		*temperature |= 1;

	//Send NAK
	I2C_IDLE_I2C();
	I2C_NOT_ACK();
	while(I2C_ACK_IN_PROGRESS_BIT)
		;

	//Send Stop
	I2C_IDLE_I2C();
	I2C_STOP_I2C();
	while(I2C_STOP_IN_PROGRESS_BIT)
		;

	
	return(1);



read_temperature_fail:
	//Send Stop
	I2C_IDLE_I2C();
	I2C_STOP_I2C();
	while(I2C_STOP_IN_PROGRESS_BIT)
		;

	*temperature = 0;

	return(0);
}

int16_t putcUART(char c)
{
	if (U1STAbits.TRMT ==  1) 	// if the register equals 1, then the buffer is empty
								// and therefore data can be written on again
	{
		U1TXREG = c;
		return 1;
	}
	else
	{
		return 0;
	}

}

int16_t putsUART(const char *str)
{
	int i = 0;
	int ret;
    while (str[i]!= '\0') 
	{
        ret = putcUART(str[i]); // this function is already going to check if the buffer is
								// free or not
		i++;

		// couldn't transmit entire string
		if (ret == 0)
		{
			return -i;
		}
	}
	
	// entire string was transmitted
    return i;
}

// I HAVE PROBLEMS WITH THIS
int16_t getcUART(char *c)
{
    // it is not working because the U1RXREG is always = 0 
    // the interrupt is never triggered
    *c = U1RXREG;
    
    return 1;
}


//int16_t getsUART(char *s)
//{
//    
//    int i;
//    for (i = 0 ; i<STR_SIZE; i++)
//    {
//        getcUART(s[i]);
//        if ( s[i] == '\0')
//        {
//            break;
//        }
//    }
//    
//    return 1;
//}


void ZeroArray(char *str)
{
    int16_t i;
    for (i = 0; i< STR_SIZE; i++)
    {
        str[i] = '\0';
    }
}


void init_UART1(int baudrate)
{
    int16_t brgval = (FP/baudrate)/16 -1;
    
//    U1BRG = BRGVAL1;   
    U1BRG = brgval;   
    U1MODEbits.BRGH = 0;
    U1MODEbits.PDSEL = 0b00; // 8 bit, no parity
    U1MODEbits.STSEL = 0;    // 1 Stop Bit
    U1MODEbits.UEN = 0b00;   // nur RX und TX
    U1STAbits.URXISEL = 0;   // Interrupt after one RX character is received;
    U1MODEbits.UARTEN = 1;   // UART Enable
    U1STAbits.UTXEN = 1;     // TX Enable
    _U1TXIE = 1;
    _U1RXIE = 1;            
}

// counter variable - counts ms until 1000
int16_t counter_to_1s = 0; 

// für primitive Sendefunktionen Main Loop und Interrupt
char buffer[10];


int value = 10;

volatile char    SendString[] = "Hello World\n";
volatile int16_t SendIndex = 0;
// Global variables are automatically initialized to zero
volatile char    ReceiveString[STR_SIZE]; 
volatile int16_t ReceiveIndex = 0;


void __attribute__((interrupt,auto_psv)) _U1TXInterrupt(void)
{
    _U1TXIF = 0;

    
    //
    sprintf(buffer, "%d", value);
    
    if (SendString[SendIndex] != 0)
    {
        U1TXREG = SendString[SendIndex];
        SendIndex++;
    }

}

// UART_RECEIVE INTERRUPT ROUTINE WITHOUT STRCMP()

void __attribute__((interrupt,auto_psv)) _U1RXInterrupt(void)
{
    _U1RXIF = 0;
    
    getcUART(&ReceiveString[ReceiveIndex]);
    

     
    //echo example 
    /*    
    char c;    
    getcUART(c);
    putcUART(c);
    */    
}



// UART_RECEIVE INTERRUPT ROUTINE with STRCMP()

//void __attribute__((interrupt,auto_psv)) _U1RXInterrupt(void)
//{
//    _U1RXIF = 0;
//    
//    getcUART(&ReceiveString[ReceiveIndex]);
//    ReceiveIndex++; 
//
//    // I could have use the function "strcmp(const char* str1, const char* str2)"
//    if ( ReceiveString[ReceiveIndex] == '\n')
//    {
//        if (!strcmp( ReceiveString, "B0\n" ))
//        {
//            LATBbits.LATB8 = !LATBbits.LATB8;
//            
//        } 
//        else if(!strcmp( ReceiveString, "R1\n" ))
//        {
//            LATBbits.LATB9 = !LATBbits.LATB9;
//            
//        }
//        else if (!strcmp( ReceiveString, "S2\n" ))
//        {
//            LATBbits.LATB10 = !LATBbits.LATB10;
//        }
//        
//        // Don't need this function
//        ZeroArray(ReceiveString);
//        // Resets the ReceiveIndex
//        ReceiveIndex = 0; 
//    }
//     
//    //echo example 
//    
//    //char c;    
//    //getcUART(c);
//    //putcUART(c);
//       
//}

//---------------------------------------------------------------------------------
// Main
//---------------------------------------------------------------------------------
int16_t main(void)
{
    ConfigureOscillator();
    
    
    // Pins für LEDs auf Ausgang, Pins sind nach dem Reset auf Eingang 
    //  (Ausgangstreiber deaktiviert)
    TRISB &= 0xF0FF;             // Alternativ, nur <11:8> auf 0, Rest unverändert
    
    // Pins für die LEDs haben auch eine Analogfunktionalität.
    // Diese ist nach dem Reset aktiviert, bei digitaler Verwendung, muss Analog
    // deaktiviert werden
    ANSELB &= 0xF0FF;     // ANSELB<11:8>  Pin auf digital -> 0
    
    // Taster ziehen die Leitung nach 0 Volt, Taster offen -> Eingang offen
    // -> floatend -> Pull Up Widerstand nötig
    CNPUG |= 0xF000;  // Alternativ <15:12> Pull Up Enable, Rest bleibt unverändert
    CNPUGbits.CNPUG9 = 1; // Inkrementalgeber Taster
    
    // EXP2_3 _LATB0 und EXP2_4 _LATB1 als Ausgänge
    TRISB  &= ~0x03;  // Auf Bit 0/1 auf 0 -> Ausgang
    ANSELB &= ~0x03;  // Auf Bit 0/1 auf 0 -> Digital
	
	// set UART input to digital (Port E) See EDA table
    ANSELEbits.ANSE8 = 0;
    // to connect RX PIN (see EDA's pin table) with UART1
    RPINR18bits.U1RXR = 88; 

    RPINR3bits.T2CKR = 32;  // EXP_2_3: RB0 - RPI32 auf T2 Clock Input mappen

    CNENGbits.CNIEG9 = 1;   // interrupt on change für alle Taster
    CNENG |= 0xF000;
    
    _CNIF = 0; 
    _CNIP = 1;
    _CNIE = 1;    // enable CN 
        
    // timer initialization
    init_ms_t1();
    init_ms_t2();
    I2C_Init();
    
    uint32_t betriebszeit = 0;

    // Prozessorauslastung
    //    init_T23_gated();
    RPINR3bits.T2CKR = 32;  // EXP_2_3: RB0 - RPI32 auf T2 Clock Input mappen

	// Pin Remap
    RPOR1bits.RP66R   = _RPOUT_U1TX;

	// UART Initialisierung
    init_UART1(9600);

    

    
    
    while(1)
    {
        // All LEDs
            if(_T1IF == 1)
			{
                _T1IF = 0;
                
                if (counter_to_1s == 1000)
                {
                    counter_to_1s = 0;
                }
                // the uart is working in an interrupt
                // i actually wanted to print a message with the current
                // every second to the console through uart.

                
                // missing reading temperature at the right time
                // setting variable and sending data via UART
            }            
                            
    }    


}