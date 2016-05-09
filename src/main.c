// CONFIG4
#pragma config DSWDTPS = DSWDTPSF // DSWDT Postscale Select (1:2,147,483,648 (25.7 days))
#pragma config DSWDTOSC = LPRC // Deep Sleep Watchdog Timer Oscillator Select (DSWDT uses Low Power RC Oscillator (LPRC))
#pragma config RTCOSC = SOSC // RTCC Reference Oscillator Select (RTCC uses Secondary Oscillator (SOSC))
#pragma config DSBOREN = ON // Deep Sleep BOR Enable bit (BOR enabled in Deep Sleep)
#pragma config DSWDTEN = ON // Deep Sleep Watchdog Timer (DSWDT enabled)

// CONFIG3
#pragma config WPFP = WPFP63 // Write Protection Flash Page Segment Boundary (Highest Page (same as page 21))
#pragma config SOSCSEL = IO // Secondary Oscillator Pin Mode Select (SOSC pins have digital I/O functions (RA4, RB4))
#pragma config WUTSEL = LEG // Voltage Regulator Wake-up Time Select (Default regulator start-up time used)
#pragma config WPDIS = WPDIS // Segment Write Protection Disable (Segmented code protection disabled)
#pragma config WPCFG = WPCFGDIS // Write Protect Configuration Page Select (Last page and Flash Configuration words are unprotected)
#pragma config WPEND = WPENDMEM // Segment Write Protection End Page Select (Write Protect from WPFP to the last page of memory)

// CONFIG2
#pragma config POSCMOD = NONE // Primary Oscillator Select (Primary Oscillator disabled)
#pragma config I2C1SEL = PRI // I2C1 Pin Select bit (Use default SCL1/SDA1 pins for I2C1 )
#pragma config IOL1WAY = OFF // IOLOCK One-Way Set Enable (The IOLOCK bit can be set and cleared using the unlock sequence)
#pragma config OSCIOFNC = ON // OSCO Pin Configuration (OSCO pin functions as port I/O (RA3))
#pragma config FCKSM = CSDCMD // Clock Switching and Fail-Safe Clock Monitor (Sw Disabled, Mon Disabled)
#pragma config FNOSC = FRC // Initial Oscillator Select (Fast RC Oscillator (FRC))
#pragma config PLL96MHZ = OFF // 96MHz PLL Startup Select (96 MHz PLL Startup is enabled by user in software( controlled with the PLLEN bit))
#pragma config PLLDIV = NODIV // USB 96 MHz PLL Prescaler Select (Oscillator input used directly (4 MHz input))
#pragma config IESO = OFF // Internal External Switchover (IESO mode (Two-Speed Start-up) disabled)

// CONFIG1
#pragma config WDTPS = PS32768 // Watchdog Timer Postscaler (1:32,768)
#pragma config FWPSA = PR128 // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config WINDIS = OFF // Windowed WDT (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF // Watchdog Timer (Watchdog Timer is disabled)
#pragma config ICS = PGx1 // Emulator Pin Placement Select bits (Emulator functions are shared with PGEC1/PGED1)
#pragma config GWRP = OFF // General Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF // General Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF // JTAG Port Enable (JTAG port is disabled)

#include <libpic30.h>
#include <energy_management.h>
#include <xc.h>
#include <stdio.h>

void shiftOut(char, char);
void initLCD(void);
void writeChar(unsigned char);
void writeString(char *);
void setCursor(unsigned char, unsigned char);
void initADC(void);
void resetADC(void);
void writeNum(int);

int main() {
    // Initialize the LIN interface
    if (l_sys_init())
        return -1;

    // Initialize the interface
    if (l_ifc_init_UART1())
        return -1;

    // Set UART TX to interrupt level 6
    // Set UART RX to interrupt level 6
    struct l_irqmask irqmask = { 6, 6 };
    l_sys_irq_restore(irqmask);
    
    __delay_ms(100);
    
    //setting output pins for LCD
    TRISAbits.TRISA4 = 0;
    TRISAbits.TRISA3 = 0;
    TRISAbits.TRISA2 = 0;
    
    //setting input for ADC Vrefs
    TRISAbits.TRISA0 = 1;
    TRISAbits.TRISA1 = 1;
    
    //setting input for analog channel
    TRISBbits.TRISB2 = 1;

    initLCD();
    
    initADC();
    resetADC();
    
    writeString("Current:");
   
    AD1CON1bits.ADON = 0b1; //Turning ADC on
    
    
    while(1) {
    }
    
    return 0;
}

char buf[64];
void __attribute__((__interrupt__)) _ADC1Interrupt(void) {
    setCursor(9,1);

    l_u16 x = ADC1BUF0;
    writeNum(x);
    
    if(l_flg_tst_usage_current()) {
        l_flg_clr_usage_current();
        l_u16_wr_usage_current(x);
    }
    
    resetADC();//reset interrupt flag
}

void shiftOut(char highByte, char lowByte){
    char temp;
    char i;
    for(i = 0; i < 8; i++){
        temp = lowByte;
        LATAbits.LATA4 = lowByte >> 7;
        lowByte = temp << 1;
        
        LATAbits.LATA3 = 0b1; //triggering clock pin
        LATAbits.LATA3 = 0b0;
    }
    for(i = 0; i < 8; i++){
        temp = highByte;
        LATAbits.LATA4 = highByte >> 7;
        highByte = temp << 1;
        
        LATAbits.LATA3 = 0b1; //triggering clock pin
        LATAbits.LATA3 = 0b0;
    }
    
    LATAbits.LATA2 = 0b1; //triggering latch pin
    LATAbits.LATA2 = 0b0;
}

void setCursor(unsigned char x, unsigned char y){
    int n;
    if(y == 2){
        x+=192;
        shiftOut(x, 0x01);
        for(n = 0; n < 1000; n++);
        shiftOut(0x00, 0x00);
    }else{
        x+=128;
        shiftOut(x, 0x01);
        for(n = 0; n < 1000; n++);
        shiftOut(0x00, 0x00);
    }
}

void writeString(char * string){
    int i = 0;
    while(string[i] != '\0'){
        writeChar(string[i]);
        i++;
    }
}

void writeChar(unsigned char c){
    int n;
    shiftOut(c, 0b00000011);
    for(n = 0; n < 1000; n++);
    shiftOut(0x00, 0x00);
}

void initLCD(void){
    shiftOut(0, 0);
    
    shiftOut(0b00001111, 0b00000001); //turn LCD on; cursor on; cursor blink on
    shiftOut(0b00001111, 0b00000000);
    
    shiftOut(0b00111100, 0b00000001); //8-bit bus mode; 2-line display mode; 5x11 dots format
    shiftOut(0b00111100, 0);
    
    shiftOut(0b00000001, 0b00000001); //clear display
    shiftOut(0b00000001, 0);
    
    shiftOut(0b00000010, 0b00000001); //return cursor home
    shiftOut(0b00000010, 0);
}

void writeNum(int num){
    num *= 100;
    
    char hun = (num / 10000);
    writeChar(hun + 48);
    num -= 10000 * hun;
    
    char ten = (num / 1000);
    writeChar(ten + 48);
    num -= 1000 * ten;
    
    char fir = (num / 100);
    writeChar(fir + 48);
    num -= 100 * fir;
    
    writeChar('.');
    
    char tenths = (num / 10);
    writeChar(tenths+48);
    num -= 10 * tenths;
    
    char thous = (num / 1);
    writeChar(thous+48);
}

void initADC(void){
    AD1CON1bits.FORM = 0b00; //Data output format as 0000 00dd dddd dddd
    AD1CON1bits.SSRC = 0b111; //Internal counter ends sampling and starts conversion (auto-convert)
    AD1CON1bits.ASAM = 0b1; //Sampling begins immediately after last conversion completes; SAMP bit is auto-set
    
    AD1CON2bits.VCFG = 0b000; //Voltage Reference as VDD and VSS
    AD1CON2bits.CSCNA = 0b0; //Does not scan inputs
    AD1CON2bits.SMPI = 0b111; //Interrupts at the completion of conversion for each 16th sample/convert sequence
    AD1CON2bits.BUFM = 0b0; //Buffer configured as one 16-word buffer(ADC1BUFn<15:0>)
    AD1CON2bits.ALTS = 0b0; //Always uses MUX A input multiplexer settings
    
    AD1CON3bits.ADRC = 0b1; //A/D internal RC clock
    AD1CON3bits.SAMC = 0b11111; //31 ? TAD
    AD1CON3bits.ADCS = 0b00111111; //64 ? TCY
    
    IPC3bits.AD1IP = 0b011; //Interrupt is Priority 3 (highest priority interrupt)
    
    AD1CHSbits.CH0SA = 4; //Channel 0 positive input is AN4
    
    AD1PCFGbits.PCFG4 = 0; //AN4-Pin is configured in Analog mode; I/O port read is disabled, A/D samples pin voltage
    
    AD1CON1bits.SAMP = 0b1; //Start sampling
}

void resetADC(void){
    IFS0bits.AD1IF = 0b0; //clearing ADC interrupt flag
    IEC0bits.AD1IE = 0b1; //enabling ADC interrupt
}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt()
{
    if (IFS0bits.U1TXIF) {
        IFS0bits.U1TXIF = 0;
        l_ifc_tx_UART1();
    }
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt()
{
    if (IFS0bits.U1RXIF) {
        IFS0bits.U1RXIF = 0;
        l_ifc_rx_UART1();
    }
}

struct l_irqmask l_sys_irq_disable()
{
    IEC0bits.U1RXIE = 0;
    IEC0bits.U1TXIE = 0;
    IFS0bits.U1TXIF = 0;
    IFS0bits.U1RXIF = 0;
    struct l_irqmask mask = { IPC2bits.U1RXIP, IPC3bits.U1TXIP };
    return mask;
}

void l_sys_irq_restore(struct l_irqmask previous)
{
    IPC2bits.U1RXIP = previous.rx_level;
    IFS0bits.U1TXIF = 0;
    IEC0bits.U1RXIE = 1;

    IPC3bits.U1TXIP = previous.tx_level;
    IFS0bits.U1RXIF = 0;
    IEC0bits.U1TXIE = 1;
}
