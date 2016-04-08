// CONFIG4
#pragma config DSWDTPS = DSWDTPSF // DSWDT Postscale Select (1:2,147,483,648 (25.7 days))
#pragma config DSWDTOSC = LPRC // Deep Sleep Watchdog Timer Oscillator Select (DSWDT uses Low Power RC Oscillator (LPRC))
#pragma config RTCOSC = SOSC // RTCC Reference Oscillator Select (RTCC uses Secondary Oscillator (SOSC))
#pragma config DSBOREN = OFF // Deep Sleep BOR Enable bit (BOR disabled in Deep Sleep)
#pragma config DSWDTEN = OFF // Deep Sleep Watchdog Timer (DSWDT disabled)

// CONFIG3
#pragma config WPFP = WPFP63 // Write Protection Flash Page Segment Boundary (Highest Page (same as page 21))
#pragma config SOSCSEL = IO // Secondary Oscillator Pin Mode Select (SOSC pins have digital I/O functions (RA4, RB4))
#pragma config WUTSEL = LEG // Voltage Regulator Wake-up Time Select (Default regulator start-up time used)
#pragma config WPDIS = WPDIS // Segment Write Protection Disable (Segmented code protection disabled)
#pragma config WPCFG = WPCFGDIS // Write Protect Configuration Page Select (Last page and Flash Configuration words are unprotected)
#pragma config WPEND = WPENDMEM // Segment Write Protection End Page Select (Write Protect from WPFP to the last page of memory)

// CONFIG2
#pragma config POSCMOD = HS // Primary Oscillator Select (HS Oscillator mode selected)
#pragma config I2C1SEL = PRI // I2C1 Pin Select bit (Use default SCL1/SDA1 pins for I2C1 )
#pragma config IOL1WAY = OFF // IOLOCK One-Way Set Enable (Once set, the IOLOCK bit cannot be cleared)
#pragma config OSCIOFNC = OFF // OSCO Pin Configuration (OSCO pin functions as clock output (CLKO))
#pragma config FCKSM = CSDCMD // Clock Switching and Fail-Safe Clock Monitor (Sw Disabled, Mon Disabled)
#pragma config FNOSC = FRCDIV // Initial Oscillator Select (Fast RC Oscillator with Postscaler (FRCDIV))
#pragma config PLL96MHZ = OFF // 96MHz PLL Startup Select (96 MHz PLL Startup is enabled automatically on start-up)
#pragma config PLLDIV = NODIV // USB 96 MHz PLL Prescaler Select (Oscillator input used directly (4 MHz input))
#pragma config IESO = ON // Internal External Switchover (IESO mode (Two-Speed Start-up) enabled)

// CONFIG1
#pragma config WDTPS = PS32768 // Watchdog Timer Postscaler (1:32,768)
#pragma config FWPSA = PR128 // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config WINDIS = OFF // Windowed WDT (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF // Watchdog Timer (Watchdog Timer is disabled)
#pragma config ICS = PGx1 // Emulator Pin Placement Select bits (Emulator functions are shared with PGEC1/PGED1)
#pragma config GWRP = OFF // General Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF // General Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF // JTAG Port Enable (JTAG port is disabled)

#include <xc.h>
#include <libpic30.h>
#include <energy_management.h>

void main_task()
{
    // Put the node specific function here!
}

int main()
{
    // Initialize the LIN interface
    if (l_sys_init())
        return -1;

    // Initialize the interface
    if (l_ifc_init_UART1())
        return -1;

    struct l_irqmask irqmask = { 4, 4, 7 };
    l_sys_irq_restore(irqmask);
    
    l_bool configuration_ok = false;
    l_u16 configuration_timeout = 1000;
    do {
        if (l_ifc_read_status_UART1() & (1 << 6)) {
            configuration_ok = true;
            break;
        }
        __delay_ms(5);
        configuration_timeout--;
    } while (configuration_timeout || !configuration_ok);
    
    if (!configuration_ok) {
        // Master did not configure this node.
        return -1;
    }

    // TODO move this because RA0 is used for VBATRATIO
    TRISAbits.TRISA0 = 1; // AN0 input
    AD1PCFGbits.PCFG0 = 0; // AN0 analog

    AD1CON2bits.VCFG = 0; // Vr+ = AVdd and Vr- = AVss
    AD1CON3bits.ADCS = 0; // TCY
    AD1CON1bits.SSRC = 7; // Internal counter ends sampling and starts conversion (auto-convert)
    AD1CON3bits.SAMC = 1; // 1 TAD
    AD1CON1bits.FORM = 0; // Integer (0000 00dd dddd dddd)
    AD1CON2bits.SMPI = 0; // Interrupts are at the completion of conversion for each sample/convert sequence
    AD1CON1bits.ADON = 1; // Turn on the A/D

    IFS0bits.AD1IF = 0; // Clear the A/D interrupt flag
    IPC3bits.AD1IP = 3; // Set the interrupt A/D priority to 3
    IEC0bits.AD1IE = 1; // Enable the A/D interrupt

    while (1) {
        main_task();
    }

    return -1;
}

void __attribute__((interrupt, no_auto_psv)) _ADC1Interrupt()
{
    if (IFS0bits.AD1IF) {
        IFS0bits.AD1IF = 0;
        l_u16_wr_battery_voltage(ADC1BUF0); // Save the battery voltage and schedule it for LIN transmission.
    }
}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt()
{
    if (IFS0bits.U1TXIF) {
        IFS0bits.U1TXIF = 0;

        l_ifc_tx_UART1();

        if (U1STAbits.FERR)
            U1STAbits.FERR = 0;
    }
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt()
{
    if (IFS0bits.U1RXIF) {
        IFS0bits.U1RXIF = 0;

        l_ifc_rx_UART1();

        if (U1STAbits.FERR)
            U1STAbits.FERR = 0;
    }
}

struct l_irqmask l_sys_irq_disable()
{
    IEC0bits.U1RXIE = 0;
    IEC0bits.U1TXIE = 0;
    IEC0bits.IC1IE = 0;
    IFS0bits.U1TXIF = 0;
    IFS0bits.U1RXIF = 0;
    struct l_irqmask mask = { IPC2bits.U1RXIP, IPC3bits.U1TXIP, IPC0bits.IC1IP };
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

    IPC0bits.IC1IP = previous.t1_level;
    IFS0bits.IC1IF = 0;
    IEC0bits.IC1IE = 1;
}
