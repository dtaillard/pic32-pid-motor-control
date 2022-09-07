#include <xc.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <math.h>

#include "lcd_display_driver.h"

//
// Configure CPU and perpherial bus clock to run at 80MHz
//
#pragma config POSCMOD = HS
#pragma config FNOSC = PRIPLL
#pragma config FPLLMUL = MUL_20
#pragma config FPLLIDIV = DIV_2
#pragma config FPLLODIV = DIV_1
#pragma config FPBDIV = DIV_1

#define CP0_FREQUENCY 40000000.0

#define ENCODER_COUNTS_PER_REVOLUTION 48
#define GEARTRAIN_RATIO 99

#define ADC_SAMP_DELAY 100

static volatile int encoderState = 0;
static volatile int encoderCount = 0;
static volatile double targetDegrees = 0;

static volatile int prevLine1Length = 0;
static volatile int prevLine2Length = 0;

static volatile int transmitCount = 0;
static volatile int receivedTime = 0;

static volatile double proportionalGain = 0;
static volatile double integralGain = 0;
static volatile double integralSum = 0;
static volatile double referenceAmplitude = 0;
static volatile double referenceFrequency = 0;

static volatile double periodicTime = 0;

double degreesFromCounts(double counts) {
    return 360 * (counts / ENCODER_COUNTS_PER_REVOLUTION) / GEARTRAIN_RATIO;
}

void read_uart2(char *message, int max_length) {
    char data = 0;
    int complete = 0, num_bytes = 0;
    while(!complete) {
        if(U2STAbits.URXDA) {
            data = U2RXREG;
            if((data == '\n') || (data == '\r')) {
                complete = 1;
            } else {
                message[num_bytes] = data;
                num_bytes++;
                if(num_bytes >= max_length) {
                    num_bytes = 0;
                }
            }
        }
    }
    message[num_bytes] = '\0';
}

void write_uart2(const char *string) {
    while (*string != '\0') {
        while (U2STAbits.UTXBF) {
           ; // wait until TX buffer isn't full 
        }
        U2TXREG = *string;
        ++string;
    }
}

void updateDisplay() {
    int length1, length2;
    char line1[128], line2[128];

    length1 = sprintf(line1, "Target: %.2f\xDF", targetDegrees);
    length2 = sprintf(line2, "Current: %.2f\xDF", degreesFromCounts(encoderCount));

    if((length1 < prevLine1Length) || (length2 < prevLine2Length)) {
        lcd_display_driver_clear();
    }
    
    display_driver_use_first_line();
    lcd_display_driver_write(line1, length1);
    prevLine1Length = length1;

    display_driver_use_second_line();
    lcd_display_driver_write(line2, length2);
    prevLine2Length = length2;
}

void __ISR(_CHANGE_NOTICE_VECTOR, IPL5SOFT) CNISR(void) {
    int nextEncoderState = (PORTG & 0xC0) >> 6;
        
    if(nextEncoderState == 0 && encoderState == 1) {
        encoderCount--;
    } else if(nextEncoderState == 0 && encoderState == 2) {
        encoderCount++;
    } else if(nextEncoderState == 1 && encoderState == 0) {
        encoderCount++;
    } else if(nextEncoderState == 1 && encoderState == 3) {
        encoderCount--;
    } else if(nextEncoderState == 2 && encoderState == 0) {
        encoderCount--;
    } else if(nextEncoderState == 2 && encoderState == 3) {
        encoderCount++;
    } else if(nextEncoderState == 3 && encoderState == 1) {
        encoderCount++;
    } else if(nextEncoderState == 3 && encoderState == 2) {
        encoderCount--;
    }
    
    encoderState = nextEncoderState;
    IFS1bits.CNIF = 0;
}

void __ISR(_TIMER_4_VECTOR, IPL4SOFT) IntTimer4Handler(void) {
    if(transmitCount == 0) {
        // Stop motor
        LATFbits.LATF0 = 0;
        LATFbits.LATF1 = 0;
        IFS0bits.T4IF = 0;
        return;
    }
    
    double elapsedTime = 0.05; // Timer4 running at 20Hz: 1/20Hz = 0.05s
    periodicTime += elapsedTime;
    targetDegrees = referenceAmplitude * sin(2*M_PI * referenceFrequency * periodicTime);
    
    double errorDegrees = degreesFromCounts(encoderCount) - targetDegrees;
    int output = proportionalGain * errorDegrees + integralGain * integralSum;
    integralSum += elapsedTime * errorDegrees;
    
    // Set OC4 compare value to clamped output (0 - 1023)
    OC4RS = (output > 1023) ? 1023 : output;
    
    if(errorDegrees > 0) {
        // Decrease encoder count
        LATFbits.LATF0 = 0;
        LATFbits.LATF1 = 1;
    } else if(errorDegrees < 0) {        
        // Increase encoder count
        LATFbits.LATF0 = 1;
        LATFbits.LATF1 = 0;
    }

    IFS0bits.T4IF = 0;
}

void __ISR(_TIMER_2_VECTOR, IPL4SOFT) IntTimer2Handler(void) {
    updateDisplay();
    IFS0bits.T2IF = 0;
}

void __ISR(_UART_2_VECTOR, IPL3SOFT) IntUart2Handler(void) {    
    char received_str[64];
    double tempRefAmplitude, tempRefFrequency;

    if(IFS1bits.U2RXIF) {
        // Parse reference amplitude
        read_uart2(received_str, 64);
        tempRefAmplitude = atof(received_str);
        
        // Parse reference frequency
        read_uart2(received_str, 64);
        tempRefFrequency = atof(received_str);
        
        // Parse proportional gain
        read_uart2(received_str, 64);
        proportionalGain = atof(received_str);
         
        // Parse integral gain
        read_uart2(received_str, 64);
        integralGain = atof(received_str);
        
        receivedTime = _CP0_GET_COUNT();    // Save the received time
        IEC0bits.T5IE = 1;                  // Enable Timer5 interrupt
        
        referenceAmplitude = tempRefAmplitude;
        referenceFrequency = tempRefFrequency;
        
        IFS1bits.U2RXIF = 0;                // Clear interrupt flag
    } else if(IFS1bits.U2TXIF) {
    } else if(IFS1bits.U2EIF) {
    }
}

void __ISR(_TIMER_5_VECTOR, IPL6SOFT) IntTimer5Handler(void) {
    char message[16];
    
    // Publish 1000 samples of the transient response
    if((transmitCount++) == 1000) {        
        IEC0bits.T5IE = 0;      // Disable Timer5 interrupt again
        transmitCount = 0;      // Reset transmit count for next transmission
    } else {
        sprintf(message, "%0.4f\r\n", (float)((_CP0_GET_COUNT() - receivedTime) / CP0_FREQUENCY));
        write_uart2(message);

        sprintf(message, "%0.4f\r\n", (float)targetDegrees);
        write_uart2(message);

        sprintf(message, "%0.4f\r\n", (float)degreesFromCounts(encoderCount));
        write_uart2(message);
    }
    
    IFS0bits.T5IF = 0;
}

int main() {
    // Turn off JTAG
    DDPCON = 0;
    
    // debugging
    LATA = 0;
    TRISAbits.TRISA0 = 0;
    
    // G6/G7 -> Encoder inputs
    TRISGbits.TRISG6 = 1;
    TRISGbits.TRISG7 = 1;

    // G0/F1 -> IN1/IN2 on the H-bridge
    LATF = 0;
    TRISFbits.TRISF0 = 0;
    TRISFbits.TRISF1 = 0;
    
    // Enable internal pull-ups
    CNPUEbits.CNPUE8 = 1;
    CNPUEbits.CNPUE9 = 1;
    
    // Enable multi-vectored interrupts
    INTCONbits.MVEC = 1;
    
    __builtin_disable_interrupts();

    //
    // Setup CN interrupt
    //
    CNCONbits.ON = 1;
    CNENbits.CNEN8 = 1;
    CNENbits.CNEN9 = 1;
    
    IPC6bits.CNIP = 5;
    IFS1bits.CNIF = 0;
    IEC1bits.CNIE = 1;
    
    //
    // Setup Timer4 interrupt
    //
    PR4 = 15625; // ((80MHz) / 256) / 15625 = 20Hz
    TMR4 = 0;

    T4CONbits.TCKPS = 7;
    T4CONbits.TGATE = 0;
    T4CONbits.TCS = 0;
    T4CONbits.ON = 1;
    
    IPC4bits.T4IP = 4;
    IPC4bits.T4IS = 1;
    IFS0bits.T4IF = 0;
    IEC0bits.T4IE = 1;
    
    //
    // Setup Timer2 interrupt
    //
    PR2 = 20833; // ((80MHz) / 256) / 20833 = 15Hz
    TMR2 = 0;

    T2CONbits.TCKPS = 7;
    T2CONbits.TGATE = 0;
    T2CONbits.TCS = 0;
    T2CONbits.ON = 1;
    
    IPC2bits.T2IP = 4;
    IPC4bits.T4IS = 2;
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 1;
    
    //
    // Setup Timer5 interrupt
    //
    PR5 = 6250; // ((80MHz) / 256) / 6250 = 50Hz
    TMR5 = 0;

    T5CONbits.TCKPS = 7;
    T5CONbits.TGATE = 0;
    T5CONbits.TCS = 0;
    T5CONbits.ON = 1;
    
    IPC5bits.T5IP = 6;
    IFS0bits.T5IF = 0;
    IEC0bits.T5IE = 0; // Disable timer until message is received from UART2

    //
    // Setup output compare
    //
    PR3 = 1022;
    T3CONbits.TCKPS = 3; // prescalar = 8
    T3CONbits.TGATE = 0;
    T3CONbits.TCS = 0;

    OC4CONbits.OCM = 6; // PWM on OC4, no fault pin
    OC4CONbits.OCTSEL = 1;

    OC4RS = 1023;
    OC4R = 1023;

    // Turn on OC4 and Timer3
    T3CONbits.ON = 1;
    OC4CONbits.ON = 1;

    //
    // Setup UART2
    //
    U2MODEbits.PDSEL = 0b00;
    U2MODEbits.STSEL = 0;     // 0 = 1 stop bit
    U2STAbits.UTXEN = 1;
    U2STAbits.URXEN = 1;
    U2MODEbits.BRGH = 0;
    U2BRG = 21;               // 230400 baud rate
    U2MODEbits.ON = 1;
    //U2MODEbits.UEN = 0;     // Disable RTS and CTS
        
    U2STAbits.URXISEL = 0;    // RX interrupt when receive buffer not empty
    IFS1bits.U2RXIF = 0;      // clear RX interrupt flag
    IPC8bits.U2IP = 3;        // set interrupt priority to 3
    IEC1bits.U2RXIE = 1;      // enable RX interrupt
    
    __builtin_enable_interrupts();

    // Initialize display driver
    lcd_display_driver_initialize();
    lcd_display_driver_clear();
    
    while(1) {
    }
}
