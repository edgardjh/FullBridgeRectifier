#include <stdint.h>
#include <stdbool.h>
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/qei.h"
#include "driverlib/pwm.h"
#include "driverlib/uart.h"
#include "driverlib/fpu.h"
#include "driverlib/pin_map.h"
#include "driverlib/flash.h"
#include "driverlib/rom.h"
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

// INPUTs and OUTPUTs
#define LED_Rojo    GPIO_PIN_1
#define LED_Azul    GPIO_PIN_2
#define LED_Verde   GPIO_PIN_3
#define SW1         GPIO_PIN_4
#define SW2         GPIO_PIN_0
#define TMRsPRESCLR	20

// With this variables we allow the firing of the SCR from a couple of deg to 60deg, aprox.
#define minAlpha	10666                 	// Minimum time to fire and SCR, 2.79mS
#define maxAlpha  	20952                 	// Maximum time to fire and SCR, 5.49mS
#define dlyAlpha	10467					// Tiempo de retraso de 2.77mS o 60°

// Flags to detect positive or negative sine wave
struct BanderasTemporizadores{
	volatile uint8_t	phA;
	volatile uint8_t	phB;
	volatile uint8_t	phC;
}BandTemp;

// Variable for timers
volatile int16_t alpha = maxAlpha - 5000;      // El temporizador es de 16-bits
//volatile int16_t alpha_ts = 2095;				// 5.5mS

// Variables for switchs and LEDs
    // Controlan el encendido/apagado de los LEDs
volatile uint8_t azul_OnOff = LED_Azul;
volatile uint8_t rojo_OnOff = LED_Rojo;
volatile uint8_t verde_OnOff = LED_Verde;



void CrucePorCero(void);    //  Zero crossing function, is gonna be used for interrupts trigger by GPIOA pins

void configuracion(void){
    
	// MASTER CLOCK CONFIG, AND MODULE CLOCKs ENABLE
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ); // Master clock at 80 MHz
    IntMasterDisable();                                                                 // Disables interrupts
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);                                        // Clk for PORTA, Zero crossing signals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);                                        // Clk for PORTF, Firing signals for SCRs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                                        // Clk for PORTE, LEDs and switches
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);                                       // Clk for TIMER0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);                                       // Clk for TIMER1

 /***************************************************************************************************************************/
 /******************************   INPUT, OUTPUTS AND PERIPHERALs CONFIGURATION ********************************/
 /***************************************************************************************************************************/

    // CONFIGURACION DEL MODULO GPIO A - Detectores de cruce por cero
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2);                                        // Detectores de cruce por cero, el pin 2 es la fase A...
    GPIOPadConfigSet(GPIO_PORTA_BASE,(GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4),GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);   // Se configura la resistencia de pull-up

    // CONFIGURACION DEL MODULO GPIO E - Señales de disparo para los SCRs
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, (GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5));    // Los tres primeros bits disparan la parte alta, el resto la parte baja
                                                                                                                        // del puente rectificador controlado de SCRs
    // Configuracion del puerto digital GPIOF
            // Se desbloquea el GPIOF_0, para poder ocupar SW2
        HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
        HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
        HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_Rojo|LED_Azul|LED_Verde);                     // Los LEDs se declaran como salidas
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, SW1 | SW2);                                       // Se declara pin de entrada, SW1 y SW2
    GPIOPadConfigSet(GPIO_PORTF_BASE, SW1 , GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);      // Se configura la resistencia pull-up PF_4
    GPIOPadConfigSet(GPIO_PORTF_BASE, SW2 , GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);      // Se configura la resistencia pull-up PF_0

    // Se configuran los temporizadores TIMER0 A y B
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT | TIMER_CFG_B_ONE_SHOT);    // El TIMER0 se divide en TIMER_A Y TIMER_B de 16-bits cada uno
    TimerPrescaleSet(TIMER0_BASE, TIMER_A | TIMER_B, TMRsPRESCLR);                       // El prescaler de los TIMERs_A/B se configura para tener una resolucion de 0.2625uS y un maximo de 17.2mS
    TimerUpdateMode(TIMER0_BASE, TIMER_A | TIMER_B, TIMER_UP_LOAD_IMMEDIATE);   // Se carga de manera imnediata el valor del temporizador
    TimerLoadSet(TIMER0_BASE, TIMER_A | TIMER_A, alpha);                        // Se precarga el valor de disparo -alpha- para los TIMERs A y B.
        // TIMER1_B
    TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT);   // El TIMER1 se divide en TIMER A y B
    TimerPrescaleSet(TIMER1_BASE, TIMER_A, TMRsPRESCLR);                                 // Cono el prescaler se tiene una resolucion de 0.2625uS y un maximo de 17.2mS
    TimerUpdateMode(TIMER1_BASE, TIMER_A,TIMER_UP_LOAD_IMMEDIATE);              // Se carga de manera imnediata el valor del temporizador
    TimerLoadSet(TIMER1_BASE, TIMER_A, alpha);                                   // Se precarga el valor de disparo -alpha- para los TIMERs A y B.



/***************************************************************************************************************************/
/************************************ 				INTERRUPTS SECTION					************************************/
/***************************************************************************************************************************/

    // Interrupts for TIMERS

    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);                            // Interrupcion TIMER0_A, activada al llegar a 0
    TimerIntEnable(TIMER0_BASE, TIMER_TIMB_TIMEOUT);                            // Interrupcion TIMER0_B, activada al llegar a 0
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);                            // Interrupcion TIMER1_B, activada al llegar a 0
    IntEnable(INT_TIMER0A);                                                     // Se habilitan las interrupciones para los TIMER0s A y B
    IntEnable(INT_TIMER0B);                                                     // y TIMER1 A.
    IntEnable(INT_TIMER1A);

    //  Interrupts for pins in GPIA port 
   GPIOIntDisable(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4);        // Se deshabilitan las interrupciones para los pines del GPIOA.
   GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4);          // Se limpian las interrupcioens pendientes para los pines del GPIOA
   GPIOIntRegister(GPIO_PORTA_BASE, CrucePorCero);                           // Registra la funcion para la interrupcion del puerto GPIOA
   GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_FALLING_EDGE);   // Se configura el flanco que disparara la interrupcion del GPIOA
   GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4);         // Se habilita la interrupcion para los cruces por cero

    // Interruption priorities
    IntPrioritySet(INT_GPIOA,0x00);                     // Prioridad 0, para el cruce por cero (Es la mas alta)
    IntPrioritySet(INT_TIMER0A, 0x20);                  // Prioridad 1, del TIMER0 activado en el cruce por cero
    IntPrioritySet(INT_TIMER0B, 0x20);                  // Prioridad 1, del TIMER0 activado en el cruce por cero
    IntPrioritySet(INT_TIMER1A, 0x20);                  // Prioridad 1, del TIMER0 activado en el cruce por cero



}


void main(void) {
    configuracion();            				// Configuration of peripheral.
    while((GPIOPinRead(GPIO_PORTF_BASE,SW1)));
    IntMasterEnable();          				// Global interrupts enabled
   
	BandTemp.phA = 1;
    BandTemp.phB = 1;
    BandTemp.phC = 1;

    while(1){                   // Forever Loop
    SysCtlDelay(4000000);
	
			// here we can control the firing angle of the SCRs thereby, the output voltage
        if((!GPIOPinRead(GPIO_PORTF_BASE,SW1)) & (alpha  < maxAlpha - 50)){
            alpha = alpha + 100;
        }
        if((!GPIOPinRead(GPIO_PORTF_BASE,SW2)) & (alpha > minAlpha + 200)){
            alpha = alpha - 100;
        }
    }
}

/*
	 Timer0AIntHandler(void), this interrupt fires/enables the SCRs in phase A
*/
void Timer0AIntHandler(void){
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);         	// Clear the timer interrupt
    TimerDisable(TIMER0_BASE, TIMER_A);

    if(BandTemp.phA){
    	TimerLoadSet(TIMER0_BASE, TIMER_A, dlyAlpha);			// Load the value for a second firing
        TimerEnable(TIMER0_BASE, TIMER_A);						// 60° delay
        GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_0,GPIO_PIN_0);	// Enables the proper diode
        SysCtlDelay(5333);                                  	//  200us ~ (3 clk cicles)(12.5ns)(5333)
        GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_0,0x00);
        BandTemp.phA = 0;
    }
    else{                                                   	// Segundo disparo a 60°
        GPIOPinWrite(GPIO_PORTE_BASE,  GPIO_PIN_1 ,GPIO_PIN_1);	// Enables the proper diode
        SysCtlDelay(5333);                                  	// 200us ~ (3 clk cicles)(12.5ns)(5333)
        GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1,0x00);
        BandTemp.phA = 1;
    }
}

/*
	 Timer0BIntHandler(void), this interrupt fires/enables the SCRs in phase B
*/
void Timer0BIntHandler(void){
    TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);         		// Clear the timer interrupt
    TimerDisable(TIMER0_BASE, TIMER_B);

    TimerLoadSet(TIMER0_BASE, TIMER_B, alpha);
    TimerEnable(TIMER1_BASE, TIMER_B);

    if(BandTemp.phB){
        	TimerLoadSet(TIMER0_BASE, TIMER_B, dlyAlpha);			// Load the value for a second firing
            TimerEnable(TIMER0_BASE, TIMER_B);						// 60° delay
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_2,GPIO_PIN_2);	// Enables the proper diode
            SysCtlDelay(5333);                                  	// 200us ~ (3 clk cicles)(12.5ns)(5333)
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_2,0x00);
            BandTemp.phB = 0;
        }
        else{                                                   	// firing at 60°
            GPIOPinWrite(GPIO_PORTE_BASE,  GPIO_PIN_3 ,GPIO_PIN_3);	// Enables the proper diode
            SysCtlDelay(5333);                                  	// 200us ~ (3 clk cicles)(12.5ns)(5333)
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_3,0x00);
            BandTemp.phB= 1;
        }
}

/*
	 Timer1AIntHandler(void), this interrupt fires/enables the SCRs in phase C
*/
void Timer1AIntHandler(void){
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);         		// Clear the timer interrupt
    TimerDisable(TIMER1_BASE, TIMER_A);
    if(BandTemp.phC){
        	TimerLoadSet(TIMER1_BASE, TIMER_A, dlyAlpha);			// Se vuelve a cargar el temporizador para el segundo disparo
            TimerEnable(TIMER1_BASE, TIMER_A);						// con un delay de 60°
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_4,GPIO_PIN_4);	// Enables the proper diode
            SysCtlDelay(5333);                                  	//  200us ~ (3 clk cicles)(12.5ns)(5333)
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_4,0x00);
            BandTemp.phC = 0;
        }
        else{                                                   	// Segundo disparo a 60°
            GPIOPinWrite(GPIO_PORTE_BASE,  GPIO_PIN_5 ,GPIO_PIN_5);	// Enables the proper diode
            SysCtlDelay(5333);                                  	// 200us ~ (3 clk cicles)(12.5ns)(5333)
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_5,0x00);
            BandTemp.phC = 1;
        }
}

/*
	CrucePorCero(void) interruption is called when is detected a zero crossing in
    in each phase.
	It's trigger by a digital signal provived by a optocoupler located in each
	single phase, this interrupt updates the value of the TIMER0_A, TIMER0_B or TIMER1_A
	The time of the TIMERs indicate when SCRs will be enable-
*/
void CrucePorCero(void) {
    if (GPIOIntStatus(GPIO_PORTA_BASE, false) & GPIO_PIN_2){                    // Interruption for A-phase
        GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_2);                              // Clear interruption of  pin 2
        GPIOPinWrite(GPIO_PORTF_BASE,LED_Verde,verde_OnOff);
        verde_OnOff = ~verde_OnOff;
        TimerLoadSet(TIMER0_BASE, TIMER_A, alpha);
        TimerEnable(TIMER0_BASE, TIMER_A);
    }
    if (GPIOIntStatus(GPIO_PORTA_BASE, false) & GPIO_PIN_3){                    // Interruption for B-phase
        GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_3);                              // Clear interruption of  pin 3
        GPIOPinWrite(GPIO_PORTF_BASE,LED_Verde,verde_OnOff);
        verde_OnOff = ~verde_OnOff;
        TimerLoadSet(TIMER0_BASE, TIMER_B, alpha);
        TimerEnable(TIMER0_BASE, TIMER_B);
    }
    if (GPIOIntStatus(GPIO_PORTA_BASE, false) & GPIO_PIN_4){                    // Interruption for C-phase
        GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_4);                              // Clear interruption of  pin 4.
        GPIOPinWrite(GPIO_PORTF_BASE,LED_Verde,verde_OnOff);
        verde_OnOff = ~verde_OnOff;
        TimerLoadSet(TIMER1_BASE, TIMER_A, alpha);
        TimerEnable(TIMER1_BASE, TIMER_A);
        }
}
