


#include <xc.h>
#include <stdio.h>
#include "configBits.h"
#include "constants.h"
#include "lcd.h"

#define __lcd_newline() lcdInst(0b11000000);

// blocks
const int RESET_POS = 52;
const int FORWARD_POS_1 = 35;
const int FORWARD_POS_2 = 25;

const int RESET_POS_9V = 44;//52
const int FORWARD_POS_1_9V = 26;//29
const int FORWARD_POS_2_9V = 13;//12

const int RESET_POS_C = 40;
const int FORWARD_POS_0_C = 34;
const int FORWARD_POS_1_C = 23;
const int FORWARD_POS_2_C = 8;

const int RESET_POS_AA = 36;
const int FORWARD_POS_1_AA = 26;
const int FORWARD_POS_2_AA = 6;

// electrode
const int ELEC_UP = 34;
const int ELEC_DOWN = 20;

// ramp turns
const int RAMP_DEAD = 41;
const int RAMP_9V = 52;
const int RAMP_C = 20;
const int RAMP_AA = 31;

// wait timers
const int SERVO_WAIT = 25000;
const unsigned int SERVO_F_WAIT = 50000;
const int VOLT_WAIT = 12500;
const int AGITATE_WAIT_1 = 2500;
const int AGITATE_WAIT_2 = 10000;
const int AGITATE_DC = 25000;
const int SENSOR_WAIT = 25000;

// max count
const int MAX_COUNT = 2;

// Function declarations
void FSM();
void intro();
void running();
void print_batteries();
void print_simu();
void keypad();

const char keys[] = "123A456B789C*0#D";

enum MACHINE_STATE {
    RESET = 0,
    WAIT = 1,
    F_9V = 20,
    V_9V = 21,
    VU_9V = 25,
    FA_9V = 22,
    A_9V = 23,
    R_9V = 24,
    F_C = 30,
    V_C = 31,
    VU_C = 35,
    FA_C = 32,
    A_C = 33,
    R_C = 34,
    F_AA = 40,
    V_AA = 41,
    VU_AA = 45,
    FA_AA = 42,
    A_AA = 43,
    R_AA = 44,
    TEST = 98,
    END = 99,
    EMERGENCY = 100
           
};

enum SENSOR_STATE {
    RESET_SENSOR = 0,
    WAIT_SENSOR = 1,
    WAIT_SENSOR_C = 2 // special state for C block that might contain AA (because holes are not aligned, AA might have trouble falling through)
};

enum SERVO_MOTORS {
    SERVO_9V = 0,
    SERVO_C = 1,
    SERVO_AA = 2,
    SERVO_ELEC = 3,
    SERVO_RAMP = 4
};

enum TEST_STATES {
    TEST_1 = 1,
    TEST_2 = 2
};

//<editor-fold defaultstate="collapsed" desc=" Global Variables "> 

// state values
int state = RESET;
int sensor_state = RESET_SENSOR;
int test_state = TEST_1; // for some dumb test
int test_counter = 0;

// counter values
int count9V = 0;
int countC = 0;
int countAA = 0;
int countDead = 0;
// sensor counter values, it counts to 1s (which is 5000 * 0.0002))
int sensor9V = 0;
int sensorC = 0;
int sensorAA = 0;

int sensor_incrementer = 0;

// voltage counter values
int volt9V = 0;
int voltC = 0;
int voltAA = 0;

int voltcheck_1 = 0;
int voltcheck_2 = 0;
int voltcheck_3 = 0;

// PWM, frequency 50 Hz, or period 20 ms

// This array stores the duty cycle values for each servo, indexed by the enumeration SERVO_MOTORS
// duty cycle in units of 0.0002 s or 0.01 * 20 ms
int servo_duties[5];

int AC_wait = 0;
    
// timer_counter counts to 20 ms (500 * 0.00004s)
int timer_counter = 0;

// timer_counter2 counts to varying # of seconds depending on the state the machine is in
unsigned int timer_counter2 = 0; // for timing states
int timer_counter3 = 0; // for the DC motors
int timer_counter4 = 1; // for block agitation
int timer_on = 0;

unsigned int operation_time = 0;


// Simulation values
/*
int simu_sensor9V = 0;
int simu_sensorC = 0;
int simu_sensorAA = 0;

int simu_volt9V = 0;
int simu_voltC = 0;
int simu_voltAA = 0;
 */

// LCD
int lcd_mode = 1;

// TEST MODE
int test_mode = 0;

unsigned int tenDigit = 0;
unsigned int oneDigit = 0;
unsigned char tenDigitChar = '0';
unsigned char oneDigitChar = '0';

// Rate limiter forward
int rlf = 0;

// Rate limiter backward
int rlb = 0;

// Keypad
unsigned char temp = 'L';
unsigned char keypress = 'L';

//</editor-fold>

void interrupt handler(void) {
    di();
    if(TMR1IF){
        TMR1IF = 0;
        
        TMR1ON = 0;
        
        // Initialize timer again!
        
        T1CON = 0b10000000;
        TMR1H = 0b11111111;
        TMR1L = 0b01011111;
        
        
        if (timer_on == 1) {
            
        //<editor-fold defaultstate="collapsed" desc="TIMER1 actions"> 
        // Apply voltage low to servos depending on their duty cycle
        // 9V
        if (state != EMERGENCY) {
            timer_counter++;
            // 9V
            if (timer_counter >= servo_duties[SERVO_9V]) {
                if (LATCbits.LATC0 != 0)
                    LATCbits.LATC0 = 0;
            }            
            // C
            if (timer_counter >= servo_duties[SERVO_C]) {
                if (LATCbits.LATC1 != 0)
                    LATCbits.LATC1 = 0;
            }
            // AA

            if (timer_counter >= servo_duties[SERVO_AA]) {
                if (LATCbits.LATC2 != 0)
                    LATCbits.LATC2 = 0;
            }

            // Electrode
            if (timer_counter >= servo_duties[SERVO_ELEC]) {
                if (LATCbits.LATC3 != 0)
                    LATCbits.LATC3 = 0;
            }

            // Ramp
            if (timer_counter >= servo_duties[SERVO_RAMP]) {
                if (LATCbits.LATC4 != 0)
                    LATCbits.LATC4 = 0;
            }

            // reset to 0 after 20 ms
            if (timer_counter >= 500) {
                timer_counter = 0;

                // apply voltage high to all servos
                LATCbits.LATC0 = 1;
                LATCbits.LATC1 = 1;
                LATCbits.LATC2 = 1;
                LATCbits.LATC3 = 1;
                LATCbits.LATC4 = 1;

            }
        }

        // RATE LIMITER in WAIT MODE

        /*
        if (state == WAIT && test_mode != 1) {
            // RATE LIMITER
            timer_counter3++;
            if (timer_counter3 >= AGITATE_DC) {
                if (rlf == 0 && rlb == 0) {
                    LATCbits.LATC6 = 1;
                    rlf = 1;

                } else if (rlb == 0){
                    
                    LATCbits.LATC6 = 0;
                    rlf = 0;
                }
                timer_counter3 = 0;
            }
        }*/

            /*
            // Actuation
            timer_counter4++;
            if (timer_counter4 >= AGITATE_WAIT_1) {
                if (servo_duties[SERVO_9V] == RESET_POS) {
                    servo_duties[SERVO_9V] = RESET_POS_9V - 2;
                    servo_duties[SERVO_C] = RESET_POS_C - 2;
                    servo_duties[SERVO_AA] = RESET_POS_AA - 2;
                } else {
                    servo_duties[SERVO_9V] = RESET_POS_9V;
                    servo_duties[SERVO_C] = RESET_POS_C;
                    servo_duties[SERVO_AA] = RESET_POS_AA;
                }

                timer_counter4 = 0;
            }
            */


        /*
        // AGITATION SPECIFIC TO THE 9V BLOCK (since battery tilting is a problem for electrodes)
        if (state == V_9V) {
            timer_counter4++;
            if (timer_counter4 >= AGITATE_WAIT_1) {
                if (servo_duties[SERVO_9V] == FORWARD_POS_1_9V) {
                    servo_duties[SERVO_9V] = FORWARD_POS_1_9V - 2;
                } else {
                    servo_duties[SERVO_9V] = FORWARD_POS_1_9V - 2;
                }

                timer_counter4 = 0;
            }
        }
        */
        // Count the photo sensors

        /*
        if (sensor_state == WAIT_SENSOR || sensor_state == WAIT_SENSOR_C) {
            if (sensor_incrementer == 0) {
                if (PORTAbits.RA0 == 1) {
                    if (sensor9V < SENSOR_WAIT + 1000) {
                        //sensor9V++;
                    }
                } else {
                    if (sensor9V > 0)
                        sensor9V--;
                }
                sensor_incrementer = 1;
            }
            else if (sensor_incrementer == 1) {
                if (PORTAbits.RA1 == 1) {
                    if (sensorC < SENSOR_WAIT + 1000) {
                        //sensorC++;
                    }
                } else {
                    if (sensorC > 0)
                        sensorC--;
                }
                sensor_incrementer = 2;
            }
            else if (sensor_incrementer == 2) {
                if (PORTEbits.RE0 == 1) {
                    if (sensorAA < SENSOR_WAIT + 1000) {
                        //sensorAA++;
                    }
                } else {
                    if (sensorAA > 0)
                        sensorAA--;
                }
                sensor_incrementer = 0;
            }
        }
         */ 


        if (timer_counter2 < 50000)
            timer_counter2++;


        //</editor-fold>
             
        // Dummy pin checking to balance PWM
        //<editor-fold defaultstate="collapsed" desc="dummy and real pin checking">   
        switch(state) {
            case WAIT:
                
                
                //USED FOR AGITATION OF BLOCKS IN WAIT STATE
                if (timer_counter4 < 50000) 
                    timer_counter4++;
            
                switch(sensor_state) {
                    case WAIT_SENSOR:
                        if (PORTAbits.RA0 == 1) {
                            if (sensor9V < SENSOR_WAIT + 1000) {
                                sensor9V++;
                            }
                        } else {
                            if (sensor9V > 0)
                                sensor9V--;
                        }
                        if (PORTAbits.RA1 == 1) {
                            if (sensorC < SENSOR_WAIT + 1000) {
                                sensorC++;
                            }
                        } else {
                            if (sensorC > 0)
                                sensorC--;
                        }
                        if (PORTEbits.RE0 == 1) {
                            if (sensorAA < SENSOR_WAIT + 1000) {
                                sensorAA++;
                            }
                        } else {
                            if (sensorAA > 0)
                                sensorAA--;
                        }
                        //sensor9V = SENSOR_WAIT + 1000;
                        break;
                    case WAIT_SENSOR_C:
                        /*
                        if (PORTAbits.RA0 == 1) {
                            if (sensor9V < SENSOR_WAIT + 1000) {
                                sensor9V++;
                            }
                        } else {
                            if (sensor9V > 0)
                                sensor9V--;
                        }*/
                        
                        if (PORTAbits.RA1 == 1) {
                            if (sensorC < SENSOR_WAIT + 1000) {
                                sensorC++;
                            }
                        } else {
                            if (sensorC > 0)
                                sensorC--;
                        }
                        if (PORTEbits.RE0 == 1) {
                            if (sensorAA < SENSOR_WAIT + 1000) {
                                sensorAA++;
                            }
                        } else {
                            if (sensorAA > 0)
                                sensorAA--;
                        }
                        //sensor9V = SENSOR_WAIT + 1000;
                        break;
                    default:
                        if (PORTAbits.RA0 == 1) {
                            int x = 5 + 5;
                        }
                        if (PORTAbits.RA1 == 1) {
                            int y = 6 + 4;
                        }
                        if (PORTEbits.RE0 == 1) {
                            int w = 3 * 3;
                        }
                        if (PORTAbits.RA0 == 1) {
                            int L = 123;
                        }
                        if (PORTEbits.RE0 == 1) {
                            int z = 43 * 2;
                        }
                        if (PORTAbits.RA0 == 1) {
                            int x = 5 + 5;
                        }
                        if (PORTAbits.RA1 == 1) {
                            int y = 6 + 4;
                        }
                        if (PORTEbits.RE0 == 1) {
                            int w = 3 * 3;
                        }
                        if (PORTAbits.RA0 == 1) {
                            int L = 123;
                        }
                        if (PORTEbits.RE0 == 1) {
                            int z = 43 * 2;
                        }
                        break;
                }
                break;
            case V_9V:
                //volt9V = 1;
                voltcheck_1 = PORTEbits.RE1;
                voltcheck_2 = PORTAbits.RA4;
                voltcheck_3 = PORTAbits.RA5;
                if ((voltcheck_1 == 1) || (voltcheck_2 == 1) || (voltcheck_3 == 1)) {
                    if (volt9V < VOLT_WAIT + 1000)
                        //volt9V++;
                        volt9V += 100;
                } else {
                    if (volt9V > 0) {}
                        //volt9V--;
                }
            case V_C:
                if (PORTEbits.RE2 == 1) {
                    if (voltC < VOLT_WAIT + 1000)
                        voltC += 10;
                } else {
                    if (voltC > 0)
                        voltC--;
                }
                break;
            case V_AA:
                if (PORTCbits.RC7 == 1) {
                    if (voltAA < VOLT_WAIT + 1000)
                        voltAA += 10;
                } else {
                    if (voltAA > 0)
                        voltAA--;
                }
                break;
            default:
                if (PORTAbits.RA0 == 1) {
                    int x = 5 + 5;
                }
                if (PORTAbits.RA1 == 1) {
                    int y = 6 + 4;
                }
                if (PORTEbits.RE0 == 1) {
                    int w = 3 * 3;
                }
                if (PORTAbits.RA0 == 1) {
                    int L = 123;
                }
                if (PORTEbits.RE0 == 1) {
                    int z = 43 * 2;
                }
                if (PORTAbits.RA0 == 1) {
                    int x = 5 + 5;
                }
                if (PORTAbits.RA1 == 1) {
                    int y = 6 + 4;
                }
                if (PORTEbits.RE0 == 1) {
                    int w = 3 * 3;
                }
                if (PORTAbits.RA0 == 1) {
                    int L = 123;
                }
                if (PORTEbits.RE0 == 1) {
                    int z = 43 * 2;
                }
                
                break;
        }
        
        //</editor-fold>
        
        TMR1ON = 1;
        
        
        }
        
        
        
    }
    
    
    if (TMR0IF) {
        TMR0IF = 0;
        TMR0ON = 0;
        
        // Initialize timer again!
        
        T0CON = 0b00000111;
        TMR0H = 0b10000101;
        TMR0L = 0b11101101;
        
        //<editor-fold defaultstate="collapsed" desc="TIMER0 actions">
        
        if (state != RESET && state != EMERGENCY && state != END) {
            operation_time += 1;
        }
        
        
        //</editor-fold>
        TMR0ON = 1;
    }
     
    ei();
}

void FSM() {
    // this function reads what state we are in, and does something accordingly
    
    
    
    // does its autonomous task depending on the current state
    // different task depending on the state
    switch(state) {
        
        // The 'off' state, before the machine is running
        case RESET:
            // DISPLAYS DATE AND TIME
            
            // DC motors off
            //LATCbits.LATC5 = 0;
            //LATCbits.LATC6 = 0;
            timer_counter2 = 0;
            break;
            
            
        //<editor-fold defaultstate="collapsed" desc=" WAIT ">   
        // The 'start' state, it is waiting for the photo-resistors to stop sending a 1 signal (meaning that it
        // is being blocked by a battery for more than a second)
        case WAIT:
            servo_duties[SERVO_RAMP] = RAMP_DEAD;
            
            // AGITATION OF BLOCKS IN WAIT STATE
            
            if (timer_counter4 >= AGITATE_WAIT_1) {
                if (servo_duties[SERVO_AA] == RESET_POS_AA) {
                    servo_duties[SERVO_9V] = RESET_POS_9V - 3;
                    if (sensor_state == WAIT_SENSOR_C)
                        servo_duties[SERVO_C] = FORWARD_POS_0_C - 3;
                    else
                        servo_duties[SERVO_C] = RESET_POS_C - 3;
                    servo_duties[SERVO_AA] = RESET_POS_AA - 3;
                } else {
                    servo_duties[SERVO_9V] = RESET_POS_9V;
                    if (sensor_state == WAIT_SENSOR_C)
                        servo_duties[SERVO_C] = FORWARD_POS_0_C;
                    else
                        servo_duties[SERVO_C] = RESET_POS_C;
                    servo_duties[SERVO_AA] = RESET_POS_AA;
                }

                timer_counter4 = 0;
            }
            
            switch(sensor_state) {
                case RESET_SENSOR:
                    sensor_state = WAIT_SENSOR;
                    sensor9V = 0;
                    sensorC = 0;
                    sensorAA = 0;
                    if (lcd_mode == 1)
                        print_simu();
                    break;
                case WAIT_SENSOR:
                    // ordered in a way such that the bottom block always moves first
               
                    
                    if (sensor9V >= SENSOR_WAIT) {
                        state = F_9V;
                        

                        // reset the sensor state
                        sensor_state = RESET_SENSOR;

                        // set up timer_counter2 for w.e the next state is
                        timer_counter2 = 0;

                        // DC motors off
                        LATCbits.LATC5 = 0;
                        LATCbits.LATC6 = 0;
                        
                        // LCD
                        if (lcd_mode == 1) {
                            
                            print_simu();
                        }
                        
                    }
                    if (sensorC >= SENSOR_WAIT) {
                        
                        // reset the sensor state
                        sensor_state = WAIT_SENSOR_C;
                        sensorC = 0; // Check this block again
                        // set up timer_counter2 for w.e the next state is
                        timer_counter2 = 0;

                        // DC motors off
                        LATCbits.LATC5 = 0;
                        LATCbits.LATC6 = 0;
                        
                        // LCD
                        if (lcd_mode == 1) {
                            
                            print_simu();
                        }
                        
                        
                    }
                    if (sensorAA >= SENSOR_WAIT) {
                        state = F_AA;

                        // reset the sensor state
                        sensor_state = RESET_SENSOR;

                        // set up timer_counter2 for w.e the next state is
                        timer_counter2 = 0;

                        // DC motors off
                        LATCbits.LATC5 = 0;
                        LATCbits.LATC6 = 0;
                        
                        // LCD
                        if (lcd_mode == 1) {
                            
                            print_simu();
                        }
                        
                    }
                    
                    break;
                    
                // SPECIAL STATE BECAUSE C BLOCK HOLE IS NOT PERFECTLY ALLIGNED WITH A BLOCK HOLE IN THE CONSTRUCTION
                case WAIT_SENSOR_C:
                    
                    servo_duties[SERVO_C] = FORWARD_POS_0_C;
                    
                    if (sensorC >= SENSOR_WAIT) {
                        state = F_C;
                        
                        // reset the sensor state
                        sensor_state = RESET_SENSOR;
                        
                        // set up timer_counter2 for w.e the next state is
                        timer_counter2 = 0;

                        // DC motors off
                        LATCbits.LATC5 = 0;
                        LATCbits.LATC6 = 0;
                        
                        // LCD
                        if (lcd_mode == 1) {
                            
                            print_simu();
                        }
                        
                        
                    }
                    if (sensorAA >= SENSOR_WAIT) {
                        state = F_AA;

                        // reset the sensor state
                        sensor_state = RESET_SENSOR;

                        // set up timer_counter2 for w.e the next state is
                        timer_counter2 = 0;

                        // DC motors off
                        LATCbits.LATC5 = 0;
                        LATCbits.LATC6 = 0;
                        
                        // LCD
                        if (lcd_mode == 1) {
                            
                            print_simu();
                        }
                        
                    }
                    break;
                default:
                    break;
            }
            
            break;
        //</editor-fold>
            
            
        //<editor-fold defaultstate="collapsed" desc=" 9V FSM ">   
            
        // The state to drive the solenoid motor controlling the 9V block from the initial position
        // to the voltage checking position
        case F_9V:
            
            // set duty cycle to first forward value for 9V block
            servo_duties[SERVO_9V] = FORWARD_POS_1_9V;
            
            // move other blocks back to prevent jamming
            servo_duties[SERVO_C] = RESET_POS_C + 3;
            servo_duties[SERVO_AA] = RESET_POS_AA + 3;
            
            // wait approx 1 s
            if (timer_counter2 >= SERVO_WAIT) {
                
                // prepare transition to next V state
                state = V_9V;
                volt9V = 0;
                timer_counter2 = 0;
                
                // LCD
                if (lcd_mode == 1)
                    print_simu();
                
                //servo_duties[SERVO_ELEC] = ELEC_DOWN;
            }
            break;
        
        // The state to drive the solenoid motor controlling the voltage measuring electrodes
        // and specifically check the voltage of the 9V battery.
        // The state is finished after driving the voltage measuring electrodes back up to their initial positions
        case V_9V:
            // set duty cycle to push electrodes down to measure voltage
            servo_duties[SERVO_ELEC] = ELEC_DOWN;
            // wait approx 1 s
            if (timer_counter2 >= SERVO_WAIT) {
                // now read voltage values
                // PORTA, bit 3: 9V, 4: C, 5: AA
                // and set the ramp servo duty cycle (servo_duties[4]) to the correct bucket
                // Also increment battery counts
                
                if (volt9V >= VOLT_WAIT) {
                    servo_duties[SERVO_RAMP] = RAMP_9V;
                    count9V += 1;
                } else {
                    //servo_duties[SERVO_RAMP] = RAMP_9V;
                    //count9V += 1;
                    servo_duties[SERVO_RAMP] = RAMP_DEAD;
                    countDead += 1;
                }
                
               
                
                // Assuming one of these things happen, else an error occurred
                
                // prepare transition to FA state
                
                state = VU_9V;
                timer_counter2 = 0;
                
                // LCD
                if (lcd_mode == 1)
                    print_simu();
            }
            break;
            
        case VU_9V:
            servo_duties[SERVO_ELEC] = ELEC_UP;
            if (timer_counter2 >= SERVO_WAIT) {
                timer_counter2 = 0;
                state = FA_9V;
            }
            break;
            
        // The state to drive the solenoid motor controlling the 9V block from the voltage checking position
        // to the release position. This state also rotates the ramp (which leads the batteries to the buckets)
        // to the 9V bucket or the uncharged bucket
        case FA_9V:
            // set duty cycle to second forward value for 9V block
            servo_duties[SERVO_9V] = FORWARD_POS_2_9V;
            
            // wait approx 1 s
            if (timer_counter2 >= SERVO_WAIT) {
                state = A_9V;
                // reset timer_counter2 for the next state
                timer_counter2 = 0;
                
                // LCD
                if (lcd_mode == 1)
                    print_simu();
                
            }
            
            
            break;
            
        // The state to slightly move the 9V block back and forth, to ensure that the battery is out.
        // Ends when the photo-resistor starts sending a signal again
        case A_9V:
            
            if (timer_counter2 >= SERVO_WAIT/2) {
                state = R_9V;
                timer_counter2 = 0;
                
                // LCD
                if (lcd_mode == 1)
                    print_simu();
                
                // reset 9V block
                //servo_duties[SERVO_9V] = RESET_POS_9V;
            }
            break;
            
        // The state to send the 9V block back to it's initial position
        case R_9V:
            
            // Reset all blocks
            servo_duties[SERVO_9V] = RESET_POS_9V;
            servo_duties[SERVO_C] = RESET_POS_C;
            servo_duties[SERVO_AA] = RESET_POS_AA;
            
            if (timer_counter2 >= SERVO_WAIT) {
                if (count9V + countC + countAA + countDead >= MAX_COUNT) {
                    state = END;
                    print_batteries();
                } else {
                    state = WAIT;
                    
                    // LCD
                    if (lcd_mode == 1) {
                       
                        print_simu();
                    }
                }
                timer_counter2 = 0;
                
                
            }
            break;
            
        //</editor-fold>
            
            
        
        //<editor-fold defaultstate="collapsed" desc=" C FSM ">
            
        // The state to drive the solenoid motor controlling the C block from the initial position
        // to the voltage checking position
        case F_C:
            /*
            // set duty cycle to the special first forward value for C block
            if (timer_counter2 < SERVO_WAIT/2) {
                servo_duties[SERVO_C] = FORWARD_POS_1_C + 8;
            }
            // wait approx 1 s
            if (timer_counter2 >= SERVO_WAIT/2 && timer_counter2 < SERVO_WAIT) {
                servo_duties[SERVO_C] = FORWARD_POS_1_C;
            }
            */
            
            // move other blocks back to prevent jamming
            servo_duties[SERVO_9V] = RESET_POS_9V + 3;
            servo_duties[SERVO_AA] = RESET_POS_AA + 3;
            
            // move C block to forward position
            servo_duties[SERVO_C] = FORWARD_POS_1_C;
            
            if (timer_counter2 >= SERVO_WAIT) {
                // prepare transition to next V state
                state = V_C;
                timer_counter2 = 0;
                
                // lcd_mode
                if (lcd_mode == 1)
                    print_simu();
                
                //servo_duties[SERVO_ELEC] = ELEC_DOWN;
            }
            break;
            
        // The state to drive the solenoid motor controlling the voltage measuring electrodes
        // and specifically check the voltage of the C battery.
        // The state is finished after driving the voltage measuring electrodes back up to their initial positions
        case V_C:
            // set duty cycle to push electrodes down to measure voltage
            servo_duties[SERVO_ELEC] = ELEC_DOWN;
            // wait approx 1 s
            if (timer_counter2 >= SERVO_WAIT) {
                // now read voltage values
                // PORTA, bit 3: 9V, 4: C, 5: AA
                // and set the ramp servo duty cycle (servo_duties[4]) to the correct bucket
                // Also increment battery counts
                

                    if (voltC >= VOLT_WAIT) {
                        servo_duties[SERVO_RAMP] = RAMP_C;
                        countC += 1;
                    } else {
                        //servo_duties[SERVO_RAMP] = RAMP_C;
                        //countC += 1;
                        servo_duties[SERVO_RAMP] = RAMP_DEAD;
                        countDead += 1;
                    }
                
                // Assuming one of these things happen, else an error occurred
                
                // prepare transition to FA state
                //servo_duties[SERVO_ELEC] = ELEC_UP;
                state = VU_C;
                timer_counter2 = 0;
                
                // lcd_mode
                if (lcd_mode == 1)
                    print_simu();
                
                
            }
            
            break;
            
        case VU_C:
            servo_duties[SERVO_ELEC] = ELEC_UP;
            if (timer_counter2 >= SERVO_WAIT) {
                timer_counter2 = 0;
                state = FA_C;
            }
            break;
            
        // The state to drive the solenoid motor controlling the C block from the voltage checking position
        // to the release position. This state also rotates the ramp (which leads the batteries to the buckets)
        // to the C bucket or the uncharged bucket
        case FA_C:
            // set duty cycle to second forward value for C block
            /*
            if (timer_counter2 < SERVO_WAIT/2) {
                servo_duties[SERVO_C] = FORWARD_POS_2_C + 8;
            }
            
            if (timer_counter2 >= SERVO_WAIT/2 && timer_counter2 < SERVO_WAIT) {
                servo_duties[SERVO_C] = FORWARD_POS_2_C;
            }
            */
            
            servo_duties[SERVO_C] = FORWARD_POS_2_C;
            
            // wait approx 1 s
            if (timer_counter2 >= SERVO_WAIT) {
                state = A_C;
                // reset timer_counter2 for the next state
                timer_counter2 = 0;
                
                // lcd_mode
                if (lcd_mode == 1)
                    print_simu();
            }
            break;
        case A_C:
            
            if (timer_counter2 >= SERVO_WAIT/2 /*&& AC_wait >= 2*/) {
                state = R_C;
                timer_counter2 = 0;
                //AC_wait = 0;
                
                // lcd_mode
                if (lcd_mode == 1)
                    print_simu();
                
                // reset C block
                servo_duties[SERVO_C] = RESET_POS_C;
            }
            break;
        case R_C:
            /*
            if (timer_counter2 >= SERVO_WAIT) {
                AC_wait += 1;
                timer_counter2 = 0;
                servo_duties[SERVO_C] += 8;
            }*/
            
            // Reset all blocks
            servo_duties[SERVO_9V] = RESET_POS_9V;
            servo_duties[SERVO_C] = RESET_POS_C;
            servo_duties[SERVO_AA] = RESET_POS_AA;
            
            if (timer_counter2 >= SERVO_WAIT /*&& AC_wait >= 3*/) {
                servo_duties[SERVO_C] = RESET_POS_C;
                if (count9V + countC + countAA + countDead >= MAX_COUNT) {
                    state = END;
                    print_batteries();
                } else {
                    state = WAIT;
                    
                    
                    // lcd_mode
                    if (lcd_mode == 1) {

                        print_simu();
                    }
                }
                timer_counter2 = 0;
                //AC_wait = 0;
                
                
            }
            break;
        
        //</editor-fold>
            
            
            
        //<editor-fold defaultstate="collapsed" desc=" AA FSM ">
            
        // The state to drive the solenoid motor controlling the AA block from the initial position
        // to the voltage checking position
            
        case F_AA:
            
            // set duty cycle to first forward value for AA block
            servo_duties[SERVO_AA] = FORWARD_POS_1_AA;
            
            // move other blocks back to prevent jamming
            servo_duties[SERVO_9V] = RESET_POS_9V + 3;
            servo_duties[SERVO_C] = RESET_POS_C + 3;
            
            
            // wait approx 1 s
            if (timer_counter2 >= SERVO_WAIT) {
                
                // prepare transition to next V state
                state = V_AA;
                timer_counter2 = 0;
                
                // lcd_mode
                if (lcd_mode == 1)
                    print_simu();
                
                servo_duties[SERVO_ELEC] = ELEC_DOWN;
            }
            break;
            
        // The state to drive the solenoid motor controlling the voltage measuring electrodes
        // and specifically check the voltage of the AA battery.
        // The state is finished after driving the voltage measuring electrodes back up to their initial positions
        case V_AA:
            // set duty cycle to push electrodes down to measure voltage
            servo_duties[SERVO_ELEC] = ELEC_DOWN;
            
            
            
            // wait approx 1 s
            if (timer_counter2 >= SERVO_WAIT) {
                // now read voltage values
                // PORTA, bit 3: 9V, 4: C, 5: AA
                // and set the ramp servo duty cycle (servo_duties[4]) to the correct bucket
                // Also increment battery counts
                
                // lcd_mode
                if (voltAA >= VOLT_WAIT) {
                    servo_duties[SERVO_RAMP] = RAMP_AA;
                    countAA += 1;
                } else {
                    //servo_duties[SERVO_RAMP] = RAMP_AA;
                    //countAA += 1;
                    servo_duties[SERVO_RAMP] = RAMP_DEAD;
                    countDead += 1;
                }

                // Assuming one of these things happen, else an error occurred
                
                // prepare transition to FA state
                servo_duties[SERVO_ELEC] = ELEC_UP;
                state = VU_AA;
                timer_counter2 = 0;
                
                // lcd_mode
                if (lcd_mode == 1)
                    print_simu();
            }
            
        case VU_AA:
            servo_duties[SERVO_ELEC] = ELEC_UP;
            if (timer_counter2 >= SERVO_WAIT) {
                timer_counter2 = 0;
                state = FA_AA;
            }
            break;
            
            break;
        case FA_AA:
            // set duty cycle to second forward value for AA block
            servo_duties[SERVO_AA] = FORWARD_POS_2_AA;
            
            
            
            // wait approx 1 s
            if (timer_counter2 >= SERVO_WAIT) {
                state = A_AA;
                // reset timer_counter2 for the next state
                timer_counter2 = 0;
                
                // lcd_mode
                if (lcd_mode == 1)
                    print_simu();
            }
            break;
        case A_AA:
             
            if (timer_counter2 >= SERVO_WAIT/2) {
                state = R_AA;
                timer_counter2 = 0;
                
                // LCD
                if (lcd_mode == 1)
                    print_simu();
                
                // reset A block
                servo_duties[SERVO_AA] = RESET_POS_AA;
            }
            break;
        case R_AA:
            
            // Reset all blocks
            servo_duties[SERVO_9V] = RESET_POS_9V;
            servo_duties[SERVO_C] = RESET_POS_C;
            servo_duties[SERVO_AA] = RESET_POS_AA;
            
            if (timer_counter2 >= SERVO_WAIT) {
                if (count9V + countC + countAA + countDead >= MAX_COUNT) {
                    state = END;
                    print_batteries();
                } else {
                    state = WAIT;
                    
                    
                    // lcd_mode
                    if (lcd_mode == 1) {
                        
                        print_simu();
                    }
                }
                timer_counter2 = 0;
                
                
                
                
            }
            break;
            
        //</editor-fold>
           
            
            
        case TEST:
            //LATCbits,LATC6 = 1;
            break;
            
            
            
        // Go here when done
        case END:
            break;
            
            
            
        default:
            break;
    }
}

void keypad() {
    //<editor-fold defaultstate="collapsed" desc="KEYPAD"> 
        // wait until RB1 interrupt pin
        
        //
        if (PORTBbits.RB1 == 1) {
            // read the 4 bit character code
            keypress = (PORTB & 0xF0)>>4;
            temp = keys[keypress];

            //
            //while(PORTBbits.RB1 == 1){
            //    // Wait until the key has been released
            //}

            Nop();  //Apply breakpoint here because of compiler optimizations
            Nop();

                switch (temp) {
                    case '1':
                        if (state == RESET) {

                            if (test_mode == 1)
                                state = TEST;
                            else
                                state = WAIT;

                            // lcd_mode
                            
                            if (lcd_mode == 1)
                                print_simu();
                            // real
                             
                            //else if (lcd_mode == 0)
                            //running();
                            sensor_state = RESET_SENSOR;
                            
                            servo_duties[SERVO_9V] = RESET_POS_9V;
                            servo_duties[SERVO_C] = RESET_POS_C;
                            servo_duties[SERVO_AA] = RESET_POS_AA;
                            servo_duties[SERVO_ELEC] = ELEC_UP;
                            servo_duties[SERVO_RAMP] = RAMP_DEAD;
                            operation_time = 0;
                            
                            timer_counter = 0;

                        }
                        break;
                    case '2':

                        state = RESET;
                        sensor_state = RESET_SENSOR;
                        // Clear everything



                        servo_duties[SERVO_9V] = RESET_POS_9V;
                        servo_duties[SERVO_C] = RESET_POS_C;
                        servo_duties[SERVO_AA] = RESET_POS_AA;
                        servo_duties[SERVO_ELEC] = ELEC_UP;
                        servo_duties[SERVO_RAMP] = RAMP_DEAD;
                        //print_simu();
                        count9V = 0;
                        countC = 0;
                        countAA = 0;
                        countDead = 0;
                        sensor9V = 0;
                        sensorC = 0;
                        sensorAA = 0;
                        volt9V = 0;
                        voltC = 0;
                        voltAA = 0;
                        AC_wait = 0;

                        timer_counter = 0;
                        timer_counter2 = 0;
                        timer_counter3 = 0;
                        timer_counter4 = 0;

                        intro();


                        break;
                    case '3':

                        timer_on = 0;
                        //LATC = 0x00;
                        state = EMERGENCY;

                        break;
                    //test_state = TEST_1;
                    case 'A':
                        if (state == TEST) {
                            //servo_duties[SERVO_9V] += 1;
                            //servo_duties[SERVO_C] += 1;
                            //servo_duties[SERVO_AA] += 1;
                            servo_duties[SERVO_ELEC] += 1;
                            //servo_duties[SERVO_RAMP] += 1;

                            initLCD();
                            __delay_ms(10);
                            tenDigit = servo_duties[SERVO_ELEC] / 10 + 48;
                            oneDigit = servo_duties[SERVO_ELEC] % 10 + 48;
                            tenDigitChar = tenDigit;
                            oneDigitChar = oneDigit;
                            putch(tenDigitChar); putch(oneDigitChar);
                        }
                        break;
                    case '4':
                        if (state == TEST) {
                            //servo_duties[SERVO_9V] -= 1;
                            //servo_duties[SERVO_C] -= 1;
                            //servo_duties[SERVO_AA] -= 1;
                            servo_duties[SERVO_ELEC] -= 1;
                            //servo_duties[SERVO_RAMP] -= 1;

                            initLCD();
                            __delay_ms(10);
                            tenDigit = servo_duties[SERVO_ELEC] / 10 + 48;
                            oneDigit = servo_duties[SERVO_ELEC] % 10 + 48;
                            tenDigitChar = tenDigit;
                            oneDigitChar = oneDigit;
                            putch(tenDigitChar); putch(oneDigitChar);
                        }
                        break;
                    default:
                        break;
                }
            }
                //</editor-fold>
}

void main(void) {
    
    // Write to LATX
    // Read from PORTX
    // Configure from TRISX, 0 = output, 1 = input
    
    /*
     * RC7: AA voltage
     * RE0: AA photo
     * RE1: 9V volt, aso RA4, RA5
     * 
     * A:   bits 0 - 2, photo-resistors
     *      bits 3 - 5, 9V voltage bits
     *      bits 6 - 7, C and AA voltage bits
     * B:   occupied by Keypad
     * C:   bits 0 - 4, servos
     * RC0: 9V block servo
     * RC1: C block servo
     * RC2: AA block servo
     * RC3: Electrode servo
     * RC4: Ramp servo
     * RC6: Rate limiter forward (DC motor)
     * RC5: Rate limiter back (DC motor)
     * D:   occupied by LCD
     */
    
    // Make sure rate limiter pins set to 0, so they don't short out
    // can never have both be 1 at the same time
    TRISC = 0b10000000;
    LATCbits.LATC5 = 0;
    rlb = 0;
    
    LATCbits.LATC6 = 0;
    rlf = 0;
    
    TRISEbits.RE1 = 1;
    
    // reading photo-resistors and voltage bits from these
    TRISA = 0xFF;
    
    // For LCD
    TRISD = 0x00;   //All output mode
    
    // For keypad
    TRISB = 0xFF;   //All input mode (For Keypad)
    ADCON0 = 0x00;  //Disable ADC
    ADCON1 = 0xFF;  //Set PORTB to be digital instead of analog default
    LATB = 0x00;
    
    
    
    // For PWM
    // Set internal oscillator to run at 8 MHz 
    OSCCON = OSCCON | 0b01110000; 
    // Enable PLL for the internal oscillator, Processor now runs at 32MHZ
    OSCTUNEbits.PLLEN = 1; 
    
    
    
    // Enable timer 0 interrupts, 0.0002 s, so 100 * 0.0002 s = 20 ms
    
    //TMR0IE = 1; // Enable Timer0 interrupts
    TMR1IE = 1; // Enable Timer1 interrupts
    //TMR2IE = 1; // Enable Timer2 interrupts // for date/time
    TMR3IE = 1; // Enable Timer3 interrupts
    
    PEIE = 1; // Set peripheral interrupt enable bit (YOU MUST DO THIS IF YOU
              // DIDN'T SET INTERRUPT PRIORITIES (i.e. if IPEN=0)
    
    // Timer control register. ////Bit 2-0 = 011, means 16 pre-scaler value, 111 means 256
    // (2^16 - 1)-S(clock frequency in Hz)/4 * 1/pre-scaler
    
    
    // 0.0002 s timer (i think)
    T1CON = 0b10000000;
    TMR1H = 0b11111111;
    TMR1L = 0b01011111;
 
    TMR1ON = 1;
    
    // 1 s timer
    T0CON = 0b00000111;
    TMR0H = 0b10000101;
    TMR0L = 0b11101101;
    
    TMR0ON = 1;
    
    
    servo_duties[SERVO_9V] = RESET_POS_9V;
    servo_duties[SERVO_C] = RESET_POS_C;
    servo_duties[SERVO_AA] = RESET_POS_AA;
    servo_duties[SERVO_ELEC] = ELEC_UP;
    servo_duties[SERVO_RAMP] = RAMP_DEAD;
    timer_on = 1;
    
    
    ei(); // enable all interrupts for now
    
    
    
    // introduction text
    intro();
    
    // Machine Loop
    while(1){
        keypad();
        FSM();
    }
}

void print_batteries() {
    initLCD();
    unsigned int ten9V = count9V / 10 + 48;
    unsigned int one9V = count9V % 10 + 48;
    unsigned int tenC = countC / 10 + 48;
    unsigned int oneC = countC % 10 + 48;
    unsigned int tenAA = countAA / 10 + 48;
    unsigned int oneAA = countAA % 10 + 48;
    unsigned int tenDrained = countDead / 10 + 48;
    unsigned int oneDrained = countDead % 10 + 48;
    
    unsigned int minOpDigit = operation_time / 60;
    unsigned int tenOpDigit = operation_time - minOpDigit * 60;
    tenOpDigit /= 10;
    tenOpDigit += 48;
    minOpDigit += 48;
    unsigned int oneOpDigit = operation_time % 10;
    while (oneOpDigit > 10) {
        oneOpDigit -= 10;
    }
    oneOpDigit += 48;
    unsigned char minOpChar = minOpDigit;
    unsigned char tenOpChar = tenOpDigit;
    unsigned char oneOpChar = oneOpDigit;
    
    unsigned char chten9V;
    chten9V = ten9V;
    unsigned char chone9V;
    chone9V = one9V;
    unsigned char chtenC;
    chtenC = tenC;
    unsigned char choneC;
    choneC = oneC;
    unsigned char chtenAA;
    chtenAA = tenAA;
    unsigned char choneAA;
    choneAA = oneAA;
    unsigned char chtenDrained;
    chtenDrained = tenDrained;
    unsigned char choneDrained;
    choneDrained = oneDrained;
    putch('9');
    putch('V');
    putch(':');
    putch(chten9V);//chten9V
    putch(chone9V);//chone9V
    putch(' ');
    putch('C');
    putch(':');
    putch(chtenC);
    putch(choneC);
    putch(' ');
    putch('A');
    putch('A');
    putch(':');
    putch(chtenAA);
    putch(choneAA);
    __lcd_newline();
    putch('D');putch('R');putch('N');putch(':');
    putch(chtenDrained);
    putch(choneDrained);
    putch(' ');
    putch('T');
    putch(':'); putch(' ');
    putch(minOpChar); putch(':'); putch(tenOpChar); putch(oneOpChar);
    __delay_ms(10);
}

void print_simu() {
    // print the lcd_mode state
    initLCD();
    switch(state) {
        case WAIT:
            switch(sensor_state) {
                case RESET_SENSOR:
                    putch('R');putch('S');
                    break;
                case WAIT_SENSOR:
                    putch('W');putch('S');
                    break;
                case WAIT_SENSOR_C:
                    putch('W');putch('S');putch('C');
                    break;
                default:
                    break;
            }
            //putch('W'); putch('A'); putch('I'); putch('T');
            break;
        case F_9V:
            putch('F'); putch(' '); putch('9'); putch('V');
            break;
        case V_9V:
            putch('V'); putch(' '); putch('9'); putch('V');
            break;
        case VU_9V:
            putch('V'); putch('U'); putch(' '); putch('9'); putch('V');
            break;
        case A_9V:
            putch('A'); putch(' '); putch('9'); putch('V');
            break;
        case FA_9V:
            putch('F'); putch('A'); putch(' '); putch('9'); putch('V');
            break;
        case R_9V:
            putch('R'); putch(' '); putch('9'); putch('V');
            break;
        case F_C:
            putch('F'); putch(' '); putch('C');
            break;
        case V_C:
            putch('V'); putch(' '); putch('C');
            break;
        case VU_C:
            putch('V'); putch('U'); putch(' '); putch('C');
            break;
        case A_C:
            putch('A'); putch(' '); putch('C');
            break;
        case FA_C:
            putch('F'); putch('A'); putch(' '); putch('C');
            break;
        case R_C:
            putch('R'); putch(' '); putch('C');
            break;
        case F_AA:
            putch('F'); putch(' '); putch('A'); putch('A');
            break;
        case V_AA:
            putch('V'); putch(' '); putch('A'); putch('A');
            break;
        case VU_AA:
            putch('V'); putch('U'); putch(' '); putch('A'); putch('A');
            break;
        case A_AA:
            putch('A'); putch(' '); putch('A'); putch('A');
            break;
        case FA_AA:
            putch('F'); putch('A'); putch(' '); putch('A'); putch('A');
            break;
        case R_AA:
            putch('R'); putch(' '); putch('A'); putch('A');
            break;
        case END:
            putch('E'); putch('N'); putch('D');
            break;
        case TEST:
            break;
        default:
            break;
    }
    __delay_ms(10);
}