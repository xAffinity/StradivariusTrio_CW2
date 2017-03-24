#include "mbed.h"
#include "rtos.h"
#include "slre.h"  //for regex parsing
#include "ctype.h" //for isDigit
#include "RawSerial.h" //for serial interrupts

//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

//serial connection between mbed and pc via usb
RawSerial pc(USBTX,USBRX);

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
DigitalIn I1(I1pin);
DigitalIn I2(I2pin);
DigitalIn I3(I3pin);

//Quadrature Encoder Inputs
InterruptIn CHA1(CHA);
InterruptIn CHB1(CHB);

//Motor Drive outputs
PwmOut L1L(L1Lpin);
PwmOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
PwmOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
PwmOut L3H(L3Hpin);

//Threads
Thread thread1;

//Timers
Timer timer;
Timer timer1;

//Global Variables
//@@@@@@@@@MOTOR@@@@@@@@@@@
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00}; //Drive state to output table
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  //Mapping from interrupter inputs to sequential rotor states
int8_t lead = 2;
int8_t orState = 0;    //Rotot offset at motor state 0
int8_t intState = 0;
int8_t intStateOld = 0;
float duty_cycle = 0.5;
//@@@@@@@@@MOTOR@@@@@@@@@@@

//@@@@@@@@@GET VELOCITY@@@@@@@@@@@
int8_t count =0;
float current_time = 0;
float previous_time = 0;
float time_per_state = 0;
float angular_velocity = 0;
float previous_angular_velocity = 0;
float print_velocity =0;
float time_per_revolution=0;
//@@@@@@@@@GET VELOCITY@@@@@@@@@@@

//@@@@@@@@@VELOCITY CONTROL@@@@@@@@@@@
float kvp = 50;
float kvi = 0.01;
float desired_angular_velocity = 0;
float v_dt = 0;
float change_in_duty_cycle = 0;
float v_theoretical = 0;
float velocity_control_duty_cycle = 0;
float v_error = 0;
float v_integral = 0;
//@@@@@@@@@VELOCITY CONTROL@@@@@@@@@@@

//@@@@@@@@POSITION CONTROL@@@@@@@@@@@
float channel_state = 0;
float current_position = 0;
float desired_position = 0;
float p_error = 0;
float p_integral = 0;
float p_derivative =0;
float kpp = 10;
float kpi = 0;
float kpd = -7.5;
float p_dt = 0;
int8_t CHA_value =-1;
int8_t change_lead = 0;
float force_duty_cycle =0.9;
float theoretical_duty_cycle = 0;
int8_t direction=1;

//@@@@@@@@POSITION CONTROL@@@@@@@@@@@

//@@@@@@@@@COMMAND DECODER@@@@@@@@@@@
static const char *regex = "(R|V)-?(\\d+\\.?\\d*)(V?)(\\d+\\.?\\d*)?";
struct slre_cap caps[4];
float nrotations=0;
float nrps=0;
int commandchecker=0;
char *str;
bool newcommand = false;
char command[128];
int m;
bool print=false;
bool music=false;
//@@@@@@@@@COMMAND DECODER@@@@@@@@@@@

//@@@@@@@@@TUNE PARSER@@@@@@@@@@@
int k =0;
int l=0;
char notes[]={'C', 'S', 'D' ,'S','E', 'F', 'S', 'G', 'S', 'A', 'S', 'B'};
float frequency[]={1046.50, 1108.73, 1174.66, 1244.51, 1318.51, 1396.91, 1479.98, 1567.98, 1661.22, 1760.00, 1864.66, 1975.53};
int duration[16]={NULL};
float melody[16]={NULL};
int noteperiod=10;
//@@@@@@@@@TUNE PARSER@@@@@@@@@@@

//@@@@@@@@FUNCTION DEFINITIONS@@@@@@@@@
void motorOut(int8_t driveState); //drive the motor(low level)
inline int8_t readRotorState();//Convert photointerrupter inputs to a rotor state
int8_t motorHome();//Basic synchronisation routine
inline float min(float a, float b); //to return minimum of a and b
void motor_on(); //drive the motor(high level)thread
void get_velocity(); //get+control the velocity
void add_count(); //add count to get print velocity
void commanddecode(); //decode the input commands and change commandchecker
void tuneparse();//parse the command for tune
void control_velocity(); //velocity control system
void control_position(); //position control system
void stop_motor(); //apply 'brakes' to stop the motor
void check_CHA(); //check the quadrature output
void receivecommand(); //receive command from the user
void empty(); //clear the command buffer
void initialize();//initialize a few constants to original values
//@@@@@@@@FUNCTION DEFINITIONS@@@@@@@@@

//Main
int main() {

    pc.printf("Hello\n\r");
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    pc.attach(&receivecommand);

    while(1){

        while (command[0]!='\0'){//If the command buffer is not empty

                    wait(0.1);//wait to make sure receive all the char from user input
                    pc.printf("Input: %s",command);
                    str = command; //
                    initialize();
                    commanddecode(); //decode the command
                    wait(0.5);
                    if(commandchecker==1){  //Play melody                                                                                   //Play tune
                            tuneparse(); //put the notes and their durations to their respective buffer
                            newcommand=true;
                            empty(); //clear the command buffer to ready for the next command
                            thread1.start(motor_on);
                            music=true;
                            while(music){
                             for(int i = 0; i < 15; i++){
                                    if(melody[i]!=0){//Check if there is any zeros to avoid PWM out of range
                                    noteperiod = 1000000/melody[i]; //Calculate the note period (in us)by dividing its frequency
                                    wait(0.5f*duration[i]); //multiply by 0.5 to set the time for each beat
                                    }
                             }
                        }
                    }
                    else if(commandchecker==2){ //Velocity control (V)
                            empty(); //clear the command buffer to ready for the next command
                            nrotations=0;
                            printf("Rotating with:\n"); //print the command by the user
                            printf("R=%0.2f\n",nrotations);
                            printf("V=%0.2f\n",nrps);
                            printf("Lead=%d\n",lead);
                            desired_angular_velocity=nrps;
                            timer.start();
                            timer1.start();
                            newcommand=true;
                            thread1.start(motor_on);
                            CHA1.rise(&get_velocity);
                            CHB1.rise(&add_count);
                    }
                     else if(commandchecker==3){   //Position control (R and RV)
                             empty(); //clear the command buffer to ready for the next command
                             printf("Rotating with:\n"); //print the command by the user
                             printf("R=%0.2f\n",nrotations);
                             printf("V=%0.2f\n",nrps);
                             printf("Lead=%d\n",lead);
                             desired_angular_velocity=nrps;
                             desired_position=nrotations;
                             newcommand=true;
                             timer.start();
                             timer1.start();
                             thread1.start(motor_on);

                             CHA1.rise(&control_velocity);
                             CHB1.rise(&add_count);
                             CHA1.fall(&control_position);
                             CHB1.fall(&check_CHA);
                    }
                    else{
                         printf("Incorrect Command %s !/n", str); //If the command is invalid
                         empty();
                    }

                    }
        wait(0.1);
     }
}


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@FUNCTIONS@@@@@@@@@@@@@@@@@@@@@@@@@@@@

//-@@@@-M-O-T-O-R-O-U-T-@@@@-
void motorOut(int8_t driveState){

    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];

    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;

    //Then turn on
    if (driveOut & 0x01) {
        L1L.period_us(noteperiod);//variable to vary the period of PWM
        L1L.write (duty_cycle); //variable to vary the duty cycle
    };

    if (driveOut & 0x02) L1H = 0;

    if (driveOut & 0x04) {
        L2L.period_us(noteperiod);
        L2L.write (duty_cycle);
    };

    if (driveOut & 0x08) L2H = 0;

    if (driveOut & 0x10) {
        L3L.period_us(noteperiod);
        L3L.write (duty_cycle);
    };

    if (driveOut & 0x20) L3H = 0;

}

//-@@@@-R-E-A-D-R-O-T-O-R-S-T-A-T-E-@@@@-
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
}

//-@@@@-M-O-T-O-R-H-O-M-E-@@@@-
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(1.0);

    //Get the rotor state
    return readRotorState();
}

//-@@@@-M-I-N-@@@@-
inline float min(float a, float b){//declare a min function to obtain minimum numbers between 2 number
    if (a>b) {
        return b;
    }
    else {
        return a;
    }
}

//-@@@@-M-O-T-O-R-O-N-@@@@-
void motor_on(){
  while(newcommand){
    while(change_lead != 1){

        intState = readRotorState();
        if (intState != intStateOld) {
            intStateOld = intState;
            motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
        }
    }
 }
}

//-@@@@-G-E-T-V-E-L-O-C-I-T-Y-@@@@-                     //for V
void get_velocity(){//control parameters for velocity control PI
    current_time = timer.read_us();
    time_per_state = (current_time - previous_time)/1000000;
    previous_time = current_time;
    angular_velocity = 1 / (time_per_state * 117);

    if (angular_velocity != desired_angular_velocity){

        v_dt = time_per_state;
        v_error = desired_angular_velocity - angular_velocity;
        v_integral = v_integral + v_error * v_dt * kvi;

        v_theoretical = (kvp * v_error) +  v_integral ;

        if (v_theoretical < 0){
            duty_cycle = 0;
        }
        duty_cycle = min (0.9, v_theoretical);
    }
}

//-@@@@-C-O-N-T-R-O-L-V-E-L-O-C-I-T-Y-@@@@-                    // for R n RV
void control_velocity(){//control parameters to control velocity for the RV function
    current_time = timer.read_us();
    time_per_state = (current_time - previous_time)/1000000;
    previous_time = current_time;
    angular_velocity = 1 / (time_per_state * 117);

//    control the speed
    if (angular_velocity != desired_angular_velocity){

        v_dt = time_per_state;
        v_error = desired_angular_velocity - angular_velocity;
        v_integral = v_integral + v_error * v_dt * kvi;

        v_theoretical = (kvp * v_error) +  v_integral ;


        if (v_theoretical < 0){
            velocity_control_duty_cycle = 0;
        }
        velocity_control_duty_cycle = min (0.9, v_theoretical);
    }
}
//-@@@@-S-T-O-P-M-O-T-O-R-@@@@-
void stop_motor(){//apply negative torque when transfer function goes negative by changing the lead to the opposite sign
    //the duty_cycle is then forced to be zero to prevent motor from spinning in the other direction
    if(direction == -1){
        if (CHA_value == 1){
            if(force_duty_cycle !=0){
                lead = +2;
                force_duty_cycle =0.5;
            }
        }
    }

    if(direction == -1){
        if((CHA_value == 0 ) || (angular_velocity < 2)){
            change_lead = 1;
            lead = -2;
            force_duty_cycle = 0;
        }
    }
    if (direction == 1){
        if (CHA_value == 0){
            if(force_duty_cycle !=0){
                lead = -2;
            }
        }
    }
    if(direction == 1){
        if((CHA_value == 1 ) || (angular_velocity < 2)){
            change_lead = 1;
            lead = 2;
            force_duty_cycle = 0;
        }
    }
}
//-@@@@-C-O-N-T-R-O-L-P-O-S-I-T-I-O-N-@@@@-
void control_position(){//control system for the position
    channel_state++;
    current_position = channel_state/117;
    if (current_position != desired_position){

        p_dt = time_per_state;
        p_error = desired_position - current_position;
        p_integral = p_integral + p_error * p_dt* kpi;
        p_derivative = angular_velocity;

        theoretical_duty_cycle = (kpp * p_error) + p_integral + kpd* p_derivative;

        if (theoretical_duty_cycle < 0){
            stop_motor();
        }
        else {
            duty_cycle = min (min(velocity_control_duty_cycle, theoretical_duty_cycle), force_duty_cycle);
        }
    }
}

//-@@@@-A-D-D-V-E-L-O-C-I-T-Y-@@@@-
void add_count(){//read velocity after each full revolution as specified in the coursework specifications
    count++;
    if(count>116){
        count = 0;
        time_per_revolution = timer1.read_us();
        print_velocity = 1000000/time_per_revolution;
        timer1.reset();
        timer1.start();
    }
}

//-@@@@-C-H-E-C-K-C-H-A-@@@@-
void check_CHA(){
    CHA_value = CHA1.read();
}

//-@@@@-C-O-M-M-A-N-D-D-E-C-O-D-E-@@@@-
void commanddecode(){   // decodes the command and returns commandchecker to see what type of command it is. Parses R and V commands ( eg R100=[R][100], R20V5=R[20]V[5] )
        if(command[0]=='T'){
            commandchecker=1;
        }
        else if( command[0]=='R'||'V'){

            if(command[1]=='-'){
            lead=-2;
            direction=-1;
            }
            else {
            lead=2;
            direction=1;
            }

            if (slre_match(regex,str, strlen(str), caps, 4, 0) > 0) {
                if(command[0]=='V'){
                     commandchecker=2;
                     nrps=atof(caps[1].ptr); //convert to float for use in functions
                }
             //r
                else {//rv
                     commandchecker=3;
                     nrotations=atof(caps[1].ptr);
                     if(*caps[2].ptr=='V'){ //convert to float for use in functions
                            nrps=atof(caps[3].ptr); //convert to float for use in functions
                      }
                     else{
                         nrps=5;//turn at V5 when no V is set
                    }
                }
            }
            else {

                commandchecker=-1; //invalid command

            }
        }
}

//-@@@@-T-U-N-E-P-A-R-S-E-@@@@-
void tuneparse(){//decodes T command and puts notes and durations into melody[] and duration[] buffer accordingly
    int k =0;
    int l=0;
    for(int i=1; i<sizeof(command);i++){
              if(isdigit(command[i])){
                        duration[k]=command[i]-'0'; //convert from ASCII into int
                        k++;
              }
              for(int j=0;j<sizeof(notes);j++){
                  if(command[i]==notes[j]){
                      if(command[i+1]=='#'){
                          melody[l]=frequency[j+1];
                        }
                      else if(command[i+1]=='^'){
                          melody[l]=frequency[j-1];
                        }
                      else{
                          melody[l]=frequency[j];
                        }
                      l++;
                 }
            }
      }
}


//-@@@@-R-E-C-E-I-V-E-C-O-M-M-A-N-D-@@@@-
void receivecommand(){
    newcommand=false;
    music=false;
    command[m] = pc.getc(); //save the char into command[] buffer
    m++;

}

//-@@@@-E-M-P-T-Y-@@@@-
void empty(){
    for (m=0;m<128;m++){ //clear the command[] buffer
        command[m]='\0';
    }
    m=0;
}

//-@@@@-I-N-I-T-I-A-L-I-Z-E-@@@@-
void initialize(){ //Reset the variables for
          CHA1.fall(NULL);
          CHB1.fall(NULL);
          lead = 2;
          duty_cycle = 0.5;
          count =0;
          channel_state = 0;
          current_position = 0;
          CHA_value =-1;
          change_lead = 0;
          force_duty_cycle =0.9;
          theoretical_duty_cycle = 0;
          direction=1;
          noteperiod=10;
}
