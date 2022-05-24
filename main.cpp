#include "mbed.h"
#include "drivers/DigitalOut.h"

#include "erpc_simple_server.h"
#include "erpc_basic_codec.h"
#include "erpc_crc16.h"
#include "UARTTransport.h"
#include "DynamicMessageBufferFactory.h"
#include "bbcar_control_server.h"
// Uncomment for actual BB Car operations
#include "bbcar.h"
#include <cmath>
#include <exception>
//
#define CENTER_BASE 1500
 Ticker servo_ticker;
 Ticker encoder_ticker;
 PwmOut pin5(D5), pin6(D6);
 BusInOut qti(D3,D4,D7,D8);
 //DigitalInOut pin10(D10);//ping
 DigitalIn encoder(D11);
 BBCar car(pin5, pin6, servo_ticker);
 EventQueue queue(32 * EVENTS_EVENT_SIZE);
 EventQueue queue2(32 * EVENTS_EVENT_SIZE);
 EventQueue queue3(32 * EVENTS_EVENT_SIZE);
 EventQueue queue_1(32 * EVENTS_EVENT_SIZE);
 Timer timer;
 Thread t,t2,t3,t_1;
 //parallax_ping  ping1(pin10);
 float  time_pass=0;
 double pingbef=0,pingminu=0,judge=1;
 double pingtemp[2];
 bool K=0,jud=1;
 int ok=1,distant=0,timess=1,timess2=1,speed=0,qtirec=0,state=0;
 volatile int steps;
 volatile int last;
/**
 * Macros for setting console flow control.
 */
#define CONSOLE_FLOWCONTROL_RTS     1
#define CONSOLE_FLOWCONTROL_CTS     2
#define CONSOLE_FLOWCONTROL_RTSCTS  3
#define mbed_console_concat_(x) CONSOLE_FLOWCONTROL_##x
#define mbed_console_concat(x) mbed_console_concat_(x)
#define CONSOLE_FLOWCONTROL mbed_console_concat(MBED_CONF_TARGET_CONSOLE_UART_FLOW_CONTROL)

mbed::DigitalOut led1(LED1, 1);
mbed::DigitalOut led2(LED2, 1);
mbed::DigitalOut led3(LED3, 1);
mbed::DigitalOut* leds[] = { &led1, &led2, &led3 };
// Uncomment for actual BB Car operations
 BBCar* cars[] = {&car}; //Control only one car

/****** erpc  *******/
void stop(uint8_t car){
    if(car == 1) {}
}
void goStraight(uint8_t car, int32_t  speed){
    if(car == 1) {}
}
void turn(uint8_t car, int32_t speed, double factor){
    if(1) {
        printf(" %d , %f , \n",qtirec, time_pass,steps*6.5*3.14/32 );
    }
}
/****** erpc  *******/



void encoder_control() {
   int value = encoder;
   if (!last && value) steps++;
   last = value;
}
void go_1status(){
    if(state==1) {
       steps=0;
       car.goStraight(20);
       ThisThread::sleep_for(1000ms);
       state=2;
    } 
}
void Qtirecord(){
    qti.output();
    qti=0b1111;
    wait_us(250);
    qti.input();
    wait_us(250);
    qtirec=qti;
}
void suddenstop(){
        speed=0;
        if (speed > 200)       speed = 200;
        else if (speed < -200) speed = -200;
        pin5  = (CENTER_BASE+speed)/20000.0f;
        pin6  = (CENTER_BASE+speed)/20000.0f;
}
int facter=0.4;
int speedt=50;
void Qtijudge(){
    car.stop();
    //ThisThread::sleep_for(1ms);
    if(qtirec==0b1000) car.turn(100,0.4); //left
    else if(qtirec==0b1100) car.turn(80,0.4);
    else if(qtirec==0b0100) car.turn(50,0.4);
    else if(qtirec==0b0110) car.goStraight(10);
    else if(qtirec==0b0010) car.turn(50,-0.4);//right
    else if(qtirec==0b0011) car.turn(80,-0.4);
    else if(qtirec==0b0001) car.turn(100,-0.4);
    else if(qtirec==0b0000) car.goStraight(-20);
}

/** erpc infrastructure */
ep::UARTTransport uart_transport(D1, D0, 9600);
ep::DynamicMessageBufferFactory dynamic_mbf;
erpc::BasicCodecFactory basic_cf;
erpc::Crc16 crc16;
erpc::SimpleServer rpc_server;
/** LED service */
BBCarService_service car_control_service;

int main(void) {

// Initialize the rpc server
  uart_transport.setCrc16(&crc16);
// Set up hardware flow control, if needed
#if CONSOLE_FLOWCONTROL == CONSOLE_FLOWCONTROL_RTS
  uart_transport.set_flow_control(mbed::SerialBase::RTS, STDIO_UART_RTS, NC);
#elif CONSOLE_FLOWCONTROL == CONSOLE_FLOWCONTROL_CTS
  uart_transport.set_flow_control(mbed::SerialBase::CTS, NC, STDIO_UART_CTS);
#elif CONSOLE_FLOWCONTROL == CONSOLE_FLOWCONTROL_RTSCTS
  uart_transport.set_flow_control(mbed::SerialBase::RTSCTS, STDIO_UART_RTS, STDIO_UART_CTS);
#endif
  printf("Initializing server.\n");
  rpc_server.setTransport(&uart_transport);
  rpc_server.setCodecFactory(&basic_cf);
  rpc_server.setMessageBufferFactory(&dynamic_mbf);
  // Add the led service to the server
  printf("Adding BBCar server.\n");
  rpc_server.addService(&car_control_service);
  // Run the server. This should never exit
  printf("Running server.\n");
//*****erpc**********



  //t_1.start(callback(&queue_1, &EventQueue::dispatch_forever));
  //queue_1.call(erpc_call);
   /*double pwm_table0[] = {150, 120, 90, 60, 30, 0, -30, -60, -90, -120, -150};
   double speed_table0[] = {42.252,41.136,37.947,28.620,13.393,0.000,16.024,30.214,38.346,41.694,42.970};
   double pwm_table1[] = {-150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150};
   double speed_table1[] = {41.376,40.020,37.150,28.381,12.755,0.000,16.741,30.693,40.180,41.056};
   // first and fourth argument : length of table
   car.setCalibTable(11, pwm_table0, speed_table0, 11, pwm_table1, speed_table1);*/
  encoder_ticker.attach(&encoder_control, 1ms);
  pin5.period_ms(20);
  pin6.period_ms(20);
  t.start(callback(&queue, &EventQueue::dispatch_forever));
  queue.call_every(1ms, Qtirecord);
  t3.start(callback(&queue3, &EventQueue::dispatch_forever));
  queue3.call_every(10ms, Qtijudge);
  t2.start(callback(&queue2, &EventQueue::dispatch_forever));
  //queue2.call_every(1ms, go_1status);
  //printf("distance=%f\n",steps*6.5*3.14/32);
  rpc_server.run();
}
