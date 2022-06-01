#include "mbed.h"
#include "drivers/DigitalOut.h"

#include "erpc_simple_server.h"
#include "erpc_basic_codec.h"
#include "erpc_crc16.h"
#include "UARTTransport.h"
#include "DynamicMessageBufferFactory.h"
#include "bbcar.h"
#include <cmath>
#include <exception>

#include "bbcar_control_server.h"
//
#define CENTER_BASE 1500
 Ticker servo_ticker;
 Ticker encoder_ticker;
 PwmOut pin5(D5), pin6(D6);//pin6 servo1 正常  右手馬達
 BusInOut qti(D3,D4,D7,D8);
 //DigitalInOut pin10(D10);//ping
 DigitalIn encoder(D11);
 DigitalInOut pin10(D10);
 BBCar car(pin5, pin6, servo_ticker);//p5 servo 0
 EventQueue queue(32 * EVENTS_EVENT_SIZE);
 EventQueue queue2(32 * EVENTS_EVENT_SIZE);
 EventQueue queue3(32 * EVENTS_EVENT_SIZE);
 EventQueue queue_3(32 * EVENTS_EVENT_SIZE);
 EventQueue queue_1(32 * EVENTS_EVENT_SIZE);
 parallax_ping  ping1(pin10);
 Timer timer;
 Thread t,t2,t3,t_1,t_2,t_3,t_encoder;
 //parallax_ping  ping1(pin10);
 float  time_pass=0,pdistant=0,speednow=0;
 double pingbef=0,pingminu=0,judge=1;
 double pingtemp[2];
 bool K=0,jud=1;
 int ok=1,timess=0,timesstemp=1,speed=0,qtirec=0,state=0,tempstate,obstacle=0,pingbeg=0,final=0;
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
#define CONSOLE_FLOWCONTROL      mbed_console_concat(MBED_CONF_TARGET_CONSOLE_UART_FLOW_CONTROL)

mbed::DigitalOut led1(LED1, 1);
mbed::DigitalOut led2(LED2, 1);
mbed::DigitalOut led3(LED3, 1);
mbed::DigitalOut* leds[] = { &led1, &led2, &led3 };
// Uncomment for actual BB Car operations
 BBCar* cars[] = {&car}; //Control only one car

/****** erpc  *******/

void stop(uint8_t car){
    if(car == 1) { //there is only one car
          *leds[car - 1] = 0;
        // Uncomment for actual BB Car operations
        // (*cars[car -1]).stop();
        //printf("Car %d stop.\n", car);
        printf("%d ,%d,t=%d,%f,obs=%d,temp=%d,dis=%f,speed=%f\n",qtirec,state,timess,pdistant,obstacle,tempstate,steps*6.5*3.14/32,speednow);
  }
}

void goStraight(uint8_t car, int32_t  speed){
    if(car == 1) { //there is only one car
          *leds[car - 1] = 0;
        // Uncomment for actual BB Car operations
        // (*cars[car -1]).goStraight(speed);
        //printf("Car %d go straight at speed %d.\n", car, speed);
        printf("%d ,%d,t=%d,%f,obs=%d,temp=%d,dis=%f,speed=%f\n",qtirec,state,timess,pdistant,obstacle,tempstate,steps*6.5*3.14/32,speednow);
  }
}

void turn(uint8_t car, int32_t speed, double factor){
    if(car == 1) { //there is only one car
          *leds[car - 1] = 0;
        // Uncomment for actual BB Car operations
        // (*cars[car -1]).turn(speed, factor);
        //printf("Car %d turn at speed %d with a factor of %f.\n", car, speed, factor);
        printf("%d ,%d,t=%d,%f,obs=%d,temp=%d,dis=%f,speed=%f\n",qtirec,state,timess,pdistant,obstacle,tempstate,steps*6.5*3.14/32,speednow);
  }
}
/****** erpc  *******/



void encoder_control() {
    timer.start();
   int value = encoder;
   if (!last && value){
       steps++;
       timer.stop();
       time_pass=timer.read();
       speednow=1*6.5*3.14/32/time_pass;
       timer.reset();
    } 
   last = value;
}
int once=0;
void statusjudge(){
        //ThisThread::sleep_for(100ms);
        if(qtirec==0b1111&&timess==0&&obstacle==0){
            state++;
            timess=1;
            //printf("stst++");
            ThisThread::sleep_for(1s);//要得
        }
        if(pdistant>5){
            pingbeg=1;
        }//將一開始ping雜質刷掉
        /*if(qtirec==0b1111&&timess==0&&obstacle==1){
            timess=1;
        }*/
        if(pdistant<20&&pingbeg==1){
            if(timesstemp==0){
            tempstate=state;
            timesstemp=1;
            }//確保temp只被刷新一次
            state=-1;
            obstacle=1;
            //printf("ping\n");
        }
        /*if(state==-1&&qtirec==0b1111&&obstacle==1){
            state=tempstate;
        }*/
        if(qtirec==0b1111&&obstacle==1&&once==0){
        state=tempstate--;
        once=1;
        //obstacle=0;
        } //timess證明1111以過去
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


double facter=0.4;
double speedt=35,speedt2=38,speedt3=38;//25 38 38 18.5
double straight=19;
void Qtijudge(){
    car.stop();
    if(qtirec==0b1000&&state!=-1){
            car.turn(45,facter); //right
            timess=0;
    } 
    else if((qtirec==0b1100||qtirec==0b1110)&&state!=-1){
            car.turn(45,facter); //right
            timess=0;
    }
    else if(qtirec==0b0100&&state!=-1){
            car.turn(40,facter);//right
            timess=0;
    } 
    else if(qtirec==0b0110&&state!=-1){
            car.goStraight(straight); //left
            timess=0;
    } 
    else if(qtirec==0b0010&&state!=-1){
            car.turn(29,-facter); //left
            timess=0;
    } 
    else if((qtirec==0b0011||qtirec==0b0111)&&state!=-1){
            car.turn(39,-facter); //left
            timess=0;
    } 
    else if(qtirec==0b0001&&state!=-1){
            car.turn(speedt3,-facter); //left
            timess=0;
    }///以下未改
    else if(qtirec==0b0000&&state!=-1&&state!=2){
        car.goStraight(-straight);
        timess=0;
    }//怕超過岔點所以state!=2
    else if(qtirec==0b1111&&state!=-1&&state!=2&&obstacle==0){
        car.goStraight(straight);
        timess=0;
    } 
    else if(qtirec==0b1111&&state==2&&obstacle==0){
        car.turn(45,facter);
        ThisThread::sleep_for(1s);//要得
        state++;
    } //1111其一狀況第一叉點

    else if(qtirec==0b1111&&obstacle==1&&timess==0){
        car.turn(45,facter);
        //printf("in\n");
        ThisThread::sleep_for(1s);//要得
        obstacle=2;
        //timess=1;//未測試
        car.goStraight(straight);
        ThisThread::sleep_for(500ms);
    } //1111其一狀況 回頭  //以上未改
    else if(qtirec==0b1111&&obstacle==2&&timess==0){
        car.turn(38,facter);
        ThisThread::sleep_for(880ms);
        car.goStraight(straight);
        ThisThread::sleep_for(300ms);
        //timess=1;//未測試
        final++;
        /*if(final>3){
            car.turn(50,1);
            ThisThread::sleep_for(1000ms);
        }*/
    }
    else if((qtirec==0b1001||qtirec==0b1101||qtirec==0b1011||qtirec==0b0101||qtirec==0b1010)&&state!=-1){
            car.goStraight(straight);//left
            //printf("bug");
            timess=0;
    } //排除特例
    else if(state==-1){
        //printf("in\n");
        car.turn(75,1);
        ThisThread::sleep_for(350ms);//要得
        state=tempstate;
        obstacle=1;
        timess=1;
    }
}
int i=1;
void pingprint(){
    pdistant=ping1;
}
void print(){
    //printf("%d ,%d,t=%d,%f,obs=%d,temp=%d\n",qtirec,state,timess,pdistant,obstacle,tempstate);
}
void encoder_t(){
    encoder_ticker.attach(&encoder_control, 1ms);
}

/** erpc infrastructure */
ep::UARTTransport uart_transport(D1, D0, 9600);
ep::DynamicMessageBufferFactory dynamic_mbf;
erpc::BasicCodecFactory basic_cf;
erpc::Crc16 crc16;
erpc::SimpleServer rpc_server;

/** LED service */
BBCarService_service car_control_service;
/*void erpc_call(){
    rpc_server.run();
}*/
void erpc_init(){
  printf("Initializing server.\n");
  rpc_server.setTransport(&uart_transport);
  rpc_server.setCodecFactory(&basic_cf);
  rpc_server.setMessageBufferFactory(&dynamic_mbf);

  // Add the led service to the server
  printf("Adding BBCar server.\n");
  rpc_server.addService(&car_control_service);

  // Run the server. This should never exit
  printf("Running server.\n");
  rpc_server.run();
}


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
 /* double pwm_table0[] = {150, 120, 90, 60, 30, 0, -30, -60, -90, -120, -150};
   double speed_table0[] = {42.252,41.136,37.947,28.620,13.393,0.000,16.024,30.214,38.346,41.694,42.970};*/
   double pwm_table0[] = {-150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150};
   double speed_table0[] = {41.376,40.020,37.150,28.381,12.755,0.000,16.741,30.693,40.180,41.056};
   double pwm_table1[] = {-150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150};
   double speed_table1[] = {41.376,40.020,37.150,28.381,12.755,0.000,16.741,30.693,40.180,41.056};

  // first and fourth argument : length of table
  car.setCalibTable(11, pwm_table0, speed_table0, 11, pwm_table1, speed_table1);
  t.start(callback(&queue, &EventQueue::dispatch_forever));
  queue.call_every(1ms, Qtirecord);
  t3.start(callback(&queue3, &EventQueue::dispatch_forever));
  queue3.call_every(2ms, Qtijudge);
  t2.start(callback(&queue2, &EventQueue::dispatch_forever));
  queue2.call_every(3ms, statusjudge);
  t_2.start(erpc_init);
  t_encoder.start(encoder_t);
  t_1.start(callback(&queue_1, &EventQueue::dispatch_forever));
  queue_1.call_every(700ms, pingprint);
  t_3.start(callback(&queue_3, &EventQueue::dispatch_forever));
  queue_3.call_every(10ms, print);
}
