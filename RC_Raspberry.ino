#include <Servo.h> 

#define PINCAMERAY 2 
#define PINCAMERAZ 3
#define PINSERVO 4
#define PINMOTOR 5
#define STARTPWM 116


bool printAngle=false;

/*
 * Servo code
 * For servos items we have to use PIN: 2, 3, 4, 5, 6, 7, 8
 */

int pulsoMotor=103; //Initial valor to initialice the ESC, if you are using a different esc, this valor maybe change.
int pulsoDireccion=90; // Initial valor for the direction servo

int direccion=0;

Servo servoY,servoZ,servoD, motor;

byte recibiendoByte ;
boolean iniciado = false;

int x,y, mov, dire;

int option=0;


/*
 * 
 */

void setup() {
  Serial.begin(115200);
  /*
   * Setup servos
   */

  servoY.attach(PINCAMERAY);
  servoZ.attach(PINCAMERAZ);
  servoD.attach(PINSERVO);
  motor.attach(PINMOTOR);


  while ( iniciado==false ){
          motor.write(0);   // Armado
          servoD.write(pulsoDireccion); 
          
          iniciado=true;
            
  } 
  motor.write(pulsoMotor);
   //motor.write(117);
   //delay(100);
 
}
void loop() {
 

 if (Serial.available()>0){
    //leemos la opcion enviada
    option=Serial.read();
    if(option=='a') {
      servoD.write(0);
      //Serial.println("derecha");
    }
    if(option=='k') {
      servoD.write(45);
      //Serial.println("derecha");
    }
    if(option=='b') {
      servoD.write(180);
      //Serial.println("izquierda");
    }
    if(option=='j') {
      servoD.write(135);
      //Serial.println("izquierda");
    }
    if(option=='g') {
      servoD.write(90);
      //Serial.println("centro");
    }
    if (option=='c') {
          motor.write(117);
          
          delay(200);
          motor.write(102);
          delay(500);
          servoD.write(90);
          //Serial.println("adelante");
        }
    
    
    if(option=='d'){
          motor.write(102);
          servoD.write(90);
          //Serial.println("para");
        }
    if(option=='e'){
          servoD.write(90);
          motor.write(70);
          delay(500);
          motor.write(102);
          servoD.write(90);
          //Serial.println("atras");
    }
    if(option=='f'){
          motor.write(117);
          
    }
    if(option=='h'){
          motor.write(117);
          motor.write(102);
          
    }
    if(option=='i'){
          motor.write(40);
          
          servoD.write(90);
          delay(50);
          motor.write(102);
          //Serial.println("para");
        }
  }
}
