//Include the SoftwareSerial library
#include "SoftwareSerial.h"
#include <Joystick.h>

// Criando um joystick
Joystick_ Joystick(0x03,0x04,0,0,true,true,true,true,true,true,true,true,true,true,true);

//Create a new software  serial
SoftwareSerial bluetooth(10, 11); //TX, RX (Bluetooth){RX no pino 10 pela necessidade de RX do arduino Leonardo}
int incomingByte;      // a variable to read incoming serial data into

// Pino de entrada para leitura ADC (Pino A0 - 5)
int analogPinX = 0;
int analogPinY = 1;

long int valX = 0;
long int valY = 0;
int count = 0;


// Variaveis de controle
int posIndex = 0;
int dadosPos[11];


void setup() {
  // Inicialiando comunicacao serial com PC
  Serial.begin(9600);
  
  //Initialize the software serial
  bluetooth.begin(9600);

  Joystick.begin();
  // 3 Eixos posicao
  Joystick.setXAxisRange(0,1023);
  Joystick.setYAxisRange(0,1023);
  Joystick.setZAxisRange(0,1023);  

  // 3 Eixos Rotacao
  Joystick.setRxAxisRange(0,1023);  
  Joystick.setRyAxisRange(0,1023);  
  Joystick.setRzAxisRange(0,1023);      

  // Restante eixos
  Joystick.setRudderRange(0,1023);
  Joystick.setThrottleRange(0,1023);
  Joystick.setAcceleratorRange(0,1023);  
  Joystick.setBrakeRange(0,1023);
  Joystick.setSteeringRange(0,1023);
}

void loop() {
  // Verificando se tem mensagem vinda da serial
  if(Serial.available() > 0){
    bluetooth.println((char)Serial.read());
  }
  
  // see if there's incoming serial data:
  if (bluetooth.available() > 0) {
    // read the oldest byte in the serial buffer:
    incomingByte = bluetooth.read();
  //  Serial.println((char)incomingByte);

    // Reseta o indice da posicao recebida
    if((char)incomingByte == ';'){
      posIndex = 0;
      dadosPos[posIndex] = 0;
    }
    else{
      // Separador de dados
      if((char)incomingByte == ':'){
        posIndex++;        
      }
      else
      {
        // Vai deslocando o numero para o lado (o -48 converte de ASCII para inteiro de acordo com a tabela)
        dadosPos[posIndex] = dadosPos[posIndex] * 10 + ((int)incomingByte - 48);
      }

    }
  }

  Serial.println(dadosPos[0]);
  /*Serial.println(dadosPos[1]);
  Serial.println(dadosPos[2]);
  Serial.println(dadosPos[3]);
  Serial.println(dadosPos[4]);
  Serial.println(dadosPos[5]);  
   */     
  
  
  // Define os valores
 // valX = analogRead(analogPinX); 
 valX = dadosPos[0];
 // Serial.print("Valor X:"); 
  Joystick.setXAxis(valX);
  Joystick.setZAxis(valX);  
//  Serial.print(valX);
  
  valY = analogRead(analogPinY);   
//  Serial.print("Valor Y:");
  Joystick.setYAxis(valY);  

   // 3 Eixos Rotacao
  Joystick.setRxAxis(valY);  
  Joystick.setRyAxis(valY);  
  Joystick.setRzAxis(valY);      

  // Restante eixos
  Joystick.setRudder(valY);
  Joystick.setThrottle(valY);
  Joystick.setAccelerator(valY);  
  Joystick.setBrake(valY);
  Joystick.setSteering(valY); 
//  Serial.print(valY);

 // Serial.println("\n");  
  
  // Conversão para escrever no pino de saida
  // 0.249 Conversão para 1024 do ADC para 255 do analogWrite
//  analogWrite(5, 0.249*val);
  //val += analogRead(analogPin);
  
/*  count++;
  if(count > 39){
    val = val/40;
    //val = 110;
    //Serial.println(val*val*val*val);
    double dist = (0.000000005*(val*val*val*val)) +
                  (-0.000007*(val*val*val)) + 
                  (0.0036*(val*val))+
                  (-0.8023*(val))+
                  (82.454);
    Serial.println(dist);
    count = 0;
    val = 0;
    
  }
  */
}
