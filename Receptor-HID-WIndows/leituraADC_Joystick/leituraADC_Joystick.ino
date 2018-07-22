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
const int nPosDados = 11;
int dadosPos[nPosDados];
int tempPos[nPosDados];

void atualizaControle(char incomingByte){
  /*
   * Funcao utilizada para obter todos os parametros e atualizar de uma unica 
   * vez a colecao a ser utilizada
   */
    // Reseta o indice da posicao recebida
    if((char)incomingByte == ';'){
      posIndex = 0;
      
      // Transfere os dados e limpa o vetor temporario para novos dados
      for(int i = 0; i < nPosDados; i++){
        dadosPos[i] = tempPos[i];
        tempPos[i] = 0;
      }
    }
    else{
      // Separador de dados
      if((char)incomingByte == ':'){
        posIndex++;        
      }
      else
      {
        // Vai deslocando o numero para o lado (o -48 converte de ASCII para inteiro de acordo com a tabela)
        tempPos[posIndex] = tempPos[posIndex] * 10 + ((int)incomingByte - 48);
      }

    }
}

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

    atualizaControle(incomingByte);
    // Serial.println((char)incomingByte);
  }

//  Serial.println(dadosPos[0]);
  /*Serial.println(dadosPos[1]);
  Serial.println(dadosPos[2]);
  Serial.println(dadosPos[3]);
  Serial.println(dadosPos[4]);
  Serial.println(dadosPos[5]);  
   */     
  
  
  // Define os valores obtidos pelo bluetooth
  // X,Y,Z posicao
  Joystick.setXAxis(dadosPos[0]);
  Joystick.setYAxis(dadosPos[1]);  
  Joystick.setZAxis(dadosPos[2]);  
  
   // 3 Eixos Rotacao
  Joystick.setRxAxis(dadosPos[3]);  
  Joystick.setRyAxis(dadosPos[4]);  
  Joystick.setRzAxis(dadosPos[5]);      

  // Restante eixos
  Joystick.setRudder(dadosPos[6]);
  Joystick.setThrottle(dadosPos[7]);
  Joystick.setAccelerator(dadosPos[8]);  
  Joystick.setBrake(dadosPos[9]);
  Joystick.setSteering(dadosPos[10]); 

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
