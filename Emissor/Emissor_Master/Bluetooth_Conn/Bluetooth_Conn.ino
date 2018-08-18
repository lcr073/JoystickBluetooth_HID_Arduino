#define SCL_PIN 7
#define SCL_PORT PORTD
#define SDA_PIN 6
#define SDA_PORT PORTD

// Biblioteca utilizada no I2C
#include <SoftI2CMaster.h>
#include <SoftWire.h>

// Biblioteca de serial em outras portas
#include <SoftwareSerial.h>

int bluetoothTx = 2;
int bluetoothRx = 3;

// Cria objeto para I2C
//SoftWire Wire = SoftWire();

// Cria objeto para outra serial
SoftwareSerial bluetooth(bluetoothTx,bluetoothRx);

int incomingByte = 0;
int desligaConfig = 0;

// Caso o pino A0 estiver desligado e 0x68,
// caso estiver ligado vira 0x69;
const int MPU = 0x68;

// Obtem informações acelerometro, giroscopio e temperatura
int AcX,AcY,AcZ,Tmp, GyX,GyY,GyZ;


void DefineMaster(){
    // Colocando atraso no modulo
    delay(3000);
    // Resetando o modulo
    Serial.write("R,1");
    // Atraso
    delay(3000);
    // Entrando no modo de comandos
    Serial.write("$$$");
    //Atraso
    delay(3000);
    // Habilitando modo de pareamentro
    Serial.write("SM,6");
  
}

void setup() {
  // Inicializa a comunicacao Serial com baud Rate 9600bps
  Serial.begin(57600);

  // ### Inicio ligar I2C ###
//  Wire.begin();
//  Wire.beginTransmission(MPU);
//  Wire.write(0x6B);

  // Inicializando o MPU-6050
 // Wire.write(0);
 // Wire.endTransmission(true);
  // ### Fim ligacao I2C ###  
  
  // ### Inicio definicoes bluetooth ###  
  // Altera velocidade de comunicacao default do modulo bluetooth
/*  bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
  bluetooth.print("$");  // Print three times individually
  bluetooth.print("$");
  bluetooth.print("$");  // Enter command mode
  delay(100);  // Short delay, wait for the Mate to send back CMD
  bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
  */
  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
  bluetooth.begin(57600);  // Start bluetooth serial at 9600
  // ### Fim definicoes bluetooth ###
}

void loop() {
  /* 
   * Obtem valores do ADC e proveniente de outros protocolos
   * (I2C) para montar uma string contendo os dados e separados por :
   * e encerrado por ;
   * modelo:
   * 
   * posX:posY:posZ:rotX:RotY,RotZ:Dedo1:Dedo2:Dedo3:Dedo4:Dedo5;
   */
  
  // Iniciando captura valores do MPU-6050
/*  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  
  //Solicita os dados do sensor
  Wire.requestFrom(MPU,14,true);  
  
  //Armazena o valor dos sensores nas variaveis correspondentes
  AcX=Wire.read()<<8|Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read(); //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

*/

  GyX = 0;
  GyY = 0;
  GyZ = 0;
  
  // Le valores dos ADC's
  // Le posicoes
  int readX = analogRead(0);
  int readY = analogRead(2);
  //int readZ = analogRead(2);  

  // Le flex sensors para os dedos
  //int dedo1 = analogRead(3)
  //int dedo2 = analogRead(4)
  //int dedo3 = analogRead(5)
  //int dedo4 = analogRead(6)
  //int dedo5 = analogRead(7)

  char Data[150];
  strcpy(Data,""); 

// Posicao X,Y,Z
  // X
  char valX[5];
  sprintf(valX, "%d", readX);
  strcat(Data, valX);

  // Y
  strcat(Data,":");
  char valY[5];
  sprintf(valY, "%d", readY);  
  strcat(Data, valY);

  // Z  
  strcat(Data,":");
  strcat(Data,"0");

// Rotacao X,Y,Z
  // rotX  
  strcat(Data,":");
  char rotX[5];
  sprintf(rotX, "%d", GyX);  
  strcat(Data, rotX);

  // rotY  
  strcat(Data,":");
  char rotY[5];
  sprintf(rotY, "%d", GyY);  
  strcat(Data, rotY);

  // rotZ
  strcat(Data,":");
  char rotZ[5];
  sprintf(rotZ, "%d", GyZ);  
  strcat(Data, rotZ);
  
// Eixos
  // Eixo 7
  strcat(Data,":");  
  strcat(Data,"0");  

  // Eixo 8
  strcat(Data,":");
  strcat(Data,"0");  

  // Eixo 9
  strcat(Data,":");
  strcat(Data,"0");    

  // Eixo 10
  strcat(Data,":");
  strcat(Data,"0");      

  // Eixo 11
  strcat(Data,":");
  strcat(Data,"0");      
  
  strcat(Data,";"); 

 Serial.println(Data);
 
 // Quebra de linha
// Serial.println("");

  bluetooth.print(Data);

  delay(40);
  if(bluetooth.available())  // If the bluetooth sent any characters
  {
    // Send any characters the bluetooth prints to the serial monitor
    Serial.print((char)bluetooth.read());  
  }
  
  if(Serial.available())  // If stuff was typed in the serial monitor
  {

  /*
       char SerialRec = (char) Serial.read();
       Serial.print(SerialRec);
        if(SerialRec == 'a'){
         // Anda();
        }
        else{
          bluetooth.print(SerialRec);                       
         }
*/
        //  Manda os caracteres enviados pela Serial do PC para o bluetooth
        bluetooth.print((char)Serial.read());      


  }
}
