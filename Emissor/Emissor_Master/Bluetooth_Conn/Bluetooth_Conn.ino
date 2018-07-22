#include <SoftwareSerial.h>

int bluetoothTx = 2;
int bluetoothRx = 3;

SoftwareSerial bluetooth(bluetoothTx,bluetoothRx);

int incomingByte = 0;
int desligaConfig = 0;

void Anda(){
delay (1000);  
  bluetooth.write((byte)0xFD); //Start HID Report
  bluetooth.write((byte)0x6);  //Length byte

// Buttons
bluetooth.write(B10000001); // Second Byte (Buttons 1-8)
bluetooth.write(B10000000); // Second Byte (Buttons 9-16)  

// 1. X/Y-Axis
bluetooth.write(45);  //First X coordinate
bluetooth.write(-33); //First Y coordinate

// 2. X/Y-Axis
bluetooth.write(45);  //Second X coordinate
bluetooth.write(-33); //Second Y coordinate

delay (1000);

  bluetooth.write((byte)0xFD); //Start HID Report
  bluetooth.write((byte)0x6);  //Length byte

// Buttons
bluetooth.write(B10000001); // Second Byte (Buttons 1-8)
bluetooth.write(B10000000); // Second Byte (Buttons 9-16)  

// 1. X/Y-Axis
bluetooth.write(77);  //First X coordinate
bluetooth.write(-67); //First Y coordinate

// 2. X/Y-Axis
bluetooth.write(04);  //Second X coordinate
bluetooth.write(-24); //Second Y coordinate
delay (1000);
}

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

void LigaBW(){
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
    //Atraso
    delay(3000);  
    // Definindo o nome
    Serial.write("SN, VasconnectionBT");  
    //Atraso
    delay(3000);    
    // Definindo p hid que é um joystick
    Serial.write("SH, 0240");
    //SH,208 é GAMEPAD    
    // Colocando atraso no modulo
    delay(3000);
    // Resetando o modulo
    Serial.write("R,1");    
}

void setup() {
  Serial.begin(9600);  // Begin the serial monitor at 9600bps

  bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
  bluetooth.print("$");  // Print three times individually
  bluetooth.print("$");
  bluetooth.print("$");  // Enter command mode
  delay(100);  // Short delay, wait for the Mate to send back CMD
  bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
  bluetooth.begin(9600);  // Start bluetooth serial at 9600
}

void loop() {
//  Anda();
  if(bluetooth.available())  // If the bluetooth sent any characters
  {
    // Send any characters the bluetooth prints to the serial monitor
    Serial.print((char)bluetooth.read());  
  }
  if(Serial.available())  // If stuff was typed in the serial monitor
  {

  //      char SerialRec = (char) Serial.read();
// Serial.print(SerialRec);
 //       if(SerialRec == 'a'){
         // Anda();
 //       }
  //      else{
  //        bluetooth.print(SerialRec);               

          
  //      }


        // Send any characters the Serial monitor prints to the bluetooth
        bluetooth.print((char)Serial.read());      


  }
  // and loop forever and ever!
}
