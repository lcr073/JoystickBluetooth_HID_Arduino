// Biblioteca de serial em outras portas
#include <SoftwareSerial.h>

SoftwareSerial bluetooth(10, 11); //TX, RX (Bluetooth){RX no pino 10 pela necessidade de RX do arduino Leonardo}


void configuraBaudBT(){
  bluetooth.begin(115200);  // Baud Rate default do bluetooth
  delay(320);
/*  bluetooth.print("$");  // Print three times individually
  bluetooth.print("$");
  bluetooth.print("$");  // Enter command mode
  delay(100);  // Short delay, wait for the Mate to send back CMD
  bluetooth.println("U,57.6,N");  // Temporarily Change the baudrate to 9600, no parity  
  //bluetooth.println("SU,57");  // Temporarily Change the baudrate to 9600, no parity  
  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
  */
}

void setup() {

  // Ligacao com PC
  Serial.begin(115200); 

  // Define baud rate
  configuraBaudBT();
 
  bluetooth.begin(57600);

  
}

void loop() {
  if(Serial.available() > 0){
    bluetooth.print((char)Serial.read());
   }

  // Printa no monitor serial caso bluetooth mande algo
  if (bluetooth.available() > 0) {

    // Le oque o bluetooth mandou
   // incomingByte = bluetooth.read();

     Serial.print((char)bluetooth.read());
  }

}
