// Biblioteca de serial em outras portas
#include <SoftwareSerial.h>

int bluetoothTx = 10;
int bluetoothRx = 11;

// Cria objeto para outra serial
SoftwareSerial bluetooth(bluetoothTx,bluetoothRx);

void setup() {
  // Inicializa serial com o PC
  Serial.begin(9600);
  
  // Inicializa serial da biblioteca com bluetooth
  // 38400 e o baud rate do modo AT (Modo de configuracao)
  bluetooth.begin(38400);
}

void loop() {
  // Se o dispositivo receber mensagens do bluetooth
  if(bluetooth.available()) 
  {
    // Manda essas mensagens para o monitor serial
    Serial.print((char)bluetooth.read());  
  }

  // Se alguma coisa for digitada no monitor serial
  if(Serial.available()) 
  {
    //  Manda os caracteres enviados pela Serial do PC para o bluetooth
    bluetooth.print((char)Serial.read());      
  }
}
