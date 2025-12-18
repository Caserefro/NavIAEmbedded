#ifndef DEVICE_NETWORKING.h
#define DEVICE_NETWORKING_H

void checkUSBInput() {
  static String inputBuffer = "";

  while (Serial.available()) {
    char incomingByte = Serial.read();

    if (incomingByte == '\n') {
      UART_SEND(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += incomingByte;
    }
  }
}


void UART_SEND(String message) {
  Serial.print("Sent to UART1: ");
  Serial.println(message);
  for (int i = 0; i < message.length() + 1; i++)
    SystemUART.write(message[i]);
}

String readUARTLine(HardwareSerial &uart) {
  String result = "";
  while (uart.available()) {
    char c = uart.read();
    if (c == '\n') break;  // end of message
    result += c;
  }
  return result;
}

#endif