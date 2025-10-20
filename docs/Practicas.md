## Práctica 1

Esta practiica podemos observar el uso de un circuito integrado 555 de manera astable en la cual, hace parpadear un diodo led dependiendo del valor de las resistencias.


[Practica 1][doc-ref]

[doc-ref]: https://github.com/user-attachments/assets/a55c972d-851a-4645-9918-f97dc848011a "Practica 1"


## Práctica 2

Esta practica consistio en usar un ESP32 para controlar de distantas maneras un led sea desde solo el ESP32, como con este y un botón y por medio de bluetooth.


### ESP32 solo:<br>
<br>
 const int led=33<br>
<br>
  void setup () {<br>
  Serial. begin (115200);<br>
  pinMode (Led, OUTPUT) ;<br>
 }<br>
<br>
<br>
 void lo0p() {<br>
 digitalWrite(led,1);<br>
 delay (1000);<br>
 digitalwrite(led,0);<br>
 delay (1000);<br>
 }<br>

[ESP32 solo] https://github.com/user-attachments/assets/6e66b9b6-49fc-4279-bbd0-478fd867dd4a 

### ESP32 con botón:<br>
<br>
const int led=33;<br>
const int btn=34;<br>
void setup() {<br>
  Serial.begin(115200);<br>
  pinMode(led,OUTPUT);<br>
  pinMode(btn,INPUT);<br>
}<br>

void loop() {<br>
  int estado = digitalRead(btn);<br>
  if(estado == 1 ){<br>
    digitalWrite(led,1);<br>
  }<br>
  else{<br>
    digitalWrite(led,0);
  }
}<br>

[ESP32 con botón] https://github.com/user-attachments/assets/2fb48297-2228-4e13-8b8e-80b5b23b4017

### ESP32 Bluetooth:<br>
<br>
#include "BluetoothSerial.h"<br>
BluetoothSerial SerialBT;<br>
const int led=33;<br>
void setup() {<br>
  pinMode(led,OUTPUT);<br>
    Serial.begin(115200);<br>
    SerialBT.begin("AbrahamEsp32"); // Nombre del dispositivo Bluetooth<br>
}<br>
<br>
void loop() {<br>
    if (SerialBT.available()) {<br>
        String mensaje = SerialBT.readString();<br>
        Serial.println("Recibido: " + mensaje);<br>
        if(mensaje == "on" ){<br>
    digitalWrite(led,1);<br>
  }<br>
  else{<br>
    digitalWrite(led,0);<br>
    }}<br>
    delay(100);<br>
}<br>
<br>
[ESP32 Bluetooth]https://github.com/user-attachments/assets/15d742b1-f1fd-4519-a911-5364dd9c4c94 

