## Práctica 1

Esta practiica podemos observar el uso de un circuito integrado 555 de manera astable en la cual, hace parpadear un diodo led dependiendo del valor de las resistencias.


[Practica 1][doc-ref]

[doc-ref]: https://github.com/user-attachments/assets/a55c972d-851a-4645-9918-f97dc848011a "Practica 1"


## Práctica 2

Esta practica consistio en usar un ESP32 para controlar de distantas maneras un led sea desde solo el ESP32, como con este y un botón y por medio de bluetooth.


### ESP32 solo:<br>

```
const int led=33

  void setup () {
  Serial. begin (115200);
  pinMode (Led, OUTPUT) ;
 }


 void lo0p() {
 digitalWrite(led,1);
 delay (1000);
 digitalwrite(led,0);
 delay (1000);
 }
``` 

[ESP32 solo](https://github.com/user-attachments/assets/6e66b9b6-49fc-4279-bbd0-478fd867dd4a)

### ESP32 con botón:<br>
``` 
const int led=33;
const int btn=34;
void setup() {
  Serial.begin(115200);
  pinMode(led,OUTPUT);
  pinMode(btn,INPUT);
}

void loop() {
  int estado = digitalRead(btn);
  if(estado == 1 ){
    digitalWrite(led,1);
  }
  else{
    digitalWrite(led,0);
  }
}
``` 

[ESP32 con botón](https://github.com/user-attachments/assets/2fb48297-2228-4e13-8b8e-80b5b23b4017)

### ESP32 Bluetooth:<br>
``` 
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
const int led=33;
void setup() {
  pinMode(led,OUTPUT);
    Serial.begin(115200);
    SerialBT.begin("AbrahamEsp32"); // Nombre del dispositivo Bluetooth
}

void loop() {
    if (SerialBT.available()) {
        String mensaje = SerialBT.readString();
        Serial.println("Recibido: " + mensaje);
        if(mensaje == "on" ){
    digitalWrite(led,1);
  }
  else{
    digitalWrite(led,0);
    }}
    delay(100);
}
``` 
[ESP32 Bluetooth](https://github.com/user-attachments/assets/15d742b1-f1fd-4519-a911-5364dd9c4c94)

## Práctica 3

En esta practica observaremos a base de un ESP32 combinado a un puente H, sumado de un motor el cambio de dirección y su acelereación y desaceleración.

### Aceleración:<br>
``` 
#define in1 25
#define in2 26
int var=20;
 <br>
void setup() {
 <br>
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  ledcAttachChannel(3, 1000, 8 , 0);
  Serial.begin(115200);
 
}
 
void loop() {
  Serial.println(var);
  ledcWrite(18, var);
  digitalWrite(in1,1);
  digitalWrite(in2,0);
  delay(1000);
  var=var+20;
  if(var>255){
     var=var-80;
  }  
  delay(1000);
}
``` 

![Diagrama del sistema](recursos/imgs/FotoESP32motoraceleracion.jpg) 


### Cambio de dirección:<br>
``` 
#define in1 25
#define in2 26

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

void loop() {

    digitalWrite(in1, 1); 
    digitalWrite(in2, 0); 
    delay(3000);
    digitalWrite(in1, 0); 
    digitalWrite(in2, 0); 
    delay(1000);
    digitalWrite(in1, 0); 
    digitalWrite(in2, 1); 
    delay(1000); 
  }
``` 
  
  
[Cambio de dirección](https://github.com/user-attachments/assets/4e1e3c5b-193a-47d9-be2c-e228a90f1efe)

### Esp32 Bluetooth:<br>

```
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
const int led=33;
void setup() {
  pinMode(led,OUTPUT);
    Serial.begin(115200);
    SerialBT.begin("AbrahamEsp32"); // Nombre del dispositivo Bluetooth
}

void loop() {
    if (SerialBT.available()) {
        String mensaje = SerialBT.readString();
        Serial.println("Recibido: " + mensaje);
        if(mensaje == "on" ){
    digitalWrite(led,1);
  }
  else{
    digitalWrite(led,0);
    }}
    delay(100);
}
```
[ESP32 Bluetooth](https://github.com/user-attachments/assets/a7df4006-0df3-4002-8bfc-e393138ed8cc)





https://github.com/user-attachments/assets/81669f9e-aa6b-495b-ab91-c8a8a92758bc

