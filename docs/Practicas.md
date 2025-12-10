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

#  Proyecto Final: Carro Robot de Fútbol Controlado por PS4 (ESP32)

##  Objetivo del Proyecto

El proyecto consistió en el diseño y construcción de un *carro robot teledirigido* utilizando el microcontrolador *ESP32* y un *control PS4* (vía Bluetooth). El objetivo principal era crear una plataforma móvil con alta maniobrabilidad para participar y competir exitosamente en un torneo de fútbol de robots, moviendo una pelota y marcando goles.

---

##  Arquitectura y Tecnologías

La implementación se centró en la integración de tres áreas principales:

### Marco Teórico
* *Microcontrolador:* Se seleccionó el *ESP32* por su capacidad de doble núcleo y, fundamentalmente, por su *conectividad Bluetooth* para recibir comandos en tiempo real desde el control de PlayStation.
* *Driver de Motor:* Se utilizó un *Puente H (L298N o similar)* para gestionar la alta corriente requerida por los motores DC, ya que el ESP32 no puede alimentarlos directamente.
* *Movimiento:* Se implementó *Tracción Diferencial* mediante el envío de señales PWM (Modulación por Ancho de Pulso) a cada motor, permitiendo movimientos precisos (avance, retroceso y giros).

###  Materiales Clave
| Componente | Función Principal |
| :--- | :--- |
| *ESP32 DevKit V1* | Cerebro del sistema, gestor de Bluetooth. |
| *Puente H* | Driver de potencia para los motores. |
| *Control PS4* | Interfaz de usuario para comandos inalámbricos. |
| Pilas 3.7V / 2600 mAh | Fuente de alimentación. |
| Motores DC | Actuadores de tracción. |
| MDF / Impresión 3D | Construcción del chasis, carcasas y pala. |

---

##  Procedimiento General

El proyecto se ejecutó mediante la colaboración de equipos especializados:

1.  *Electrónica:* Diseño del diagrama del circuito, conexión del driver de motor, fusibles, baterías y cableado general.
2.  *Programación:* Desarrollo del firmware para la conexión Bluetooth y la lógica de control de motores, traduciendo los comandos del joystick a señales de movimiento.
3.  *Mecánica:* Diseño y ensamblaje del chasis de cuatro ruedas y la pala frontal para la interacción con la pelota.

---

##  Código de Programación (Arduino para ESP32)

Este firmware gestiona la conexión con el control PS4 y utiliza la lógica de tracción diferencial y ajuste de velocidad (con el gatillo R2) para el control del carro.

```cpp
/**
 * @file Robot_Futbol_PS4_ESP32.ino
 * @brief Código para controlar un carro robot de fútbol usando un ESP32 y un control PS4.
 * Incluye funciones para avance, retroceso, giros y tracción diferencial mediante Joysticks.
 */

// Bibliotecas necesarias:
#include <Arduino.h>
#include <PS4Controller.h> 

// --- Configuración de Pines y Variables ---
int enA = 25; int enB = 14; // Pines de Enable (PWM)
int IN1 = 26; int IN2 = 27; int IN3 = 32; int IN4 = 33; // Pines de Dirección
#define R 0 // Canal LEDC para Motor Derecho
#define L 1 // Canal LEDC para Motor Izquierdo
int Speed = 210; // Velocidad base inicial
int threshold = 10; // Umbral de sensibilidad para Joysticks

// --- Declaración de Funciones de Movimiento ---
void forward(); void backward(); void left(); void right(); void stop();
void setMotor(int leftMotor, int rightMotor);

// --- Setup (Configuración Inicial) ---
void setup() {
  Serial.begin(115200);
  // *IMPORTANTE*: Reemplace la MAC Address con la de su control PS4.
  PS4.begin("98:3b:8f:fc:0c:82"); 
  Serial.println("Esperando control PS4...");
  ledcAttachChannel(enA, 5000, 8, R);
  ledcAttachChannel(enB, 5000, 8, L);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stop();
}

// --- Loop Principal (Ejecución Continua) ---
void loop() {
  if (PS4.isConnected()) {
    // 1. Ajuste de Velocidad con R2
    Speed = map(PS4.R2Value(), 0, 255, 210, 255);


