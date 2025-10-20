*Práctica 1*

Esta practiica podemos observar el uso de un circuito integrado 555 de manera astable en la cual, hace parpadear un diodo led dependiendo del valor de las resistencias.


[Practica 1][doc-ref]

[doc-ref]: https://github.com/user-attachments/assets/a55c972d-851a-4645-9918-f97dc848011a "Practica 1"


*Práctica 2*

Esta practica consistio en usar un ESP32 para controlar de distantas maneras un led sea desde solo el ESP32, como con este y un botón y por medio de bluetooth.


ESP32:
const int led=33;


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

