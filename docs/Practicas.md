*Práctica 1*

Esta practiica podemos observar el uso de un circuito integrado 555 de manera astable en la cual, hace parpadear un diodo led dependiendo del valor de las resistencias.


[Practica 1][doc-ref]

[doc-ref]: https://github.com/user-attachments/assets/a55c972d-851a-4645-9918-f97dc848011a "Practica 1"


*Práctica 2*

Esta practica consistio en usar un ESP32 para controlar de distantas maneras un led sea desde solo el ESP32, como con este y un botón y por medio de bluetooth.


ESP32 solo:
1  const int led=33<br>
2<br>
3<br>
4  void setup () {<br>
5  Serial. begin (115200);<br>
6  pinMode (Led, OUTPUT) ;<br>
7  }<br>
8<br>
9<br>
10 void lo0p() {<br>
11 digitalWrite(led,1);<br>
12 delay (1000);<br>
13 digitalwrite(led,0);<br>
14 delay (1000);<br>
15 }<br>

https://github.com/user-attachments/assets/6e66b9b6-49fc-4279-bbd0-478fd867dd4a 

