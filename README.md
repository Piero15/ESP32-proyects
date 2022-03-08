# ROBERT_ESP_32_CAM

el proyecto principal le pertenece a alfajor144 yo hicen un fork a su proyecto para poder trabajar con el mejorarlo y modificarlo, en el proyecto le agrege nuevas
funciones y un nuevo modulo


**nuevos gadgets añadidos:**
```javascript
-pantalla Oled 128x32 I2C
-se agrego una tira de leds blancos para mejorar la vision de la camara por las noches controlado por un mosfet
-se agrego un pulsador para llamar a otras funciones adicionales
```

**mientras que sus nuevas funciones son:**
```javascript
-programacion via OTA (se puede programar el robot de forma inalambrica mediante la direccion IP que aparecera en la pantalla Oled)
-el robot al iniciar podra conectarse a cualquier red wifi que decida el anfitrion gracias a la libreria WifiManager (ojo que tendras q' tener la contraseña de la red para conectar al robot a la misma) ya no sera necesario colocar tu red en el codigo para que se conecte, ahora podras conectar el robot a una red directamente desde el movil
-Otra funcion muy util es que al momento de encender el robot si el pulsador esta precionado, ya no tratara de conectarse a alguna red local por WifiManager, si
 no directamente creara un red propia con el mismo nombre del robot, al conectarte y entrar a la IP que se muestra en el OLED podras controlarlo estes donde estes sin
 necesidad de una red intermediaria
```
