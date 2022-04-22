//----------LIBRERÍAS----------
//Librería para la comunicación protocolo SPI
#include <SPI.h>
//Librería para la transmisión con módulo radio
#include <RH_RF69.h>
//Librería del protocolo/controlador de generación/comunicación de paquetes. Funciones de espera, acks...direciones emisor, receptor...
#include <RHReliableDatagram.h>

/************ Configuración de la radio ***************/
//FRECUENCIA
//Constante que define la frecuencia de transmisión y recepción. 
//Dar un nombre a una variable constante; se sustituyen en la compilación, no ocupan memoria en el procesador.
#define RF69_FREQ 868.0 //Frecuencia expresada en MHz


//IDENTIFICADORES (Número del 1 al 100)
//Cada dispositivo conectado deberá tener una dirección única.
//Constante que define el identificador del receptor/destino: la estación base.
#define MY_ADDRESS     1 //Identificador de la estación base=1
//En el receptor no declaramos la constante de definición de la dirección del CANSAT puesto que no vamos a enviarle mensajes.
//Se le enviarán mensajes al CANSAT de confirmación, mensajes ACK, pero cogiendo la dirección de los paquetes que el CANSAT envío.


//DEFINICIÓN DE PINES
//Constantes que definen los pines donde están conectadas nuestras placas, definidos por el fabricante. 
//Pines para la comunicación de la placa de radio con la placa Arduino UNO.
//Depende de las placas que estemos usando.
#define RFM69_INT     2  
#define RFM69_RST     3 
#define RFM69_CS      10 
//#define LED           13 

//CREACIÓN OBJETO RADIO: Parámetros el CS y el INT, pines 10 y 2 respectivamente. "MI CANAL DE RADIO"
RH_RF69 rf69(RFM69_CS, RFM69_INT);

//CREACIÓN DEL OBJETO GESTOR de los datagramas/paquetes.
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);// Datagramas al objeto "MI RADIO" y dirección 1=ESTACIÓN BASE




// Se ejecuta 1 VEZ cuando el Arduino se enciende. INICIALIZACIONES Y MENSAJES DE ERROR
void setup()
{
  //INICIALIZACIÓN PUERTO SERIE. Para la recepción de datos.
  Serial.begin(9600);
  
  //INICIALIZACIÓN PINES
  //pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);// Pin 3 es de salida.
  digitalWrite(RFM69_RST, LOW);// Escribir 0 en el pin 3.

  //RESETEO MANUAL INICIAL
  digitalWrite(RFM69_RST, HIGH);//Pin 13 a "1" durante 10ms
  delay(10);
  digitalWrite(RFM69_RST, LOW);//Pin 13 a "0" durante 10ms
  delay(10);


  //ESCRIBIR en el monitor serie:
  Serial.println("RECEPCIÓN EN LA ESTACIÓN BASE >>-TECHNO-ARROW->");
  Serial.println();


  //INICIALIZACIÓN DE "MI RADIO"
  //MENSAJES ARRANQUE
  if (!rf69_manager.init()) { //Si mi GESTOR de PAQUETES no se inicia/arranca: mensaje de error.
    Serial.println("Fallo inicialización Radio ESTACIÓN BASE >>-TECHNO-ARROW->"); //Arranca mal
    while (1);
  }
  Serial.println("RADIO ESTACIÓN BASE >>-TECHNO-ARROW->: ¡TODO BIEN, todo correcto y yo que me alegro!"); //Si el GESTOR de PAQUETES inicia/arranca correctamente: mensaje ok
  if (!rf69.setFrequency(RF69_FREQ))
  {
    Serial.println("La asignación de frecuencia en LA ESTACIÓN BASE ha fallado"); //Si no se asigna correctamente la frecuencia a "MI RADIO": mensaje de error.
  }//Si se asigna correctamente: No hace nada, continua el programa.


  //Asignar la POTENCIA de 20dBm al emisor radio. Poner true para correcto funcionamiento.
  rf69.setTxPower(20,true);//Rango de 14-20. Potencia de transmisión.
  //Con la poetencia máxima 20dBm y antena pequeña, no se va a recibir bien la señal.

  //Asignar la CLAVE DE CIFRADO: debe ser la misma en el emisor y el receptor.
  //Definición de la clave.
  uint8_t key[] = { 0x37, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
                  };
  //Asignación de la clave a "MI RADIO"
  rf69.setEncryptionKey(key);

  //Salida por el puerto serie de la información: Nombre radio y frecuencia de la radio.
  Serial.print("TECHNO-ARROW ESTACIÓN BASE transmitiendo en la banda de: ");
  Serial.print((int)RF69_FREQ);//Saca por el puerto serie la constante RF69_FREQ en formato NÚMERO ENTERO
  Serial.println(" MHz");
}//FIN SETUP


//VARIABLES VARIAS Tamaño del mensaje 
uint8_t data[] = "Ha llegado tu mensaje a la estación base TECHNO-ARROW";//Variable array para el envío del ACK al CANSAT. Mensaje ACK.
// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];//Variable array para guardar el mensaje recibido del CANSAT.

//¿?¿?¿? No está definina la constante en el programa del receptor
//buf es un buffer/vector/arrray de enteros sin signo de 8 bits.
//Serial.print((int)RH_RF69_MAX_MESSAGE_LEN);

//Se ejecuta en BUCLE de manera INDEFINIDA
void loop() 
{
  //Esperamos a recibir un mensaje. ¿Hay algo para mí? 
  
  if (rf69_manager.available())//Si el GESTOR de PAQUETES ha recibido paquetes:
  {
  //Si se ha recibido algún mensaje. Lo fragmentamos para extraer las partes.
  //VARIABLES: TAMAÑO DEL MENSAJE REDIBIDO Y DIRECCIÓN DEL EMISOR DEL MENSAJE  
    uint8_t len = sizeof(buf); //Se guarda en la variable "len" el tamaño del paquete. 
    uint8_t from; //Se crea la variable "from" el origen de los paquetes.
  //Podríamos tener varios satélites, o compartir estación base.
  
  
  //Al GESTOR de PAQUETES le aplico la función recvfromAck, 
  // pasando por parámetros el array donde se va a guardar el mensaje, 
  // la variable donde se va a guardar la longitud,
  // la variable donde se va a guardar la dirección del emisor.
  // Esta función va a extraer la información guardada en el paquete que proviene de la estación base.
    if (rf69_manager.recvfromAck(buf, &len, &from)) 
    {
      buf[len] = 0; // Esto se llama cero terminador, necesario a nivel interno. 
    // Buf es un vector de enteros sin signo y de 8 bits.
      // EJEMPLO vector[enteros]={´1´,`2`, `s`,`m`,`s`,/0 }

  //INFORMACIÓN RECIBIDA en el monitor serie
    //Una vez extraída toda la información se la imprimos al usuario por el monitor serie.
      //Serial.print("Emisor del paquete: CANSAT número #"); 
      //Serial.print(from);//Dirección del CANSAT
      //Serial.print(" [RSSI :");//Potencia de la señal recibida en dBm (con respecto a 1mW); 0RSSI = 0dBm = 1mW
      //Serial.print(rf69.lastRssi());//Función que recoge la potencia recibida del objeto radio.
      //Serial.print("dBm] : ");
      //Serial.print("Recibiendo del CANSAT paquete Nº: ");
      Serial.println((char*)buf);//Impresión de la información guardada en "buf", que es el contenido del paquete, es decir, el mensaje.
      
  //ENVÍO DEL ACK del paquete recibido.  
  //Para la generación del paquete ACK se le pasa por parámetros: 
  // El array con el mensaje; el tamaño del array con el mensaje; la dirección de destino.
      if (!rf69_manager.sendtoWait(data, sizeof(data), from))//El GESTOR de PAQUETES intenta enviar un paquete ACK al CANSAT 
      {
        Serial.println("Error al enviar el ACK al CANSAT TECHNO-ARROW");//Si no se puede mandar el paquete ACK, mensaje de error.
      }
    }
  }
}
