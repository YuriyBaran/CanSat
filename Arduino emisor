//----------LIBRERÍAS----------
//Librería para la comunicación protocolo SPI
#include <SPI.h>
//Librería para la transmisión con módulo radio
#include <RH_RF69.h>
//Librería del protocolo/controlador de generación/comunicación de paquetes. Funciones de espera, acks...direciones emisor, receptor...
#include <RHReliableDatagram.h>
//Librería del protocolo I2C comunicación dispositivos
#include <Wire.h> 
//Librería sensor de presión y temperatura
#include <Adafruit_BMP280.h>



/************ Configuración de la Radio  ***************/
//FRECUENCIA
//Constante que define la frecuencia de transmisión y recepción. 
//Dar un nombre a una variable constante; se sustituyen en la compilación, no ocupan memoria en el procesador.
#define RF69_FREQ 868.0//Frecuencia expresada en MHz

//IDENTIFICADORES
//Cada dispositivo conectado deberá tener una dirección única.
//Constante que define el identificador del receptor/destino: la estación base.
#define DEST_ADDRESS   1 //Identificador de la estación base=1
//Constante que define el identificador del emisor/origen: el cansat.
#define MY_ADDRESS     2 //Identificador del cansat=2

//El cansat, dirección 2, manda información a la estación base, dirección 1.


//DEFINICIÓN DE CONSTANTES
//Las constantes: No gastan memoria y ahorran tiempo de ejecución.
//Constantes que definen los pines donde están conectadas nuestras placas, definidos por el fabricante. 
//Pines para la comunicación de la placa de radio con la placa Arduino.
#define RFM69_INT     2  
#define RFM69_RST     3 
#define RFM69_CS      10 
#define LED           13 

//CREACIÓN OBJETO RADIO: Parámetros el CS y el INT, pines 10 y 2 respectivamente. "MI CANAL DE RADIO"
RH_RF69 rf69(RFM69_CS, RFM69_INT);

//CONTROL DE MENSAJES, entrantes y salientes.
//El objeto radio GESTOR MENSAJES TRANSMISOR-CANSAT sólo acepta mensajes/acuses de recibo "ACK", que van dirigidos a él (dirección 2).
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);// Datagramas al objeto "MI RADIO" y dirección 2

//VARIABLE ENTERA que va a guardar el número de paquetes/mensajes recibidos.
//Entero: número de paquete
int paquete = 0;



/************ BMP280 Setup ***************/
//OBJETO I2C: sensor
Adafruit_BMP280 bmp; // Creación de un objeto I2C denominado bmp

//VARIABLES SENSOR TEMPERATURA-PRESIÓN
//Real: números reales que representan el valor de las magnitudes físicas
double temperatura;
double presion;
double altitud;


/************ variables millies ***************/
const int parpadeo = 1000;
const int sonido = 500;
int tiempo1 = 0;
int tiempo2 = 0;
int tiempo3 = 0;
int tiempo4 = 0;
int tiempo5 = 0;
int tiempo6 = 0;
int miLED = 4;
int ZUM = 6;
int contadorL = 0;
int contadorZ = 0;

// Se ejecuta 1 VEZ cuando el Arduino se enciende. INICIALIZACIONES Y MENSAJES DE ERROR
void setup() {

  //CONFIG PINES LED Y ZUMBADOR
  pinMode(miLED,OUTPUT);
  pinMode(ZUM,OUTPUT);
  
  //INICIALIZACIÓN PUERTO SERIE. Para la recepción de datos (acks).
  Serial.begin(9600);

  //INICIALIZACIÓN PINES
  pinMode(LED, OUTPUT); // Pin 13 es de salida.
  pinMode(RFM69_RST, OUTPUT);// Pin 3 es de salida.
  digitalWrite(RFM69_RST, LOW);// Escribir 0 en el pin 3.


  //RESETEO MANUAL INICIAL
  digitalWrite(RFM69_RST, HIGH); //Pin 13 a "1" durante 10ms
  delay(10);
  digitalWrite(RFM69_RST, LOW); //Pin 13 a "0" durante 10ms
  delay(10);
  
  //ESCRIBIR en el monitor serie:
  Serial.println("TRANSMISIÓN DESDE EL SATÉLITE >>-TECHNO-ARROW->");
  
  //INICIALIZACIÓN DE "MI RADIO"
  //MENSAJES ARRANQUE
  if (!rf69_manager.init()) //Si "MI RADIO-paquetes" no se inicia/arranca: mensaje de error.
  {
    Serial.println("Fallo inicialización Radio >>-TECHNO-ARROW->");//Arranca mal
    while (1);//¿?¿?¿?¿?¿?¿?¿?¿
  }
  Serial.println("RADIO >>-TECHNO-ARROW->: ¡TODO BIEN, todo correcto y yo que me alegro!"); //Si la radio se inicia/arranca correctamente: mensaje ok



  //MENSAJES ASIGNACIÓN FRECUENCIA
  if (!rf69.setFrequency(RF69_FREQ)) //Si no se asigna correctamente la frecuencia a "MI RADIO": mensaje de error.
  {
    Serial.println("La asignación de frecuencia ha fallado");
  }//Si se asigna correctamente: No hace nada, continua el programa.

  
  //Asignar la POTENCIA de 20dBm al emisor radio. Poner true para correcto funcionamiento.
  rf69.setTxPower(20,true);//Rango de 14-20.
  

  //Asignar la CLAVE DE CIFRADO: debe ser la misma en el emisor y el receptor.
  //Definición de la clave.
  uint8_t key[] = { 0x37, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
                  };
  //Asignación de la clave a "MI RADIO"
  rf69.setEncryptionKey(key);


  //Salida por el puerto serie de la información: Nombre radio y frecuencia de la radio.
  Serial.print("TECHNO-ARROW transmitiendo en la banda de: ");
  Serial.print((int)RF69_FREQ);//Saca por el puerto serie la constante RF69_FREQ en formato NÚMERO ENTERO
  Serial.println(" MHz");

  //INICIALIZACIÓN SENSORES BMP 280
  //Si el sensor de temperatura-presión no arranca correctamente, no responde en la dirección hexadecimal 76. Mensaje de error.
  if (!bmp.begin(0x76))//Tenemos que indicar la dirección de nuestro barómetro(que usa protocolo I2C 0x76)
  {
    Serial.println(F("No se encuentra el sensor BMP280 sensor. ¡Revisa el cableado!"));
    while (1); //¿?¿?¿?¿?¿?¿?¿?¿
  }

  
  /* Valores por defecto del sensor */
  //Configuraciones del sensor para la función de transferencia.
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Modo de operación. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Compensación de temperatura */
                  Adafruit_BMP280::SAMPLING_X16,    /* Compensación de presión */
                  Adafruit_BMP280::FILTER_X16,      /* Filtrado. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Tiempo de espera para sus cálculos internos. */
}//FIN SETUP


//VARIABLES VARIAS Tamaño del mensaje 

  #define RH_RF69_MAX_ENCRYPTABLE_PAYLOAD_LEN 64 //Tamaño máximo mensaje codificable=64
  #define RH_RF69_HEADER_LEN 4 //Tamaño de la cabecera=4
  //tamaño del mensaje = Tamaño máximo del mensaje codificable menos tamaño de la cabecera = 64-4=60
  #define RH_RF69_MAX_MESSAGE_LEN (RH_RF69_MAX_ENCRYPTABLE_PAYLOAD_LEN - RH_RF69_HEADER_LEN)



uint8_t buf[RH_RF69_MAX_MESSAGE_LEN]; //Crea un array de tamaño 60 Bytes (uint8= enteros sin signo tamaño 8bits)


//Se ejecuta en BUCLE de manera INDEFINIDA
void loop()
{
  //PITIDO Y LED
  tiempo2 = millis();
  if (tiempo2 > (tiempo1 + parpadeo)) {
    if (contadorL == 0){
      digitalWrite(miLED,HIGH);
      tiempo1 = millis();
      contadorL = 1;
      }
    else if (contadorL == 1){
      digitalWrite(miLED,LOW);
      tiempo1 = millis();
      contadorL = 0;
      }
  }
  tiempo4 = millis();
  if (tiempo4 > (tiempo3 + sonido)) {
    if (contadorZ == 0){
      digitalWrite(ZUM,HIGH);
      tiempo3 = millis();
      contadorZ = 1;
      }
    else if (contadorZ == 1){
      digitalWrite(ZUM,LOW);
      tiempo3 = millis();
      contadorZ = 0;
      }
    }
    
  tiempo6 = millis();
  if (tiempo6 > (tiempo5 + parpadeo)) {
    //LECTURA MAGNITUDES FÍSICAS
    temperatura = bmp.readTemperature(); //Lectura del valor de temperatura con la función de la librería adafruit
    presion = bmp.readPressure(); //Lectura del valor de prsión con la función de la librería adafruit
    altitud = bmp.readAltitude(1013.25);//Cálculo del valor de altitud con la función de la librería adafruit
    
    //CREACIÓN OBJETO MENSAJE "s" CON LA INFORMACIÓN DE LOS SENSORES
    String s = ""; //Variable donde se va a guardar el mensaje a enviar
    
    
    //CONSTRUCCIÓN DEL MENSAJE: 
    //Construimos un mensaje de texto adjuntando los valores de las magnitudes físicar guardados en las variables
    //Se guardan como strings, aunque sean doubles/reales.
    //Cadena vacía+número paquete+;+valortemperatura+;+valorpresión+;+valoraltitud;+soylaNoe
    s = s + paquete + ";" + temperatura + ";" + presion + ";" + altitud + ";PYTHON";
    
    //char=caracter ASCCI = letra
    //radiopacket= Array de letras.
    //Crea un array de letras de tamaño 20.
    //EL MENSAJE CON FORMATO PARA EL GESTOR DE MENSAJES
    char radiopacket[20];
    
    //CONVERSIÓN de la CADENA "s" en un ARRAY de letras.
    //Metemos el mensaje/paquete (la cadena de texto) en un vector (letra a letra).
    //Ejemplo: la cadena de texto "hola" la transforma en un array donde en cada posición del array/vector hay una letra.
    //hola --> [´h´, ´o´, ´l´, ´a´,\0´]
    //Añade al final \0 = el \0 indica que se ha acabado el texto.
    //La función sprintf retorna el número de caracteres escritos al array, sin contar el carácter nulo al final.
    //  sprintf(array, "texto con %variable1 %variable2...", variable1, variable2); Sustituye en %variable1 el contenido
     
     
     //INFORMACIÓN EMPAQUETADA PARA EL GESTOR DE MENSAJES: en radiopacket
    sprintf(radiopacket, "%s", s.c_str()); //Mete en el array radiopacket la cadena "%s" = el contenido de s
    
    
    // Serial.print(">>-TECHNO-ARROW-> Enviando a estación base paquete Nº ");
    //Saca por el puerto serie el contenido del paquete.
    Serial.print(radiopacket);
    
    //ENVÍO DEL MENSAJE AL DESTINO: ESTACIÓN BASE = 1 
    //El CONTROLADOR DE PAQUETES manda el PAQUETE a la estación base = 1. Si consigue enviarlo:
    if (rf69_manager.sendtoWait((uint8_t *)radiopacket, strlen(radiopacket), DEST_ADDRESS))
    {
     
      uint8_t len = sizeof(buf);//60
      uint8_t from;
     //El CONTROLADOR DE PAQUETES espera un acuse de recibo de la estación base, con tiempo limitado (2000ms).
      if (rf69_manager.recvfromAckTimeout(buf, &len, 2000, &from)) //Espera como mucho 2 segundos para ver que recibo el ack
      {
        buf[len] = 0; // zero out remaining string ¿?¿?¿?¿? inicializa a cero todo ¿?¿?¿
        Serial.println("ACK: Confirmación de paquete recibido en la ESTACIÓN BASE"); //Si recibe el acuse de recibo indica que todo Ok.
      } 
      else 
      {
        Serial.println("No hay respuesta de la estación base >>-TECHNO-ARROW->, ¿hay alguien ahí?"); //Si no recibe el acuse de recibo: mensaje.
      }
    } 
    else 
    {
      Serial.println(" Fallo en el envío del paquete"); //Si no consigue enviar el paquete.
    }
    paquete++;
    tiempo5 = millis();
  }
}
