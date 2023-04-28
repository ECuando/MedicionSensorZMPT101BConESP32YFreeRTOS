/*     
 *     Código para utilizar el medidor de voltaje ZMPT101B con el microcontrolador ESP32 y 
 *     el sistema FreeRTOS mostrando la lectura de voltaje en el puerto serie 
 *     actualizandola cada 5 segundos. 
 *     
 *     Se calcula la desviación estándar:
 *      
 *     1. Se calcula la media de las lecturas tomadas del sensor.
 *     2. Luego, se calcula la varianza de las lecturas. La varianza es la media de los cuadrados de las desviaciones de cada dato respecto a la media.
 *     3. Finalmente, se obtiene la desviación estándar, que es la raíz cuadrada de la varianza.
 * 
 *     Una vez que se tiene la desviación estándar, se utiliza la siguiente fórmula para calcular el voltaje:
 *     voltaje = intercepción de recta + pendiente * desviación estándar
 *     
 *     Donde la intercepción de recta y la pendiente son parámetros que 
 *     deben ser ajustados mediante calibración previa del sensor. 
 *     En este código, se asume que estos valores ya han sido ajustados 
 *     previamente mediante pruebas de calibración. Luego, el resultado 
 *     de esta ecuación se multiplica por un factor de calibración 
 *     específico para el sensor utilizado.
 */

//Bibliotecas de FreeRTOS y comunicación serial
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Definición de constantes
const int PinADCSensor1 = 25; //Pin ADC1_8 del ESP32.
const float FrecuenciaDeRed = 60.0; //Frecuencia de la señal de la red de energía pública.
const float LongitudDeOndaPromedio = 40.0; // Promedio de la señal
const float IntercepcionDeRecta = -0.04; 
const float Pendiente = 0.0405;
const unsigned long Periodo = 1000;

//Se declaran algunas variables globales que se utilizarán en la tarea.
float Volts;
float SumatoriaLecturas;
float SumatoriaLecturasAlCuadrado;
int NumeroDeLecturas;
unsigned long TiempoAnterior;

// Prototipo de la tarea
void LecturaSensor(void* Parametro);

//Inicializa la comunicación serial
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

// Se y crea la tarea "LecturaSensor" con una prioridad de 1 y un tamaño de pila de 2048 bytes.
  xTaskCreate(LecturaSensor, "LecturaSensor", 2048, NULL, 1, NULL);
}

// El bucle principal no hace nada en este código.
void loop() {
}

//  Esta parte del código define la tarea principal del programa que se encarga de leer las 
//  señales del sensor conectado al pin analógico del microcontrolador.
void LecturaSensor(void* Parametro) {
  pinMode(PinADCSensor1, INPUT); //Se establece el Pin ADC como entrada.

//Bucle infinito para leer la señal analógica del sensor.
  while (true) {
    int ValorDeSensor = analogRead(PinADCSensor1);
    SumatoriaLecturas += ValorDeSensor;
    SumatoriaLecturasAlCuadrado += pow(ValorDeSensor, 2);
    NumeroDeLecturas++;

    if ((unsigned long)(millis() - TiempoAnterior) >= Periodo) {
      float Media = SumatoriaLecturas / NumeroDeLecturas;
      float Varianza = (SumatoriaLecturasAlCuadrado / NumeroDeLecturas) - pow(Media, 2);
      float DesviacionEstandar = sqrt(Varianza);

      Volts = IntercepcionDeRecta + Pendiente * DesviacionEstandar;
      Volts *= 40.3231;

      Serial.print("\tVoltage: ");
      Serial.print(Volts);
      Serial.println(" V AC"); // Muestra el mensaje "Voltage" en el monitor serial

      SumatoriaLecturas = 0.0;
      SumatoriaLecturasAlCuadrado = 0.0;
      NumeroDeLecturas = 0;
      TiempoAnterior = millis();
    }

    vTaskDelay(1);
  }
}
