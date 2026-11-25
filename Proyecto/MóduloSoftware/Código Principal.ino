#include <Arduino.h>
#include <NimBLEDevice.h>
//CONFIGURACIÓN

#define SERVICE_UUID "a1000000-1234-5678-1234-56789abcdef0"
#define TELEMETRY_UUID "a1000001-1234-5678-1234-56789abcdef0"
#define COMMAND_UUID   "a1000002-1234-5678-1234-56789abcdef0"
#define STATUS_UUID    "a1000003-1234-5678-1234-56789abcdef0"

const int PIN_EMG = 27;
const int PIN_LM[2] = {13,14};
const int PIN_MVC[3] = {34, 22, 21};
const int PIN_MVF[3] = {33, 32, 35};
const int PIN_FSR[5] = {34, 35, 32, 33, 25};
const int PIN_MVI[2] = {22, 21};
const int PIN_IFC = 26;
const int PIN_IFE = 25;
const int PIN_HEN = 23;
const int PIN_HLV = 19;
const int PIN_HRV = 18;
const int PIN_LED = 2;

const uint32_t EMG_PERIOD_US = 2000; // 500 Hz, objetivo
const uint32_t MIN_INTERVAL_MS = 100;
const uint32_t MAX_INTERVAL_MS = 2000;

const unsigned long TIEMPO_CALIBRACION = 4000;
const unsigned long INTERVALO_MUESTREO = 5;
unsigned long tiempoAnterior = 0;

uint32_t intervaloBLE = 200;
uint32_t ultimoEnvio = 0;
uint32_t ultimaCalemg = 0;

int emgMin = 0;
int emgMax = 4095;
int estadoLED = LOW;

// Estados operativos
bool emgCalibraOrd = false;
bool emgActivo = false;
bool envioDatos = true;
bool flexion = true;
const char Modo = 'L'; //'M' = matriz, 'S' = directo, 'L' = simulado

// Comando pendiente desde callback BLE
portMUX_TYPE comandoMux = portMUX_INITIALIZER_UNLOCKED;
char comandoPendiente[24] = "";
bool hayComando = false;

// Características
NimBLECharacteristic* chrTelemetria = nullptr;
NimBLECharacteristic* chrEstado = nullptr;
NimBLEServer* bleServer = nullptr;

//LECTURA DE SENSORES

//Lectura Temperatura
float leerTemperaturaC(int Pin) {
  unsigned long Tiempo_Muestreo = 10;
  return (obtenerPromedioSensor(Tiempo_Muestreo, PIN_LM[Pin])/10.0);
}

//Lectura FSR
uint16_t leerPresionADC(int Pin_col, int Pin_fill) {
  unsigned long Tiempo_Muestreo = 10;
  if (Modo == 'M') {
    static int columnaActiva = -1;
    if (Pin_col != columnaActiva) {
      for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_MVC[i], LOW);
      }
      digitalWrite(PIN_MVC[Pin_col], HIGH);
      delay(5);
    }
    digitalWrite(PIN_MVC[Pin_col], HIGH);
    return obtenerPromedioSensor(Tiempo_Muestreo, PIN_MVF[Pin_fill]);
  }else if (Modo == 'S') {
    if ((Pin_col == 0 && (Pin_fill == 0 || Pin_fill == 2)) || (Pin_col == 1 && Pin_fill == 1) || (Pin_col == 2 && (Pin_fill == 0 || Pin_fill == 2))) {
    int pin = (Pin_col * 3 + Pin_fill) / 2;
      return obtenerPromedioSensor(Tiempo_Muestreo, PIN_FSR[pin]);
    }
  }else if (Modo == 'L') {
    long numero_az=random(100,300);
    return numero_az;
  }
  return 0;
}

//Filtrado de lecturas
int obtenerPromedioSensor(unsigned long duracion, int PIN) {
  unsigned long inicio = millis();
  uint32_t suma = 0;
  uint32_t muestras = 0;
  while (millis() - inicio < duracion) {
    int lectura = analogReadMilliVolts(PIN);
    suma += lectura;
    muestras++;
  }
  return suma / muestras;
}

int obtenerPromedio(unsigned long duracion) {
  unsigned long inicio = millis();
  uint32_t suma = 0;
  uint32_t muestras = 0;
  while (millis() - inicio < duracion) {
    int lectura = analogRead(PIN_EMG);
    suma += lectura;
    muestras++;
    parpadeoLed(500);
    delay(INTERVALO_MUESTREO);
  }
  return suma / muestras;
}

int obtenerMaximo(unsigned long duracion) {
  unsigned long inicio = millis();
  int maximo = 0;
  while (millis() - inicio < duracion) {
    int lectura = analogRead(PIN_EMG);
    if (lectura > maximo) {
      maximo = lectura;
    }
    delay(INTERVALO_MUESTREO);
  }

  return maximo;
}

//Confirmacion de carrera
void confirmacion (unsigned long inicio) {
  unsigned long actual;
  if(flexion || digitalRead(PIN_IFC) == HIGH) {
    do {
      //Espera bro, todo ta chill
      actual = millis();
    } while (digitalRead(PIN_IFE) != HIGH && (actual-inicio) < 5000);
    analogWrite(PIN_HRV, 0);
    analogWrite(PIN_HLV, 0);
  } else if (!flexion || digitalRead(PIN_IFE) == HIGH) {
    do {
      //espera pero pa sentarte ahora ps
      actual = millis();
    } while (digitalRead(PIN_IFC) != HIGH && (actual-inicio) < 5000);
    analogWrite(PIN_HLV, 0);
    analogWrite(PIN_HRV, 0);
  }
}

void parpadeoLed (unsigned long intervalo) {
  unsigned long tiempoActual = millis();
  if (tiempoActual - tiempoAnterior >= intervalo) {
    tiempoAnterior = tiempoActual;
    if (estadoLED == LOW) {
      estadoLED = HIGH;
    } else {
      estadoLED = LOW;
    }
  digitalWrite(PIN_LED, estadoLED);
  }
}

// Calibracion local del EMG
void CalibrarEMG() {
  if (!emgCalibraOrd) return;
  unsigned long inicio = millis();
  while (millis()-inicio < 3000){
    parpadeoLed(200);
  }
  digitalWrite(PIN_LED, HIGH);
  emgMin = obtenerPromedio(TIEMPO_CALIBRACION);
  inicio = millis();
  while (millis()-inicio < 2000){
    parpadeoLed(100);
  }
  digitalWrite(PIN_LED, HIGH);
  emgMax = obtenerMaximo(TIEMPO_CALIBRACION);
  digitalWrite(PIN_LED, LOW);
  emgActivo = true;
  publicarEstado("Calibra_Termino");
  emgCalibraOrd = false;
}

bool sincronizarFlexion() {
  bool flexionado = digitalRead(PIN_IFC) == HIGH;
  bool extendido  = digitalRead(PIN_IFE) == HIGH;

  if (flexionado && extendido) return false;

  if (flexionado)      flexion = true;
  else if (extendido)  flexion = false;

  return true;
}

//Asistencia mediante actuadores
void AccionarApoyo(int Lectura) {
    unsigned long inicio;
  if (!emgActivo) return;
  int porcentaje = map(
    Lectura,
    emgMin,
    emgMax,
    0,
    100
  );
  porcentaje = constrain(porcentaje, 0, 100);

  static bool avisado = false;
  if (!sincronizarFlexion()) {
    if (!avisado) {
      publicarEstado("ERR:FINALES");
      avisado = true;
    }
    return;                     // no mover nada con sensores incoherentes
  }
  avisado = false;

  if (porcentaje > 75){
    if(flexion) {
      analogWrite(PIN_HRV, 254);
      inicio = millis();
      confirmacion(inicio);
      flexion = false;
    } else {
      analogWrite(PIN_HLV, 254);
      inicio = millis();
      confirmacion(inicio);
      flexion = true;
    }
  }
}

//COMUNICACIÓN DE ESTADO

void publicarEstado(const char* mensaje) {
  Serial.print("ESTADO: ");
  Serial.println(mensaje);

  char linea[40];
  snprintf(linea, sizeof(linea), "%s\n", mensaje);
  chrEstado->setValue(linea);

  if (bleServer->getConnectedCount() > 0) {
    chrEstado->notify();
  }
}

void publicarEstadoGeneral() {
  char mensaje[20];

  snprintf(
    mensaje,
    sizeof(mensaje),
    "S,E%d,D%d,R%lu",
    emgActivo ? 1 : 0,
    envioDatos ? 1 : 0,
    (unsigned long)intervaloBLE
  );

  publicarEstado(mensaje);
}

//PROCESAMIENTO DE COMANDOS

void ejecutarComando(char* comando) {

  size_t n = strlen(comando);
  while (n > 0 && (comando[n-1] == '\n' || comando[n-1] == '\r' || comando[n-1] == ' ')) {
    comando[--n] = '\0';
  }

  if (strcmp(comando, "EMG_START") == 0) {
    emgCalibraOrd = true;
    ultimaCalemg = micros();
    publicarEstado("Calibra_Inicio");
  }

    else if (strcmp(comando, "DATA_ON") == 0) {
    envioDatos = true;
    publicarEstado("ACK:DATA_ON");
  }

  else if (strcmp(comando, "DATA_OFF") == 0) {
    envioDatos = false;
    publicarEstado("ACK:DATA_OFF");
  }

  else if (strcmp(comando, "STATUS") == 0) {
    publicarEstadoGeneral();
  }

  else if (strncmp(comando, "RATE:", 5) == 0) {
    const char* numero = comando + 5;
    bool valido = (*numero != '\0');

    for (const char* p = numero; *p; ++p) {
      if (*p < '0' || *p > '9') {
        valido = false;
        break;
      }
    }

    unsigned long valor = valido ? strtoul(numero, nullptr, 10) : 0;

    if (valido &&
        valor >= MIN_INTERVAL_MS &&
        valor <= MAX_INTERVAL_MS) {
      intervaloBLE = (uint32_t)valor;
      publicarEstado("ACK:RATE");
    } else {
      publicarEstado("ERR:RATE");
    }
  }
}

//CALLBACK DE RECEPCIÓN BLE

class CommandCallbacks : public NimBLECharacteristicCallbacks {

  void onWrite(
    NimBLECharacteristic* characteristic,
    NimBLEConnInfo& connInfo
  ) override {

    std::string valor = characteristic->getValue();

    if (valor.empty() || valor.size() >= sizeof(comandoPendiente)) {
      return;
    }

    // Cola de un elemento: no sustituir un comando pendiente.
    portENTER_CRITICAL(&comandoMux);

    if (!hayComando) {
      memcpy(comandoPendiente, valor.data(), valor.size());
      comandoPendiente[valor.size()] = '\0';
      hayComando = true;
    }

    portEXIT_CRITICAL(&comandoMux);
  }
};

CommandCallbacks commandCallbacks;

//TRANSMISIÓN DE SENSORES

void transmitirSensores() {
  float temperatura[2];
  uint16_t presion[9] = {0};

  for (int i = 0; i < 2; i++) temperatura[i] = leerTemperaturaC(i);
  int j = 0;
  for (int i = 0; i < 3; i++) {
    for (int l = 0; l < 3; l++) {
      presion[j] = leerPresionADC(i,l);
      j++;
    }
  }

  char mensaje[120];

  snprintf(
    mensaje, sizeof(mensaje),
    "V1,T1:%.1f,T2:%.1f,P1:%u,P2:%u,P3:%u,P4:%u,P5:%u,P6:%u,P7:%u,P8:%u,P9:%u\n",
    temperatura[0], temperatura[1],
    presion[0], presion[1], presion[2], presion[3], presion[4],
    presion[5], presion[6], presion[7], presion[8]
  );

  Serial.print(mensaje);

  chrTelemetria->setValue(mensaje);
  chrTelemetria->notify();
}

//INICIALIZACIÓN

void setup() {
  Serial.begin(115200);

  for (int i=0 ; i<2 ; i++) {
    pinMode(PIN_LM[i], INPUT);
  }
  if (Modo == 'M') {
    for (int i = 0; i < 3; i++){
      pinMode(PIN_MVC[i],OUTPUT);
      pinMode(PIN_MVF[i],INPUT);
    }
  }else if (Modo == 'S') {
    for (int i = 0; i < 3; i++){
      pinMode(PIN_FSR[i],INPUT);
    }
  }
  pinMode(PIN_HEN, OUTPUT);
  pinMode(PIN_HLV, OUTPUT);
  pinMode(PIN_HRV, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_IFC, OUTPUT);
  pinMode(PIN_IFE, OUTPUT);

  digitalWrite(PIN_HEN, HIGH);

  analogReadResolution(12);

  NimBLEDevice::init("ESP32_BIOMEDICO");

  bleServer = NimBLEDevice::createServer();

  // Volver a anunciarse tras desconexión
  bleServer->advertiseOnDisconnect(true);

  NimBLEService* servicio =
      bleServer->createService(SERVICE_UUID);

  chrTelemetria = servicio->createCharacteristic(
    TELEMETRY_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );

  NimBLECharacteristic* chrComando =
      servicio->createCharacteristic(
        COMMAND_UUID,
        NIMBLE_PROPERTY::WRITE
      );

  chrEstado = servicio->createCharacteristic(
    STATUS_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );

  chrComando->setCallbacks(&commandCallbacks);

  chrTelemetria->setValue("V1,T1:NA,T2:NA,P1:NA,P2:NA,P3:NA,P4:NA,P5:NA,P6:NA,P7:NA,P8:NA,P9:NA\n");
  chrEstado->setValue("S,E0,D1,R200\n");

  servicio->start();

  NimBLEAdvertising* publicidad =
      NimBLEDevice::getAdvertising();

  publicidad->addServiceUUID(SERVICE_UUID);
  publicidad->enableScanResponse(true);
  publicidad->setName("ESP32_BIOMEDICO");
  publicidad->start();

  Serial.println("ESP32 BLE listo");
}

//BUCLE PRINCIPAL

void loop() {

  // Procesamiento local EMG
  CalibrarEMG();
  if (emgActivo) {
    int lecturaEMG = analogRead(PIN_EMG);
    AccionarApoyo(lecturaEMG);
  }

  // Extraer comando sin ejecutar lógica en el callback
  char comandoLocal[24] = "";
  bool procesar = false;

  portENTER_CRITICAL(&comandoMux);

  if (hayComando) {
    strcpy(comandoLocal, comandoPendiente);
    hayComando = false;
    procesar = true;
  }

  portEXIT_CRITICAL(&comandoMux);

  if (procesar) {
    ejecutarComando(comandoLocal);
  }

  // Telemetría periódica
  uint32_t ahora = millis();

  if (envioDatos &&
      bleServer->getConnectedCount() > 0 &&
      (uint32_t)(ahora - ultimoEnvio) >= intervaloBLE) {

    ultimoEnvio = ahora;
    Serial.print("enviado, conectados: ");
    Serial.println(bleServer->getConnectedCount());
    transmitirSensores();
  }

  delay(1);
}
