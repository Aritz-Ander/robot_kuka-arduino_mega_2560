#include "EjeMotor.h"

const float STEPS_PER_REVGIRO   = 200.0;                                                  // Pasos por revolución del motor (1.8° por paso)
const float MICROSTEPSGIRO      = 16.0;                                                   // Micropasos (ajustar según configuración del driver)
const float PULLEY_TEETHGIRO    = 20.0;                                                   // Número de dientes de la polea
const float BELT_PITCHGIRO      = 2.0;                                                    // Paso de la correa en mm (típicamente GT2=2mm)
const float MM_PER_REVGIRO      = PULLEY_TEETHGIRO * BELT_PITCHGIRO;                      // mm por revolución
const float STEPS_PER_MMGIRO    = (STEPS_PER_REVGIRO * MICROSTEPSGIRO) / MM_PER_REVGIRO;  // Pasos por mm
// Variables globales
long currentPositionGIROSteps = 0;  // Posición actual en pasos
float currentPositionGIROMM = 0.0;  // Posición actual en mm
String inputString = "";            // String para almacenar la entrada
boolean stringComplete = false;     // Indica si la cadena está completa

const float STEPS_PER_REVmano = 200.0;                                                 // Pasos por revolución del motor (1.8° por paso)
const float MICROSTEPSmano = 16.0;                                                     // Micropasos (ajustar según configuración del driver)
const float PULLEY_TEETHmano = 20.0;                                                   // Número de dientes de la polea
const float BELT_PITCHmano = 2.0;                                                      // Paso de la correa en mm (típicamente GT2=2mm)
const float MM_PER_REVmano = PULLEY_TEETHmano * BELT_PITCHmano;                        // mm por revolución
const float STEPS_PER_MMmano = (STEPS_PER_REVmano * MICROSTEPSmano) / MM_PER_REVmano;  // Pasos por mm
// Variables globales
long currentPositionmanoSteps = 0;  // Posición actual en pasos
float currentPositionmanoMM = 0.0;  // Posición actual en mm

// Buffer para comandos seriales
String commandBuffer = "";

enum MotorID {
  BASE,
  HOMBRO,
  CODO,
  MUNECA,
  MANO,
  GIRO,
  MOTOR_COUNT  // Cantidad total de motores
};

MotorID charToMotorID(char c) {
  switch (c) {
    case 'B': return BASE;
    case 'H': return HOMBRO;
    case 'C': return CODO;
    case 'M': return MUNECA;
    case 'A': return MANO;
    case 'G': return GIRO;
    default: return MOTOR_COUNT;  // Indicador de inválido
  }
}

EjeMotor base("BASE", 2, 3, 40, 1000);
EjeMotor hombro("HOMBRO", 4, 5, 41, 1000);
EjeMotor codo("CODO", 6, 7, 42, 1000);
EjeMotor muneca("MUNECA", 8, 9, 43, 1000);
EjeMotor mano("MANO", 10, 11, 44, 1000);
EjeMotor giro("GIRO", 12, 13, 45, 1000);

// Crear array de punteros con los motores
EjeMotor* motores[MOTOR_COUNT] = { &base, &hombro, &codo, &muneca, &mano, &giro };

void setup() {
  // Configuración de pines
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(40, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(41, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(42, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(43, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(44, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(45, OUTPUT);

  // Habilitar drivers (lógica invertida en algunos modelos)
  digitalWrite(40, LOW);
  digitalWrite(41, LOW);
  digitalWrite(42, LOW);
  digitalWrite(43, LOW);
  digitalWrite(44, LOW);
  digitalWrite(45, LOW);

  // Inicializar comunicación serial
  Serial.begin(115200);
  Serial.println("Control de NAIbot iniciado");
  Serial.println("Comandos disponibles:");
  Serial.println("Movimiento individual: B[posición], H[posición], C[posición], M[posición], A[posición], G[posición]");
  Serial.println("Movimiento múltiple: B[posición],H[posición],C[posición],M[posición],A[posición],G[posición]");
  Serial.println("Homing: BO, HO, CO, MO, AO, GO");
  Serial.println("Estado actual: ?");
}

void loop() {
  // Leer comandos desde el puerto serial
  readSerialCommands();

  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (motores[i]->isRunning) {
      motores[i]->updateMotor();
    }
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    // Si es un carácter de nueva línea, procesar el comando completo
    if (c == '\n' || c == '\r') {
      if (commandBuffer.length() > 0) {
        processCommand(commandBuffer);
        commandBuffer = "";
      }
    } else {
      // Acumular caracteres en el buffer
      commandBuffer += c;
    }
  }
}

void processCommand(String command) {
  Serial.println("command: "+command);
  if (command.length() < 1) {
    Serial.println("Comando vacío");
    return;
  }

  if (command == "?") {
    showCurrentInfo();
    return;
  }

  int startPos = 0;
  int endPos = 0;
  do{
    endPos = command.indexOf(',', startPos);  // obtener trozo de comando
    String msg= (endPos == -1)  
      ? command.substring(startPos)          // si no hay "," coger todo el el mensaje a partir del indice start
      : command.substring(startPos,endPos);   // si hay "," coger desde start hasta el coma                            
    
    processSingleCommand(msg);                // procesar el comando obtenido  
    startPos= endPos + 1;                     // saltar el coma

  }while(endPos != -1);                       // si no habia "," significa que no hay mas comandos despues
}

void processSingleCommand(String cmd) {
  if (cmd.length() < 1) return;

  MotorID motorId = charToMotorID(cmd.charAt(0));
  if (motorId == MOTOR_COUNT) {
    Serial.println("Comando no reconocido");
    return;
  }

  int position = 0;
  if (cmd.length() > 1) {
    position = cmd.substring(1).toInt();
  }

  motores[motorId]->move(position);
}

void showCurrentInfo() {
  Serial.println("\n--- Estado Actual ---");
  Serial.print("BASE: ");
  Serial.println(motores[BASE]->currentPosition);
  Serial.print("HOMBRO: ");
  Serial.println(motores[HOMBRO]->currentPosition);
  Serial.print("CODO: ");
  Serial.println(motores[CODO]->currentPosition);
  Serial.print("MUÑECA: ");
  Serial.println(motores[MUNECA]->currentPosition);
  Serial.print("MANO: ");
  Serial.print(currentPositionmanoMM);
  Serial.println(" mm");
  Serial.print("GIRO: ");
  Serial.print(currentPositionGIROMM);
  Serial.println(" mm");
  Serial.println("-------------------");
}