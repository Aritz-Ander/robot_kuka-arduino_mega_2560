const float STEPS_PER_REVGIRO = 200.0;                                                 // Pasos por revolución del motor (1.8° por paso)
const float MICROSTEPSGIRO = 16.0;                                                     // Micropasos (ajustar según configuración del driver)
const float PULLEY_TEETHGIRO = 20.0;                                                   // Número de dientes de la polea
const float BELT_PITCHGIRO = 2.0;                                                      // Paso de la correa en mm (típicamente GT2=2mm)
const float MM_PER_REVGIRO = PULLEY_TEETHGIRO * BELT_PITCHGIRO;                        // mm por revolución
const float STEPS_PER_MMGIRO = (STEPS_PER_REVGIRO * MICROSTEPSGIRO) / MM_PER_REVGIRO;  // Pasos por mm
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

struct EjeMotor {
  const String name;
  // Definición de pines
  const int pulPin;     // Pin de pulso
  const int dirPin;     // Pin de dirección
  const int enablePin;  // Pin de habilitación (opcional)

  // Parámetros de los motores
  const int stepsPerRevolution;  // Pasos por revolución (ajustar según configuración del HSS86)

  // Variables para control de tiempo y pulsos
  unsigned long previousMicros = 0;
  unsigned long currentMicros = 0;
  unsigned long pulseDuration = 0;
  bool pulseState = LOW;
  int currentPosition = 0;
  int targetPosition = 0;
  bool isRunning = false;
  int velocidad = 50;
  int motorSpeed = 1000;

  // Constructor
  EjeMotor(String n, int pul, int dir, int en, int stepsRev)
    : name(n), pulPin(pul), dirPin(dir), enablePin(en), stepsPerRevolution(stepsRev) {}

  void move(int position) {
    if (position == 0) {
      doHoming();
    } else {
      moveSelectedPosition(position);
    }
  }

  void moveSelectedPosition(int position) {
    // Establecer dirección
    if (position > currentPosition) {
      digitalWrite(dirPin, HIGH);  // Sentido horario (CW)
    } else {
      digitalWrite(dirPin, LOW);  // Sentido antihorario (CCW)
    }

    // Iniciar movimiento
    isRunning = true;
    Serial.print(name + " moviendo a: ");
    Serial.println(position);
  }

  void updateMotor() {
    currentMicros = micros();

    // Verificar si es tiempo de enviar un pulso
    if (currentMicros - previousMicros >= pulseDuration) {
      previousMicros = currentMicros;

      if (currentPosition != targetPosition) {
        // Cambiar estado del pin de pulso
        pulseState = !pulseState;
        digitalWrite(pulPin, pulseState);

        // Sólo contamos un paso completo cuando el pulso va de HIGH a LOW
        if (pulseState == LOW) {
          // Actualizar posición según dirección
          if (digitalRead(dirPin) == HIGH) {
            currentPosition++;
          } else {
            currentPosition--;
          }
        }
      } else {
        // Llegamos a la posición objetivo
        isRunning = false;
        Serial.print(name + ": Posición alcanzada: ");
        Serial.println(currentPosition);
      }
    }
  }

  void doHoming() {
    Serial.println(name + ": Iniciando secuencia de homing ...");

    // Configurar una velocidad más lenta para homing
    int originalSpeed = motorSpeed;
    motorSpeed = velocidad * 4;  // Velocidad lenta para homing
    pulseDuration = velocidad;

    targetPosition = 0;
    isRunning = true;
    // Establecer dirección
    if (targetPosition > currentPosition) {
      digitalWrite(dirPin, HIGH);  // Sentido horario (CW)
    } else {
      digitalWrite(dirPin, LOW);  // Sentido antihorario (CCW)
    }
    Serial.print(name + ": Moviendo a posición: ");
    Serial.println(targetPosition);

    // Restablecer velocidad original
    motorSpeed = originalSpeed;
    pulseDuration = velocidad;

    // Restablecer posición a cero
    currentPosition = 0;
    targetPosition = 0;
    Serial.println(name + ": Homing completado. Posición establecida a 0.");
  }
};

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
  pinMode(pulPinBASE, OUTPUT);
  pinMode(dirPinBASE, OUTPUT);
  pinMode(enablePinBASE, OUTPUT);
  pinMode(pulPinHOMBRO, OUTPUT);
  pinMode(dirPinHOMBRO, OUTPUT);
  pinMode(enablePinHOMBRO, OUTPUT);
  pinMode(pulPinCODO, OUTPUT);
  pinMode(dirPinCODO, OUTPUT);
  pinMode(enablePinCODO, OUTPUT);
  pinMode(pulPinMUNECA, OUTPUT);
  pinMode(dirPinMUNECA, OUTPUT);
  pinMode(enablePinMUNECA, OUTPUT);
  pinMode(pulPinMANO, OUTPUT);
  pinMode(dirPinMANO, OUTPUT);
  pinMode(enablePinMANO, OUTPUT);
  pinMode(pulPinGIRO, OUTPUT);
  pinMode(dirPinGIRO, OUTPUT);
  pinMode(enablePinGIRO, OUTPUT);

  // Inicializar comunicación serial
  Serial.begin(115200);
  Serial.println("Control de NAIbot iniciado");
  Serial.println("Comandos disponibles:");
  Serial.println("Movimiento individual: B[posición], H[posición], C[posición], M[posición], A[posición], G[posición]");
  Serial.println("Movimiento múltiple: B[posición],H[posición],C[posición],M[posición],A[posición],G[posición]");
  Serial.println("Homing: BO, HO, CO, MO, AO, GO");
  Serial.println("Estado actual: ?");

  // Calcular duración de pulso inicial según velocidad por defecto
  pulseDurationBASE = velocidadBASE;
  pulseDurationHOMBRO = velocidadHOMBRO;
  pulseDurationCODO = velocidadCODO;
  pulseDurationMUNECA = velocidadMUNECA;
  pulseDurationMANO = velocidadMANO;
  pulseDurationGIRO = velocidadGIRO;

  // Habilitar drivers (lógica invertida en algunos modelos)
  digitalWrite(enablePinBASE, LOW);
  digitalWrite(enablePinHOMBRO, LOW);
  digitalWrite(enablePinCODO, LOW);
  digitalWrite(enablePinMUNECA, LOW);
  digitalWrite(enablePinMANO, LOW);
  digitalWrite(enablePinGIRO, LOW);
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

void processCommand(String command) {
  if (command.length() < 1) {
    Serial.println("Comando vacío");
    return;
  }

  if (command == "?") {
    showCurrentInfo();
    return;
  }

  // Verificar si es un comando múltiple
  if (command.indexOf(',') != -1) {
    int startPos = 0;
    int endPos = 0;
    while (endPos != -1) {
      endPos = command.indexOf(',', startPos);
      String part;
      if (endPos != -1) {
        part = command.substring(startPos, endPos);
        startPos = endPos + 1;
      } else {
        part = command.substring(startPos);
      }
      // Procesar cada parte como comando simple
      processSingleCommand(part);
    }
    Serial.println("Iniciando movimiento multimotores");
  } else {
    // Si no es múltiple, procesa como comando simple
    processSingleCommand(command);
  }
}

void showCurrentInfo() {
  Serial.println("\n--- Estado Actual ---");
  Serial.print("BASE: ");
  Serial.println(currentPositionBASE);
  Serial.print("HOMBRO: ");
  Serial.println(currentPositionHOMBRO);
  Serial.print("CODO: ");
  Serial.println(currentPositionCODO);
  Serial.print("MUÑECA: ");
  Serial.println(currentPositionMUNECA);
  Serial.print("MANO: ");
  Serial.print(currentPositionmanoMM);
  Serial.println(" mm");
  Serial.print("GIRO: ");
  Serial.print(currentPositionGIROMM);
  Serial.println(" mm");
  Serial.println("-------------------");
}