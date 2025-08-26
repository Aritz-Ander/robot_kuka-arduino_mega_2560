// Definición de pines
const int pulPinBASE = 2;   // Pin de pulso
const int dirPinBASE = 3;   // Pin de dirección
const int enablePinBASE = 40; // Pin de habilitación (opcional)
const int pulPinHOMBRO = 4;   // Pin de pulso
const int dirPinHOMBRO = 5;   // Pin de dirección
const int enablePinHOMBRO = 41; // Pin de habilitación (opcional)
const int pulPinCODO = 6;   // Pin de pulso
const int dirPinCODO = 7;   // Pin de dirección
const int enablePinCODO = 42; // Pin de habilitación (opcional)
const int pulPinMUNECA = 8;   // Pin de pulso
const int dirPinMUNECA = 9;   // Pin de dirección
const int enablePinMUNECA = 43; // Pin de habilitación (opcional)
const int pulPinMANO = 10;   // Pin de pulso
const int dirPinMANO = 11;   // Pin de dirección
const int enablePinMANO = 44; // Pin de habilitación (opcional)
const int pulPinGIRO = 12;   // Pin de pulso
const int dirPinGIRO = 13;   // Pin de dirección
const int enablePinGIRO = 45; // Pin de habilitación (opcional)

// Parámetros de los motores
const int stepsPerRevolutionBASE = 1000;  // Pasos por revolución (ajustar según configuración del HSS86)
const int stepsPerRevolutionHOMBRO = 1000;  // Pasos por revolución (ajustar según configuración del HSS86)
const int stepsPerRevolutionCODO = 1000;  // Pasos por revolución (ajustar según configuración del HSS86)
const int stepsPerRevolutionMUNECA = 1000;  // Pasos por revolución (ajustar según configuración del HSS86)
const int stepsPerRevolutionMANO = 200;  // Pasos por revolución (ajustar según configuración del HSS86)
const int stepsPerRevolutionGIRO = 200;  // Pasos por revolución (ajustar según configuración del HSS86)

// Variables para control de tiempo y pulsos
unsigned long previousMicrosBASE = 0;
unsigned long currentMicrosBASE = 0;
unsigned long pulseDurationBASE = 0;
bool pulseStateBASE = LOW;
int currentPositionBASE = 0;
int targetPositionBASE = 0;
bool isRunningBASE = false;
int velocidadBASE = 50;
int motorSpeedBASE = 1000;

unsigned long previousMicrosHOMBRO = 0;
unsigned long currentMicrosHOMBRO = 0;
unsigned long pulseDurationHOMBRO = 0;
bool pulseStateHOMBRO = LOW;
int currentPositionHOMBRO = 0;
int targetPositionHOMBRO = 0;
bool isRunningHOMBRO = false;
int velocidadHOMBRO = 50;  
int motorSpeedHOMBRO = 1000;

unsigned long previousMicrosCODO = 0;
unsigned long currentMicrosCODO = 0;
unsigned long pulseDurationCODO = 0;
bool pulseStateCODO = LOW;
int currentPositionCODO = 0;
int targetPositionCODO = 0;
bool isRunningCODO = false;
int velocidadCODO = 50; 
int motorSpeedCODO = 1000;

unsigned long previousMicrosMUNECA = 0;
unsigned long currentMicrosMUNECA = 0;
unsigned long pulseDurationMUNECA = 0;
bool pulseStateMUNECA = LOW;
int currentPositionMUNECA = 0;
int targetPositionMUNECA = 0;
bool isRunningMUNECA = false;
int velocidadMUNECA = 250; 
int motorSpeedMUNECA = 1000;

// Variables para control de tiempo y pulsos
unsigned long previousMicrosMANO = 0;
unsigned long currentMicrosMANO = 0;
unsigned long pulseDurationMANO = 0;
bool pulseStateMANO = LOW;
int currentPositionMANO = 0;
int targetPositionMANO = 0;
bool isRunningMANO = false;
int velocidadMANO = 50;
int motorSpeedMANO = 1000;

unsigned long previousMicrosGIRO = 0;
unsigned long currentMicrosGIRO = 0;
unsigned long pulseDurationGIRO = 0;
bool pulseStateGIRO = LOW;
int currentPositionGIRO = 0;
int targetPositionGIRO = 0;
bool isRunningGIRO = false;
int velocidadGIRO = 50;
int motorSpeedGIRO = 1000;

const float STEPS_PER_REVGIRO = 200.0;       // Pasos por revolución del motor (1.8° por paso)
const float MICROSTEPSGIRO = 16.0;           // Micropasos (ajustar según configuración del driver)
const float PULLEY_TEETHGIRO = 20.0;         // Número de dientes de la polea
const float BELT_PITCHGIRO = 2.0;            // Paso de la correa en mm (típicamente GT2=2mm)
const float MM_PER_REVGIRO = PULLEY_TEETHGIRO * BELT_PITCHGIRO;  // mm por revolución
const float STEPS_PER_MMGIRO = (STEPS_PER_REVGIRO * MICROSTEPSGIRO) / MM_PER_REVGIRO;  // Pasos por mm
// Variables globales
long currentPositionGIROSteps = 0;  // Posición actual en pasos
float currentPositionGIROMM = 0.0;  // Posición actual en mm
String inputString = "";       // String para almacenar la entrada
boolean stringComplete = false;  // Indica si la cadena está completa

const float STEPS_PER_REVmano = 200.0;       // Pasos por revolución del motor (1.8° por paso)
const float MICROSTEPSmano = 16.0;           // Micropasos (ajustar según configuración del driver)
const float PULLEY_TEETHmano = 20.0;         // Número de dientes de la polea
const float BELT_PITCHmano = 2.0;            // Paso de la correa en mm (típicamente GT2=2mm)
const float MM_PER_REVmano = PULLEY_TEETHmano * BELT_PITCHmano;  // mm por revolución
const float STEPS_PER_MMmano = (STEPS_PER_REVmano * MICROSTEPSmano) / MM_PER_REVmano;  // Pasos por mm
// Variables globales
long currentPositionmanoSteps = 0;  // Posición actual en pasos
float currentPositionmanoMM = 0.0;  // Posición actual en mm

// Buffer para comandos seriales
String commandBuffer = "";

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
  
  // Actualizar todos los motores si están en movimiento
  if (isRunningBASE) {
    updateMotorBASE();
  }
  if (isRunningHOMBRO) {
    updateMotorHOMBRO();
  }
  if (isRunningCODO) {
    updateMotorCODO();
  }
  if (isRunningMUNECA) {
    updateMotorMUNECA();
  }
  if (isRunningMANO) {
    updateMotorMANO();
  }
  if (isRunningGIRO) {
    updateMotorGIRO();
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
  // Verificar si es un comando múltiple (contiene comas)
  if (command.indexOf(',') != -1) {
    processMultiCommand(command);
    return;
  }
  
  // Comandos individuales
  if (command.startsWith("B") && command.length() > 1) {
    if (command.substring(1, 3) == "O") {
      doHomingBASE();
    } else {
      targetPositionBASE = command.substring(1).toInt();
      moveMotorBASE(targetPositionBASE);
    }
  }
  else if (command.startsWith("H") && command.length() > 1) {
    if (command.substring(1, 3) == "O") {
      doHomingHOMBRO();
    } else {
      targetPositionHOMBRO = command.substring(1).toInt();
      moveMotorHOMBRO(targetPositionHOMBRO);
    }
  }
  else if (command.startsWith("C") && command.length() > 1) {
    if (command.substring(1, 3) == "O") {
      doHomingCODO();
    } else {
      targetPositionCODO = command.substring(1).toInt();
      moveMotorCODO(targetPositionCODO);
    }
  }
  else if (command.startsWith("M") && command.length() > 1) {
    if (command.substring(1, 3) == "O") {
      doHomingMUNECA();
    } else {
      targetPositionMUNECA = command.substring(1).toInt();
      moveMotorMUNECA(targetPositionMUNECA);
    }
  }
  else if (command.startsWith("A") && command.length() > 1) {
    if (command.substring(1, 3) == "O") {
      homePositionMANO();
    } else {
      float targetMM = command.substring(1).toFloat();
      if (targetMM >= 0 && targetMM <= 80000) {
        moveToPositionMANO(targetMM);
      }
    }
  }
  else if (command.startsWith("G") && command.length() > 1) {
    if (command.substring(1, 3) == "O") {
      homePositionGIRO();
    } else {
      float targetMM = command.substring(1).toFloat();
      if (targetMM >= 0 && targetMM <= 80000) {
        moveToPositionGIRO(targetMM);
      }
    }
  }
  else if (command == "?") {
    showCurrentInfo();
  }
  else {
    Serial.println("Comando no reconocido");
  }
}

void processMultiCommand(String command) {
  // Dividir el comando en partes separadas por comas
  int startPos = 0;
  int endPos = 0;
  
  Serial.println("Procesando comando múltiple");
  
  while (endPos != -1) {
    endPos = command.indexOf(',', startPos);
    String part;
    
    if (endPos != -1) {
      part = command.substring(startPos, endPos);
      startPos = endPos + 1;
    } else {
      part = command.substring(startPos);
    }
    
    // Procesar cada parte del comando
    if (part.startsWith("B") && part.length() > 1) {
      targetPositionBASE = part.substring(1).toInt();
      moveMotorBASE(targetPositionBASE);
    }
    else if (part.startsWith("H") && part.length() > 1) {
      targetPositionHOMBRO = part.substring(1).toInt();
      moveMotorHOMBRO(targetPositionHOMBRO);
    }
    else if (part.startsWith("C") && part.length() > 1) {
      targetPositionCODO = part.substring(1).toInt();
      moveMotorCODO(targetPositionCODO);
    }
    else if (part.startsWith("M") && part.length() > 1) {
      targetPositionMUNECA = part.substring(1).toInt();
      moveMotorMUNECA(targetPositionMUNECA);
    }
    else if (part.startsWith("A") && part.length() > 1) {
      float targetMM = part.substring(1).toFloat();
      if (targetMM >= 0 && targetMM <= 80000) {
        moveToPositionMANO(targetMM);
      }
    }
    else if (part.startsWith("G") && part.length() > 1) {
      float targetMM = part.substring(1).toFloat();
      if (targetMM >= 0 && targetMM <= 80000) {
        moveToPositionGIRO(targetMM);
      }
    }
  }
  
  Serial.println("Iniciando movimiento multimotores");
}

void moveMotorBASE(int position) {
  // Establecer dirección
  if (position > currentPositionBASE) {
    digitalWrite(dirPinBASE, HIGH); // Sentido horario (CW)
  } else {
    digitalWrite(dirPinBASE, LOW);  // Sentido antihorario (CCW)
  }
  
  // Iniciar movimiento
  isRunningBASE = true;
  Serial.print("BASE moviendo a: ");
  Serial.println(position);
}

void moveMotorHOMBRO(int position) {
  // Establecer dirección
  if (position > currentPositionHOMBRO) {
    digitalWrite(dirPinHOMBRO, HIGH); // Sentido horario (CW)
  } else {
    digitalWrite(dirPinHOMBRO, LOW);  // Sentido antihorario (CCW)
  }
  
  // Iniciar movimiento
  isRunningHOMBRO = true;
  Serial.print("HOMBRO moviendo a: ");
  Serial.println(position);
}

void moveMotorCODO(int position) {
  // Establecer dirección
  if (position > currentPositionCODO) {
    digitalWrite(dirPinCODO, HIGH); // Sentido horario (CW)
  } else {
    digitalWrite(dirPinCODO, LOW);  // Sentido antihorario (CCW)
  }
  
  // Iniciar movimiento
  isRunningCODO = true;
  Serial.print("CODO moviendo a: ");
  Serial.println(position);
}

void moveMotorMUNECA(int position) {
  // Establecer dirección
  if (position > currentPositionMUNECA) {
    digitalWrite(dirPinMUNECA, HIGH); // Sentido horario (CW)
  } else {
    digitalWrite(dirPinMUNECA, LOW);  // Sentido antihorario (CCW)
  }
  
  // Iniciar movimiento
  isRunningMUNECA = true;
  Serial.print("MUÑECA moviendo a: ");
  Serial.println(position);
}

void updateMotorBASE() {
  currentMicrosBASE = micros();
  
  // Verificar si es tiempo de enviar un pulso
  if (currentMicrosBASE - previousMicrosBASE >= pulseDurationBASE) {
    previousMicrosBASE = currentMicrosBASE;
    
    if (currentPositionBASE != targetPositionBASE) {
      // Cambiar estado del pin de pulso
      pulseStateBASE = !pulseStateBASE;
      digitalWrite(pulPinBASE, pulseStateBASE);
      Serial.println(currentPositionBASE);
      // Sólo contamos un paso completo cuando el pulso va de HIGH a LOW
      if (pulseStateBASE == LOW) {
        // Actualizar posición según dirección
        if (digitalRead(dirPinBASE) == HIGH) {
          currentPositionBASE++;
        } else {
          currentPositionBASE--;
        }
      }
    } else {
      // Llegamos a la posición objetivo
      isRunningBASE = false;
      Serial.print("BASE: Posición alcanzada: ");
      Serial.println(currentPositionBASE);
    }
  }
}

void doHomingBASE() {
  Serial.println("Iniciando secuencia de homing BASE...");
  
  // Aquí puedes implementar tu lógica de homing
  // Por ejemplo, moverte lentamente hasta detectar un interruptor de límite
  
  // Configurar una velocidad más lenta para homing
  int originalSpeed = motorSpeedBASE;
  motorSpeedBASE = velocidadBASE * 4; // Velocidad lenta para homing
  pulseDurationBASE = velocidadBASE;
  
  targetPositionBASE = 0;
  isRunningBASE = true;
  // Establecer dirección
  if (targetPositionBASE > currentPositionBASE) {
    digitalWrite(dirPinBASE, HIGH); // Sentido horario (CW)
  } else {
    digitalWrite(dirPinBASE, LOW);  // Sentido antihorario (CCW)
  }
  Serial.print("BASE: Moviendo a posición: ");
  Serial.println(targetPositionBASE);
  
  // Restablecer velocidad original
  motorSpeedBASE = originalSpeed;
  pulseDurationBASE = velocidadBASE;
  
  // Restablecer posición a cero
  currentPositionBASE = 0;
  targetPositionBASE = 0;
  Serial.println("BASE: Homing completado. Posición establecida a 0.");
}

void updateMotorHOMBRO() {
  currentMicrosHOMBRO = micros();
  
  // Verificar si es tiempo de enviar un pulso
  if (currentMicrosHOMBRO - previousMicrosHOMBRO >= pulseDurationHOMBRO) {
    previousMicrosHOMBRO = currentMicrosHOMBRO;
    
    if (currentPositionHOMBRO != targetPositionHOMBRO) {
      // Cambiar estado del pin de pulso
      pulseStateHOMBRO = !pulseStateHOMBRO;
      digitalWrite(pulPinHOMBRO, pulseStateHOMBRO);
      
      // Sólo contamos un paso completo cuando el pulso va de HIGH a LOW
      if (pulseStateHOMBRO == LOW) {
        // Actualizar posición según dirección
        if (digitalRead(dirPinHOMBRO) == HIGH) {
          currentPositionHOMBRO++;
        } else {
          currentPositionHOMBRO--;
        }
      }
    } else {
      // Llegamos a la posición objetivo
      isRunningHOMBRO = false;
      Serial.print("HOMBRO: Posición alcanzada: ");
      Serial.println(currentPositionHOMBRO);
    }
  }
}

void doHomingHOMBRO() {
  Serial.println("Iniciando secuencia de homing HOMBRO...");
  
  // Configurar una velocidad más lenta para homing
  int originalSpeed = motorSpeedHOMBRO;
  motorSpeedHOMBRO = velocidadHOMBRO * 4; // Velocidad lenta para homing
  pulseDurationHOMBRO = velocidadHOMBRO;
  
  targetPositionHOMBRO = 0;
  isRunningHOMBRO = true;
  // Establecer dirección
  if (targetPositionHOMBRO > currentPositionHOMBRO) {
    digitalWrite(dirPinHOMBRO, HIGH); // Sentido horario (CW)
  } else {
    digitalWrite(dirPinHOMBRO, LOW);  // Sentido antihorario (CCW)
  }
  Serial.print("HOMBRO: Moviendo a posición: ");
  Serial.println(targetPositionHOMBRO);
  
  // Restablecer velocidad original
  motorSpeedHOMBRO = originalSpeed;
  pulseDurationHOMBRO = velocidadHOMBRO;
  
  // Restablecer posición a cero
  currentPositionHOMBRO = 0;
  targetPositionHOMBRO = 0;
  Serial.println("HOMBRO: Homing completado. Posición establecida a 0.");
}

void updateMotorCODO() {
  currentMicrosCODO = micros();
  
  // Verificar si es tiempo de enviar un pulso
  if (currentMicrosCODO - previousMicrosCODO >= pulseDurationCODO) {
    previousMicrosCODO = currentMicrosCODO;
    
    if (currentPositionCODO != targetPositionCODO) {
      // Cambiar estado del pin de pulso
      pulseStateCODO = !pulseStateCODO;
      digitalWrite(pulPinCODO, pulseStateCODO);
      
      // Sólo contamos un paso completo cuando el pulso va de HIGH a LOW
      if (pulseStateCODO == LOW) {
        // Actualizar posición según dirección
        if (digitalRead(dirPinCODO) == HIGH) {
          currentPositionCODO++;
        } else {
          currentPositionCODO--;
        }
      }
    } else {
      // Llegamos a la posición objetivo
      isRunningCODO = false;
      Serial.print("CODO: Posición alcanzada: ");
      Serial.println(currentPositionCODO);
    }
  }
}

void doHomingCODO() {
  Serial.println("Iniciando secuencia de homing CODO...");
  
  // Configurar una velocidad más lenta para homing
  int originalSpeed = motorSpeedCODO;
  motorSpeedCODO = velocidadCODO * 4; // Velocidad lenta para homing
  pulseDurationCODO = velocidadCODO;
  
  targetPositionCODO = 0;
  isRunningCODO = true;
  // Establecer dirección
  if (targetPositionCODO > currentPositionCODO) {
    digitalWrite(dirPinCODO, HIGH); // Sentido horario (CW)
  } else {
    digitalWrite(dirPinCODO, LOW);  // Sentido antihorario (CCW)
  }
  Serial.print("CODO: Moviendo a posición: ");
  Serial.println(targetPositionCODO);
  
  // Restablecer velocidad original
  motorSpeedCODO = originalSpeed;
  pulseDurationCODO = velocidadCODO;
  
  // Restablecer posición a cero
  currentPositionCODO = 0;
  targetPositionCODO = 0;
  Serial.println("CODO: Homing completado. Posición establecida a 0.");
}

void updateMotorMUNECA() {
  currentMicrosMUNECA = micros();
  
  // Verificar si es tiempo de enviar un pulso
  if (currentMicrosMUNECA - previousMicrosMUNECA >= pulseDurationMUNECA) {
    previousMicrosMUNECA = currentMicrosMUNECA;
    
    if (currentPositionMUNECA != targetPositionMUNECA) {
      // Cambiar estado del pin de pulso
      pulseStateMUNECA = !pulseStateMUNECA;
      digitalWrite(pulPinMUNECA, pulseStateMUNECA);
      Serial.println(currentMicrosMUNECA);
      // Sólo contamos un paso completo cuando el pulso va de HIGH a LOW
      if (pulseStateMUNECA == LOW) {
        // Actualizar posición según dirección
        if (digitalRead(dirPinMUNECA) == HIGH) {
          currentPositionMUNECA++;
        } else {
          currentPositionMUNECA--;
        }
      }
    } else {
      // Llegamos a la posición objetivo
      isRunningMUNECA = false;
      Serial.print("MUÑECA: Posición alcanzada: ");
      Serial.println(currentPositionMUNECA);
    }
  }
}

void doHomingMUNECA() {
  Serial.println("Iniciando secuencia de homing MUÑECA...");
  
  // Configurar una velocidad más lenta para homing
  int originalSpeed = motorSpeedMUNECA;
  motorSpeedMUNECA = velocidadMUNECA * 4; // Velocidad lenta para homing
  pulseDurationMUNECA = velocidadMUNECA;
  
  targetPositionMUNECA = 0;
  isRunningMUNECA = true;
  // Establecer dirección
  if (targetPositionMUNECA > currentPositionMUNECA) {
    digitalWrite(dirPinMUNECA, HIGH); // Sentido horario (CW)
  } else {
    digitalWrite(dirPinMUNECA, LOW);  // Sentido antihorario (CCW)
  }
  Serial.print("MUÑECA: Moviendo a posición: ");
  Serial.println(targetPositionMUNECA);
  
  // Restablecer velocidad original
  motorSpeedMUNECA = originalSpeed;
  pulseDurationMUNECA = velocidadMUNECA;
  
  // Restablecer posición a cero
  currentPositionMUNECA = 0;
  targetPositionMUNECA = 0;
  Serial.println("MUÑECA: Homing completado. Posición establecida a 0.");
}

// Nuevas funciones para controlar MANO y GIRO de manera no bloqueante
void updateMotorMANO() {
  currentMicrosMANO = micros();
  
  // Verificar si es tiempo de enviar un pulso
  if (currentMicrosMANO - previousMicrosMANO >= pulseDurationMANO) {
    previousMicrosMANO = currentMicrosMANO;
    
    if (currentPositionMANO != targetPositionMANO) {
      // Cambiar estado del pin de pulso
      pulseStateMANO = !pulseStateMANO;
      digitalWrite(pulPinMANO, pulseStateMANO);
      
      // Sólo contamos un paso completo cuando el pulso va de HIGH a LOW
      if (pulseStateMANO == LOW) {
        // Actualizar posición según dirección
        if (digitalRead(dirPinMANO) == HIGH) {
          currentPositionMANO++;
          currentPositionmanoSteps++;
        } else {
          currentPositionMANO--;
          currentPositionmanoSteps--;
        }
        
        // Actualizar posición en mm
        currentPositionmanoMM = currentPositionmanoSteps / STEPS_PER_MMmano;
      }
    } else {
      // Llegamos a la posición objetivo
      isRunningMANO = false;
      Serial.print("MANO: Posición alcanzada: ");
      Serial.print(currentPositionMANO);
      Serial.print(" pasos, ");
      Serial.print(currentPositionmanoMM);
      Serial.println(" mm");
    }
  }
}

void updateMotorGIRO() {
  currentMicrosGIRO = micros();
  
  // Verificar si es tiempo de enviar un pulso
  if (currentMicrosGIRO - previousMicrosGIRO >= pulseDurationGIRO) {
    previousMicrosGIRO = currentMicrosGIRO;
    
    if (currentPositionGIRO != targetPositionGIRO) {
      // Cambiar estado del pin de pulso
      pulseStateGIRO = !pulseStateGIRO;
      digitalWrite(pulPinGIRO, pulseStateGIRO);
      
      // Sólo contamos un paso completo cuando el pulso va de HIGH a LOW
      if (pulseStateGIRO == LOW) {
        // Actualizar posición según dirección
        if (digitalRead(dirPinGIRO) == HIGH) {
          currentPositionGIRO++;
          currentPositionGIROSteps++;
        } else {
          currentPositionGIRO--;
          currentPositionGIROSteps--;
        }
        
        // Actualizar posición en mm
        currentPositionGIROMM = currentPositionGIROSteps / STEPS_PER_MMGIRO;
      }
    } else {
      // Llegamos a la posición objetivo
      isRunningGIRO = false;
      Serial.print("GIRO: Posición alcanzada: ");
      Serial.print(currentPositionGIRO);
      Serial.print(" pasos, ");
      Serial.print(currentPositionGIROMM);
      Serial.println(" mm");
    }
  }
}

void moveToPositionMANO(float targetPositionMM) {
  // Calcular los pasos necesarios para llegar a la posición objetivo
  long targetPositionSteps = (long)(targetPositionMM * STEPS_PER_MMmano);
  
  // Determinar dirección
  if (targetPositionSteps > currentPositionmanoSteps) {
    digitalWrite(dirPinMANO, HIGH);  // Ajusta HIGH/LOW según tu configuración
  } else {
    digitalWrite(dirPinMANO, LOW);   // Ajusta HIGH/LOW según tu configuración
  }
  
  // Asignar posición objetivo para movimiento no bloqueante
  targetPositionMANO = targetPositionSteps;
  isRunningMANO = true;
  
  Serial.print("MANO: Moviendo a ");
  Serial.print(targetPositionMM);
  Serial.println(" mm");
}

void homePositionMANO() {
  // Aquí puedes implementar la lógica para buscar el punto de referencia
  // utilizando un sensor de final de carrera
  
  Serial.println("Iniciando homing MANO...");
  
  // Configurar dirección hacia el punto de referencia
  digitalWrite(dirPinMANO, LOW);  // Ajustar según configuración
  
  // Por ahora, simplemente establecemos la posición a 0
  currentPositionMANO = 0;
  currentPositionmanoSteps = 0;
  currentPositionmanoMM = 0.0;
  targetPositionMANO = 0;
  isRunningMANO = false;
  
  Serial.println("MANO: Homing completado. Posición establecida a 0.");
}

void moveToPositionGIRO(float targetPositionMM) {
  // Calcular los pasos necesarios para llegar a la posición objetivo
  long targetPositionSteps = (long)(targetPositionMM * STEPS_PER_MMGIRO);
  
  // Determinar dirección
  if (targetPositionSteps > currentPositionGIROSteps) {
    digitalWrite(dirPinGIRO, HIGH);  // Ajusta HIGH/LOW según tu configuración
  } else {
    digitalWrite(dirPinGIRO, LOW);   // Ajusta HIGH/LOW según tu configuración
  }
  
  // Asignar posición objetivo para movimiento no bloqueante
  targetPositionGIRO = targetPositionSteps;
  isRunningGIRO = true;
  
  Serial.print("GIRO: Moviendo a ");
  Serial.print(targetPositionMM);
  Serial.println(" mm");
}

void homePositionGIRO() {
  // Aquí puedes implementar la lógica para buscar el punto de referencia
  // utilizando un sensor de final de carrera
  
  Serial.println("Iniciando homing GIRO...");
  
  // Configurar dirección hacia el punto de referencia
  digitalWrite(dirPinGIRO, LOW);  // Ajustar según configuración
  
  // Por ahora, simplemente establecemos la posición a 0
  currentPositionGIRO = 0;
  currentPositionGIROSteps = 0;
  currentPositionGIROMM = 0.0;
  targetPositionGIRO = 0;
  isRunningGIRO = false;
  
  Serial.println("GIRO: Homing completado. Posición establecida a 0.");
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
