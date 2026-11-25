 
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/*
  =====================================
             SPEEDGUARD V4.0
  =====================================

  Componentes:
  - Arduino Uno R3
  - 2 sensores HC-SR04
  - LCD 16x2 I2C
  - LDR e resistor de 10k
  - Potenciometro de 10k
  - 2 botoes
  - 3 LEDs com resistores de 330 ohms
  - Buzzer
  - Servo SG90

  Funcionamento:
  - Mede velocidade com dois HC-SR04
  - Classifica NORMAL, ATENCAO, EXCESSO
  - Alterna entre dia e noite
  - Permite configurar limites
  - Identifica falhas e recupera
  - Utiliza millis() e micros()
  - Sem delay() e sem pulseIn()

  Os limites sao de demonstracao.
  Recalibrar na montagem fisica.
*/

// =====================================
// 1. PINOS
// =====================================

const byte TRIG1 = 8;
const byte ECHO1 = 7;
const byte TRIG2 = 10;
const byte ECHO2 = 11;

const byte LDR_PIN = A0;
const byte POT_PIN = A1;

const byte BOTAO_CONFIG = 12;
const byte BOTAO_OK = 13;

const byte LED_VERDE = 2;
const byte LED_AMARELO = 3;
const byte LED_VERMELHO = 4;

const byte BUZZER = 5;
const byte SERVO_PIN = 6;

const byte trigPins[2] = {TRIG1, TRIG2};
const byte echoPins[2] = {ECHO1, ECHO2};

Servo placa;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// =====================================
// 2. PARAMETROS
// =====================================

const float DISTANCIA_PONTOS = 30.0;
const float DETECTAR = 30.0;
const float LIBERAR = 40.0;

// Limites iniciais para simulacao
float limiteDia = 0.12;
float limiteNoite = 0.08;

const float VELOCIDADE_MIN = 0.01;
const float VELOCIDADE_MAX = 5.0;

// Microssegundos
const unsigned long INTERVALO_PING = 70000UL;
const unsigned long TIMEOUT_ECO = 25000UL;
const unsigned long TEMPO_MAXIMO = 12000000UL;

// Milissegundos
const unsigned long TEMPO_RESULTADO = 3000UL;
const unsigned long TEMPO_PRONTO = 2000UL;
const unsigned long TEMPO_RECUPERACAO = 1500UL;
const unsigned long TEMPO_LUZ = 1000UL;
const unsigned long DEBOUNCE = 35UL;

const int LIMIAR_NOITE = 350;
const int LIMIAR_DIA = 600;

// =====================================
// 3. ESTADOS PRINCIPAIS
// =====================================

enum Estado {
  INICIALIZANDO,
  AGUARDANDO,
  MEDINDO,
  RESULTADO,
  FALHA
};

Estado estado = INICIALIZANDO;

// =====================================
// 4. ESTADOS DOS HC-SR04
// =====================================

enum FaseSonar {
  SONAR_PRONTO,
  TRIGGER_ALTO,
  ESPERAR_ECHO_ALTO,
  ESPERAR_ECHO_BAIXO
};

FaseSonar faseSonar = SONAR_PRONTO;

// =====================================
// 5. VARIAVEIS
// =====================================

// Luminosidade
bool modoNoturno = false;
bool candidatoNoturno = false;
bool modoDaPassagem = false;

unsigned long ultimaLeituraLuz = 0;
unsigned long inicioMudancaLuz = 0;

// Sensores
bool presente[2] = {false, false};
bool leituraValida[2] = {false, false};

byte contPresenca[2] = {0, 0};
byte contAusencia[2] = {0, 0};
byte contFalha[2] = {0, 0};

// Leitura HC-SR04
byte sensorAtual = 0;

unsigned long ultimaMedicao = 0;
unsigned long inicioTrigger = 0;
unsigned long inicioEsperaEcho = 0;
unsigned long inicioEcho = 0;

// Passagem
unsigned long inicioPassagem = 0;
unsigned long inicioPistaLivre = 0;

bool contagemResultadoAtiva = false;

// Recuperacao
unsigned long inicioRecuperacao = 0;

// Mensagens temporarias
bool mostrandoPronto = false;
unsigned long inicioMensagem = 0;

// Configuracao
// 0 = normal
// 1 = ajustar dia
// 2 = ajustar noite

byte menuConfig = 0;
int ultimoValorConfig = -1;

// Botao CONFIG
bool configLeituraAnterior = HIGH;
bool configEstavel = HIGH;
unsigned long configMudou = 0;

// Botao OK
bool okLeituraAnterior = HIGH;
bool okEstavel = HIGH;
unsigned long okMudou = 0;

// Estatisticas
unsigned int totalVeiculos = 0;
unsigned int totalExcessos = 0;
unsigned int totalFalhas = 0;

// =====================================
// 6. LCD
// =====================================

void escreverLinha(byte linha, const char* texto) {

  lcd.setCursor(0, linha);

  byte i = 0;

  while (texto[i] != '\0' && i < 16) {
    lcd.print(texto[i]);
    i++;
  }

  while (i < 16) {
    lcd.print(' ');
    i++;
  }
}

void mostrarLCD(const char* linha1,
                const char* linha2) {

  escreverLinha(0, linha1);
  escreverLinha(1, linha2);
}

void telaInicializacao() {

  mostrarLCD(
    "SPEEDGUARD",
    "VERIF. SENSORES"
  );
}

void telaEspera() {

  if (estado != AGUARDANDO ||
      mostrandoPronto ||
      menuConfig != 0) {
    return;
  }

  if (modoNoturno) {
    mostrarLCD("SPEEDGUARD", "MODO: NOTURNO");
  }
  else {
    mostrarLCD("SPEEDGUARD", "MODO: DIURNO");
  }
}

void telaMedicao() {

  mostrarLCD(
    "MEDINDO...",
    "AGUARDE SENSOR 2"
  );
}

void telaResultado(float velocidade,
                   const char* status) {

  // Apaga a linha para evitar sobras.
  escreverLinha(0, "");

  lcd.setCursor(0, 0);
  lcd.print("VEL: ");
  lcd.print(velocidade, 3);
  lcd.print(" m/s");

  escreverLinha(1, status);

  Serial.print("LCD: ");
  Serial.println(status);
}

void telaFalha() {

  mostrarLCD(
    "FALHA NO SIST.",
    "VERIFIQUE"
  );
}

void telaPronto(const char* linha1,
                const char* linha2) {

  mostrarLCD(linha1, linha2);

  mostrandoPronto = true;
  inicioMensagem = millis();
}

void atualizarMensagem() {

  if (!mostrandoPronto) {
    return;
  }

  if (millis() - inicioMensagem >= TEMPO_PRONTO) {

    mostrandoPronto = false;

    if (estado == AGUARDANDO) {
      telaEspera();
    }
  }
}

// =====================================
// 7. SINALIZACAO
// =====================================

void apagarLeds() {

  digitalWrite(LED_VERDE, LOW);
  digitalWrite(LED_AMARELO, LOW);
  digitalWrite(LED_VERMELHO, LOW);
}

void sinalizarNormal() {

  apagarLeds();
  digitalWrite(LED_VERDE, HIGH);

  noTone(BUZZER);
  placa.write(0);
}

void sinalizarAtencao() {

  apagarLeds();
  digitalWrite(LED_AMARELO, HIGH);

  noTone(BUZZER);
  placa.write(45);
}

void sinalizarExcesso() {

  apagarLeds();
  digitalWrite(LED_VERMELHO, HIGH);

  placa.write(90);
  tone(BUZZER, 1000, 1800);
}

void sinalizarFalha() {

  apagarLeds();
  digitalWrite(LED_VERMELHO, HIGH);

  placa.write(0);
  tone(BUZZER, 600, 1800);
}

// =====================================
// 8. FALHAS
// =====================================

void registrarFalha(const char* motivo) {

  if (estado == FALHA) {
    return;
  }

  estado = FALHA;

  totalFalhas++;

  inicioRecuperacao = 0;
  contagemResultadoAtiva = false;
  mostrandoPronto = false;
  menuConfig = 0;

  sinalizarFalha();
  telaFalha();

  Serial.println();
  Serial.println("===== FALHA =====");

  Serial.print("Motivo: ");
  Serial.println(motivo);

  Serial.print("Total de falhas: ");
  Serial.println(totalFalhas);
}

// =====================================
// 9. LDR
// =====================================

void atualizarLuz() {

  unsigned long agora = millis();

  if (agora - ultimaLeituraLuz < 100) {
    return;
  }

  ultimaLeituraLuz = agora;

  int luz = analogRead(LDR_PIN);

  bool desejado = modoNoturno;

  if (luz < LIMIAR_NOITE) {
    desejado = true;
  }
  else if (luz > LIMIAR_DIA) {
    desejado = false;
  }

  if (desejado != modoNoturno) {

    if (candidatoNoturno != desejado) {

      candidatoNoturno = desejado;
      inicioMudancaLuz = agora;
    }

    if (agora - inicioMudancaLuz >= TEMPO_LUZ) {

      modoNoturno = desejado;

      Serial.println(
        modoNoturno ? "MODO NOTURNO" : "MODO DIURNO"
      );

      if (estado == AGUARDANDO &&
          !mostrandoPronto &&
          menuConfig == 0) {

        telaEspera();
      }
    }
  }
  else {

    candidatoNoturno = modoNoturno;
    inicioMudancaLuz = agora;
  }
}

// =====================================
// 10. POTENCIOMETRO E CONFIGURACAO
// =====================================

// Retorna valor inteiro em centesimos
// de m/s para evitar pequenas oscilacoes.
int lerValorConfig() {

  int leitura = analogRead(POT_PIN);

  if (menuConfig == 1) {
    // Dia: 0.05 a 0.30 m/s
    return map(leitura, 0, 1023, 5, 30);
  }

  // Noite: 0.04 a 0.20 m/s
  return map(leitura, 0, 1023, 4, 20);
}

void telaConfiguracao() {

  if (menuConfig == 0) {
    return;
  }

  int valor = lerValorConfig();

  if (valor == ultimoValorConfig) {
    return;
  }

  ultimoValorConfig = valor;

  if (menuConfig == 1) {
    escreverLinha(0, "AJUSTAR DIA");
  }
  else {
    escreverLinha(0, "AJUSTAR NOITE");
  }

  escreverLinha(1, "");

  lcd.setCursor(0, 1);
  lcd.print("LIM: ");
  lcd.print(valor / 100.0, 2);
  lcd.print(" m/s");
}

void botaoConfigPressionado() {

  // Configuracao apenas em espera,
  // com sensores livres.
  if (estado != AGUARDANDO ||
      mostrandoPronto ||
      presente[0] ||
      presente[1]) {
    return;
  }

  if (menuConfig == 0) {
    menuConfig = 1;
  }
  else if (menuConfig == 1) {
    menuConfig = 2;
  }
  else {
    menuConfig = 0;
  }

  ultimoValorConfig = -1;

  if (menuConfig == 0) {

    Serial.println("Configuracao encerrada");
    telaEspera();
  }
  else {

    Serial.println(
      menuConfig == 1 ?
      "Ajustando limite diurno" :
      "Ajustando limite noturno"
    );

    telaConfiguracao();
  }
}

void botaoOkPressionado() {

  if (menuConfig != 0) {

    int valor = lerValorConfig();

    if (menuConfig == 1) {

      limiteDia = valor / 100.0;

      Serial.print("Novo limite diurno: ");
      Serial.println(limiteDia, 2);
    }
    else {

      limiteNoite = valor / 100.0;

      Serial.print("Novo limite noturno: ");
      Serial.println(limiteNoite, 2);
    }

    menuConfig = 0;
    ultimoValorConfig = -1;

    telaPronto(
      "CONFIG. SALVA",
      "SISTEMA PRONTO"
    );

    return;
  }

  // Fora do menu, interrompe apenas
  // o som atual, sem desativar falhas.
  noTone(BUZZER);

  Serial.println("Aviso sonoro reconhecido");
}

// =====================================
// 11. LEITURA DOS BOTOES
// =====================================

void atualizarBotoes() {

  unsigned long agora = millis();

  bool leituraConfig = digitalRead(BOTAO_CONFIG);
  bool leituraOk = digitalRead(BOTAO_OK);

  // Debounce do CONFIG
  if (leituraConfig != configLeituraAnterior) {

    configMudou = agora;
    configLeituraAnterior = leituraConfig;
  }

  if (agora - configMudou >= DEBOUNCE &&
      leituraConfig != configEstavel) {

    configEstavel = leituraConfig;

    if (configEstavel == LOW) {
      botaoConfigPressionado();
    }
  }

  // Debounce do OK
  if (leituraOk != okLeituraAnterior) {

    okMudou = agora;
    okLeituraAnterior = leituraOk;
  }

  if (agora - okMudou >= DEBOUNCE &&
      leituraOk != okEstavel) {

    okEstavel = leituraOk;

    if (okEstavel == LOW) {
      botaoOkPressionado();
    }
  }

  if (menuConfig != 0) {
    telaConfiguracao();
  }
}

// =====================================
// 12. CALCULO DA VELOCIDADE
// =====================================

void calcularVelocidade(unsigned long fim) {

  unsigned long tempo = fim - inicioPassagem;

  if (tempo == 0) {
    registrarFalha("Tempo invalido");
    return;
  }

  float segundos = tempo / 1000000.0;

  float velocidade =
    (DISTANCIA_PONTOS / 100.0) / segundos;

  Serial.println();
  Serial.println("===== RESULTADO =====");

  Serial.print("Tempo: ");
  Serial.print(segundos, 3);
  Serial.println(" s");

  Serial.print("Velocidade: ");
  Serial.print(velocidade, 3);
  Serial.println(" m/s");

  if (velocidade < VELOCIDADE_MIN ||
      velocidade > VELOCIDADE_MAX) {

    registrarFalha("Velocidade invalida");
    return;
  }

  float limite =
    modoDaPassagem ? limiteNoite : limiteDia;

  Serial.print("Modo da passagem: ");
  Serial.println(
    modoDaPassagem ? "NOTURNO" : "DIURNO"
  );

  Serial.print("Limite: ");
  Serial.print(limite, 2);
  Serial.println(" m/s");

  totalVeiculos++;

  estado = RESULTADO;
  contagemResultadoAtiva = false;

  if (velocidade > limite) {

    totalExcessos++;

    sinalizarExcesso();
    telaResultado(velocidade, "STATUS: EXCESSO");

    Serial.println("CLASSIFICACAO: EXCESSO");
  }
  else if (velocidade >= limite * 0.8) {

    sinalizarAtencao();
    telaResultado(velocidade, "STATUS: ATENCAO");

    Serial.println("CLASSIFICACAO: ATENCAO");
  }
  else {

    sinalizarNormal();
    telaResultado(velocidade, "STATUS: NORMAL");

    Serial.println("CLASSIFICACAO: NORMAL");
  }

  Serial.print("Total de veiculos: ");
  Serial.println(totalVeiculos);

  Serial.print("Total de excessos: ");
  Serial.println(totalExcessos);
}

// =====================================
// 13. PROCESSAMENTO DAS DISTANCIAS
// =====================================

void processarDistancia(
  byte sensor,
  float distancia,
  unsigned long instante
) {

  if (distancia < 2 || distancia > 300) {
    return;
  }

  leituraValida[sensor] = true;
  contFalha[sensor] = 0;

  bool antes = presente[sensor];

  // Confirma veiculo com duas leituras
  if (distancia < DETECTAR) {

    contAusencia[sensor] = 0;

    if (contPresenca[sensor] < 2) {
      contPresenca[sensor]++;
    }

    if (contPresenca[sensor] >= 2) {
      presente[sensor] = true;
    }
  }

  // Confirma pista livre
  else if (distancia > LIBERAR) {

    contPresenca[sensor] = 0;

    if (contAusencia[sensor] < 2) {
      contAusencia[sensor]++;
    }

    if (contAusencia[sensor] >= 2) {
      presente[sensor] = false;
    }
  }

  bool novaDeteccao =
    !antes && presente[sensor];

  if (!novaDeteccao) {
    return;
  }

  // Impede medir durante configuracao,
  // falha ou mensagem temporaria.
  if (estado == INICIALIZANDO ||
      estado == FALHA ||
      mostrandoPronto ||
      menuConfig != 0) {
    return;
  }

  if (estado == AGUARDANDO && sensor == 0) {

    if (presente[1]) {

      registrarFalha("Sensor 2 ocupado");
      return;
    }

    inicioPassagem = instante;
    modoDaPassagem = modoNoturno;

    estado = MEDINDO;

    Serial.println();
    Serial.println("SENSOR 1: INICIO");

    telaMedicao();
  }

  else if (estado == MEDINDO &&
           sensor == 1) {

    Serial.println("SENSOR 2: FIM");

    calcularVelocidade(instante);
  }
}

// =====================================
// 14. TRATAMENTO DE ECO INVALIDO
// =====================================

void ecoInvalido() {

  byte s = sensorAtual;

  if (contFalha[s] < 255) {
    contFalha[s]++;
  }

  leituraValida[s] = false;

  contPresenca[s] = 0;
  contAusencia[s] = 0;

  if (contFalha[s] >= 3) {

    if (s == 0) {
      registrarFalha("HC-SR04 1 sem eco");
    }
    else {
      registrarFalha("HC-SR04 2 sem eco");
    }
  }
}

// =====================================
// 15. ALTERNANCIA DOS SENSORES
// =====================================

void proximoSensor(unsigned long agora) {

  ultimaMedicao = agora;

  sensorAtual = 1 - sensorAtual;

  faseSonar = SONAR_PRONTO;
}

// =====================================
// 16. LEITURA NAO BLOQUEANTE HC-SR04
// =====================================

void atualizarSonar() {

  unsigned long agora = micros();

  byte trig = trigPins[sensorAtual];
  byte echo = echoPins[sensorAtual];

  switch (faseSonar) {

    case SONAR_PRONTO:

      if (agora - ultimaMedicao >=
          INTERVALO_PING) {

        digitalWrite(trig, HIGH);

        inicioTrigger = agora;
        faseSonar = TRIGGER_ALTO;
      }

      break;

    case TRIGGER_ALTO:

      if (agora - inicioTrigger >= 12) {

        digitalWrite(trig, LOW);

        inicioEsperaEcho = agora;
        faseSonar = ESPERAR_ECHO_ALTO;
      }

      break;

    case ESPERAR_ECHO_ALTO:

      if (digitalRead(echo) == HIGH) {

        inicioEcho = agora;
        faseSonar = ESPERAR_ECHO_BAIXO;
      }
      else if (agora - inicioEsperaEcho >
               TIMEOUT_ECO) {

        ecoInvalido();
        proximoSensor(agora);
      }

      break;

    case ESPERAR_ECHO_BAIXO:

      if (digitalRead(echo) == LOW) {

        unsigned long duracao =
          agora - inicioEcho;

        float distancia =
          duracao * 0.0343 / 2.0;

        if (distancia >= 2 &&
            distancia <= 300) {

          processarDistancia(
            sensorAtual,
            distancia,
            agora
          );
        }
        else {

          ecoInvalido();
        }

        proximoSensor(agora);
      }
      else if (agora - inicioEcho >
               TIMEOUT_ECO) {

        ecoInvalido();
        proximoSensor(agora);
      }

      break;
  }
}

// =====================================
// 17. VERIFICACAO INICIAL
// =====================================

void verificarInicializacao() {

  if (estado != INICIALIZANDO) {
    return;
  }

  bool sensoresValidos =
    leituraValida[0] &&
    leituraValida[1] &&
    contFalha[0] == 0 &&
    contFalha[1] == 0;

  bool pistaLivre =
    !presente[0] &&
    !presente[1] &&
    contAusencia[0] >= 2 &&
    contAusencia[1] >= 2;

  if (sensoresValidos && pistaLivre) {

    estado = AGUARDANDO;

    sinalizarNormal();
    telaEspera();

    Serial.println();
    Serial.println("SENSORES VERIFICADOS");
    Serial.println("PISTA LIVRE");
    Serial.println("SISTEMA PRONTO");
  }
}

// =====================================
// 18. CONTROLE DOS ESTADOS
// =====================================

void atualizarControle() {

  unsigned long agoraMs = millis();
  unsigned long agoraUs = micros();

  // Passagem incompleta
  if (estado == MEDINDO) {

    if (agoraUs - inicioPassagem >
        TEMPO_MAXIMO) {

      registrarFalha("Passagem incompleta");
    }
  }

  // Exibicao do resultado
  if (estado == RESULTADO) {

    bool pistaLivre =
      leituraValida[0] &&
      leituraValida[1] &&
      contFalha[0] == 0 &&
      contFalha[1] == 0 &&
      !presente[0] &&
      !presente[1] &&
      contAusencia[0] >= 2 &&
      contAusencia[1] >= 2;

    if (pistaLivre) {

      if (!contagemResultadoAtiva) {

        contagemResultadoAtiva = true;
        inicioPistaLivre = agoraMs;

        Serial.println("PISTA LIVRE");
        Serial.println("Resultado por mais 3 s");
      }

      if (agoraMs - inicioPistaLivre >=
          TEMPO_RESULTADO) {

        estado = AGUARDANDO;
        contagemResultadoAtiva = false;

        sinalizarNormal();

        telaPronto(
          "NOVA PASSAGEM",
          "SISTEMA PRONTO"
        );

        Serial.println(
          "PRONTO PARA NOVA PASSAGEM"
        );
      }
    }
    else {

      contagemResultadoAtiva = false;
    }
  }

  // Recuperacao de falhas
  if (estado == FALHA) {

    bool tudoNormal =
      leituraValida[0] &&
      leituraValida[1] &&
      contFalha[0] == 0 &&
      contFalha[1] == 0 &&
      !presente[0] &&
      !presente[1] &&
      contAusencia[0] >= 2 &&
      contAusencia[1] >= 2;

    if (tudoNormal) {

      if (inicioRecuperacao == 0) {
        inicioRecuperacao = agoraMs;
      }

      if (agoraMs - inicioRecuperacao >=
          TEMPO_RECUPERACAO) {

        estado = AGUARDANDO;
        inicioRecuperacao = 0;

        sinalizarNormal();

        telaPronto(
          "FALHA RECUPERADA",
          "SISTEMA PRONTO"
        );

        Serial.println("FALHA RECUPERADA");
      }
    }
    else {

      inicioRecuperacao = 0;
    }
  }
}

// =====================================
// 19. SETUP
// =====================================

void setup() {

  Serial.begin(9600);

  // HC-SR04
  pinMode(TRIG1, OUTPUT);
  pinMode(TRIG2, OUTPUT);

  pinMode(ECHO1, INPUT);
  pinMode(ECHO2, INPUT);

  digitalWrite(TRIG1, LOW);
  digitalWrite(TRIG2, LOW);

  // Botoes
  pinMode(BOTAO_CONFIG, INPUT_PULLUP);
  pinMode(BOTAO_OK, INPUT_PULLUP);

  // LEDs
  pinMode(LED_VERDE, OUTPUT);
  pinMode(LED_AMARELO, OUTPUT);
  pinMode(LED_VERMELHO, OUTPUT);

  pinMode(BUZZER, OUTPUT);

  // Servo
  placa.attach(SERVO_PIN);

  // LCD
  Wire.begin();
  lcd.init();
  lcd.backlight();

  // Luminosidade inicial
  modoNoturno =
    analogRead(LDR_PIN) < 475;

  candidatoNoturno = modoNoturno;

  estado = INICIALIZANDO;

  apagarLeds();
  noTone(BUZZER);
  placa.write(0);

  telaInicializacao();

  // Primeira medicao sem atraso inicial
  ultimaMedicao =
    micros() - INTERVALO_PING;

  Serial.println();
  Serial.println("==========================");
  Serial.println("SPEEDGUARD V4.0");
  Serial.println("==========================");

  Serial.println("Verificando HC-SR04...");
  Serial.println("Limite dia: 0.12 m/s");
  Serial.println("Limite noite: 0.08 m/s");
}

// =====================================
// 20. LOOP PRINCIPAL
// =====================================

void loop() {

  atualizarSonar();

  atualizarLuz();

  atualizarBotoes();

  verificarInicializacao();

  atualizarControle();

  atualizarMensagem();
}

