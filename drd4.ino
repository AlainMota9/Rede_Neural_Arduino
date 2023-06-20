///Variáveis Método LDR
// Nos de entrada
#define x1 A2 //LDR DIREITA
int val1 = 0;
int valRef1 = 0;
#define x2 A3 //LDR ESQUERDA
int val2 = 0;
int valRef2 = 0;

//Dados neurais do motor dianteiro
float wDi[12]; //Pesos Sinapticos
float thEscDi[3]; //threshold da camada escondida
float thSadDi[2]; //threshold da camada de saida
int y4Di; //direcao
float y5Di; //velocidade
float avaliacaoCicloDi;
int pesosEntradaDi = 0; //controle de atualização
int thresholdEscondidaDi = 0;
int pesosEscondidoDi = 6;
int thresholdSaidaDi = 0;

//Dados neurais do motor traseiro
float wTr[12]; //Pesos Sinapticos
float thEscTr[3]; //threshold da camada escondida
float thSadTr[2]; //threshold da camada de saida
int y4Tr; //direcao
float y5Tr; //velocidade
float avaliacaoCicloTr;
int pesosEntradaTr = 0; //controle de atualização
int thresholdEscondidaTr = 0;
int pesosEscondidoTr = 6;
int thresholdSaidaTr = 0;

//Variavel da funçao redeneural
const float e = 2.72;

//Varíavel Método Temperatura
#define LM35 A0

//Variáveis Metodo Movimento
#define direcao_Dianteiro 7
#define velocidade_Dianteiro 6
#define direcao_Traseiro 8
#define velocidade_Traseiro 9
#define direcao_Direito 4
#define velocidade_Direito 5
#define direcao_Esquerdo 12
#define velocidade_Esquerdo 10
unsigned long tempoMotor = 0;
unsigned long tempoMotorDelay = 50;

//Variveis movimento motor esquerdo e direito
int contadorEsquerdo = 0;
int contadorDireito = 0;


//Variáveis de Indicador de giro
#define botaoEsquerdo 13
int valorEsquerdo = 1;
#define botaoDireito 2
int valorDireito = 1;
#define botaoDianteiro 3
int valorDianteiro = 1;
#define sensorTraseiro A1
int valorTraseiroInicial = 0;
int valorTraseiroFinal = 0;
bool movimento = false;

//Gerador de numeros aleatorios decimais
float randomFloat (float minimo, float maximo) {
  return minimo + random(1UL << 31) * (maximo - minimo) / (1UL << 31);
}


void redeNeuralDi(float x1, float x2) {

  //Calculo da matriz produto
  int indice1 = 0;
  float x[2] = {x1, x2};
  float produto1[3][2]; //Matriz Produto
  for (int j = 0; j < 2; j++) { //Percorrendo as colunas
    for (int i = 0; i < 3; i++) { //Percorrendo as linhas
      produto1[i][j] = wDi[indice1] * x[j];
      indice1++;
    }
  }

  //Calculo do Potencial de ativaçao
  float a[3];
  for (int i = 0; i < 3; i++) { //Percorrendo a matriz de potencial
    a[i] = (produto1[i][0] + produto1[i][1]) - thEscDi[i];
  }

  //Função de Ativação Sigmoide
  float f1[3];
  for (int i = 0; i < 3; i++) {
    float sig = pow(e, a[i]);
    f1[i] = sig / (sig + 1);
  }

  //Condiçao de ativaçao
  float y1[3]; //resultado da camada escondida
  for (int i = 0; i < 3; i++) {
    if (f1[i] > thEscDi[i]) {
      y1[i] = 1;
    } else {
      y1[i] = 0;
    }
  }

  //Calculo da matriz produto
  int indice2 = 6;
  float produto2[2][3]; //Matriz Produto
  for (int j = 0; j < 3; j++) { //Percorrendo as colunas
    for (int i = 0; i < 2; i++) { //Percorrendo as linhas
      produto2[i][j] = wDi[indice2] * y1[j];
      indice2++;
    }
  }

  //Calculo do Potencial de ativaçao
  float b[2];
  for (int i = 0; i < 2; i++) { //Percorrendo a matriz de potencial
    b[i] = (produto2[i][0] + produto2[i][1] + produto2[i][2]) - thSadDi[i];
  }

  //Função de Ativação Sigmoide
  float f2[2];
  for (int i = 0; i < 2; i++) {
    float sig = pow(e, b[i]);
    f2[i] = sig / (sig + 1);
  }

  //Condiçao de ativaçao
  float y2[2]; //resultado da camada de saida
  if (f2[0] > thSadDi[0]) {//análise do valor para direção
    y2[0] = 1;
  } else {
    y2[0] = 0;
  }
  if (f2[1] > thSadDi[1]) {//análise do valor para velocidade
    y2[1] = map(f2[1], -1, 1, 0, 255); //Mapea os números para o intervalo do PWM
  }


  //armazenamento do resultado
  y4Di = int(y2[0]); //direcao
  y5Di = y2[1]; //velocidade

  //Avaliaçao do final do ciclo
  float fx2 = 2 * x[1];
  float gx2 = x[1] / 2;
  float avaliacaoCicloAnterior = avaliacaoCicloDi;
  if (x[0] < fx2 && x[0] > gx2) {
    avaliacaoCicloDi = max(x[1], x[0]) - min(x[1], x[0]);;
  }

  //Correção do ciclo neural
  //caso o valor do ciclo atual for maior que anterior a correção é necessária
  if (avaliacaoCicloAnterior < avaliacaoCicloDi) {
    randomSeed(analogRead(A5));
    //Correção dos pesos de entrada
    if (pesosEntradaDi < 6) {
      wDi[pesosEntradaDi] = randomFloat(-1, 1);
      pesosEntradaDi++;
      wDi[pesosEntradaDi] = randomFloat(-1, 1);
      pesosEntradaDi++;
    } else {
      pesosEntradaDi = 0;
    }
    //correção dos threshold da camda escondida
    if (thresholdEscondidaDi < 3) {
      thEscDi[thresholdEscondidaDi] = randomFloat(-1, 1);
      thresholdEscondidaDi++;
    } else {
      thresholdEscondidaDi = 0;
    }
    //correção dos pesos da camada escondida
    if (pesosEscondidoDi < 12) {
      wDi[pesosEscondidoDi] = randomFloat(-1, 1);
      pesosEscondidoDi++;
      wDi[pesosEscondidoDi] = randomFloat(-1, 1);
      pesosEscondidoDi++;
    } else {
      pesosEscondidoDi = 6;
    }
    //correção dos threshold da camada de saída
    if (thresholdSaidaDi < 2) {
      thSadDi[thresholdSaidaDi] = randomFloat(-1, 1);
      thresholdSaidaDi++;
    } else {
      thresholdSaidaDi = 0;
    }
  }

}//fim da rede neural do motor dianteiro

void redeNeuralTr(float x1, float x2) {

  //Calculo da matriz produto
  int indice1 = 0;
  float x[2] = {x1, x2};
  float produto1[3][2]; //Matriz Produto
  for (int j = 0; j < 2; j++) { //Percorrendo as colunas
    for (int i = 0; i < 3; i++) { //Percorrendo as linhas
      produto1[i][j] = wTr[indice1] * x[j];
      indice1++;
    }
  }

  //Calculo do Potencial de ativaçao
  float a[3];
  for (int i = 0; i < 3; i++) { //Percorrendo a matriz de potencial
    a[i] = (produto1[i][0] + produto1[i][1]) - thEscTr[i];
  }

  //Função de Ativação Sigmoide
  float f1[3];
  for (int i = 0; i < 3; i++) {
    float sig = pow(e, a[i]);
    f1[i] = sig / (sig + 1);
  }

  //Condiçao de ativaçao
  float y1[3]; //resultado da camada escondida
  for (int i = 0; i < 3; i++) {
    if (f1[i] > thEscTr[i]) {
      y1[i] = 1;
    } else {
      y1[i] = 0;
    }
  }

  //Calculo da matriz produto
  int indice2 = 6;
  float produto2[2][3]; //Matriz Produto
  for (int j = 0; j < 3; j++) { //Percorrendo as colunas
    for (int i = 0; i < 2; i++) { //Percorrendo as linhas
      produto2[i][j] = wTr[indice2] * y1[j];
      indice2++;
    }
  }

  //Calculo do Potencial de ativaçao
  float b[2];
  for (int i = 0; i < 2; i++) { //Percorrendo a matriz de potencial
    b[i] = (produto2[i][0] + produto2[i][1] + produto2[i][2]) - thSadTr[i];
  }

  //Função de Ativação Sigmoide
  float f2[2];
  for (int i = 0; i < 2; i++) {
    float sig = pow(e, b[i]);
    f2[i] = sig / (sig + 1);
  }

  //Condiçao de ativaçao
  float y2[2]; //resultado da camada de saida
  if (f2[0] > thSadTr[0]) {//análise do valor para direção
    y2[0] = 1;
  } else {
    y2[0] = 0;
  }
  if (f2[1] > thSadTr[1]) {//análise do valor para velocidade
    y2[1] = map(f2[1], -1, 1, 0, 255); //Mapea os números para o intervalo do PWM
  }

  //armazenamento do resultado
  y4Tr = int(y2[0]); //direcao
  y5Tr = y2[1]; //velocidade

  //Avaliaçao do final do ciclo
  float fx2 = 2 * x[1];
  float gx2 = x[1] / 2;
  float avaliacaoCicloAnterior = avaliacaoCicloTr;
  if (x[0] < fx2 && x[0] > gx2) {
    avaliacaoCicloTr = max(x[1], x[0]) - min(x[1], x[0]);
  }

  //Correção do ciclo neural
  //caso o valor do ciclo atual for maior que anterior a correção é necessária
  if (avaliacaoCicloAnterior < avaliacaoCicloTr) {
    randomSeed(analogRead(A5));
    //Correção dos pesos de entrada
    if (pesosEntradaTr < 6) {
      wTr[pesosEntradaTr] = randomFloat(-1, 1);
      pesosEntradaTr++;
      wTr[pesosEntradaTr] = randomFloat(-1, 1);
      pesosEntradaTr++;
    } else {
      pesosEntradaTr = 0;
    }
    //correção dos threshold da camda escondida
    if (thresholdEscondidaTr < 3) {
      thEscTr[thresholdEscondidaTr] = randomFloat(-1, 1);
      thresholdEscondidaTr++;
    } else {
      thresholdEscondidaTr = 0;
    }
    //correção dos pesos da camada escondida
    if (pesosEscondidoTr < 12) {
      wTr[pesosEscondidoTr] = randomFloat(-1, 1);
      pesosEscondidoTr++;
      wTr[pesosEscondidoTr] = randomFloat(-1, 1);
      pesosEscondidoTr++;
    } else {
      pesosEscondidoTr = 6;
    }
    //correção dos threshold da camada de saída
    if (thresholdSaidaTr < 2) {
      thSadTr[thresholdSaidaTr] = randomFloat(-1, 1);
      thresholdSaidaTr++;
    } else {
      thresholdSaidaTr = 0;
    }
  }


}//fim da rede neural motor traseiro

float temperaturaL293D () {
  float temperatura = (float(analogRead(LM35)) * 5 / (1023)) / 0.023;
  return temperatura;

}//fim de temperaturaL293D

int LDR1 () {
  val1 = analogRead(x1);
  if (val1 != valRef1 && val1 != (valRef1 + 1) && val1 != (valRef1 - 1) && val1 != (valRef1 + 2) && val1 != (valRef1 - 2)) {
    valRef1 = val1;
  }//fim de if
  return valRef1;
}//fim de LDR1

int LDR2 () {
  val2 = analogRead(x2);
  if (val2 != valRef2 && val2 != (valRef2 + 1) && val2 != (valRef2 - 1) && val2 != (valRef2 + 2) && val2 != (valRef2 - 2)) {
    valRef2 = val2;
  }//fim de if
  return valRef2;
}//fim de LDR2

void motorDianteiro ( int direcao, int velocidade) {
  valorDianteiro = digitalRead(botaoDianteiro);
  if (temperaturaL293D () > 85) {
    analogWrite(velocidade_Dianteiro, 0);
  } else if (valorDianteiro = 1) { //quando o valor for zero o botao esta pressionado
    digitalWrite(direcao_Dianteiro, direcao);
    analogWrite(velocidade_Dianteiro, velocidade);
  } else {
    analogWrite(velocidade_Dianteiro, 0);
  }
}//fim de motorDianteiro

void motorTraseiro (int direcao, int velocidade) {
  digitalWrite(direcao_Traseiro, direcao);
  analogWrite(velocidade_Traseiro, velocidade);

  tempoMotor = millis();
  do {
    if ((millis() - tempoMotor) > tempoMotorDelay) {
      valorTraseiroInicial = analogRead(sensorTraseiro);
    }
  } while ((millis() - tempoMotor) < tempoMotorDelay); //fim de while

  do {
    if ((millis() - tempoMotor) > tempoMotorDelay) {
      valorTraseiroFinal = analogRead(sensorTraseiro);
    }
  } while ((millis() - tempoMotor) < tempoMotorDelay); //fim de while

  if ((valorTraseiroInicial < 60 && valorTraseiroFinal > 100) || (valorTraseiroInicial > 100 && valorTraseiroFinal < 60)) {
    movimento = true;
  } else {
    movimento = false;
  }

  if ( movimento == false || (temperaturaL293D () > 85)) {
    analogWrite(velocidade_Traseiro, 0);
  }
}//fim de motorTraseiro

void motorDireito ( int direcao, int velocidade) {
  valorDireito = digitalRead(botaoDireito);
  if (valorDireito = 1) {
    digitalWrite(direcao_Direito, direcao);
    analogWrite(velocidade_Direito, velocidade);
  } else {
    analogWrite(velocidade_Direito, 0);
  }
}//fim de motorDireito

void motorEsquerdo ( int direcao, int velocidade) {
  valorEsquerdo = digitalRead(botaoEsquerdo);
  if (valorEsquerdo = 1) {
    digitalWrite(direcao_Esquerdo, direcao);
    analogWrite(velocidade_Esquerdo, velocidade);
  } else {
    analogWrite(velocidade_Esquerdo, 0);
  }
}//fim de motorEsquerdo

void setup() {
  //Comunicação Serial
  Serial.begin(9600); // abre a porta serial, configura a taxa de transferência para 9600 bps

  //Para Movimento
  pinMode(direcao_Dianteiro, OUTPUT);
  pinMode(velocidade_Dianteiro, OUTPUT);
  pinMode(direcao_Traseiro, OUTPUT);
  pinMode(velocidade_Traseiro, OUTPUT);
  pinMode(direcao_Direito, OUTPUT);
  pinMode(velocidade_Direito, OUTPUT);
  pinMode(direcao_Esquerdo, OUTPUT);
  pinMode(velocidade_Esquerdo, OUTPUT);

  //Para Indicador de giro
  pinMode(botaoEsquerdo, INPUT_PULLUP);
  pinMode(botaoDianteiro, INPUT_PULLUP);
  pinMode(botaoDireito, INPUT_PULLUP);

  //Para iniciar aleatoriamento o valor dos pesos
  for (int p = 0; p < 12; p++) {
    randomSeed(analogRead(A5));
    wDi[p] = randomFloat(-1, 1);
    wTr[p] = randomFloat(-1, 1);
  }
  //Para iniciar aleatoriamento o valor do threshold da camada escondida
  for (int p = 0; p < 3; p++) {
    randomSeed(analogRead(A5));
    thEscDi[p] = randomFloat(-1, 1);
    thEscTr[p] = randomFloat(-1, 1);
  }
  //Para iniciar aleatoriamento o valor do threshold da camada de saida
  for (int p = 0; p < 2; p++) {
    randomSeed(analogRead(A5));
    thSadDi[p] = randomFloat(-1, 1);
    thSadTr[p] = randomFloat(-1, 1);
  }


}//fim setup

void loop() {

  //Faz a varedura da area com o LDR esquerdo
  if (contadorEsquerdo = 0) {
    motorEsquerdo(contadorEsquerdo, 100);
    contadorEsquerdo++;
  } else {
    motorEsquerdo(contadorEsquerdo, 100);
    contadorEsquerdo = 0;
  }

  //Faz a varredura da area com o LDR direito
  if (contadorDireito = 0) {
    motorDireito(contadorDireito, 100);
    contadorDireito++;
  } else {
    motorDireito(contadorDireito, 100);
    contadorDireito = 0;
  }

  redeNeuralDi(LDR1 (), LDR2 ());
  motorDianteiro (y4Di, y5Di);

  redeNeuralTr(LDR1 (), LDR2 ());
  motorTraseiro (y4Tr, y5Tr);


}//fim de loop
