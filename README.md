# pasteurization.machine
// ----------------------------------------------------------------------------------------------
//_________________________________PINAGEM DOS DISPOSITIVOS__________________________________

#define BOTAOUP A3    // botao de aumento do Valor de PU DESEJADO
#define BOTAODOWN A2  // botao de diminuição do Valor de PU DESEJADO
#define BOTAOMENU A1  // botao de menu
#define BOTAOSTART 9  // botao de iniciar

#define buzzer 2

#define rele_resistencia 5 // 


// ----------------------------------------------------------------------------------------------
//____________________________________DISPLAY LCD__________________________________
#include <LiquidCrystal_I2C.h>  // Biblioteca i2c que tem as funções para controlar o lcd
#include <Wire.h>               // Biblioteca que a bilitoeca i2c usa pra se comunicar

LiquidCrystal_I2C lcd(0x27, 20, 4);  // Criando um objeto do tipo liquid crytal (endereço,colunas, linhas)

// ----------------------------------------------------------------------------------------------
//____________________________________SENSOR DE TEMPERATURA DS18B20__________________________________
#include <OneWire.h>            // Biblioteca OneWire para sensores de um fio de transmissão
#include <DallasTemperature.h>  // Biblioteca do sensor da Dallas

#define PINO_COMUNICAO 8          // pino de dados
OneWire oneWire(PINO_COMUNICAO);  // objeto da classe OneWire, responsável pro representar o dispostivo que esta se comunicando por onewire

DallasTemperature sensors(&oneWire);  // objeto da classe Dallas, responsável pro representar o dispositivo de temperatura da dallas, ele inicializa com o onewire para dizer qual dispositivo da dallas esta senso usado por meio do endereço

float temperatura;  // variável que armazena temperatura do DS18B20


// ----------------------------------------------------------------------------------------------
//____________________________________CONTROLADOR ONOFF__________________________________________________
            

double temperatura_setpoint = 88;      


// ----------------------------------------------------------------------------------------------
//_________________________________VARIÁVEIS DE PU GLOBAIS DO SISTEMA__________________________________
float PU_setpoint = 70000
; 
float PU_total= 0;
float PUraw = 0;

// ----------------------------------------------------------------------------------------------
//_________________________________VARIÁVEIS DE CONTROLE GLOBAIS DO SISTEMA__________________________________

int iniciar = 0;
int tempo_pasteurizacao = 0;
int modo = 0, menu = 0;


void setup() 
{
    //_________________________________TIMER 1__________________________________
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << CS10) | (1 << CS12);
  TCNT1 = 0xC2F7;
  //_________________________________PINOS__________________________________
  pinMode(BOTAOUP, INPUT_PULLUP);
  pinMode(BOTAODOWN, INPUT_PULLUP);
  pinMode(BOTAOMENU, INPUT_PULLUP);
  pinMode(BOTAOSTART, INPUT_PULLUP);
  pinMode(buzzer, OUTPUT);
  pinMode(rele_resistencia,OUTPUT);


  //_________________INICIALIZAÇÃO DAS BIBLIOTECAS_______________________
  lcd.init(); // LCD
  sensors.begin(); // DS18B20
  Serial.begin(9600);
  
  //_________________TELA DE INICIO_______________________________________
  lcd.setBacklight(HIGH);
  lcd.setCursor(2,1); lcd.print("Empresa SUNTECH");
  delay(2000);

  lcd.clear();
  lcd.setCursor(0,1); lcd.print("Universidade Federal");
  lcd.setCursor(5,2); lcd.print("da Paraiba");
  delay(2000);


  lcd.clear();
  lcd.setCursor(5,1); lcd.print("Iniciando");
  lcd.setCursor(3,2); lcd.print("Pasteurizadora");
  delay(2000);

  lcd.clear();
  lcd.setCursor(1,1); lcd.print("Esperando Comando");

  digitalWrite(rele_resistencia,0);
}

void loop() 
{
  //________________________________________Botao de Start______________________________________
  if(digitalRead(BOTAOSTART)==0)
  {
    iniciar++;
    if(iniciar > 1){ // Se o botão de começar for apertado mais de uma vez
      parar_pasteurizacao();
    }
    if(iniciar == 1) // Ele irá iniciar o processo pasteurização
    {
      lcd.clear();
      lcd.setCursor(5,1); lcd.print("Iniciando"); 
      lcd.setCursor(3,2); lcd.print("Pasteurizacao");/* Printa "A temperatura é:" */
      delay(2000);
      IMPRIMI_PU();
      TIMSK1 |= (1 << TOIE1); // habilita interrupção
      digitalWrite(rele_resistencia,HIGH);
    }
  }
  //________________________________________Botao de Menu______________________________________
  if(digitalRead(BOTAOMENU) == 0)
  {
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("       MODO 1       ");
    lcd.setCursor(0,1); lcd.print("  Ajustar Setpoint  ");
    delay(2000);

    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Aperte e Segure");
    lcd.setCursor(0,1); lcd.print("para acrescentar ou");
    lcd.setCursor(0,2); lcd.print("diminuir o PU");   
    menu++;
    PU_setpoint = mudar_parametro_setpoint(PU_setpoint);
    IMPRIMI_PU();
    delay(2000);

    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Aperte e Segure");
    lcd.setCursor(0,1); lcd.print("para acrescentar ou");
    lcd.setCursor(0,2); lcd.print("diminuir a temp");
    menu++;
    temperatura_setpoint = mudar_parametro_setpoint(temperatura_setpoint);
    IMPRIMI_PU();
    delay(2000);

    menu = 0;
    lcd.clear();
    lcd.setCursor(1,1); lcd.print("Esperando Comando");
  }
}

ISR(TIMER1_OVF_vect) // Função de interrupção para temporizar cada processo de integralização do PU a cada 1 segundo
{
  if(iniciar == 1)
  {
    tempo_pasteurizacao++;
    TCNT1 = 0xC2F7;
    Calculo_PU_total();
    ONOFF_CONTROLE();
    ALARME();
    parametros_durante_pasteurizacao();
    if(PU_total >= PU_setpoint) // Para o processo de pasteurização
    {
      parar_pasteurizacao();
      noTone(buzzer);
    }
  } 
}

void parar_pasteurizacao()
{
  TIMSK1 &= ~(1 << TOIE1); // Ele irá parar o processo de pasteurização
    // reinicia as variáveis
  digitalWrite(rele_resistencia,LOW);
  PRINTAR_RESULTADO(); // Mostrara os valores até o momento que foi
  iniciar = 0;
  tempo_pasteurizacao = 0;
  PU_total= 0;
  PUraw = 0;
  
   
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Esperando Comando");
}

float aumentar_parametro_setpoint(String nome_parametro_setpoint, float valor_parametro_setpoint)
{
  while(digitalRead(BOTAOUP) == 0){
    lcd.clear();
    lcd.setCursor(0,0);
    valor_parametro_setpoint++;
    
    lcd.print(nome_parametro_setpoint); lcd.print(valor_parametro_setpoint);
    delay(200);
  }
  return valor_parametro_setpoint;
}

float diminuir_parametro_setpoint(String nome_parametro_setpoint, float valor_parametro_setpoint)
{
  while(digitalRead(BOTAODOWN) == 0){
    lcd.clear();
    lcd.setCursor(0,0);
    valor_parametro_setpoint--;
    if(valor_parametro_setpoint < 0)
      valor_parametro_setpoint = 0;

    lcd.print(nome_parametro_setpoint); lcd.print(valor_parametro_setpoint);
    delay(200);
  }  
  return valor_parametro_setpoint;
}

float mudar_parametro_setpoint(float valor_parametro_setpoint)
{
  String nome_parametro_setpoint;

  if(menu == 1) nome_parametro_setpoint = "PU Setpoint: ";
  else nome_parametro_setpoint = "Temp Setpoint: ";

  while(true){ 
    if(digitalRead(BOTAOMENU) == 0) break;
    
    valor_parametro_setpoint = aumentar_parametro_setpoint(nome_parametro_setpoint, valor_parametro_setpoint);
    
    valor_parametro_setpoint = diminuir_parametro_setpoint(nome_parametro_setpoint, valor_parametro_setpoint);
  }
  return valor_parametro_setpoint;
}

void parametros_durante_pasteurizacao()
{
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Temp Atual:"); lcd.print(temperatura);
  lcd.setCursor(0,1); lcd.print("PU atual: "); lcd.print(PU_total,3); // era 5 mudei para 3
}

void Calculo_PU_total() // Calculo do PU acumulado
{
  sensors.requestTemperatures();
  temperatura = sensors.getTempCByIndex(0);
  adicionar_ao_buffer(temperatura);
  temperatura = calcular_media_buffer();

  float formula_PU = pow(1.393, temperatura - 60);   // 1.393^(temperatura - 60) * tempo(minutos)
  PUraw = (formula_PU)/60;        // PU crua. Naquele momento.Divide por 60 para transformar em minutos
  
  PU_total += PUraw;  // PU = formula_PU * tempo(segundos); 

}

void IMPRIMI_PU() // Mostra os valores dos Setpoints das variáveis
{
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("PU Desejado: "); lcd.print(PU_setpoint);

  lcd.setCursor(0,2); lcd.print("Temp Desejada: "); lcd.print(temperatura_setpoint);
  delay(2000);
}

void PRINTAR_RESULTADO() // Mostra o valor final do processo de pasteurização
{
  lcd.clear(); // limpa
  lcd.setCursor(0, 0); lcd.print("PU Total: ");  lcd.print(PU_total, 5); // Escreve o PU TOTAL

  lcd.setCursor(0, 1); lcd.print("Temp Atual: "); lcd.print(temperatura);// Escreve temperatura atual

  tempo_pasteurizacao = (int)tempo_pasteurizacao/60;

  lcd.setCursor(0, 2); lcd.print("Tempo Gasto: "); lcd.print(tempo_pasteurizacao); lcd.print(" min"); // Escreve temperatura atual

  delay(5000);
}



void ALARME()
{
  float porcentagem_PU = (PU_total/PU_setpoint) * 100;
  // Serial.print("Porcentagem_PU: "); Serial.println(porcentagem_PU);
  if(porcentagem_PU >= 80 && porcentagem_PU < 100) {
    tone(buzzer, 1800);
  }
  if(porcentagem_PU >= 100 && porcentagem_PU < 120) {
    tone(buzzer, 1200);
  }
  if(porcentagem_PU >= 120) {
    tone(buzzer, 300);
  }
}

void ONOFF_CONTROLE()
{
  if (temperatura < temperatura_setpoint - 0.2 )
    digitalWrite(rele_resistencia,HIGH);
  else
    digitalWrite(rele_resistencia,LOW);
}

byte posicao_leitura = 0; // Definir variável para controlar a posição de leitura do buffer circular, usando byte por que ocupa menos memória(0 a 255)
byte posicao_escrita = 0; // Definir variável para controlar a posição de escrita do buffer circular
float buffer_media = 0; // variavel para armazenar media do buffer

#define TAMANHO_BUFFER 5
float buffer[TAMANHO_BUFFER];
void adicionar_ao_buffer(float valor) {
  buffer[posicao_escrita] = valor; // Adicionar o valor na posição de escrita do buffer circular
  posicao_escrita++;
  if (posicao_escrita == TAMANHO_BUFFER) { // Incrementar a posição de escrita com tratamento de overflow
    posicao_escrita = 0;
  } 
}
float calcular_media_buffer() {
  float soma = 0.0; // Inicializar a soma dos valores com zero
  for (int i = 0; i < TAMANHO_BUFFER; i++) { 
    soma += buffer[i]; // 
  }
  return soma / TAMANHO_BUFFER; //Dividir a soma pelo número de valores armazenados para calcular a média
}
