/* PROGRAMA DE CONTROLE, VERSÃO 10. by IGOR ESTEVES 2020.07.22

INPUTS:
 
 *  LEITURA DO SENSOR DE PH
 *  LEITURA DE LUMINOSIDADE LDR
 *  LEITURA DE TEMPERATURA NOVUS
 *  LEITURA DE UMIDADE NOVUS
 *  LEITURA DO NÍVEL DO TANQUE ULTRASONICO
 *  LEITURA DA ALTURA DAS PLANTAS ULTRASONICO

OUTPUTS:

 *  FAN EXAUSTÃO
 *  FAN LADO QUENTE DIR
 *  FAN LADO QUENTE ESQ
 *  LED DA ESTUFA
 *  LED DA AQUAPONIA
 *  BOMBA DE CIRCULAÇÃO
 *  BOMBA PH+
 *  BOMBA PH-  
*/

/* Bibliotecas utilizadas */

#include <SPI.h>
#include <Ethernet.h>
#include "Mudbus.h"
#include <Ultrasonic.h>
#include <Servo.h>

/* Declaração dos pinos */

#define LED_ETF1 2 // Fita de LED da estufa
#define PUMP_PH1 4 // Bomba peristáltica PH+
#define PUMP_PH2 5 // Bomba peristáltica PH-
#define FISH_FEEDER 6 // PWM do servo fish feeder
#define FAN_HOT 7 // Fan HOT DIR
//#define TRIG_AQP 8 // Trigger do ultrasonico da aquaponia
//#define ECHO_AQP 9 // Echo do ultrasonico da aquaponia
//#define TRIG_ETF 10 // Trigger ultrasonico da estufa
//#define ECHO_ETF 11 // Echo ultrasonico da estufa
#define FAN_HOT1 12 // Fan HOT ESQ
#define LED_ETF 22 // LED da estufa
#define PELT_DIR 26 // Peltier lado direito
#define PELT_ESQ 30 // Peltier lado esquerdo
#define FAN_EXT 34 // FAN EXAUSTÃO
#define LED_AQP 38 // LED da Aquaponia
#define PUMP 42 // Bomba de circulação
#define DIR_PH2 46 // Bomba de circulação
#define DIR_PH1 48 // Bomba de circulação
#define ENB_PH 50 // Habilita a ligação das bombas de PH 

/* Declaração das variáveis */

// variáveis do sensor NOVUS
int measure2 = 0;
float alpha2 = 0.1;
float N_temp = 0.0;

int measure3 = 0;
float alpha3 = 0.1;
float N_hum = 0.0;

// Variáveis dos sensores ultrasonicos
long microsec_AQP = 0;
float distanciaCM_AQP = 0;
long microsec_ETF = 0;
float distanciaCM_ETF = 0;
int alt_cultura = 0;
int h_reservatorio = 0;
float volume_aqp = 0;

// Variáveis do sensor de PH
int measure = 0;
float Po = 0;
float alpha = 0.05;
const int bufferSize = 10;
int buffer[bufferSize];
int index;
int lastread = 0;
int lastread1 = 0;
int lastRead2 = 0;

// Variáveis para controle de pH
float Po_convertido = 0.0; // Valor do sensor convertido para pH
float setPoint_PH = 0.00;  // Recebe setpoint do indusoft
float setPoint_PHf = 0.00; // Variável para converter o valor int do indu pra float
float erro_PH = 0.0;
float kP_PH = 0.0;
float kI_PH = 0.0;
float I_PH = 0.0;
float PI_PH = 0.0;
float pulsos_PH1 = 0.0;    // Relação entre número de pulsos e aumento no pH
int bomba_PH1 = 0;
float pulsos_PH2 = 0.0;    // Relação entre número de pulsos e decremento no pH
int bomba_PH2 = 0;
int hora_1 = 0;            // Hora que o controle de será executado primeira vez no dia
int minu_1 = 0;            // Minuto em que o controle de será executado primeira vez
int hora_2 = 0;            // Hora que o controle de será executado segunda vez no dia
int minu_2 = 0;            // Minuto em que o controle de será executado segunda vez
int status_PH = 0;


int hora_teste = 16;

// Variáveis do sensor de luminosidade
int measure1 = 0;
float alpha1 = 0.1;
float Luminosidade = 0;
int iluminacao = 0;
int setPoint_ldr = 0;
int erro_ldr = 0;
float P_ldr = 0.0;
int PWM_LED = 0;
int PWM_LED1 = 0;
int PWM1_LED1 = 0;
float kP_ldr = 1.0;
int LAST_PWM = 0;
int lum_etf = 0;
int L_min = 0;
int L_max = 0;
int CALIB_MIN = 0;

// Variáveis do FISH FEEDER
int pos1 = 0; // Posição inicial do servo

// Variáveis do FAN LADO QUENTE
int PWM_FAN_HOT = 0;

// Variáveis das bombas de PH
int x1 = 0;
int x2 = 0;

// Variáveis gerais
int cont = 0;
int pulse = 0;
int LED_estufa = 0;
int LED_AQP_STT = 0;
int FAN_ETF = 0;
int PUMP_AQP = 0;
int FISH = 0;
int FAN_HOT_STT = 0;
int PELT_STT = 0;

// Variaveis da função millis
unsigned long int tempo_atual = 0;
unsigned long ut_ph = 0;
unsigned long ut_ldr = 0;
unsigned long ut_ntemp = 0;
unsigned long ut_nhum = 0;
unsigned long ut_volaqp = 0;
unsigned long ut_altcult = 0;
unsigned long ultimo_tempo = 0;
unsigned long ut_calib = 0;


              /* Instanciamento dos objetos */
              

Ultrasonic ultrasonic_AQP(8,9); // SENSOR ULTRASONICO DA AQUAPONIA

Ultrasonic ultrasonic_ETF(10,11); // SENSOR ULTRASONICO DA ESTUFA

Servo servo1; // Variável SERVO

Mudbus Mb;
//Function codes 1(read coils), 3(read registers), 5(write coil), 6(write register)
//signed int Mb.R[0 to 125] and bool Mb.C[0 to 128] MB_N_R MB_N_C
//Port 502 (defined in Mudbus.h) MB_PORT


void setup()
{
  uint8_t mac[]     = { 0x90, 0xA2, 0xDA, 0x00, 0x51, 0x06 };
  uint8_t ip[]      = { 192, 168, 0, 234 };
  uint8_t gateway[] = { 192, 168, 0, 1 };
  uint8_t subnet[]  = { 255, 255, 255, 0 };
  Ethernet.begin(mac, ip, gateway, subnet);
  //Avoid pins 4,10,11,12,13 when using ethernet shield

  delay(3000);
  Serial.begin(115200);
  Serial.println("CONTROLE AUTOMÁTICO - ON");
  
  servo1.attach (6); // Declara o pino do PWM do servo
  

  pinMode(FISH_FEEDER, OUTPUT);
  pinMode(FAN_HOT, OUTPUT);
  pinMode(LED_ETF, OUTPUT);
  pinMode(FAN_EXT, OUTPUT);
  pinMode(LED_AQP, OUTPUT);
  pinMode(PUMP, OUTPUT);
  pinMode(FAN_HOT1, OUTPUT);
  pinMode(PELT_DIR, OUTPUT);
  pinMode(PELT_ESQ, OUTPUT);
  pinMode(PUMP_PH1, OUTPUT);
  pinMode(PUMP_PH2, OUTPUT);
  pinMode(DIR_PH2, OUTPUT);
  pinMode(DIR_PH1, OUTPUT);
  pinMode(ENB_PH, OUTPUT);
  pinMode(LED_ETF1, OUTPUT);
 

  digitalWrite (FISH_FEEDER, LOW);
  digitalWrite (FAN_HOT, LOW);
  digitalWrite (LED_ETF, HIGH);
  digitalWrite (LED_ETF1, LOW);
  digitalWrite (FAN_EXT, HIGH);
  digitalWrite (LED_AQP, HIGH);
  digitalWrite (PUMP, HIGH);
  digitalWrite (FAN_HOT1, LOW);
  digitalWrite (PELT_DIR, HIGH);
  digitalWrite (PELT_ESQ, HIGH);
  digitalWrite (ENB_PH, LOW);
  
  servo1.write(83); // Comando para travar o motor do servo na posição atual

  // Variaveis para a função millis

  // Variavel utilizada para armazenar o tempo de exeucção atual do código
  unsigned long int tempo_atual = 0; 

  // Variável utilizada para armazenar o ultimo valor armazenado na variável tempo atual
  unsigned long ultimo_tempo = 0;
  
}

void loop()
{
  Mb.Run();

    /* Receive values from PLC */
    
    Mb.R[0]; // Aciona os PELTIER
    //Mb.R[1];  
    //Mb.R[2];  
    Mb.R[3]; // Aciona o FAN exaustão
    Mb.R[4]; // Aciona o FAN HOT
    Mb.R[5]; // PWM FAN HOT
    //Mb.R[6];  
    //Mb.R[7];  

  
    /* SEND VALUES TO PLC*/
    
    //READ Analog inputs 0-1023 and Digital inputs 0-1. 
    Mb.R[12] = N_temp; // Valor de temperatura da estufa
    Mb.R[13] = N_hum; // Valor de humidade da estufa 
    //Mb.R[24] = ; // 
    //Mb.R[25] = ; // 
    //Mb.R[26] = ; // 
    //Mb.R[27] = ; // 
    //Mb.R[20] = ; //
    //Mb.R[21] = ; //

    /* SEND VALUES TO INDUSOFT*/
    
      Mb.R[31] = iluminacao ; // Valor do LDR da estufa em %
      Mb.R[32] = N_temp ; // Valor de temperatura da estufa
      Mb.R[33] = N_hum ; // Valor de umidade da estufa
      Mb.R[34] = volume_aqp ; // Valor da altura do reservatório
      Mb.R[35] = alt_cultura ; // Valor da altura da cultura
      Mb.R[36] = average() ; // Valor do PH da aquaponia
      Mb.R[37] = lum_etf; //Luminosidade da estufa Natural
      // Mb.R[38] = Colheita; // Altura ideial para colheita
      Mb.R[40] = PWM_LED; // Valor de 0 - 100%
      


    /* RECEIVE VALUES FROM INDUSOFT*/

    //  Mb.R[39] = Cultura; // Cultura selecionada
    //  Mb.R[50] : Aciona a bomba de circulação
    //  Mb.R[51] : Aciona o LED da aquaponia
    //  Mb.R[52] : Aciona o LED da estufa
    //  Mb.R[53] : Aciona o FAN de exaustão
    //  Mb.R[54] : Aciona o FISH FEEDER
    //  Mb.R[55] : Aciona o FAN HOT
    //  Mb.R[56] : Set-point FAN HOT
    //  Mb.R[57] : Aciona a bomba PH1
    //  Mb.R[58] : Set-point bomba PH1
    //  Mb.R[59] : Aciona a bomba PH2
    //  Mb.R[60] : Set-point bomba PH2
    //  Mb.R[61] : Aciona a fita de LED estufa
    //  Mb.R[62] : Set-point PWM iluminação AUTO
    //  Mb.R[63] : Set-point PWM iluminação MANUAL

    //  Mb.R[64] : Correção automática do pH
    //  Mb.R[65] : Set-point do pH para correção automática
    //  Mb.R[66] : Hora
    //  Mb.R[67] : Minutos
    //  Mb.R[68] : Sentido rotação da bomba
    //  Mb.R[69] : Set-point fish feeder
    //  Mb.R[70] : Habilita a calibração do LDR


    tempo_atual = millis();

    
                         /* LEITURA DOS SENSORES */


    /* LEITURA DO PH - SENSOR DE PH */
    
       if (tempo_atual - ut_ph >= 10000 || tempo_atual < 90000)
        {
         //Leitura do valor do sensor de PH
         measure = analogRead(A0);
         lastRead2 = (measure + lastRead2) / 2;
     
         // Valor de leitura com filtro de média móvel exp.
         Po = (alpha*lastRead2)+((1-alpha)*lastread1);

         lastread1 = (Po + lastread1) / 2;

         int reading = (Po + lastread) / 2;
         addReading(reading);
         lastread = average();

         ut_ph = tempo_atual;

         //IMPRIMIR ();
        }

    /* LEITURA DA LUMINOSIDADE - SENSOR LDR */

      if (tempo_atual - ut_ldr >= 1000 || tempo_atual < 90000)
       {
         //Leitura do valor do sensor de luminosidade
         measure1 = analogRead(A1);

         // Valor de leitura com filtro de média móvel exp.
         Luminosidade = (alpha1*measure1)+((1-alpha1)*Luminosidade);
      
         // Leitura da luminosidade, conversão para 0 - 100%
         iluminacao = map(Luminosidade, L_min, L_max, 0, 100);

         lum_etf = map(Luminosidade,0, 350, 0, 100);
                  
         ut_ldr = tempo_atual;
       }

     /* CALIBRAÇÃO (LIM.MIN. e LIM. MAX.) - SENSOR LDR */

       if (Mb.R[70] == 1 and CALIB_MIN == 0)
        {
         CALIB_MIN = 1;
         ut_calib = Mb.R[67];
         L_min = Luminosidade;
        }

       if (CALIB_MIN == 1)
       {
        analogWrite(LED_ETF1, 250);
        if (Mb.R[67] >= ut_calib+2)
        {
          L_max = Luminosidade;
          analogWrite(LED_ETF1, 0);
          CALIB_MIN = 0;  
        }
       }

     /* LEITURA DE TEMPERATURA - SENSOR NOVUS */

       if (tempo_atual - ut_ntemp >= 2000 || tempo_atual < 90000)
        {
         // Leitura do valor do sensor de temperatura
         measure2  = analogRead(A4);
      
         // Valor de leitura com filtro de média móvel exp.
         N_temp = (alpha2*measure2)+((1-alpha2)*N_temp);

         ut_ntemp = tempo_atual;

         //IMPRIMIR ();
        }


     /* LEITURA DE UMIDADE - SENSOR NOVUS */

       if (tempo_atual - ut_nhum >= 2000 || tempo_atual < 90000)
        {
         // Leitura do valor do sensor de umidade
         measure3 = analogRead(A3);

         // Valor de leitura com filtro de média móvel exp.
         N_hum = (alpha3*measure3)+((1-alpha3)*N_hum);

         ut_nhum = tempo_atual;
        }


     /* LEITURA DO NÍVEL DA ÁGUA - SENSOR ULTRASONICO */

        microsec_AQP = ultrasonic_AQP.timing();
        
       if (tempo_atual - ut_ntemp >= 5000 || tempo_atual < 90000)
        {
         //Lendo o sensor
         //microsec_AQP = ultrasonic_AQP.timing();
 
         //Convertendo a distância em CM
         distanciaCM_AQP = ultrasonic_AQP.convert(microsec_AQP,Ultrasonic::CM);

         // Calculo do volume de água
         h_reservatorio = 40 - distanciaCM_AQP;

         // Volume do Aquario
         volume_aqp = (1711.0 * h_reservatorio) / 1000.0;

         ut_volaqp = tempo_atual;
        }


     /* LEITURA DA ALTURA DAS PLANTAS - SENSOR ULTRASONICO */

        microsec_ETF = ultrasonic_ETF.timing();
       
       if (tempo_atual - ut_altcult >= 5000 || tempo_atual < 90000)
        {
         //Lendo o sensor
         //microsec_ETF = ultrasonic_ETF.timing();
 
         //Convertendo a distância em CM
         distanciaCM_ETF = ultrasonic_ETF.convert(microsec_ETF, Ultrasonic::CM); 

         // Calculo da altura da cultura
         alt_cultura = 45 - distanciaCM_ETF;

         ut_altcult = tempo_atual;
        }

        if (tempo_atual - ultimo_tempo >= 5000)
        {
          IMPRIMIR ();

          ultimo_tempo = tempo_atual;
        }



                         /* ACIONAMENTO DOS ATUADORES */


   /* ACIONAMENTO DO LED DA ESTUFA */
   
   if (Mb.R[52] == 1 and LED_estufa == 0)
   {
    LED_ESTUFA_ON();
   }
   if (Mb.R[52] == 0 and LED_estufa == 1)
   {
    LED_ESTUFA_OFF();
   }

   /* ACIONAMENTO DO LED DA AQUAPONIA */
   
   if (Mb.R[51] == 1 and LED_AQP_STT == 0)
   {
    LED_AQP_ON ();
   }
   if (Mb.R[51] == 0 and LED_AQP_STT == 1)
   {
    LED_AQP_OFF ();
   }

   /* ACIONAMENTO DO FAN DE EXAUSTÃO */
   
   if (Mb.R[53] == 1 and FAN_ETF == 0)
   {
    FAN_EXT_ON ();
   }
   if (Mb.R[53] == 0 and FAN_ETF == 1 and Mb.R[3] == 0)
   {
    FAN_EXT_OFF ();
   }

   /* ACIONAMENTO DA BOMBA DE CIRCULAÇÃO */
   
   if (Mb.R[50] == 1 and PUMP_AQP == 0)
   {
    PUMP_ON ();
   }
   if (Mb.R[50] == 0 and PUMP_AQP == 1)
   {
    PUMP_OFF ();
   }

   /* ACIONAMENTO DO FISH FEEDER */
      
   if (Mb.R[54] == 1 and FISH == 0 and Mb.R[66] == Mb.R[69])
   {
    FISH_ON ();
   }
   if (Mb.R[54] == 1 and FISH == 1 and Mb.R[66] != Mb.R[69])
   {
    FISH_OFF ();
   }
   

   /* ACIONAMENTO DO FAN HOT - controle MANUAL */

   if (Mb.R[55] == 1 and Mb.R[4] == 0)
     {
       if (FAN_HOT_STT == 0 and PELT_STT == 0)
         { 
          Serial.println ("FAN HOT: ON");
          FAN_HOT_STT = 1;
          PELT_STT = 1;
         }
    
       if (FAN_HOT_STT == 1 and PELT_STT == 1)
         {
          PWM_FAN_HOT = Mb.R[56];
          FAN_HOT_ON(PWM_FAN_HOT);
          PELT_ON ();
         }
        }
   if (Mb.R[55] == 0 and FAN_HOT_STT == 1)
     {
       FAN_HOT_OFF();
       PELT_OFF ();
     }

   /* ACIONAMENTO DO FAN HOT - controle AUTOMÁTICO */

   if (Mb.R[4] == 1 and Mb.R[55] == 0)
   {
    FAN_HOT_STT = 1;
    PWM_FAN_HOT = Mb.R[5]*255.0/1023.0;
    FAN_HOT_ON (PWM_FAN_HOT);
   }

   if(Mb.R[4] == 0 and FAN_HOT_STT == 1)
   {
    FAN_HOT_OFF();
   }

   /* ACIONAMENTO DO PELTIER - controle AUTOMÁTICO */

   if(Mb.R[0] == 1 and PELT_STT == 0)
   {
    PELT_ON ();
   }

   if(Mb.R[0] == 0 and PELT_STT == 1)
   {
    PELT_OFF ();
   }
   
   /* ACIONAMENTO DO FAN EXAUSTÃO - controle AUTOMÁTICO */
   
   if(Mb.R[3] == 1 and FAN_ETF == 0)
   {
    FAN_EXT_ON ();
   }

   if(Mb.R[3] == 0 and FAN_ETF == 1 and Mb.R[53] == 0)
   {
    FAN_EXT_OFF ();
   } 

   /* ACIONAMENTO DA FITA DE LED DA ESTUFA */

   if (Mb.R[61] == 1)
   {
     // Set point iluminação
     setPoint_ldr = Mb.R[62];

     // Calculo do erro
     erro_ldr = setPoint_ldr - iluminacao;

     // Ação proporcional
     P_ldr = (erro_ldr * kP_ldr);

     // Conversão para sinal de controle
     PWM_LED = P_ldr + setPoint_ldr;

     PWM_LED1 = map(PWM_LED,0, 100, 0 , 255);

     FITA_LED_AUTO (PWM_LED1);
   }

   if (Mb.R[61] == 0)
   {
    PWM1_LED1 = map(Mb.R[63], 0 , 100, 0, 255);

    if (PWM1_LED1 != LAST_PWM)
       {
       LAST_PWM = PWM1_LED1;
       FITA_LED_MAN (PWM1_LED1);
       }
   }

   /* SELEÇÃO DA ALTURA DA CULTURA */

   // Alface
   if(Mb.R[39] == 0 and Mb.R[38] != 25)
   {
    Mb.R[38] = 25;
   }

   // Salsinha
   if(Mb.R[39] == 1 and Mb.R[38] != 60)
   {
    Mb.R[38] = 60;
   }

   // Cheiro verde
   if(Mb.R[39] == 2 and Mb.R[38] != 75)
   {
    Mb.R[38] = 75;
   }

   // Agrião
   if(Mb.R[39] == 3 and Mb.R[38] != 10)
   {
    Mb.R[38] = 10;
   }

   // Cebolinha
   if(Mb.R[39] == 4 and Mb.R[38] != 30)
   {
    Mb.R[38] = 30;
   }
     
}

void addReading (int reading)
{
  buffer[index] = reading;
  index++;
  if(index >= bufferSize) index = 0;
}

int average ()
{
  long sum = 0;
  for (int i = 0; i < bufferSize; i++)
  {
    sum += buffer[i];
  }
  return (int)(sum / bufferSize);
}

void LED_ESTUFA_ON ()
{
 digitalWrite(LED_ETF, LOW);
 LED_estufa = 1; 
 Serial.println ("LED_ESTUFA: ON"); 
}

void LED_ESTUFA_OFF ()
{
 digitalWrite(LED_ETF, HIGH);
 LED_estufa = 0;
 Serial.println ("LED_ESTUFA: OFF"); 
}

void LED_AQP_ON ()
{
 digitalWrite(LED_AQP, LOW);
 LED_AQP_STT = 1; 
 Serial.println ("LED_AQP: ON"); 
}

void LED_AQP_OFF ()
{
 digitalWrite(LED_AQP, HIGH);
 LED_AQP_STT = 0; 
 Serial.println ("LED_AQP: OFF"); 
}

void FAN_EXT_ON ()
{
 digitalWrite(FAN_EXT, LOW);
 FAN_ETF = 1; 
 Serial.println ("FAN_EXT: ON"); 
}

void FAN_EXT_OFF ()
{
 digitalWrite(FAN_EXT, HIGH);
 FAN_ETF = 0; 
 Serial.println ("FAN_EXT: OFF"); 
}

void PUMP_ON ()
{
 digitalWrite(PUMP, LOW); 
 PUMP_AQP = 1; 
 Serial.println ("PUMP: ON");
}

void PUMP_OFF ()
{
 digitalWrite(PUMP, HIGH); 
 PUMP_AQP = 0; 
 Serial.println ("PUMP: OFF"); 
}

void FISH_ON ()
{
 servo1.write(75);
 delay(2000);
 servo1.write(83);
 FISH = 1; 
 Serial.println ("FISH: ON");
}

void FISH_OFF ()
{
 servo1.write(83); // Comando para travar o motor do servo na posição atual  
 FISH = 0;
 Serial.println ("FISH: OFF"); 
}

void FAN_HOT_ON (int PWM_FAN_HOT)
{
  analogWrite(FAN_HOT, PWM_FAN_HOT);
  analogWrite(FAN_HOT1, PWM_FAN_HOT);
  Serial.println ("FAN HOT: ON");
}

void FAN_HOT_OFF ()
{
  analogWrite(FAN_HOT, 0);
  analogWrite(FAN_HOT1, 0);
  FAN_HOT_STT = 0;
  Serial.println ("FAN HOT: OFF");
}

void PELT_ON ()
{
  PELT_STT = 1;
  digitalWrite(PELT_DIR, LOW);
  digitalWrite(PELT_ESQ, LOW);
  Serial.println ("PELTIER: ON");
}

void PELT_OFF ()
{
  digitalWrite(PELT_DIR, HIGH);
  digitalWrite(PELT_ESQ, HIGH); 
  PELT_STT = 0;
  Serial.println ("PELTIER: OFF");
}

void IMPRIMIR ()
{
  Serial.print ("Lum:");
  Serial.print (Luminosidade);
  Serial.print (" ilum.:");
  Serial.print (iluminacao);
  Serial.print (" L_min:");
  Serial.print (L_min);
  Serial.print (" L_max:");
  Serial.print (L_max);

  Serial.println ("   ");
  
  /*Serial.print ("\tIlum.:");
  Serial.print (iluminacao);
  Serial.print ("\tLum.:");
  Serial.print (Luminosidade);
  Serial.print ("\tPWM:");
  Serial.print (PWM_LED1);
  Serial.print ("\tPWM 1:");
  Serial.print (PWM1_LED1);
  Serial.print ("\tLAST:");
  Serial.print (LAST_PWM);  
  Serial.println ("\tLENDO OS SENSORES");*/
}

void FITA_LED_AUTO (int PWM_LED1)
{
  analogWrite(LED_ETF1, PWM_LED1);
}

void FITA_LED_MAN (int PWM1_LED1)
{
  analogWrite(LED_ETF1, PWM1_LED1);
}
