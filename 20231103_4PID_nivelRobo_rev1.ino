#include <SPI.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <Plotter.h>


//portas do arduino para cada pulso do motor
//motor 1
#define INT_PORT1 23 
#define INT_PORT2 25
//motor 2
#define INT_PORT3 33
#define INT_PORT4 35
//motor 3
#define INT_PORT5 43
#define INT_PORT6 45
//motor 4
#define INT_PORT7 49
#define INT_PORT8 47


#define ENABLE_MOTORS 8

//Porta para SPI(entrada dos motores)
#define SS_M4 14
#define SS_M3 13
#define SS_M2 12
#define SS_M1 11


//Direção do motor
#define DIR_M1 2
#define DIR_M2 3
#define DIR_M3 4
#define DIR_M4 7

// L9958 PWM pins
#define PWM_M1 9
#define PWM_M2 10  
#define PWM_M3 5
#define PWM_M4 6    

// Constantes do Controle PID
#define KP 24
#define KI 24
#define KD 0.10

//Limites do PWM
#define MIN_PWM 0
#define MAX_PWM 255

// Motor encoder output pulses per 360 degree revolution (measured manually)
#define ENC_COUNT_REV 2480

Encoder enc1 (INT_PORT1,INT_PORT2);
Encoder enc2 (INT_PORT3,INT_PORT4);
Encoder enc3 (INT_PORT5,INT_PORT6);
Encoder enc4 (INT_PORT7,INT_PORT8);

float pwm1, pwm2, pwm3, pwm4;
boolean dir1, dir2, dir3, dir4;

//para motores do lado esquerdo
double veloc_esquerda_rads=0;
double output_robo_esq=0; //velocidade a ser controlada em pwm
double input_1 = 0;
double veloc_esq_ms = 0;
double veloc_esq_ref_ms = 0;

long pulsos_1 = 0;
float pos_atual_motor_1 = 0.0;
float pos_anterior_1 = 0.0;
float pos_atual_roda_1 = 0.0;


//para motores do lado direito
double veloc_direita_rads=0;
double output_robo_dir=0; //velocidade a ser controlada em pwm
double input_2 = 0;
double veloc_dir_ms = 0;
double veloc_dir_ref_ms = 0;

long pulsos_2 = 0;
float pos_atual_motor_2 = 0.0;
float pos_anterior_2 = 0.0;
float pos_atual_roda_2 = 0.0;


long tempopassado = 0.0;
long tempoatual = 0.0;
int intervalo = 1000;
int reducao = 31;


//Controle da velocidade nivel robô
double raio_roda = 0.07; //raio da roda em metros
double dist_rodas = 0.21; //distância, em metros, entre as rodas de lados opostos
double b = dist_rodas/2; //distância do centro do robô até a roda em metros
double veloc_local = 0;
double veloc_ang_robo =0;

float veloc_local_ref = 0.4; //(m/s)velocidade linear de referência do chassi.
float veloc_ang_robo_ref = 0; //(rad/s)velocidade angular de referência do chassi.
double direcao1 = 1;
double direcao2 = 1;


// Cria PID para controle
//PID myPID(&Input, &Output, &Veloc_ref, Kp, Ki, Kd, DIRECT or INVERSE);
//Motores 1 e 4
PID motorPID_esq(&veloc_esq_ms, &output_robo_esq, &veloc_esq_ref_ms, KP, KI, KD, DIRECT);
//Motor 2 e 3
PID motorPID_dir(&veloc_dir_ms, &output_robo_dir, &veloc_dir_ref_ms, KP, KI, KD, DIRECT);

void setup()
{      
    Serial.begin(2400);
    pinMode(SS_M1, OUTPUT); digitalWrite(SS_M1, LOW);  // HIGH = not selected
    pinMode(SS_M2, OUTPUT); digitalWrite(SS_M2, HIGH);
    pinMode(SS_M3, OUTPUT); digitalWrite(SS_M3, HIGH);
    pinMode(SS_M4, OUTPUT); digitalWrite(SS_M4, HIGH);
  
    // L9958 DIRection pins
    pinMode(DIR_M1, OUTPUT);
    pinMode(DIR_M2, OUTPUT);
    pinMode(DIR_M3, OUTPUT);
    pinMode(DIR_M4, OUTPUT);

    // L9958 PWM pins
    pinMode(PWM_M1, OUTPUT);  digitalWrite(PWM_M1, LOW);
    pinMode(PWM_M2, OUTPUT);  digitalWrite(PWM_M2, LOW); 
    pinMode(PWM_M3, OUTPUT);  digitalWrite(PWM_M3, LOW);
    pinMode(PWM_M4, OUTPUT);  digitalWrite(PWM_M4, LOW); 
  
    // L9958 Habilitar os quatros motores
    pinMode(ENABLE_MOTORS, OUTPUT); 

    SPI.begin();
    SPI.setBitOrder(LSBFIRST);
    SPI.setDataMode(SPI_MODE1);  // clock pol = low, phase = high

    // Configura controle PID
    motorPID_esq.SetOutputLimits(MIN_PWM, MAX_PWM);
    motorPID_dir.SetOutputLimits(MIN_PWM, MAX_PWM);
    motorPID_esq.SetMode(AUTOMATIC);//ativar PID(AUTOMATIC/MANUAL)
    motorPID_dir.SetMode(AUTOMATIC);
}

void loop()
{
 
//Entradas dos valores de pwm que desejo para cada roda e a direção da roda. 
  analogWrite(PWM_M1, pwm1);  digitalWrite(DIR_M1, dir1);
  analogWrite(PWM_M2, pwm2);  digitalWrite(DIR_M2, dir2);
  analogWrite(PWM_M3, pwm3);  digitalWrite(DIR_M3, dir3);
  analogWrite(PWM_M4, pwm4);  digitalWrite(DIR_M4, dir4);

  digitalWrite(ENABLE_MOTORS, LOW);  // enable = LOW (HABILITAR A ENTRADA DOS MOTORES)

  // Configuração robô andando para frente(dir1=dir4=0 e dir2=dir3=1)
  // Setar direção (0 sentido anti-horário e 1 sentido horário)

  dir1 = 1; 
  pwm1 = output_robo_esq; // Valor que será em pwm.

  dir2 = 0; 
  pwm2 = output_robo_dir;

  dir3 = 0;
  pwm3 = output_robo_dir;

  dir4 = 1;
  pwm4 = output_robo_esq; 

  pulsos_1 = enc1.read();
  pulsos_2 = enc2.read();
  tempoatual = millis();

  // POSIÇÃO para rodas 1 e 4
  pos_atual_motor_1 = pulsos_1/(4*20);//rotação do motor
  pos_atual_roda_1 = pos_atual_motor_1*2*PI/reducao; //rotacao da roda em rad
  // POSIÇÃO para rodas 2 e 3
  pos_atual_motor_2 = pulsos_2/(4*20);
  pos_atual_roda_2 = pos_atual_motor_2*2*PI/reducao; 


  if ((tempoatual - tempopassado)>intervalo){// Calcula a cada 1 segundo, a velocidade.
    
    // para rodas 1 e 4 - Lado Esquerdo
    veloc_esquerda_rads = (-1)*(pos_atual_roda_1 - pos_anterior_1)/(intervalo/1000);//velocidade da roda 1 e 4 em rad/s
    input_1 = veloc_esquerda_rads*60/(2*PI);//velocidade da roda 1 e 4 em RPM
    veloc_esq_ms = veloc_esquerda_rads*raio_roda;//m/s
    tempopassado = tempoatual;
    pos_anterior_1 = pos_atual_roda_1;

    
    //para rodas 2 e 3 - Lado Direito
    veloc_direita_rads = (pos_atual_roda_2 - pos_anterior_2)/(intervalo/1000);//velocidade da roda 2 e 3 em rad/s
    input_2 = veloc_direita_rads*60/(2*PI);//velocidade da roda 2 e 3 em RPM 
    veloc_dir_ms = veloc_direita_rads*raio_roda;//m/s
    tempopassado = tempoatual;
    pos_anterior_2 = pos_atual_roda_2; //rad


    //CINEMÁTICA DO ROBÔ
    
    // Velocidade linear de referencia.
    veloc_esq_ref_ms = veloc_local_ref-(veloc_ang_robo_ref*b); //m/s (theta ponto 1)
    veloc_dir_ref_ms = veloc_local_ref+(veloc_ang_robo_ref*b); //m/s (theta ponto 2)
    //Velocidade linear local em relação as velocidades angulares das rodas.
    veloc_local = (veloc_esquerda_rads + veloc_direita_rads)*raio_roda/2; //(rad/s)*(m) = m/s
    //Velocidade angular do robô
    veloc_ang_robo = (veloc_esquerda_rads - veloc_direita_rads)*raio_roda/2*b; //(rad/s)*(m/m) = rad/s

  }

  // Calcula o PWM do motor conforme Controle PID. Faz o cácludo do PWM
  motorPID_esq.Compute(); //para motores 1 e 4
  motorPID_dir.Compute(); //para motores 2 e 3


  // Envio dos dados para serial monitor
  Serial.println("PWM_esq\tPWM_dir\tveloc_esq_ms[m/s]\tveloc_dir_ms[m/s]\tveloc_esq_ref_ms\tveloc_dir_ref_ms:");
  Serial.print(output_robo_esq);// a ser controlada
  Serial.print("\t");
  Serial.print(output_robo_dir);
  Serial.print("\t");
  Serial.print(veloc_esq_ms);
  Serial.print("\t\t\t");
  Serial.print(veloc_dir_ms);
  Serial.print("\t\t\t");
  Serial.print(veloc_esq_ref_ms);
  Serial.print("\t\t\t");
  Serial.println(veloc_dir_ref_ms);
}
