#include <SPI.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <Plotter.h>
#include "Thread.h"
#include "ThreadController.h"



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
#define KP 15 //10
#define KI 15 //10
#define KD 0.1 //0.1 valor menor que KP e KI

//Limites do PWM
#define MIN_PWM 0
#define MAX_PWM 255

// Motor encoder output pulses per 360 degree revolution (measured manually)
#define ENC_COUNT_REV 2480

//Leitura do encoder de cada motor(fazendo apenas de cada lado do robô)
Encoder enc1 (INT_PORT1,INT_PORT2);
Encoder enc2 (INT_PORT3,INT_PORT4);


float pwm1, pwm2, pwm3, pwm4;
boolean dir1, dir2, dir3, dir4;

//para motores do lado esquerdo
double veloc_esquerda_rads=0;
double output_robo_esq=0; //velocidade a ser controlada em pwm
double input_1 = 0;
double veloc_esq_ms = 0;
double theta_ponto_1 = 0;

long pulsos_1 = 0;
float pos_atual_motor_1 = 0.0;
float pos_anterior_1 = 0.0;
float pos_atual_roda_1 = 0.0;


//para motores do lado direito
double veloc_direita_rads=0;
double output_robo_dir=0; //velocidade a ser controlada em pwm
double input_2 = 0;
double veloc_dir_ms = 0;
double theta_ponto_2 = 0;

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
double b = dist_rodas/2; //0.105m distância do centro do robô até a roda em metros
double veloc_linear_robo = 0;
double veloc_ang_robo =0;
float veloc_linear_out = 0;
float veloc_ang_robo_out = 0;


double veloc_linear_ref = 0;
double veloc_ang_ref = 0; 


//Thread
Thread thread1;
Thread thread2;
Thread thread3;
Thread thread4;
ThreadController cpu;


//Função para calcular a velocidade angular de cada lado em relação a linear e angular de referência 
void calculaVelocidadeReferencia()
{
  theta_ponto_1 = (veloc_linear_ref+(veloc_ang_ref*b))/(raio_roda); //rad/s 
  theta_ponto_2 = (veloc_linear_ref-(veloc_ang_ref*b))/(raio_roda); //rad/s 
  //Velocidade linear local em relação as velocidades angulares das rodas.
  veloc_linear_robo = (theta_ponto_1 + theta_ponto_2)*raio_roda/2; //(rad/s)*(m) = m/s
  //Velocidade angular do robô
  veloc_ang_robo = (theta_ponto_1 - theta_ponto_2)/2*b; //(rad/s)*(m/m) = rad/s
}

/*
void atualizaMotores()
{
  // Configuração robô andando para frente(dir1=dir4=0 e dir2=dir3=1)
  // Setar direção (0 sentido anti-horário e 1 sentido horário)

  //Lado direito
  dir1 = 0; 
  pwm1 = constrain(output_robo_esq, MIN_PWM, MAX_PWM);//output_robo_esq; // Valor que será em pwm.
  dir4 = 0;
  pwm4 = constrain(output_robo_esq, MIN_PWM, MAX_PWM);//output_robo_esq;
  //Lado esquerdo
  dir2 = 1; 
  pwm2 = constrain(output_robo_dir, MIN_PWM, MAX_PWM);//output_robo_dir;
  dir3 = 1;
  pwm3 = constrain(output_robo_dir, MIN_PWM, MAX_PWM);//output_robo_dir;
}
*/
void movimento2()
{
  //Lado direito
  dir2 = 1; 
  pwm2 = output_robo_dir;
}

void movimento3()
{
  dir3 = 1;
  pwm3 = output_robo_dir;

}
void movimento1()
{
  //Lado esquerdo
  dir1 = 0; 
  pwm1 = output_robo_esq; // Valor que será em pwm.
}

void movimento4()
{
  dir4 = 0;
  pwm4 = output_robo_esq;
}

// Criação do PID para controle
//PID myPID(&Input, &Output, &Veloc_ref, Kp, Ki, Kd, DIRECT or INVERSE);
//Motores 1 e 4
PID motorPID_esq(&veloc_esquerda_rads, &output_robo_esq, &theta_ponto_1, KP, KI, KD, DIRECT);
//Motor 2 e 3
PID motorPID_dir(&veloc_direita_rads, &output_robo_dir, &theta_ponto_2, KP, KI, KD, DIRECT);




//FUnção para habilitar o controle PID.
void PID_robo()
{
  motorPID_esq.Compute(); //para motores 1 e 4 - lado esquerdo
  motorPID_dir.Compute(); //para motores 2 e 3 - lado direito 

}

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
    SPI.setBitOrder(LSBFIRST);// MSBFIRST - envia o bit mais significativo primeiro
    SPI.setDataMode(SPI_MODE1);  // sinal de clock é baixo quando a placa está desligada, a amostragem de dados ocorre na transição de subida do clock e a transição de descida do clock é usada para a mudança de dados.

    // Configura controle PID
    motorPID_esq.SetOutputLimits(MIN_PWM, MAX_PWM);
    motorPID_dir.SetOutputLimits(MIN_PWM, MAX_PWM);
    motorPID_esq.SetMode(AUTOMATIC);//ativar PID(AUTOMATIC/MANUAL)
    motorPID_dir.SetMode(AUTOMATIC);

    

    //thread1.setInterval(500);
    thread1.onRun(movimento1);
    //thread2.setInterval(500);
    thread2.onRun(movimento2);
    //thread3.setInterval(1000);
    thread3.onRun(movimento3);
    //thread4.setInterval(500);
    thread4.onRun(movimento4);
    

    cpu.add(&thread1);
    cpu.add(&thread2);
    cpu.add(&thread3);
    cpu.add(&thread4);

    
}

void loop()
{
 
//Entradas dos valores de pwm que desejo para cada roda e a direção da roda. 
  analogWrite(PWM_M1, pwm1);  digitalWrite(DIR_M1, dir1);
  analogWrite(PWM_M2, pwm2);  digitalWrite(DIR_M2, dir2);
  analogWrite(PWM_M3, pwm3);  digitalWrite(DIR_M3, dir3);
  analogWrite(PWM_M4, pwm4);  digitalWrite(DIR_M4, dir4);

  digitalWrite(ENABLE_MOTORS, LOW);  // enable = LOW (HABILITAR A ENTRADA DOS MOTORES)


  //Pulso gerado a partir da leitura do encoder
  pulsos_1 = enc1.read();// lado esquerdo
  pulsos_2 = enc2.read();//lado direito
  
  tempoatual = millis();

  // POSIÇÃO para rodas 1 e 4 - lado esquerdo
  pos_atual_motor_1 = pulsos_1/(4*20);//rotação do motor
  pos_atual_roda_1 = pos_atual_motor_1*2*PI/reducao; //rotacao da roda em rad
  // POSIÇÃO para rodas 2 e 3 - lado direito
  pos_atual_motor_2 = pulsos_2/(4*20);
  pos_atual_roda_2 = pos_atual_motor_2*2*PI/reducao; 



  if ((tempoatual - tempopassado)>intervalo)
  {// Calcula a cada 1 segundo, a velocidade.
    
    //velocidade das rodas 1 e 4 - Lado Esquerdo
    veloc_esquerda_rads = (pos_atual_roda_1 - pos_anterior_1)/(intervalo/1000);//velocidade da roda 1 e 4 em rad/s
    input_1 = veloc_esquerda_rads*60/(2*PI);//velocidade da roda 1 e 4 em RPM
    veloc_esq_ms = veloc_esquerda_rads*raio_roda;//m/s
    tempopassado = tempoatual;
    pos_anterior_1 = pos_atual_roda_1;

    //velocidadade das rodas 2 e 3 - Lado Direito
    veloc_direita_rads = (-1)*(pos_atual_roda_2 - pos_anterior_2)/(intervalo/1000);//velocidade da roda 2 e 3 em rad/s
    input_2 = veloc_direita_rads*60/(2*PI);//velocidade da roda 2 e 3 em RPM 
    veloc_dir_ms = veloc_direita_rads*raio_roda;//m/s
    tempopassado = tempoatual;
    pos_anterior_2 = pos_atual_roda_2; //rad 


    //CINEMÁTICA DO ROBÔ
    //Velocidade de referencia
    veloc_linear_ref = 0.3;
    veloc_ang_ref = 0.3; // valor positido movimento para o lado direito. Valor negativo movimento para o lado esquerdo.
    calculaVelocidadeReferencia();//(linear e angular)
    //Chama o controle PID de cada lado;
    PID_robo();
    //Direção dos motores e saída PWM de acordo com o controle PID
    //atualizaMotores();


    float theta_ponto_1_out = veloc_esquerda_rads;//rad/s
    float theta_ponto_2_out = veloc_direita_rads;//rad/s
    // Cálculo do erro da velocidade angular da cinemática
    float erro_1 = theta_ponto_1 - theta_ponto_1_out;
    float erro_2 = theta_ponto_2 - theta_ponto_2_out;

    //Cinematica inversa
    //Velocidade linear e angular de saída
    veloc_linear_out = (theta_ponto_1_out + theta_ponto_2_out)*raio_roda/2;//(rad/s)*m = m/s
    veloc_ang_robo_out = (theta_ponto_1_out - theta_ponto_2_out)/2;//rad/s
    // Cálculo do erro da velocidade na cinematica inversa
    float erro_1_vw = veloc_linear_ref - veloc_linear_out;
    float erro_2_vw = veloc_ang_ref - veloc_ang_robo_out;

  }

  cpu.run();

  // Envio dos dados para serial monitor
  //Serial.println("veloc_linear_out\tveloc_linear_ref\tveloc_ang_out\tveloc_ang_ref:");
  Serial.print(veloc_linear_out);// a ser controlada
  Serial.print("\t");
  Serial.print(veloc_linear_ref);
  Serial.print("\t");
  Serial.print(veloc_ang_robo_out);
  Serial.print("\t");
  Serial.println(veloc_ang_ref);
  
}
