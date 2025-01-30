/* Projeto Armário Móvel (Jan/2025)

Integrantes: Juniper, Yasmin e Felipe

Fontes utilizadas: https://www.youtube.com/watch?v=hXP_kQ_EbkA

Não funciona esse

*/

#include <ESP32Servo.h>
#include <analogWrite.h>
//#include <tone.h>
#include <ESP32Tone.h>
#include <ESP32PWM.h>

#include <PS4Controller.h>
#include <ps4.h>
#include <ps4_int.h>

#define rightMotorPin 25
#define leftMotorPin 26

Servo MotorEsquerdo;
Servo MotorDireito;

// MAC Adress do Controle Dualshock4 (PS4): a0:b7:65:26:db:3a (valor deve mudar a depender do controle que for usado)

void setup() {
  //inicializa o setup do PS4
  PS4.begin("a0:b7:65:26:db:3a");

  //Inicializa o monitor e a saída serial
  Serial.begin(9600);

  //Enquanto não houver um controle de PS4 conectado, o código para aqui até ser conectado a um
  while(!PS4.isConnected()){
    delay (250);
    Serial.println("Esperando Conexão de um Dual-Shock");
  }

  //Função que seta a cor do LED do controle
  PS4.setLed(255, 0, 0);
  PS4.sendToController();
  
  //instanciando os dois motores servo
  MotorEsquerdo.attach(leftMotorPin);
  MotorDireito.attach(rightMotorPin);

  //inicia parado (0 = ré no máximo, 180 = frontal no máximo)
  MotorEsquerdo.write(90);
  MotorDireito.write(90);

  Serial.println("Conexão Estabelecida com sucesso!"); 

}

void loop() {

  /*int LStickX = 0;
  int LStickY = 0;

  int LeftMotorOutput = 90;
  int RightMotorOutput = 90;*/

  
  //Condição para o motor andar é estar conectado com o controle DualShock
  if(PS4.isConnected()){
    //Ambos os motores são controlados pelo Stick Esquerdo do PS4
    int LStickX = PS4.LStickX();
    int LStickY = PS4.LStickY();

    int LeftMotorOutput  = constrain(map(LStickY, -127, 127, 0, 180) + map(LStickX, -127, 127, 0, 180), 0, 180);
    int RightMotorOutput = constrain(map(LStickY, -127, 127, 0, 180) - map(LStickX, -127, 127, 0, 180), 0, 180);

    if(LeftMotorOutput > 95 || LeftMotorOutput < 85)
    {
      MotorEsquerdo.write(LeftMotorOutput);
    }

    else{
      MotorEsquerdo.write(90); 
    }

    if(RightMotorOutput > 95 || RightMotorOutput < 85)
    {
      MotorDireito.write(LeftMotorOutput);
    }

    else{
      MotorDireito.write(90); 
    }
    Serial.print("Motor Direito: ");
    Serial.print(RightMotorOutput);
    Serial.println("");
    Serial.print("Motor Esquerdo: ");
    Serial.print(LeftMotorOutput);
    Serial.println("");
  }

  //Caso o controle não esteja conectado ou perca a conexão, o motor deve parar (failsafe)
  else{
    MotorEsquerdo.write(90);
    MotorDireito.write(90);
  }


}
