#include <Arduino.h>
#include <EEPROM.h>
//.................variaveis a ser alimentada pelo usuario..............................................

//quantidade de espira programada para rebobinar
unsigned int quant_esp = 40;

//tamanho do carretel a ser rebobinado em mm
float tam_carretel = 1;
//controle se o valor for 1 o valor sera a seccao se for 0 sera o diametro
bool control_s_d = 1;
//medida do fio a ser utilizado diametro ou seccao que sera setado na variavel control_s_d
float fio_sec_diam = 0.159;
//controle de parada se for 1 modo parar se 0 continua
bool control_parada = 0;
//indica qual o valor em mm que sera avancado em relacao ao sensor de referencia
float avanco = 4.0;
//seta o valor da velocidade de rotacao dos motores
uint8_t velocidade = 20;

//............................variaveis do sistema.......................................................

//tamanho do passo do fuso do motor 2 em milimetro por volta
const float passo_fuso = 2;
//numero de passo por volta do motor 1
const unsigned int n_passo_volta_m1 = 96;
//numero de passo por volta do motor 2
const unsigned int n_passo_volta_m2 = 96;
//largura de cada passo do motor2 e em milimetro
const float resolucao_passo_m2 =  passo_fuso / n_passo_volta_m2;
//quantidade de passo que o motor 1 precisa dar para cada passo do motor 2
unsigned int quant_passo_m1_p_passo_m2;
//pino para controle do passo motor 1
const uint8_t pin_step_m1 = 12;
//pino para controle do passo motor 2
const uint8_t pin_step_m2 = 10;
//pino para controle da direcao motor 1
const uint8_t pin_dir_m1 = 11;
//pino para controle da direcao motor 2
const uint8_t pin_dir_m2 = 9;
//pino para leitura do sensor de referencia
const uint8_t sensor_ref = 7;
//pino para interrupcao
const uint8_t interrupt = 2;
//variavel que armazena o valor da velocidade convertida
unsigned int vel_convert;
//quantidade de passo do motor 2 em funcao da largura do carretel
unsigned int quant_passo_m2_p_larg_carretel ;
//quantidade e espira em funcao da largura do carretel
unsigned int quant_espira_p_larg_carretel ;
//vairiavel de controle para iniciar a rebobinagem
bool iniciar_reb;
//sinaliza se o a rebobinagem foi finalizada ou interrompida
bool reb_finalizada;
//quantidade de passo do motor 2 para cada espira rebobinada
 unsigned int quant_passo_m2_p_esp;
          //variavel contador para controle do passo do motor 1
          unsigned int cont_passo_m1;
//variavel contador para controle do passo do motor 2

unsigned int cont_passo_m2_p_c;
//variavel de controle da direcao do motor 2
bool direcao_m2_atual;
unsigned int esp_atual;
unsigned int pass_m1_atual;
unsigned pass_m2_atual;

#define vel_ref 8
#define avancar  true
#define voltar  false
//prototipo de funcoes...............
void rebobinar();
void iniciar();
void parar();
void interrupt_parar();
void passo_m1();
void passo_m2(bool dir);
void  referenciar();
void set_velocidade(uint8_t veloc);
void atualizar();

void setup() {
  // put your setup code here, to run once:
  pinMode(pin_dir_m1,OUTPUT);
  pinMode(pin_dir_m2,OUTPUT);
  pinMode(pin_step_m1,OUTPUT);
  pinMode(pin_step_m2,OUTPUT);
  pinMode(interrupt,INPUT_PULLUP);
  pinMode(sensor_ref,INPUT_PULLUP);
  Serial.begin(9600);
  //variavel inicializada para teste
  iniciar_reb = true;
  cont_passo_m1 = 0;
  //atualizando valores
  control_parada = 0;
  cont_passo_m2_p_c = 0;
  direcao_m2_atual = true;
  esp_atual = 0;
  pass_m1_atual =0;
  pass_m2_atual =0;
  reb_finalizada = EEPROM.read(20);
  digitalWrite(pin_step_m1,LOW);
  digitalWrite(pin_step_m2,LOW);
  attachInterrupt(digitalPinToInterrupt(interrupt), interrupt_parar, LOW);
   referenciar();
} 
 unsigned long i =0;
   void interrupt_parar(){
    noInterrupts();
        
      Serial.print("interrupcao");
     control_parada = !control_parada;
     while(i < 1000000){
       i++;
              
     }
     i = 0;
     interrupts();
     //delay(4000);
         
         // Serial.print(esp_atual);
          //interrupts();
    interrupts();
   }

   void referenciar(){
     unsigned int temp;
     set_velocidade(vel_ref);
     while(!digitalRead(sensor_ref)){
        passo_m2(false);
        
     }
     if(reb_finalizada){
       temp = (avanco/resolucao_passo_m2);
      }else{
        atualizar();
        temp = (unsigned int)(avanco/resolucao_passo_m2) + cont_passo_m2_p_c ;
      }
     for(unsigned int i = 0; i < temp ;i++){
           passo_m2(true);
     }
     
   }
   
 void salvar(){
      uint8_t temp[4] ;
      EEPROM.write(0,direcao_m2_atual);

      EEPROM.write(1,pass_m1_atual);
      EEPROM.write(2,(pass_m1_atual>>8));

      EEPROM.write(3,pass_m2_atual);
      EEPROM.write(4,(pass_m2_atual>>8));

      EEPROM.write(5,esp_atual);
      EEPROM.write(6,(esp_atual>>8));

      EEPROM.write(7,cont_passo_m2_p_c);
      EEPROM.write(8,(cont_passo_m2_p_c>>8));

      EEPROM.write(9,quant_esp);
      EEPROM.write(10,(quant_esp>>8));
       
      memcpy(&temp,&fio_sec_diam,4);        
      EEPROM.write(11,temp[0]);
      EEPROM.write(12,temp[1]);
      EEPROM.write(13,temp[2]);
      EEPROM.write(14,temp[3]);

      memcpy(&temp,&tam_carretel,4);
      EEPROM.write(15,temp[0]);
      EEPROM.write(16,temp[1]);
      EEPROM.write(17,temp[2]);
      EEPROM.write(18,temp[3]);
      
      EEPROM.write(19,control_s_d);

      EEPROM.write(20,reb_finalizada);

      memcpy(&temp,&avanco,4);        
      EEPROM.write(21,temp[0]);
      EEPROM.write(22,temp[1]);
      EEPROM.write(23,temp[2]);
      EEPROM.write(24,temp[3]);

      EEPROM.write(25,velocidade);
      EEPROM.write(26,(velocidade>>8));
      
      }

   void atualizar(){
      uint8_t temp[4] ;
      float temp2;
      control_s_d = EEPROM.read(19);
      direcao_m2_atual = EEPROM.read(0);
      pass_m1_atual = (EEPROM.read(1) | (EEPROM.read(2)<<8));
      pass_m2_atual = (EEPROM.read(3) | (EEPROM.read(4)<<8));
      esp_atual =  (EEPROM.read(5) | (EEPROM.read(6)<<8));
      cont_passo_m2_p_c =  (EEPROM.read(7) | (EEPROM.read(8)<<8));
      quant_esp =  (EEPROM.read(9) | (EEPROM.read(10)<<8));
      velocidade =  (EEPROM.read(25) | (EEPROM.read(26)<<8));

      temp[0] = EEPROM.read(11);
      temp[1] = EEPROM.read(12);
      temp[2] = EEPROM.read(13);
      temp[3] = EEPROM.read(14);
      memcpy(&temp2,&temp, 4);
      fio_sec_diam = temp2;

      temp[0] = EEPROM.read(15);
      temp[1] = EEPROM.read(16);
      temp[2] = EEPROM.read(17);
      temp[3] = EEPROM.read(18);
      memcpy(&temp2,&temp, 4);
      tam_carretel = temp2;

      temp[0] = EEPROM.read(21);
      temp[1] = EEPROM.read(22);
      temp[2] = EEPROM.read(23);
      temp[3] = EEPROM.read(24);
      memcpy(&temp2,&temp, 4);
      avanco = temp2;


      }

      void set_velocidade(uint8_t veloc){
          if(veloc>0 && veloc<=20){
               vel_convert = 41500 -(veloc * 2000);
          }
             
      }
 
  void configuracao(){
    float diametro_fio;
   //diametro do fio usado para rebobinar
  
  if(control_s_d){
    diametro_fio = 2 * sqrt(fio_sec_diam/PI);
  }else{
    diametro_fio = fio_sec_diam;
  }
  
  quant_espira_p_larg_carretel = (tam_carretel / diametro_fio);
  quant_passo_m2_p_esp = (diametro_fio / resolucao_passo_m2);
  quant_passo_m2_p_larg_carretel = quant_passo_m2_p_esp * quant_espira_p_larg_carretel;
  //quant_passo_m1_p_passo_m2 = n_passo_volta_m1/quant_passo_m2_p_esp;
  
   //seleciona a direcao
    digitalWrite(pin_dir_m1,HIGH);
    set_velocidade(velocidade);
      //referenciar();

 }
 void passo_m1(){
     
      //avanca um passo no motor 1
      digitalWrite(pin_step_m1,HIGH);
      delayMicroseconds(500);
      digitalWrite(pin_step_m1,LOW);
      delayMicroseconds(vel_convert);
      //digitalWrite(pin_step_m1,!digitalRead(pin_step_m1));
      //cont_passo_m1++;
 }

 void passo_m2(bool dir){
     //seleciona a direcao
      digitalWrite(pin_dir_m2,dir);
      //avanca um passo no motor 2
      digitalWrite(pin_step_m2,HIGH);
      delayMicroseconds(500);
      digitalWrite(pin_step_m2,LOW);
      delayMicroseconds(vel_convert);
     // cont_passo_m2_p_c++;
     // Serial.print( cont_passo_m2_p_c);
      
 }
    void iniciar(){
     // Serial.print('0');
       while(control_parada){
        // Serial.print('.');
         delay(1000);
       }
       if(EEPROM.read(20)){
         salvar();
         configuracao();
       }else{
        atualizar();
        configuracao();
       }
        //Serial.print('1');
    }

    void parar(){
     if(control_parada){
       reb_finalizada = false;
         salvar();
         iniciar();
          
     }
    }
     void finalizar(){
       Serial.print("  finalizada");
      reb_finalizada = true;
      control_parada = true;
      cont_passo_m2_p_c = 0;
      direcao_m2_atual = true;
      esp_atual = 0;
      pass_m1_atual =0;
      pass_m2_atual =0;
       salvar();
      
     }


    
     
    void rebobinar(){
            //bool dir_m2 = direcao_m2_atual;
           // unsigned int pass_m1 ;
           // unsigned int esp;
            //unsigned  int pass_m2;
           for( ; esp_atual < quant_esp ; esp_atual++){
               
               //Serial.print(((quant_esp * n_passo_volta_m1)/quant_passo_m1_p_passo_m2));
              for( ; pass_m1_atual < n_passo_volta_m1; pass_m1_atual++){
               
                passo_m1();
                parar();
                               
              }
             
              if(direcao_m2_atual){
              for( ; pass_m2_atual < quant_passo_m2_p_esp ; pass_m2_atual++){
               //   if(cont_passo_m2_p_c <  quant_passo_m2_p_larg_carretel){
                    //Serial.print( quant_passo_m2_p_larg_carretel);
                  passo_m2(direcao_m2_atual);
                  cont_passo_m2_p_c++;
                  parar();
                  //delay(1);
                 // }else{
                    //cont_passo_m2_p_c = 0;
                    //direcao_m2_atual = false;
                   // passo_m2(direcao_m2_atual);
                   // parar();
                
                   
                  }
                   if(!(cont_passo_m2_p_c <  quant_passo_m2_p_larg_carretel)){
                     direcao_m2_atual = false;
                   }
                      //Serial.print(pass_m2);
                   // } 
///*
              }else{
              
               for( ; pass_m2_atual < quant_passo_m2_p_esp ; pass_m2_atual++){
                //  if(cont_passo_m2_p_c > 0){
                    //Serial.print( quant_passo_m2_p_larg_carretel);
                  passo_m2(direcao_m2_atual);
                  cont_passo_m2_p_c--;
                  parar();
                  //delay(1);
                  }
                   if(!(cont_passo_m2_p_c > 0)){
                     direcao_m2_atual = true;
                  }
                      //Serial.print(pass_m2);
                    
              }
              //Serial.print(cont_passo_m2_p_c);
             // */
                    pass_m1_atual = 0;
                    pass_m2_atual = 0;
          }

          finalizar();
          //control_parada = 1;
          // parar();
    }
       

void loop() {
 
  
   control_parada = 1;
  // salvar();
   //configuracao();
   iniciar();
     
   rebobinar();
    
}