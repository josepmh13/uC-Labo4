/** ENCABEZADOS
*@file  PID_i.c
*@brief Experimento 4. IE 0624 Laboratorio de Microcontroladores. Escrito orignalmente en el IDE de Arduino
*@autor Andrés Quesada Acosta, B04927. José Pablo Martínez Hernández, B34024.
*@date  26 de octubre del 2016.
*/
#include <avr/pgmspace.h>

//CONSTANTES
#define cte_kp  30.0   //300
#define cte_ki  1.0     //5.0
#define cte_kd  5.0
#define Margin_Int 0.9
//#define target1  25.0    //60
//#define target2  26.0    //58

//Variables de control de flujo
long tInit;
long tSerial;

//Valor Deseado
float Target = 20;

//Output
float dutycycle;

//PID
float error;
float prev_error;
float Proportional;
float Integral;
float Derivative;
float Kp;
float Ki;
float Kd;
long LastTime;
long SampleTime;
long timeOutInt;
 
void setup() 
{
  //Empiezo comunicacion serie a 115200
  Serial.begin(115200); 
  delay(10);
  //Configuro el pin 9 como salida (PWM)
  pinMode(9,OUTPUT);
  
  //Capturo tiempos para el control de mi programa.
  tInit=millis();
  LastTime=millis();
  
  //Valor de mis constantes del control PI
  Kp=cte_kp;
  Ki=cte_ki;
  Kd=cte_kd;
  
  //Modificar frecuencia PWM
  //Setting   Divisor   Frequency
  //0x01    1     31250
  //0x02    8     3906.25
  //0x03      64    488.28125
  //0x04      256     122.0703125
  //0x05    1024    30.517578125
  //TCCR1B = TCCR1B & 0b11111000 | <setting>;
  //Configuro el PWM a 30 kHz.
  TCCR1B=TCCR1B & 0b11111000 | 0x01;
}

void loop() 
{
  
  //Recojo 20 veces el dato del sensor y hago la media
  int lectura = analogRead(0);
  float temp = 0;
  
  for (int i=0; i<20;i++){
    temp = lectura * 5.0/1024 + temp ;  //Función que transforma mi lectura del ADC en grados usando una tabla guardad en memoria FLASH
  }
  
  temp/=20;
  float Input = (temp)*100;
  
  // Cambio el SetPoint a target1 y target2 cada 120 segundos
  
  //--------------
  //Target=target1;
  //long time=(millis()-tInit)/1000;
  //if ((time>60) && (time<120) )
  //{
  //  Target=target2;  
  //}else if ( time>120 ){
  //  tInit=millis();  
  //}
  //-------------

  
  //----------------------------------------------------
  //CONTROL PID
  //Hago los calculos de manera periodica
  
  SampleTime=(millis()-LastTime);  
  if (SampleTime>=100)
  { 

    LastTime=millis();
    
    //Calculo de error (diferencia entre SetPoint y temperatura actual
    prev_error=(float)error;
    error=(float)Input-(float)Target;
  
    Proportional=(float)Kp*(float)error;
    
    //El control integral solo entra cuando esta lo suficientemente cerca
    //Esto es para evitar saturar el control
    if ( (abs(error)<=Margin_Int))
    {
      
      if (timeOutInt++ >10)        //Si se cumple durante X veces el SampleTime (delay de la parte integral)
      {
        Integral+=(float)Ki*(float)error;
        timeOutInt=3000;              //Para evitar desbordamiento del numero si esta durante mucho tiempo
      }
    }else
    {
      Integral=0;
      timeOutInt=0;
    }

    //Parte derivativa
    Derivative=(float)Kd*((float)error-(float)prev_error);
    //----------------------------------------------------------------
    dutycycle=(int)Proportional+ (int)Integral + (int)Derivative;

    //Limites de la salida. PWM de Arduino 0-255 (8 bits).
    if (dutycycle <0)
    {
      dutycycle=0;
    }else if (dutycycle>255)
    {
      dutycycle=255;
    }  
   
    //Actualizo salida
    analogWrite(9,(int)dutycycle);
    
    //Mando datos a Stamplot
    Serial.print("Temperatura= ");
    Serial.print(Input);
    //Serial.print(13,BYTE);
    
    Serial.print("; D= ");
    Serial.print(dutycycle);
    Serial.print("; Error= ");
    Serial.print(error);
    Serial.print("; p= ");
    Serial.print(Proportional);
    Serial.print("; i= ");
    Serial.print(Integral);
    Serial.print("; d= ");
    Serial.println(Derivative);    
    
  }
  //----------------------------------------------------         
}
