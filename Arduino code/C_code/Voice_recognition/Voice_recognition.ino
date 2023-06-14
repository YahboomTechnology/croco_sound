/*
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         Voice recognition
* @author       Jessica
* @version      V1.0
* @date         2019.8.21
* @brief        Voice recognition
* @details
* @par History  
*
*/
int voicepin = A1;
int val = 0;//Temporary variable value from sensor

/*
* Function       setup
* @author        Jessica
* @date          2019.8.21
* @brief         Initial configuration
* @param[in]     void
* @retval        void
* @par History   no
*/
void setup()
{
  pinMode(voicepin, INPUT);
  Serial.begin(9600);
}

/*
* Function       loop
* @author        Jessica
* @date          2019.8.21
* @brief         main
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void loop() 
{
  val = analogRead(voicepin);//Read the analog value of the potentiometer and assign it to val
  Serial.println(val);//Serial port print val variable
  delay(1000);
}
