// @reference http://www.planetarduino.org/?cat=26

const int infra=3; 

const int infra2=4;  

  void setup()

 {    

Serial.begin(9600);
pinMode(13,OUTPUT);
pinMode(12,OUTPUT);
pinMode (11,OUTPUT);
pinMode (10,OUTPUT);
pinMode (infra,INPUT);
pinMode (infra2,INPUT); 

} 

 void loop()    {
if (digitalRead(infra)== HIGH)
{         driveforward();   

   }        

if(digitalRead(infra) == LOW)   

{       

  stopmot();      

  delay (500);      

  reverse();      

  delay (500);       

 right();        

delay (2000);  

  }         

  else if (digitalRead(infra2)== LOW)   

   { stopmot();       

 delay (500);      

  reverse();     

   delay (500);       

 left();        

delay (2000);     

 }   

         }             

  void driveforward()

 {   digitalWrite(13,HIGH);   

digitalWrite(10,HIGH);  

 digitalWrite (11,LOW); 

  digitalWrite (12,LOW);

 } 

 void stopmot()

 {    

digitalWrite(13,LOW);  

 digitalWrite(12,LOW);   

digitalWrite(11,LOW);  

 digitalWrite(10,LOW);

 } 

void reverse() 

{ 

  digitalWrite(12,HIGH);  

 digitalWrite(11,HIGH); 

  digitalWrite (13,LOW); 

  digitalWrite (10,LOW);

 } 

 void right()

 {   

digitalWrite (10,HIGH); 

  digitalWrite (13,LOW);  

 digitalWrite (12,LOW);  

 digitalWrite (11,LOW);

 } 

 void left()

 {  

 digitalWrite (13,HIGH); 

  digitalWrite (11,LOW);  

 digitalWrite  (12,LOW); 

  digitalWrite (10,LOW);

}
