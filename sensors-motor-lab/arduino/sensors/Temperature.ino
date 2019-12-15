/*
The simplest TMP 36 Thermometer
*/
const int analogIn = A1;

int RawValue= 0;
double Voltage = 0;
double tempC = 0;
double tempF = 0;

void setup(){  
  Serial.begin(9600);
}

void loop(){
  
  RawValue = analogRead(analogIn);
  Voltage = (RawValue / 1023.0) * 5000; // 5000 to get millivots.
  tempC = (Voltage-500) * 0.1; // 500 is the offset
  tempF = (tempC * 1.8) + 32; // conver to F  
  Serial.print("Raw Value = " );  // shows pre-scaled value                    
  Serial.print(RawValue);      
  Serial.print("\t milli volts = "); // shows the voltage measured     
  Serial.print(Voltage,0); //
  Serial.print("\t Temperature in C = ");
  Serial.print(tempC,1);
  Serial.print("\t Temperature in F = ");
  Serial.println(tempF,1);
  delay(500);  
}
