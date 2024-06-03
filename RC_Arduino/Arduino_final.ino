uint8_t data[3]; 

void setup() {
  Serial.begin(115200);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(4,INPUT);
}

void loop() {
  data[0] = digitalRead(4);//ign

  int i = analogRead(A0);
  data[1] = map(i,0,1024,0,255);//acc

  int j = analogRead(A1);
  data[2] = map(j,0,1024,0,255); //steer

  Serial.write((uint8_t*)data,sizeof(data));  
  delay(500);
}


// const int potPin = A0; // Potentiometer connected to analog pin A0

// void setup() {
//   Serial.begin(115200); // Initialize serial communication
// }

// void loop() {
//   int potValue = analogRead(potPin); // Read potentiometer value
//   Serial.write(potValue); // Send potentiometer value over serial
//   delay(100); // Adjust delay as needed
// }


// //Arduino code, acc, steer, ignition
// int data;
// void setup() {
//   Serial.begin(115200);

//   pinMode(A0,INPUT);
//   pinMode(A1,INPUT);
//   pinMode(2,INPUT);

// }

// void loop() 
// {
//   data = analogRead(A0);
//   //data[1]= analogRead(A1);
//   //data[2]= digitalRead(2);
//   Serial.write((uint8_t*)data, sizeof(data));
//   delay(1000);
// }
