#define ERROR_ARRAY_SIZE 5

double errors[ERROR_ARRAY_SIZE];
int pos = 0;

void addError(double error){
  errors[pos] = error;
  pos = (pos+1)%(ERROR_ARRAY_SIZE+1);
}

double avg(double errors[ERROR_ARRAY_SIZE]){
  double avg = 0;
  for (int i = 0; i <= ERROR_ARRAY_SIZE; i++) {
    avg += errors[i];
  }
  return avg/(ERROR_ARRAY_SIZE+1);
}

double prev_avg = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  
}

void serialEvent() {
  if (Serial.available()) {
    double incomingByte = Serial.read();
    if(incomingByte != 10){
      double newError = incomingByte - 48;
      addError(newError);
//      Serial.println(newError);
      for (int i = 0; i <= ERROR_ARRAY_SIZE; i++) {
        Serial.print(errors[i]);
        Serial.print(", ");
      }
      Serial.print(" avg: ");
      prev_avg = (prev_avg+newError)/2;
      Serial.print(prev_avg);
      //Serial.print(avg(errors));
      Serial.println();
      
    }
  }
}
