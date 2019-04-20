
#define MESSAGE_LENGTH 8

char *buff[MESSAGE_LENGTH];

void setup(){
	Serial.begin(9600);
}

void loop(){
	if(Serial.available()){
		Serial.readBytes(*buff, MESSAGE_LENGTH);
	}
	delay(1);
}
