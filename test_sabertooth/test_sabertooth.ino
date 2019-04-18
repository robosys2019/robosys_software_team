
#define LEFT 5
#define RIGHT 6 

void setup(){
	pinMode(LEFT, OUTPUT);
	pinMode(RIGHT, OUTPUT);
}

void loop(){
	for(int i=0; i<=255; i++){
		analogWrite(LEFT, i);
		analogWrite(RIGHT, i);
		delay(1);
	}
	for(int i=255; i>=0 i--){
		analogWrite(LEFT, i);
		analogWrite(RIGHT, i);
		delay(1);
	}
}
