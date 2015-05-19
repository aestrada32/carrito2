#define vel1 13  //pin 13, switch 1
#define vel2 12  //pin 12, switch 2
#define vel3 11  //pin 11, switch 3
#define standBy 10  //pin 10, switch 4
#define enableH 5  //pin 5, puenteH 5
#define direccion 6  //pin 6, puenteH 6
#define ultrasonico 0  //pin A0, ultrasonico 1 AN
#define btnIniciar 4 //pin 0, boton inicio verde
#define btnReset 7 //pin 1, boton reset rojo
#define qrd 1  //pin 2, sensorQRD 2

volatile int correr;
volatile int corriendo;
volatile int contador;
String datos;
long cm;
double dc;
double dc1;
double dcHold;
int v1;
int v2;
int v3;
int suma;
int c;
int c1;
int tiempo;
int a;
long t;
int distancia;
int revoluciones;
int distanciaTotal;
double velEsperada;
double velCalculada;
double disRecorrida;
double disObjeto;

double error; double errorAnterior1; double errorAnterior2;
double pid; double pidAnterior1; double pidAnterior2;
double lc; double lcAnterior; double carroPID;
double velEsperadaAnterior1; double velEsperadaAnterior2;
double pidFormat; double auxiliar;

void setup() {
  pinMode(vel1, INPUT);
  pinMode(vel2, INPUT);
  pinMode(vel3, INPUT);
  pinMode(standBy, INPUT);
  pinMode(direccion, OUTPUT);
  pinMode(btnIniciar, INPUT);
  pinMode(btnReset, INPUT);
  attachInterrupt(qrd, contarRevoluciones, FALLING);
  velEsperada = 0;
  velCalculada = 0;
  disRecorrida = 0;
  disObjeto = 0;
  contador = 0;
  corriendo = 0;
  correr = 0;
  c1 = 1;
  c = 0;
  tiempo = 0;
  t = 0;
  a = 1;
  carroPID = 0; lcAnterior = 0; errorAnterior1 = 0; errorAnterior2 = 0; pidAnterior1 = 0; pidAnterior2 = 0; velEsperadaAnterior1 = 0; velEsperadaAnterior2 = 0; pidFormat = 0;  //Todo para el control PID
  distanciaTotal = 400;
  Serial.begin(9600);
  interrupts();
}

void loop() {
  if(digitalRead(standBy) == 1) c = 1;
  else { c = 0; correr = 0; }
  if(digitalRead(btnIniciar) == 1 && a == 0){
    a = 1;
    iniciar();
  }
  
  while(c == 1 && c1 == 1){
    a = 0;
    if(digitalRead(standBy) == 1) c = 1;
    else { c = 0; correr = 0; }
    if(correr == 1){
      if(corriendo == 0){
        carroPID = 0; lcAnterior = 0; errorAnterior1 = 0; errorAnterior2 = 0; pidAnterior1 = 0; pidAnterior2 = 0; velEsperadaAnterior1 = 0; velEsperadaAnterior2 = 0; pidFormat = 0;  //Todo para el control PID
        velEsperada = velocidadEsperada();
        corriendo = 1;
        datos = "";
        digitalWrite(direccion, 0);
        analogWrite(enableH, 150);
        //analogWrite(enableH, 0);
      }
      if(tiempo == 0) {
        t = millis() + 454;  //4tau/10 = 4*1.135/10 = 0.454
        tiempo = 1;
        revoluciones = 0;
      }
      if(t <= millis()) {  //El ciclo
        tiempo = 0;
        disRecorrida = revoluciones * 16;
        velCalculada = disRecorrida/(0.454);
        //lcAnterior = (velCalculada)/100;
        lcAnterior = lc;
        controlPID();
        digitalWrite(direccion, 0);
        analogWrite(enableH, carroPID);
        String d = "Velocidad esperada: " + String(velEsperada*100) + " cm/s | Velocidad medida: " + String(velCalculada) + " cm/s | Calculo PID(u): " + String(carroPID) + " | Distancia Objeto: " + String(disObjeto) + " cm | Distancia Total Recorrida: " + String(distancia) + " cm";
        Serial.println(d);
        datos += d + "\n";
        contador++;
      }
      if(distancia > distanciaTotal){ c1 = 0; distancia = 0; c = 0; }
    }
   if(digitalRead(btnReset) == 1){
      reset();
    }
    revisarObjeto();
  }
  apagar();
  if(digitalRead(btnIniciar) == 1 && a == 0){
    a = 1;
    Serial.println(datos);
  }
}

void controlPID(){
  error = velEsperada - lcAnterior;
  pid = calcularPID();
  pidFormat = pid;
  if(pidFormat < 0) pidFormat = 0;
  if(pidFormat > 1) pidFormat = 1;
  lc = ((34.3939*lcAnterior)+(4.3*pidFormat))/35.3939;
  carroPID = (lc*255)/2.15;
  errorAnterior2 = errorAnterior1;
  errorAnterior1 = error;
  pidAnterior2 = pidAnterior1;
  pidAnterior1 = pid;
}

double calcularPID(){
  double n = -((-1.458/1.61396) * pidAnterior1) - ((0.048/1.61396) * pidAnterior2) + ((2.9447/1.61396) * error) + ((-3.27231/1.61396) * errorAnterior1) + ((1.61396/1.61396) * errorAnterior2);
  return n;
}

void contarRevoluciones() {
  if(tiempo == 1){
    revoluciones++;
    distancia += 16;
  }
}

void revisarOnOff(){
  if(digitalRead(standBy) == 1) c = 1;
  else c = 0;
}

void revisarObjeto(){
  leerMaxSonar();
  if(cm < 150){
    apagar();
  }
  else{
    correr = 1;
  }
}

void leerMaxSonar() {
  cm = (analogRead(ultrasonico) / 2 ) * 2.54;
  disObjeto = cm;
}

void detenerCarro(){
  double tiempoFrenado = (dc1*1500)/255;
  revoluciones = 0;
  tiempo = 0;
  t = 0;  
  analogWrite(enableH, 0);
  if(tiempoFrenado < 500) tiempoFrenado = 500;
  delay(tiempoFrenado);
  digitalWrite(direccion, 1);
  analogWrite(enableH, 255);
  delay(100);
  analogWrite(enableH, 0);
  digitalWrite(direccion, 0);
}

double velocidadEsperada(){
  v1 = digitalRead(vel1) * 1;
  v2 = digitalRead(vel2) * 2; 
  v3 = digitalRead(vel3) * 4;
  suma = v1 + v2 + v3;
  double dc2 = 0;
  dc1 = 0;
  dc = ((suma * 75) / 7) + 75;
  dc1 = (dc*255)/150;
  dc2 = (dc1*2.15)/255;
  return dc2;
}

void iniciar(){
  correr = 1;
  c1 = 1;
}

void reset(){
  c1 = 0;
  distancia = 0;
}

void apagar(){
  if(corriendo == 1){
    detenerCarro();
    corriendo = 0;
    correr = 0;
  }
}
