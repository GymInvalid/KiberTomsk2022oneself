#include <QTRSensors.h>

#define NUM_SENSORS 8 // Сколько сенсоров используется
#define Min_power -1 // 0, 10
#define Max_forward 180 //230, 220

QTRSensorsAnalog qtra((unsigned char[]) {A7, A6, A5, A4, A3, A2, A1, A0}, NUM_SENSORS); // Порта к которым подключены датчики
unsigned int sensorValues[NUM_SENSORS];

float kp = 8; //Коэффициент пропорционального звена
float kd = 16;

// Создаём переменную в которую будет записываться мощность моторов
int P1 = 0;
int P2 = 0;

float P8; //Значение, испльзуемое для изменения мощности моторов
float old_error;

//Порта к которым подключены моторы
int INA = 8;
int INB = 7;
int ENA = 9;
int INc = 5;
int IND = 4;
int ENB = 3;

void setup()
{
  Serial.begin(9600); // Устанавливаем скорость монитора порта
  
// Объявляем что порта к которым подключены моторы выход
pinMode (INA, OUTPUT);
pinMode (INB, OUTPUT);
pinMode (INc, OUTPUT);
pinMode (IND, OUTPUT);
pinMode (ENA, OUTPUT);
pinMode (ENB, OUTPUT);
delay(500); // Задержка 500 млс

pinMode(13, OUTPUT); // Объявляем светодиод на плате как выход
digitalWrite(13, HIGH); // Включаем светодиод на плате

// Калибровка датчиков и вывод значений в монитор порта
for (int i = 0; i < 400; i++) //
{
  qtra.calibrate(); // Метод функции, калибровка
}
digitalWrite(13, LOW);// выключаем светодиод на плате по окончанию колибровки
delay(1000); // Задержка 1 секунда
}

void loop()
{
unsigned int sensorValues[8];
int position = qtra.readLine(sensorValues);

for (unsigned char i = 0; i < NUM_SENSORS; i++)
{
int error = (position - 3500)/100;
Serial.println(error);

int up = (error * kp); // Рассчитываем мощность - еrror ,ошибку умножаем на коэффициент
int ud = kd * (error - old_error);
old_error = error;
P8 = up+ud;

P1 = Max_forward - P8;
P2 = Max_forward + P8;
delay(5);

if (P1 > 0) {
  digitalWrite(INA, HIGH);
  digitalWrite(INB, LOW);
}
else {
  digitalWrite(INA, LOW);
  digitalWrite(INB, HIGH);
}

if (P2 > 0) {
  digitalWrite(INc, LOW);
  digitalWrite(IND, HIGH);
}
  else {
   digitalWrite(INc, HIGH);
   digitalWrite(IND, LOW); 
  }

  if (P1 > 255){
    P1 = 255;
  }
  if (P2 > 255){
    P2 = 255;
  }
  
analogWrite(ENB, P2);
analogWrite(ENA, P1);
}
