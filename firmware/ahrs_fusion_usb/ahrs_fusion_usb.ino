#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FXOS8700.h>

#define BUFFER_SIZE 25
#define TX_THROTTLE 50
#define REJECTION_THRESHOLD 5

int tx_count = 0;

float final_value = 0;
float accel_offset = 0;
float averages[BUFFER_SIZE];

Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

sensors_event_t accel_event;

void setup()
{
  Serial.begin(115200);
  for (int x = 0; x < BUFFER_SIZE; x++) averages[x] = 0;

  accelmag.begin(ACCEL_RANGE_2G);
  delay(100);
}

void loop(void)
{
  if (Serial.available())
  {
    accel_offset = accel_event.acceleration.x;

    while (Serial.available()) Serial.read();
    for (int x = 0; x < BUFFER_SIZE; x++) averages[x] = 0;
  }

  accelmag.getEvent(&accel_event);

  float accel_value = (float)accel_event.acceleration.x - accel_offset;

  accel_value = map(accel_value, -9.8, 9.8, -45, 45);

  float sum = 0;
  for (int x = BUFFER_SIZE - 1; x > 0; x--)
  {
    averages[x] = averages[x - 1];
    sum += averages[x];
  }
  averages[0] = accel_value;
  sum += averages[0];

  final_value = constrain((sum / (float)BUFFER_SIZE), -45, 45);

  if (++tx_count == TX_THROTTLE)
  {
    tx_count = 0;

    Serial.print('#');
    Serial.println((int)(final_value));
    Serial.println();
  }
}
