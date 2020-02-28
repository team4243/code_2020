#include <Wire.h>
#include <Math.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_FXOS8700.h>

#define BUFFER_SIZE 15
#define TX_THROTTLE 60

#define REJECTION_THRESHOLD 4
#define REJECTION_COUNT_MAX 10

int tx_count = 0;
int rejection_count = 0;

float final_value = 0;
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
  while (Serial.available()) Serial.read();

  accelmag.getEvent(&accel_event);

  float raw_value = (float)accel_event.acceleration.y;
  float accel_value = constrain(raw_value, -9.8, 9.8);

  accel_value = asin(accel_value / 9.8) * (180 / M_PI);

  float absDifference = 0;
  if (final_value > accel_value) absDifference = final_value - accel_value;
  else absDifference = accel_value - final_value;

  if (absDifference < REJECTION_THRESHOLD || ++rejection_count > REJECTION_COUNT_MAX)
  {
    float sum = 0;
    for (int x = BUFFER_SIZE - 1; x > 0; x--)
    {
      averages[x] = averages[x - 1];
      sum += averages[x];
    }
    averages[0] = accel_value;
    sum += averages[0];

    final_value = constrain((sum / (float)BUFFER_SIZE), -45, 45);
    rejection_count = 0;
  }

  if (++tx_count == TX_THROTTLE)
  {
    tx_count = 0;

    Serial.print('#');
    Serial.println((int)(final_value));
  }
}
