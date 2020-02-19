#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Mahony.h>
#include <Madgwick.h>

#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>

#define MAX_REJECTIONS 5
#define REJECTION_THRESHOLD 20

#define AVG_BUFFER 10

int averages[AVG_BUFFER];

int last_angle = 0;
int rejection_count = 0;

int pitch_offset = 0;
double accel_offset = 0;

Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

float mag_offsets[3] = { 45.42F, 68.06F, 96.85F };

float mag_softiron_matrix[3][3] = {
  { 0.99, -0.029, 0.008 },
  { -0.029, 0.989, 0.012 },
  { 0.008, 0.012, 1.022 }
};

float mag_field_strength = 52.16F;
float gyro_zero_offsets[3] = { 0.0F, 0.0F, 0.0F };

//Mahony filter;
Madgwick filter;

void setup()
{
  Serial.begin(115200);
  zero();

  if (!gyro.begin()) {}

  if (!accelmag.begin(ACCEL_RANGE_2G)) {}

  // Filter expects 70 samples per second for Ada-Feather
  filter.begin(10);
}

void loop(void)
{
  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;

  gyro.getEvent(&gyro_event);
  accelmag.getEvent(&accel_event, &mag_event);

  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  float gx = gyro_event.gyro.x + gyro_zero_offsets[0];
  float gy = gyro_event.gyro.y + gyro_zero_offsets[1];
  float gz = gyro_event.gyro.z + gyro_zero_offsets[2];

  gx *= 57.2958F;
  gy *= 57.2958F;
  gz *= 57.2958F;

  filter.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz);

  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float heading = filter.getYaw();

  int angle = (int)(pitch - pitch_offset);
  angle = constrain(angle, -45, 45);

  int diff = angle - last_angle;
  if (diff > REJECTION_THRESHOLD || diff < -REJECTION_THRESHOLD)
  {
    if (++rejection_count < MAX_REJECTIONS) return;
  }

  int sum = 0;
  for (int x = 0; x < sizeof(averages) - 1; x++)
  {
    averages[x] = averages[x + 1];
    sum += averages[x];
  }
  averages[sizeof(averages) - 1] = angle;
  sum += averages[sizeof(averages) - 1];

  sum /= sizeof(averages);

  last_angle = sum;
  rejection_count = 0;

  if (Serial.available())
  {
    zero();
    pitch_offset = angle;
    accel_offset = accel_event.acceleration.x;
    while (Serial.available())Serial.read();
  }

  Serial.print('#');
  //  Serial.print(angle);
  //  Serial.print("      #");
  Serial.println((int)(map(accel_event.acceleration.x - accel_offset, -9.8, 9.8, 45, -45) * 1.5));
  delay(100);
}

void zero()
{
  for (int x = 0; x < sizeof(averages); x++) averages[x] = 0;
}
