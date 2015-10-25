//i2c Master(UNO)
#include <Wire.h>
#define unpack754_16(i) (unpack754((i), 16, 5))
float p, y, r, s;
char msg[8];

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  
}

void loop()
{
  Wire.requestFrom(5,8);
  int i = 0;
  while(Wire.available())
  {
    char c = Wire.read();
    Serial.print(c);
    msg[i] = c;
  }
  //unpack_message();
  //Serial.print("Yaw, Pitch, Roll: ");
  //Serial.print(y, 2);
  //Serial.print(", ");
  //Serial.print(p, 2);
  //Serial.print(", ");
  //Serial.println(r, 2);
  //Serial.print("Speed: "); Serial.println(s, 3);
  Serial.println(msg);
  delay(500);
}

unsigned int unpacku16(char *buf)
{
  return ((unsigned int)buf[0]<<8) | buf[1];
}

void unpack_message(){
  // int float_size = 2;
  // memcpy(&pitch, msg, float_size);
  // memcpy(&yaw, msg + 2, float_size);
  // memcpy(&roll, msg + 4, float_size);
  unsigned int fhold = unpacku16(msg);
  p = unpack754_16(fhold);
  fhold = unpacku16(msg + 2);
  y = unpack754_16(fhold);
  fhold = unpacku16(msg + 4);
  r = unpack754_16(fhold);
  fhold = unpacku16(msg + 6);
  s = unpack754_16(fhold);
}

long double unpack754(unsigned long long int i, unsigned bits, unsigned expbits)
{
  long double result;
  long long shift;
  unsigned bias;
  unsigned significandbits = bits - expbits - 1; // -1 for sign bit

  if (i == 0) return 0.0;

  // pull the significand
  result = (i&((1LL<<significandbits)-1)); // mask
  result /= (1LL<<significandbits); // convert back to float
  result += 1.0f; // add the one back on

  // deal with the exponent
  bias = (1<<(expbits-1)) - 1;
  shift = ((i>>significandbits)&((1LL<<expbits)-1)) - bias;
  while(shift > 0) { result *= 2.0; shift--; }
  while(shift < 0) { result /= 2.0; shift++; }

  // sign it
  result *= (i>>(bits-1))&1? -1.0: 1.0;

  return result;
}