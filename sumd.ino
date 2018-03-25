#define CRC_POLYNOME 0x1021

#define SUMD_SYNCBYTE 0xA8
#define SUMD_STATUS 0x01
#define SUMD_MAXCHAN 16
#define SUMD_BUFFSIZE SUMD_MAXCHAN * 2 + 5 // 8 channels + 5 -> 21 bytes for 8 channels

#define CHANNELS 4

static uint8_t sumd[SUMD_BUFFSIZE]={0};
static uint16_t crc = 0;

uint16_t CRC16(uint16_t crc, uint8_t value) {
  uint8_t i;

  crc = crc ^ (int16_t) value << 8;
  for(i = 0; i < 8; i++) {
    if (crc & 0x8000)
      crc = (crc << 1) ^ CRC_POLYNOME;
    else
      crc = (crc << 1);
  }

  return crc;
}

uint8_t* BuildSumD(uint16_t roll, uint16_t pitch, uint16_t yaw, uint16_t throttle,
uint8_t aux1, uint8_t aux2, uint8_t aux3, uint8_t aux4, uint8_t aux5, uint8_t aux6) {
  // static uint8_t sumdIndex = 0;
  // header
  sumd[0] = SUMD_SYNCBYTE;
  sumd[1] = SUMD_STATUS;
  sumd[2] = SUMD_MAXCHAN;

  int i = 1;
  uint16_t throttleValue = map(throttle, 0, 1023, 0x2260, 0x3b60);
  sumd[i*2+1] = throttleValue >> 8;
  sumd[i*2+2] = throttleValue & 0xff;

  i++; // 2

  uint16_t rollValue = map(roll, 0, 1023, 0x2260, 0x3b60);
  sumd[i*2+1] = rollValue >> 8;
  sumd[i*2+2] = rollValue & 0xff;

  i++; // 3

  uint16_t pitchValue = map(pitch, 0, 1023, 0x2260, 0x3b60);
  sumd[i*2+1] = pitchValue >> 8;
  sumd[i*2+2] = pitchValue & 0xff;

  i++; // 4

  uint16_t yawValue = map(yaw, 0, 1023, 0x2260, 0x3b60);
  sumd[i*2+1] = yawValue >> 8;
  sumd[i*2+2] = yawValue & 0xff;

  i++; // 5

  // uint16_t aux1Value = map(aux1, 0x00, 0xff, 0x2260, 0x3b60);
  uint16_t aux1Value = 0x2260;
  if (aux1) aux1Value = 0x3b60;
  sumd[i*2+1] = aux1Value >> 8;
  sumd[i*2+2] = aux1Value & 0xff;

  i++;
  uint16_t aux2Value = 0x2260;
  if (aux2) aux2Value = 0x3b60;
  sumd[i*2+1] = aux2Value >> 8;
  sumd[i*2+2] = aux2Value & 0xff;
  i++;
  uint16_t aux3Value = 0x2260;
  if (aux3) aux3Value = 0x3b60;
  sumd[i*2+1] = aux3Value >> 8;
  sumd[i*2+2] = aux3Value & 0xff;
  i++;
  uint16_t aux4Value = 0x2260;
  if (aux4) aux4Value = 0x3b60;
  sumd[i*2+1] = aux4Value >> 8;
  sumd[i*2+2] = aux4Value & 0xff;
  i++;
  uint16_t aux5Value = 0x2260;
  if (aux5) aux5Value = 0x3b60;
  sumd[i*2+1] = aux5Value >> 8;
  sumd[i*2+2] = aux5Value & 0xff;
  i++;
  uint16_t aux6Value = 0x2260;
  if (aux6) aux6Value = 0x3b60;
  sumd[i*2+1] = aux6Value >> 8;
  sumd[i*2+2] = aux6Value & 0xff;

  i++;

  // data
  for (; i <= SUMD_MAXCHAN; i++) {
    sumd[i*2+1] = 0x2e;
    sumd[i*2+2] = 0xe0;
  }

  // CRC
  for(int i = 0; i < SUMD_BUFFSIZE - 2; i++) {
    crc = CRC16(crc, sumd[i]);
  }
  sumd[(SUMD_MAXCHAN + 1)*2+1] = crc >> 8;
  sumd[(SUMD_MAXCHAN + 1)*2+2] = crc & 0xff;

  // reset data
  // for(int i = 0; i < SUMD_BUFFSIZE; i++) sumd[i] = 0;
  crc = 0;

  return sumd;
}

