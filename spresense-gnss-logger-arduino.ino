
/* include the GNSS library */
#include <GNSS.h>
#include <GNSSPositionData.h>
#include "gpsutils/cxd56_gnss_nmea.h"

#define PIN_SW0 PIN_D02
#define PIN_SW1 PIN_D09
#define PIN_SW2 PIN_D06
#define PIN_SW3 PIN_D05
#define PIN_SW4 PIN_D03


#define STRING_BUFFER_SIZE  128       /**< %Buffer size */
char StringBuffer[STRING_BUFFER_SIZE];

static SpGnss Gnss;                   /**< SpGnss object */

// NMEA
FILE *fp = NULL;
char current_filename[100];
NMEA_OUTPUT_CB funcs;
char nmea_buf[NMEA_SENTENCE_MAX_LEN];
FAR static char *reqbuf(uint16_t size);
static void freebuf(FAR char *buf);
static int outnmea(FAR char *buf);
static int outbin(FAR char *buf, uint32_t len);

/**
 * @brief Activate GNSS device and start positioning.
 */
void setup() {
  /* put your setup code here, to run once: */

  int error_flag = 0;

  /* Set serial baudrate. */
  Serial.begin(115200);

  /* Wait HW initialization done. */
  sleep(3);

  /* Turn on all LED:Setup start. */
  ledOn(PIN_LED0);
  ledOn(PIN_LED1);
  ledOn(PIN_LED2);
  ledOn(PIN_LED3);

  pinMode(PIN_SW0, OUTPUT);
  pinMode(PIN_SW1, OUTPUT);
  pinMode(PIN_SW2, OUTPUT);
  pinMode(PIN_SW3, OUTPUT);
  pinMode(PIN_SW4, OUTPUT);
  

  /* Set Debug mode to Info */
  Gnss.setDebugMode(PrintInfo);

  int result;

  /* Activate GNSS device */
  result = Gnss.begin();

  if (result != 0)
  {
    Serial.println("Gnss begin error!!");
    error_flag = 1;
  }
  else
  {
    /* Setup GNSS
     *  It is possible to setup up to two GNSS satellites systems.
     *  Depending on your location you can improve your accuracy by selecting different GNSS system than the GPS system.
     *  See: https://developer.sony.com/develop/spresense/developer-tools/get-started-using-nuttx/nuttx-developer-guide#_gnss
     *  for detailed information.
    */
    Gnss.select(GPS);
    Gnss.select(QZ_L1CA);
    Gnss.select(QZ_L1S);
    Gnss.select(GLONASS);
    Gnss.select(SBAS);
      
    /* Start positioning */
    result = Gnss.start(COLD_START);
    if (result != 0)
    {
      Serial.println("Gnss start error!!");
      error_flag = 1;
    }
    else
    {
      Serial.println("Gnss setup OK");
    }
  }

  /* Turn off all LED:Setup done. */
  ledOff(PIN_LED0);
  ledOff(PIN_LED1);
  ledOff(PIN_LED2);
  ledOff(PIN_LED3);

  /* Set error LED. */
  if (error_flag == 1)
  {
    ledOn(PIN_LED0);
    ledOn(PIN_LED1);
    ledOn(PIN_LED2);
    ledOn(PIN_LED3);
    exit(0);
  }

  log_init();
}

/**
 * @brief %Print position information.
 */
static void print_pos(SpNavData *pNavData)
{

  /* print time */
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%04d/%02d/%02d ", pNavData->time.year, pNavData->time.month, pNavData->time.day);
  Serial.print(StringBuffer);

  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%02d:%02d:%02d.%06d, ", pNavData->time.hour, pNavData->time.minute, pNavData->time.sec, pNavData->time.usec);
  Serial.print(StringBuffer);

  /* print satellites count */
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "numSat:%2d, ", pNavData->numSatellites);
  Serial.print(StringBuffer);

  /* print position data */
  if (pNavData->posFixMode == FixInvalid)
  {
    Serial.print("No-Fix, ");
  }
  else
  {
    Serial.print("Fix, ");
  }
  if (pNavData->posDataExist == 0)
  {
    Serial.print("No Position");
  }
  else
  {
    Serial.print("Lat=");
    Serial.print(pNavData->latitude, 6);
    Serial.print(", Lon=");
    Serial.print(pNavData->longitude, 6);
  }

  Serial.println("");
}

/**
 * @brief %Print satellite condition.
 */
static void print_condition(SpNavData *pNavData)
{
  char StringBuffer[STRING_BUFFER_SIZE];
  unsigned long cnt;

  /* Print satellite count. */
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "numSatellites:%2d\n", pNavData->numSatellites);
  Serial.print(StringBuffer);

  for (cnt = 0; cnt < pNavData->numSatellites; cnt++)
  {
    const char *pType = "---";
    SpSatelliteType sattype = pNavData->getSatelliteType(cnt);

    /* Get satellite type. */
    /* Keep it to three letters. */
    switch (sattype)
    {
      case GPS:
        pType = "GPS";
        break;

      case GLONASS:
        pType = "GLN";
        break;

      case QZ_L1CA:
        pType = "QCA";
        break;

      case SBAS:
        pType = "SBA";
        break;

      case QZ_L1S:
        pType = "Q1S";
        break;

      default:
        pType = "UKN";
        break;
    }

    /* Get print conditions. */
    unsigned long Id  = pNavData->getSatelliteId(cnt);
    unsigned long Elv = pNavData->getSatelliteElevation(cnt);
    unsigned long Azm = pNavData->getSatelliteAzimuth(cnt);
    float sigLevel = pNavData->getSatelliteSignalLevel(cnt);

    /* Print satellite condition. */
    snprintf(StringBuffer, STRING_BUFFER_SIZE, "[%2d] Type:%s, Id:%2d, Elv:%2d, Azm:%3d, CN0:", cnt, pType, Id, Elv, Azm );
    Serial.print(StringBuffer);
    Serial.println(sigLevel, 6);
  }
}


void log_init(){
  NMEA_InitMask();
  NMEA_SetMask(0x000040ff);
  funcs.bufReq  = reqbuf;
  funcs.out     = outnmea;
  funcs.outBin  = outbin;
  funcs.bufFree = freebuf;
  NMEA_RegistOutputFunc(&funcs);
}

void log_open(char *filename){
  printf("Open %s\n", filename);
  fp = fopen(filename, "a");
  if (fp == NULL) {
    ledOff(PIN_LED3);
    printf("Open error %s\n", filename);
  }else{
    ledOn(PIN_LED3);
  }
}


void log_write(struct cxd56_gnss_positiondata_s *posdatp){

  snprintf(
    StringBuffer,
    STRING_BUFFER_SIZE,
    "%04d/%02d/%02d %02d:%02d:%02d",
    posdatp->receiver.date.year, posdatp->receiver.date.month, posdatp->receiver.date.day,
    posdatp->receiver.time.hour, posdatp->receiver.time.minute, posdatp->receiver.time.sec
  );
  Serial.println(StringBuffer);
  
  if (fp==NULL && posdatp->receiver.date.year>2000) {
    sprintf(
      current_filename, "/mnt/sd0/%04d%02d%02d-%02d%02d%02d.nmea",
      posdatp->receiver.date.year, posdatp->receiver.date.month, posdatp->receiver.date.day,
      posdatp->receiver.time.hour, posdatp->receiver.time.minute, posdatp->receiver.time.sec
    );
    log_open(current_filename);
  }
  NMEA_Output(posdatp);
  if (fp!=NULL) {
    ledOff(PIN_LED3);
    if(posdatp->receiver.time.sec == 59 || digitalRead(PIN_SW0) == 0){
      fflush(fp);
      int fd = fileno(fp);
      fsync(fd);
    }
    if(posdatp->receiver.time.minute == 59 && posdatp->receiver.time.sec == 59){
      fclose(fp);
      fp = NULL;
    }
    ledOn(PIN_LED3);
  }
}

/* output NMEA */
FAR static char *reqbuf(uint16_t size)
{
  if (size > sizeof(nmea_buf))
    {
      snprintf(StringBuffer, STRING_BUFFER_SIZE, "reqbuf error: oversize %s\n", size);
      Serial.println(StringBuffer);
      return NULL;
    }
  return nmea_buf;
}

static void freebuf(FAR char *buf)
{
}

static int outnmea(FAR char *buf)
{
  int ret = 0;
  if (fp != NULL){
    ledOff(PIN_LED3);
    ret = fputs(buf, fp);
    ledOn(PIN_LED3);
  }
  Serial.print(buf);
  return ret;
}

static int outbin(FAR char *buf, uint32_t len)
{
  return len;
  //return write(WRITE_FD, buf, (size_t)len);
}


/**
 * @brief %Print position information and satellite condition.
 *
 * @details When the loop count reaches the RESTART_CYCLE value, GNSS device is
 *          restarted.
 */
void loop()
{
  /* put your main code here, to run repeatedly: */

  static int LoopCount = 0;
  static int LastPrintMin = 0;

  
  /* Check update. */
  if (Gnss.waitUpdate(-1))
  {
    /* Blink LED. */
    ledOn(PIN_LED0);
    /* Get NaviData. */
    SpNavData NavData;
    Gnss.getNavData(&NavData);

    /* Set posfix LED. */
    if(NavData.posDataExist && (NavData.posFixMode != FixInvalid)){
      ledOn(PIN_LED1);
    }else{
      ledOff(PIN_LED1);
    }

    /* Print satellite information every minute. */
    if (NavData.time.minute != LastPrintMin)
    {
      print_condition(&NavData);
      LastPrintMin = NavData.time.minute;
    }

    /* Print position information. */
    print_pos(&NavData);


    GnssPositionData PosData;
    Gnss.getPositionData((char *)&PosData);

    struct cxd56_gnss_positiondata_s *cxd56_gnss_positiondata_p = (struct cxd56_gnss_positiondata_s *)&PosData.Data;
    log_write(cxd56_gnss_positiondata_p);
  }
  else
  {
    /* Not update. */
    Serial.println("data not update");
  }
  ledOff(PIN_LED0);

}



