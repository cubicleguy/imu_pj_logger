/* 
 * Copyright(C) SEIKO EPSON CORPORATION 2022. All rights reserved.
 *
 * imu_pj_logger is Linux SocketCAN based J1939 Logger for Epson IMU
 * 
 * This software is distributed as is, without any warranty of any kind,
 * either express or implied as further specified in the GNU Public License. This
 * software may be used and distributed according to the terms of the GNU Public
 * License, version 2. See the file COPYING in the main directory of this archive
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * gcc main.c -o imu_pj_logger -lpthread -Wall
 */

#include <inttypes.h>
#include <linux/can/j1939.h>
#include <sys/socket.h>
#include <net/if.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <getopt.h>
#include <err.h>
#include <errno.h>
#include <sys/time.h>


#define LOGGER_SW_VERSION   "1.0"

#define PGN_ADDR_CLAIMED    0x00EE00 //< Claim an address to use
#define PGN_COMMMAND        0x00EF00 //< Command to the device
#define PGN_RESPONSE        0x00FFE0 //< Response for a command from the device
#define PGN_SOUT1           0x00FFE1 //< Angular rate data output
#define PGN_SOUT2           0x00FFE2 //< Acceleration data output
#define PGN_SOUT4           0x00FFE4 //< Temperature data
#define PGN_SOUT5           0x00FFE5 //< The acquisition time of the sensor data
#define PGN_SOUT7           0x00FFE7 //< Attitude angle data output

#define CMD_SAMPLES         0x00000001
#define CMD_REGREAD         0x00000002
#define CMD_REGWRITE        0x00000004
#define CMD_REGDUMP         0x00000008
#define CMD_REGMODIFY       0x00000010
#define CMD_MODELVERS       0x00000020
#define CMD_READERROR       0x00000040
#define CMD_TIMESTAMP_ONLY  0x00000080

#define HOST_ADDR        0xFD

#define REND             0xFF 
#define RNON             0xFE
#define STRL             512

#define TIMEOUT_TO_QUIT    1 //< sec

const char * SAMPLE_RATE_A[] = {
  "unknown",
  "1000 Sps",
  "500 Sps",
  "250 Sps",
  "200 Sps",
  "125 Sps",
  "100 Sps",
  "50 Sps",
  "25 Sps",
};
uint32_t SAMPLE_RATE_A_SIZE = sizeof SAMPLE_RATE_A / sizeof SAMPLE_RATE_A[0];

float SAMPLE_SEC_A[] = {
  0.000f,
  0.001f,
  0.002f,
  0.004f,
  0.005f,
  0.008f,
  0.010f,
  0.020f,
  0.040f
};

const char * FILTER_A[] = {
  "unknown",
  "Moving average filter TAP=2",
  "Moving average filter TAP=4",
  "Moving average filter TAP=8",
  "Moving average filter TAP=16",
  "Moving average filter TAP=32",
  "Moving average filter TAP=64",
  "Moving average filter TAP=128",
  "FIR Kaiser filter TAP=32 and fc=25Hz",
  "FIR Kaiser filter TAP=32 and fc=50Hz",
  "FIR Kaiser filter TAP=32 and fc=100Hz",
  "FIR Kaiser filter TAP=32 and fc=200Hz",
  "FIR Kaiser filter TAP=64 and fc=25Hz",
  "FIR Kaiser filter TAP=64 and fc=50Hz",
  "FIR Kaiser filter TAP=64 and fc=100Hz",
  "FIR Kaiser filter TAP=64 and fc=200Hz",
  "FIR Kaiser filter TAP=128 and fc=25Hz",
  "FIR Kaiser filter TAP=128 and fc=50Hz",
  "FIR Kaiser filter TAP=128 and fc=100Hz",
  "FIR Kaiser filter TAP=128 and fc=200Hz"
};
uint32_t FILTER_A_SIZE = sizeof FILTER_A / sizeof FILTER_A[0];

const char * SAMPLE_RATE_B[] = {
  "unknown",
  "1000 Sps",
  "500 Sps",
  "400 Sps",
  "250 Sps",
  "200 Sps",
  "125 Sps",
  "100 Sps",
  "80 Sps",
  "50 Sps",
  "25 Sps"
};
uint32_t SAMPLE_RATE_B_SIZE = sizeof SAMPLE_RATE_B / sizeof SAMPLE_RATE_B[0];

float SAMPLE_SEC_B[] = {
  0.0000f,
  0.0010f,
  0.0020f,
  0.0025f,
  0.0040f,
  0.0050f,
  0.0080f,
  0.0100f,
  0.0125f,
  0.0200f,
  0.0400f
};

const char * FILTER_B[] = {
  "unknown",
  "Moving average filter TAP=2",
  "Moving average filter TAP=4",
  "Moving average filter TAP=8",
  "Moving average filter TAP=16",
  "Moving average filter TAP=32",
  "Moving average filter TAP=64",
  "Moving average filter TAP=128",
  "FIR Kaiser filter TAP=32 and fc=50Hz",
  "FIR Kaiser filter TAP=32 and fc=100Hz",
  "FIR Kaiser filter TAP=32 and fc=200Hz",
  "FIR Kaiser filter TAP=32 and fc=400Hz",
  "FIR Kaiser filter TAP=64 and fc=50Hz",
  "FIR Kaiser filter TAP=64 and fc=100Hz",
  "FIR Kaiser filter TAP=64 and fc=200Hz",
  "FIR Kaiser filter TAP=64 and fc=400Hz",
  "FIR Kaiser filter TAP=128 and fc=50Hz",
  "FIR Kaiser filter TAP=128 and fc=100Hz",
  "FIR Kaiser filter TAP=128 and fc=200Hz",
  "FIR Kaiser filter TAP=128 and fc=400Hz"
};
uint32_t FILTER_B_SIZE = sizeof FILTER_B / sizeof FILTER_B[0];

const char * MOTION_PROFILE[] = { "3 m/s", "20 m/s", "1 m/s" };

const char * strSoutHeader[] = {
  "Gx[dps],Gy[dps],Gz[dps],",
     "Ax[mG],Ay[mG],Az[mG],",
            "[not defined],",
                 "T[deg.C],",
         "Timestamp[SS.ms],",
            "[not defined],",
                    "a1,a2,"};
                     
const char * strSoutHeaderRaw[] = {
  "Gx[hex],Gy[hex],Gz[hex],",
  "Ax[hex],Ay[hex],Az[hex],",
            "[not defined],",
                   "T[hex],",
         "Day[hex],ms[hex],",
            "[not defined],",
          "a1[hex],a2[hex],"};

const char COMMAS[] = ",,,,,,,,,,,,,,,\n";
char can_ch[] = "can0";
char strAppStartTime[32];

uint32_t LogFile = 0;
uint32_t timeUpdInt = 0;
uint32_t timeStampInHumanForm = 0;
uint32_t command = CMD_SAMPLES;
uint32_t limit_cnt = (uint32_t)-1;
uint32_t RawOutput = 0;
uint32_t todo_send = 1;
uint32_t todo_recv = 1;

uint8_t node_addr = J1939_NO_ADDR;
uint8_t host_addr = HOST_ADDR;
uint8_t id_cnt = 0;
uint8_t rx_data[128];
uint8_t tx_data[8];
uint8_t tx_mask[8];
uint8_t G552PJ_100100_Regs[] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,REND/*end*/};
uint8_t G552PJ_100700_Regs[] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,RNON,RNON,RNON,RNON,0x0B,0x0C,0x0D,REND/*end*/};

int rx_cnt = TIMEOUT_TO_QUIT;
int verbose = 0;

pthread_t idrx, idtx;
typedef void(*func_ptr_t)(uint8_t);

typedef struct {
    uint64_t  dev_name;
    uint64_t  version;
    uint64_t  serial_l;
    uint64_t  serial_h;
     int64_t  smplTime;
    uint32_t  flag;
    uint32_t  smplHwCnt;
    uint32_t  smplSwCnt;
     uint8_t  atti_ctrl1;
     uint8_t  atti_ctrl2;
     uint8_t  smpl_rate;
     uint8_t  fltr_sel;
     uint8_t  cfg1_b1;  
     uint8_t  set_addr;
     uint8_t  can_rate;
     uint8_t  can_pri;
     uint8_t  sout_en;
     uint8_t  commasTail;   
     uint8_t  regs_cnt;
     uint8_t  pass;
     uint8_t *regs;
        char  infoString[STRL];
        char  smplString[STRL];
        char  lgfString[STRL];
  func_ptr_t  func_ptr_sout1;
  func_ptr_t  func_ptr_sout2;
  func_ptr_t  func_ptr_sout4;
  func_ptr_t  func_ptr_sout5;
  func_ptr_t  func_ptr_sout7;
       float  smplSec;
        FILE *pLogFile;
        FILE *pErrFile;
} eps_device;
  
eps_device eps_dev[254];

void checkErrors(uint8_t addr, const char * msg)
{
  eps_device * p = &eps_dev[addr];
  if (p->pErrFile == NULL) {
    char file_name[128];
    sprintf(file_name, "%.6s ID:%02xh Errors %s.csv", (char*)&p->dev_name, addr, strAppStartTime);      
    if ((p->pErrFile = fopen(file_name, "wb")) == NULL) { 
      puts("Error! opening Error file");
      return;
    } 
    fputs("message,sw cnt, hw cnt,error\n", p->pErrFile);
  } 
  if (p->smplHwCnt != rx_data[0])
    fprintf(p->pErrFile,  "%s,%3u,%3u,%02x\n", msg, (uint32_t)(p->smplSwCnt+1), rx_data[0], rx_data[1]);
  else
    fprintf(p->pErrFile,  "%s,%3u,%3u,%02x\n", msg,            p->smplSwCnt,  p->smplHwCnt, rx_data[1]);
}

void printSample(uint8_t addr, uint8_t cnt, char * smpl, char * tlg)
{
  eps_device * p = &eps_dev[addr];
  
  if (p->smplSwCnt == 0) {
    float time = p->smplSwCnt * p->smplSec;
    p->smplHwCnt = cnt;
    sprintf(p->smplString, "Id:%02x Sw:%3u Hw:%3u S:%.4f %s", addr, p->smplSwCnt, p->smplHwCnt, time, smpl);
    if (p->pLogFile) {
      sprintf(p->lgfString, "%3u,%.4f%s", p->smplSwCnt, time, tlg);
    }
    p->smplSwCnt++;
  } else if (p->smplHwCnt == cnt) {
    strcat(p->smplString, smpl);
    if (p->pLogFile) {
      strcat(p->lgfString, tlg); 
    }    
  } else {
    float time = p->smplSwCnt * p->smplSec;
    if(rx_cnt<TIMEOUT_TO_QUIT) {
      rx_cnt++;
    }
    p->smplHwCnt = cnt;
    puts(p->smplString);
    sprintf(p->smplString, "Id:%02x Sw:%3u Hw:%3u S:%.4f %s", addr, p->smplSwCnt, p->smplHwCnt, time, smpl);
    if (p->pLogFile) {
      char hwcntStr[16];
      sprintf(hwcntStr, ",%3u", (uint8_t)(cnt-1));
      strcat(p->lgfString, hwcntStr);
      strcat(p->lgfString, COMMAS+p->commasTail);
      fputs(p->lgfString, p->pLogFile);
      sprintf(p->lgfString, "%3u,%.4f%s", p->smplSwCnt, time, tlg);
    }
    p->smplSwCnt++;
  }
}

void gyroscope(uint8_t addr)
{
  uint16_t x, y, z;
  char smp[64];
  char tlg[64];
    
  x   = rx_data[2];
  x  += rx_data[3]<<8;
  
  y   = rx_data[4];
  y  += rx_data[5]<<8;

  z   = rx_data[6];
  z  += rx_data[7]<<8;
  
  if ( RawOutput ) {
    sprintf(smp, "Gx:%04x Gy:%04x Gz:%04x ", x, y, z);
    if (LogFile) sprintf(tlg, ",%04x,%04x,%04x", x, y, z);
  } else {
    float Gx, Gy, Gz;
    
    Gx  = (float)x;
    Gx /= 66.0f;
    Gx += -450.0f;
    
    Gy  = (float)y;
    Gy /= 66.0f;
    Gy += -450.0f;
    
    Gz  = (float)z;
    Gz /= 66.0f;
    Gz += -450.0f;
      
    sprintf(smp, "Gx:%+.8f Gy:%+.8f Gz:%+.8f ", Gx, Gy, Gz);
    if (LogFile) sprintf(tlg, ",%+.8f,%+.8f,%+.8f", Gx, Gy, Gz);
  }
  
  if (rx_data[1]) checkErrors(addr, "gyroscope");
  printSample(addr, rx_data[0], smp, tlg);
}

void accelerometer(uint8_t addr)
{
  uint16_t x, y, z;
  char smp[64];
  char tlg[64];

  x   = rx_data[2];
  x  += rx_data[3]<<8;

  y   = rx_data[4];
  y  += rx_data[5]<<8;
  
  z   = rx_data[6];
  z  += rx_data[7]<<8;

  if ( RawOutput ) {
    sprintf(smp, "Ax:%04x Ay:%04x Az:%04x ", x, y, z);
    if (LogFile) sprintf(tlg, ",%04x,%04x,%04x", x, y, z);
  } else {
    float Ax, Ay, Az;
    
    Ax  = (float)x;
    Ax *= 0.4f;
    Ax += -10000.0f;
    
    Ay  = (float)y;
    Ay *= 0.4f;
    Ay += -10000.0f;  
    
    Az  = (float)z;
    Az *= 0.4f;
    Az += -10000.0f;  
      
    sprintf(smp, "Ax:%+.8f Ay:%+.8f Az:%+.8f ", Ax, Ay, Az);
    if (LogFile) sprintf(tlg, ",%+.8f,%+.8f,%+.8f", Ax, Ay, Az);
  }
  
  if (rx_data[1]) checkErrors(addr, "accelerometer");
  printSample(addr, rx_data[0], smp, tlg); 
}

void temperature(uint8_t addr)
{
  int16_t temp;
  char smp[64];
  char tlg[64];
  
  temp  = rx_data[2]<<0;
  temp += rx_data[3]<<8;
  
  if ( RawOutput ) {
    sprintf(smp, "Ts:%04x ", temp);
    if (LogFile) sprintf(tlg, ",%04x", temp);    
  } else {
    float Ts = (float)temp;
    Ts   -= 2634.0f;
    Ts   *= -0.0037918f;
    Ts   += 25.0f;
    sprintf(smp, "Ts:%+.8f ", Ts);
    if (LogFile) sprintf(tlg, ",%+.8f", Ts);
  }
  
  if (rx_data[1]) checkErrors(addr, "temperature");
  printSample(addr, rx_data[0], smp, tlg);  
}

void dayms(uint8_t addr)
{
  uint32_t msec, days;
  char smp[128];
  char tlg[128];
  
  msec  = rx_data[2];
  msec += rx_data[3]<<8;
  msec += rx_data[4]<<16;
  msec += rx_data[5]<<24;
  days = rx_data[6]+(rx_data[7]<<8);
  
  if ( RawOutput ) {
    sprintf(smp, "dy:%04x ms:%08x ", days, msec);
    if (LogFile) sprintf(tlg, ",%04x,%08x", days, msec);
  } else if (timeStampInHumanForm) {
    uint64_t DaysInSec = days*24*60*60;  
    char buf[80];
    time_t rawtime = (time_t)(msec/1000 + DaysInSec); /*in sec*/   
    struct tm ts = *localtime(&rawtime); 
    
    strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S", &ts);          
    sprintf(smp, "Timestamp:%s.%03u ", buf, msec%1000);
    if (LogFile) {
      sprintf(tlg, ",%s.%03u",  buf, msec%1000);
    }
  } else {
    uint64_t DaysInSec = days*24*60*60;
    sprintf(smp, "Timestamp:%lu.%03u ", DaysInSec + msec/1000, msec%1000);
    if (LogFile) sprintf(tlg, ",%lu.%03u", DaysInSec + msec/1000, msec%1000);
  }

  if (rx_data[1]) checkErrors(addr, "timestamp");
  printSample(addr, rx_data[0], smp, tlg);    
}
    
void attitude(uint8_t addr)
{
  uint16_t ang1, ang2;
  char smp[64];
  char tlg[64];
  
  ang1  = rx_data[2];
  ang1 += rx_data[3] << 8;    
  ang2  = rx_data[4];
  ang2 += rx_data[5] << 8; 
   
  if ( RawOutput ) { 
    sprintf(smp, "a1:%04x a2:%04x ", ang1, ang2);
    if (LogFile) sprintf(tlg, ",%04x,%04x", ang1, ang2);   
  } else {
    float roll  = (float)ang1;
    float pitch = (float)ang2;
    
    roll  *= 0.01f;
    roll  += -320.0f;

    pitch *= 0.01f;
    pitch += -320.0f;
    
    sprintf(smp, "a1:%+.8f a2:%+.8f ", roll, pitch);
    if (LogFile) sprintf(tlg, ",%+.8f,%+.8f", roll, pitch);
  }
  
  if (rx_data[1]) checkErrors(addr, "attitude");
  printSample(addr, rx_data[0], smp, tlg);    
}

void ignore(uint8_t addr){/*empty*/}

void response(uint8_t addr, int len)
{
  if ( rx_data[0] == 0 ) {  //< if Read without error
    if ( ! (command & CMD_REGMODIFY) ) {
      int st = (command & CMD_REGDUMP) ? 1/*show reg addr*/ : 2/*hide reg addr*/;
      printf("%02x:", addr);     
      for (int i=st; i < len; ++i) {
        printf(" %02x", rx_data[i]);
      }
      printf("\n");
    } 
  } else if ( rx_data[0] & 0x0F ) { //< if error
    printf("%02x:", addr);  
    if ( rx_data[0] & 0x80 ) printf(" Write error (not in cfg mode?)\n");  
    else                     printf(" Read error\n");  
  }
}

void printError(uint8_t addr)
{
  /* Check ERR_FLAG and ADDR = ERROR Register */
  if (rx_data[2]!=0 && rx_data[1]==0x01) {
    printf("Id:%02x errors:\n", addr); 
    if (rx_data[2] & (1<<4)) 
      puts(" - Attitude angle overrange of 1 axis or more occurred");    
    if (rx_data[2] & (1<<3)) 
      puts(" - Gyro and accelerrometer overrange of 1 axis or more occurred");

    /* ERR1 */
    if (rx_data[3] & (1<<4)) 
      puts(" - CAN Receive overflow occurred");      
    if (rx_data[3] & (1<<3)) 
      puts(" - CAN Send overflow occurred");    
    if (rx_data[3] & (1<<1)) 
      puts(" - CAN Bus heavy occurred");  
    if (rx_data[3] & (1<<0)) 
      puts(" - CAN Bus off occurred");        
    
    /* ERR2 */
    if (rx_data[4] & (1<<7)) 
      puts(" - Internal IMU error occurred");      
    if (rx_data[4] & (1<<2)) 
      puts(" - Nonvolatile memory write error occurred");    
    if (rx_data[4] & (1<<1)) 
      puts(" - Nonvolatile memory read error occurred");  
      
    /* ERR3 */
    if (rx_data[5] & (1<<7))
      puts(" - Internal IMU error occurred when executing self-test");
    if (rx_data[5] & (1<<6)) 
      puts(" - ROM error occurred when executing self-test");      
    if (rx_data[5] & (1<<5)) 
      puts(" - RAM error occurred when executing self-test (only at startup)");    
    if (rx_data[5] & (1<<1)) 
      puts(" - Nonvolatile memory read error occurred when executing self-test");  
  } 
}

void getModel(uint8_t addr)
{                        
  eps_device * p = &eps_dev[addr];

  switch (rx_data[1]) {
    case 0x02:
    p->dev_name  = (uint64_t)rx_data[7]<< 0;
    p->dev_name += (uint64_t)rx_data[6]<< 8;
    p->dev_name += (uint64_t)rx_data[5]<<16;
    p->dev_name += (uint64_t)rx_data[4]<<24;
    p->dev_name += (uint64_t)rx_data[3]<<32;
    p->dev_name += (uint64_t)rx_data[2]<<40;
    p->flag |= 1<<0;
    break;
    
    case 0x03:
    p->version  = (uint64_t)rx_data[7]<< 0;
    p->version += (uint64_t)rx_data[6]<< 8;
    p->version += (uint64_t)rx_data[5]<<16;
    p->version += (uint64_t)rx_data[4]<<24;
    p->version += (uint64_t)rx_data[3]<<32;
    p->version += (uint64_t)rx_data[2]<<40; 
    p->flag |= 1<<1;
    break;
    
    case 0x05:
    p->cfg1_b1  = rx_data[2];
    p->set_addr = rx_data[3]; //< can differ from can_addr when new
    p->can_rate = rx_data[4];
    p->can_pri  = rx_data[5];
    p->sout_en  = rx_data[6];
    p->flag |= 1<<2;
    break;
        
    case 0x06:
    p->atti_ctrl1 = rx_data[3];
    p->atti_ctrl2 = rx_data[4];
    p->smpl_rate  = rx_data[6];
    p->fltr_sel   = rx_data[7];
    p->flag |= 1<<3;
    break;
    
    case 0x0b:
    p->serial_l  = (uint64_t)rx_data[7]<< 0;
    p->serial_l += (uint64_t)rx_data[6]<< 8;
    p->serial_l += (uint64_t)rx_data[5]<<16;
    p->serial_l += (uint64_t)rx_data[4]<<24;
    p->serial_l += (uint64_t)rx_data[3]<<32;
    p->serial_l += (uint64_t)rx_data[2]<<40;
    p->flag |= 1<<4;
    break;
    
    case 0x0c:
    p->serial_h  = (uint64_t)rx_data[7]<< 0;
    p->serial_h += (uint64_t)rx_data[6]<< 8;
    p->serial_h += (uint64_t)rx_data[5]<<16;
    p->serial_h += (uint64_t)rx_data[4]<<24;
    p->serial_h += (uint64_t)rx_data[3]<<32;
    p->serial_h += (uint64_t)rx_data[2]<<40;
    p->flag |= 1<<5;
    break;
    
    default:
    break;
  }
  
  if (p->flag == 0x3f) {
    const char * psz_serial_h;
    const char * psz_sample_rate = SAMPLE_RATE_A[0];
    const char * psz_filter_sel  = FILTER_A[0]; 
    uint32_t att_capable = 0;
    char soutStr[9];  
    
    if (p->dev_name == 0x4a5032353547 /*47 35 35 32 50 4a*/) {
      if (p->version == 0x303031303031 /*31 30 30 31 30 30*/) {

        att_capable = 1;
        
        /* assign functions */
        p->func_ptr_sout1 = gyroscope;
        p->func_ptr_sout2 = accelerometer;
        p->func_ptr_sout4 = temperature;
        p->func_ptr_sout5 = dayms;
        p->func_ptr_sout7 = attitude; 
        p->regs = G552PJ_100100_Regs;
        p->regs_cnt = sizeof G552PJ_100100_Regs / sizeof G552PJ_100100_Regs[0];

        if (p->smpl_rate < SAMPLE_RATE_B_SIZE) {
          psz_sample_rate = SAMPLE_RATE_B[p->smpl_rate];
          p->smplSec      = SAMPLE_SEC_B[p->smpl_rate];
        } 
        if (p->fltr_sel < FILTER_B_SIZE) {
          psz_filter_sel = FILTER_B[p->fltr_sel];
        }              
                                          
      } else if (p->version == 0x303037303031/*31 30 30 37 30 30*/) {

        /* assign functions */
        p->func_ptr_sout1 = gyroscope; 
        p->func_ptr_sout2 = accelerometer;
        p->func_ptr_sout4 = temperature; 
        p->func_ptr_sout5 = dayms; 

        p->regs = G552PJ_100700_Regs;
        p->regs_cnt = sizeof G552PJ_100700_Regs / sizeof G552PJ_100700_Regs[0];
        
        if (p->smpl_rate < SAMPLE_RATE_A_SIZE) {
          psz_sample_rate = SAMPLE_RATE_A[p->smpl_rate];
          p->smplSec      = SAMPLE_SEC_A[p->smpl_rate];
        }
        if (p->fltr_sel < FILTER_A_SIZE) {
          psz_filter_sel = FILTER_A[p->fltr_sel];
        }   
      } else {
        /* unknown imu */
      }
    }
    
    /* remove spaces */
    psz_serial_h = strrchr((char*)&(p->serial_h), ' ');
    if ( psz_serial_h ) psz_serial_h++;
    else psz_serial_h = (char*)&(p->serial_h);  
        
    printf("* dev addr:%02xh, name:%.6s, version:%.6s, serial:%.6s%.6s\n"
           "* samples rate:%s\n"
           "* filter:%s\n",
           addr, (char*)&p->dev_name, (char*)&p->version, 
           psz_serial_h, (char*)&p->serial_l,
           psz_sample_rate,
           psz_filter_sel
           ); 
                  
    if (att_capable && (p->atti_ctrl1 & 1)) {
      printf("* attitude mode:%s, reference:%xh, motion:%s\n",
             (p->atti_ctrl1 & 2) ? "Euler" : "Inclination",
             p->atti_ctrl1 >> 3,
             MOTION_PROFILE[p->atti_ctrl2 & 3]
             );
    } else if ( att_capable ){
      puts( "* attitude:Disabled");
    }
    /* sout enabled str */
    memset(soutStr, 0, 9);
    for (int i=0; i<8; i++) {
      if (p->sout_en & 1<<i) {
        char charN = i + 1 + 48;
        strncat(soutStr, &charN, 1);
      }
    }
    printf("* auto start:%d, addr:%02xh, rate:%skbps\n"
           "* priority response:%d, sout:%d\n"
           "* enabled sout:%s\n",
           p->cfg1_b1, p->set_addr, (p->can_rate==0) ? "250" : "500", 
           p->can_pri >> 4, p->can_pri & 7,
           soutStr
           );
    putchar('\n');

    /* Log file */
    if ( LogFile && p->pLogFile==NULL) {
      char file_name[128];
      sprintf(file_name, "%.6s ID:%02xh %s.csv", (char*)&(p->dev_name), addr, strAppStartTime);

      if ((p->pLogFile = fopen(file_name, "wb")) == NULL) {
        puts("Error! opening log file");
      } else {
        fprintf(p->pLogFile,  "dev addr:,%02xh,name:,%.6s,version:,%.6s,serial:,%.6s%.6s%s",
                              addr, (char*)&(p->dev_name), (char*)&p->version, 
                              psz_serial_h, (char*)&p->serial_l, COMMAS+7);
        fprintf(p->pLogFile,  "samples rate:,%s%s", psz_sample_rate, COMMAS+1);
        fprintf(p->pLogFile,  "filter:,%s%s", psz_filter_sel, COMMAS+1);
        if (att_capable && (p->atti_ctrl1 & 1)) {
          fprintf(p->pLogFile, "attitude mode:,%s,reference:,%xh,motion:,%s,%s",
                              (p->atti_ctrl1 & 2) ? "Euler" : "Inclination",
                              p->atti_ctrl1 >> 3,
                              MOTION_PROFILE[p->atti_ctrl2 & 3],
                              COMMAS+6
                              );
        } else if ( att_capable ) {
          fprintf(p->pLogFile, "attitude:,Disabled%s", COMMAS+1);
        }      

        fprintf(p->pLogFile,  "auto start:,%d,addr:,%02xh,rate:,%skbps%s",
                              p->cfg1_b1, p->set_addr, (p->can_rate==0) ? "250" : "500", COMMAS+5);
        fprintf(p->pLogFile,  "priority response:,%d,sout:,%d%s",
                              p->can_pri >> 4, p->can_pri & 7, COMMAS+3);
        fprintf(p->pLogFile,  "enabled sout:,%s%s",
                              soutStr, COMMAS+1);
                                 
        fputs(COMMAS, p->pLogFile); 
        fputs("Sample No.,time[sec],", p->pLogFile);
        p->commasTail += 2;
        for (int i=0; i<8; i++) {
          if (p->sout_en & (1<<i)) {
            if (RawOutput) {
              fputs(strSoutHeaderRaw[i], p->pLogFile);
              if      (i==0 || i==1) p->commasTail += 3;
              else if (i==4 || i==6) p->commasTail += 2;
              else                   p->commasTail++;              
            } else {
              fputs(strSoutHeader[i], p->pLogFile);
              if      (i==0 || i==1) p->commasTail += 3;
              else if (i==6)         p->commasTail += 2;
              else                   p->commasTail++;
            }
          }
        }
        fputs("Hw Cnt", p->pLogFile);
        fputs(COMMAS + p->commasTail, p->pLogFile);          
      }
    }
  }  
}

void* receive_thread(void *arg)
{
  int ret, sock, broadcast = 1;
  struct sockaddr_can sockname = {
    .can_family = AF_CAN,
    .can_addr.j1939 = {
      .addr = host_addr,
      .name = J1939_NO_NAME,
      .pgn = J1939_NO_PGN,
    },
  }, rxsock = {
    .can_family = AF_CAN,
    .can_addr.j1939 = {
      .addr = node_addr,
      .name = J1939_NO_NAME,
      .pgn = J1939_NO_PGN,
    },
  };
  
  sockname.can_ifindex = if_nametoindex(can_ch);

  /* open socket */
  if (verbose)
    fprintf(stderr, "- socket(PF_CAN, SOCK_DGRAM, CAN_J1939);\n");
  sock = ret = socket(PF_CAN, SOCK_DGRAM, CAN_J1939);
  if (ret < 0)
    err(1, "socket(j1939)");

  if (verbose)
    fprintf(stderr, "- setsockopt(, SOL_SOCKET, SO_BROADCAST, %d, %zu);\n",
      broadcast, sizeof(broadcast));
  ret = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, 
       &broadcast, sizeof(broadcast));
  if (ret < 0)
    err(1, "setsockopt: filed to set broadcast");

  ret = bind(sock, (void *)&sockname, sizeof(sockname));
  if (ret < 0)
    err(1, "bind()");

  while (todo_recv) {

    socklen_t rxsock_len = sizeof(rxsock);
    ret = recvfrom(sock, rx_data, sizeof(rx_data), 0,
        (void *)&rxsock, &rxsock_len);
    if (ret < 0) {
      if (EINTR == errno) {
        if (verbose)
          fprintf(stderr, "-\t<interrupted>\n");
        continue;
      }
      err(1, "recvfrom()");
    }
   
    if (todo_recv && ret>0) {

      uint8_t addr = rxsock.can_addr.j1939.addr; 
      eps_device * p = &eps_dev[addr];
      if (p->pass == 0) continue;
      
      switch(rxsock.can_addr.j1939.pgn)
      {
        case PGN_ADDR_CLAIMED:
        break;
        case PGN_COMMMAND:
        break;
        case PGN_SOUT1:
        /* gyroscope */ 
        if((command & CMD_SAMPLES) && (p->smplSwCnt<=limit_cnt))          
          p->func_ptr_sout1(addr);
        break;
        case PGN_SOUT2:
        /* accelerometer */
        if((command & CMD_SAMPLES) && (p->smplSwCnt<=limit_cnt))
          p->func_ptr_sout2(addr);
        break;
        case PGN_SOUT4:
        /* temperature */
        if((command & CMD_SAMPLES) && (p->smplSwCnt<=limit_cnt))
          p->func_ptr_sout4(addr);
        break;
        case PGN_SOUT5:
        /* dayms */
        if((command & CMD_SAMPLES) && (p->smplSwCnt<=limit_cnt))
          p->func_ptr_sout5(addr);
        break;
        case PGN_SOUT7:
        /* attitude */
        if((command & CMD_SAMPLES) && (p->smplSwCnt<=limit_cnt))
          p->func_ptr_sout7(addr);
        break;
        case PGN_RESPONSE:      
        if (command & (CMD_REGREAD | CMD_REGWRITE) ) {
          response(addr, ret);
        } else if (command & CMD_REGMODIFY) {
          response(addr, ret);
          if (rx_data[0] & 0x80) {
            todo_recv = 0;
          }
        } else if (command & CMD_READERROR) {
          printError(addr);
        } else if (eps_dev[addr].regs == NULL) {
          if (command & (CMD_SAMPLES | CMD_REGDUMP | CMD_MODELVERS)) {
            getModel(addr);
          }
        } else if (command & CMD_REGDUMP) {
          if (eps_dev[addr].regs[rx_data[1]] != RNON) 
            response(addr, ret);
        } else if (timeUpdInt) {
          response(addr, ret);
        }
        break;
        
        default:
        break;
      }
    }  
  }
  
  if (verbose)
    fprintf(stderr, "- receive_thread: done\n");
  return NULL;
}

void* send_thread(void *arg)
{
  int ret, sock, broadcast = 1;
  uint8_t tx_id = *(uint8_t*)arg;
  struct sockaddr_can baddr = {
      .can_family = AF_CAN,
      .can_addr.j1939 = {
          .name = J1939_NO_NAME,
          .addr = host_addr,
          .pgn = J1939_NO_PGN,
      },
      .can_ifindex = if_nametoindex(can_ch),
  };
  
  sock = socket(PF_CAN, SOCK_DGRAM, CAN_J1939);
  
  if (verbose)
    fprintf(stderr, "- setsockopt(, SOL_SOCKET, SO_BROADCAST, %d, %zu);\n",
      broadcast, sizeof(broadcast));
  ret = setsockopt(sock, SOL_SOCKET, SO_BROADCAST,
       &broadcast, sizeof(broadcast));
  if (ret < 0)
    err(1, "send setsockopt: filed to set broadcast");
    
  bind(sock, (void *)&baddr, sizeof(baddr));

  struct sockaddr_can txsock = {
      .can_family = AF_CAN,
      .can_addr.j1939 = {
          .name = J1939_NO_NAME,
          .addr = tx_id,
          .pgn = PGN_COMMMAND, 
      },
  };
  
  if ((command & CMD_TIMESTAMP_ONLY) == 0) {
    if ( command & (CMD_SAMPLES | CMD_REGDUMP | CMD_MODELVERS)) {
      uint8_t tx[8] = {0,0,0,0,0,0,0,0};
      tx[0] = 0x00;
      tx[1] = 0x02;
      sendto(sock, tx, 8, 0, (const struct sockaddr *)&txsock, sizeof(txsock));
      
      tx[0] = 0x00;
      tx[1] = 0x03;
      sendto(sock, tx, 8, 0, (const struct sockaddr *)&txsock, sizeof(txsock));
      
      tx[0] = 0x00;
      tx[1] = 0x05;
      sendto(sock, tx, 8, 0, (const struct sockaddr *)&txsock, sizeof(txsock));
      
      tx[0] = 0x00;
      tx[1] = 0x06;
      sendto(sock, tx, 8, 0, (const struct sockaddr *)&txsock, sizeof(txsock));    
          
      tx[0] = 0x00;
      tx[1] = 0x0c;
      sendto(sock, tx, 8, 0, (const struct sockaddr *)&txsock, sizeof(txsock));
      
      tx[0] = 0x00;
      tx[1] = 0x0b;
      sendto(sock, tx, 8, 0, (const struct sockaddr *)&txsock, sizeof(txsock));    
    }

    if ( command & CMD_REGDUMP ) {
      uint8_t tx[8] = {0,0,0,0,0,0,0,0};
      for (uint8_t i=0; i<0x0e; i++) {
        sendto(sock, tx, 8, 0, (const struct sockaddr *)&txsock, sizeof(txsock));
        tx[1] += 1;   
      };
    }
    
    if ( command & (CMD_REGREAD | CMD_REGWRITE | CMD_READERROR) ) {
        sendto(sock, tx_data, 8/*tx_len*/, 0, (const struct sockaddr *)&txsock, sizeof(txsock));
    }
      
    if ( command & CMD_REGMODIFY ) {
      uint8_t rx[8] = {0,0,0,0,0,0,0,0};
      rx[1] = tx_data[1];
      sendto(sock, rx, 8, 0, (const struct sockaddr *)&txsock, sizeof(txsock));
      while(tx_data[1] != rx_data[1]){}

      for ( int i=1; i<8; i++ ) {
        if ( tx_mask[i] == 0 ) {
          tx_data[i] = rx_data[i]; //< keep same
        } else {
          rx_data[i]    &= ~tx_mask[i];
          tx_data[i] |=  rx_data[i];
        }
      }
      sendto(sock, tx_data, 8/*tx_len*/, 0, (const struct sockaddr *)&txsock, sizeof(txsock));
    }
  }
 
  /* Update Time Register Register */
  if (command & CMD_SAMPLES) {
    uint8_t tx[8] = {0x80,0x0d,0,0,0,0,0,0};
    uint16_t day;
    uint32_t ms;        
    struct timeval tv;

    gettimeofday(&tv,NULL);
    day = tv.tv_sec/(24*60*60);
    ms  = tv.tv_sec%(24*60*60);
    ms *= 1000;
    ms += tv.tv_usec / 1000;
    
    tx[2] = (uint8_t)(ms >> 0);
    tx[3] = (uint8_t)(ms >> 8);
    tx[4] = (uint8_t)(ms >>16);
    tx[5] = (uint8_t)(ms >>24);
    tx[6] = (uint8_t)(day>> 0);
    tx[7] = (uint8_t)(day>> 8); 
    sendto(sock, tx, 8, 0, (const struct sockaddr *)&txsock, sizeof(txsock));
  }

  todo_send = 0;
  
  if (verbose)
    fprintf(stderr, "- send_thread: done\n");  
    
  return NULL;
}

void str2upcase( char * s ) 
{
  for (uint32_t i = 0; s[i]!='\0'; i++) {
    if(s[i] >= 'a' && s[i] <= 'z') {
       s[i] = s[i] -32;
    }
  }
} 

void usage(char * execName) 
{
  printf("Usage: %s --id n1 [n2 ...] [COMMAND[PARAM[:SETTINGS]]]\n", execName);   
  printf("  -i, --id        \tDevice ID (CAN addr) which commands applies to(range:0-fdh<hex>, default:all<if not set>)\n"); 
  printf("                  \tThis command is mandatory\n");
  printf("                  \tExamples:\n");
  printf("                  \t 1. Get samples (if in sampling mode) from devices with ID 80h:\n");  
  printf("                  \t    %s -i 80\n", execName);   
  printf("                  \t 2. Get samples (if in sampling mode) from the devices with ID 80h and 81h:\n");  
  printf("                  \t    %s -i 80 81\n", execName);  
  printf("\n"); 
  printf("  -I, --hostid    \tSet host ID(default:FDh). This can be used to avoid conflict with Device ID.\n");
  printf("                  \tExample:\n");  
  printf("                  \t    %s -I 2 -i FD\n", execName); 
  printf("\n");
  printf("  -o, --info      \tGet info of device(s)\n"); 
  printf("                  \tExamples:\n");
  printf("                  \t    %s -i 80 --info\n", execName);   
  printf("\n"); 
  printf("  -R, --read      \tRead register's 48 bits value(Return 6 bytes:2 3 4 5 6 7)\n");   
  printf("                  \tExamples:\n");
  printf("                  \t 1. Read 'ERROR' register 0x01 from device with ID 80h and 81h:\n");    
  printf("                  \t    %s -i 80 81 -R 1\n", execName); 
  printf("                  \t    Example of return values from all devices, 80h and 81h:\n");
  printf("                  \t    80: 30 30 37 30 30 31\n");
  printf("                  \t    81: 30 30 31 30 30 31\n");
  printf("\n");  
  printf("  -W, --write     \tWrite 48 bits value (6 bytes:2 3 4 5 6 7) to register\n"); 
  printf("                  \tExamples:\n");
  printf("                  \t 1. Write value (1 0 0 0 0 0) to register 0x00 into a device with ID 80h:\n"); 
  printf("                  \t    %s -i 80 -W 0 1 0 0 0 0 0\n", execName); 
  printf("\n");       
  printf("  -a, --msc       \tMiscellaneous Command (one msc param at a time)\n"); 
  printf("                  \t(*): Device must be in config mode\n"); 
  printf("                  \tParams: save, restore, test, testimu, setatt, cancelatt\n"); 
  printf("                  \tExamples:\n");  
  printf("                  \t 1. Save parameters in device with ID 80h to non-volatile memory:\n"); 
  printf("                  \t    %s -i 80 --msc save\n", execName);   
  printf("                  \t 2. Restore default parameters in a device with ID 80h:\n"); 
  printf("                  \t    %s -i 80 --msc restore\n", execName);   
  printf("                  \t 3. Self-test (excludes self-test of internal IMU) in a device with ID 80h:\n"); 
  printf("                  \t    %s -i 80 --msc test\n", execName);   
  printf("                  \t 4. Self-test of internal IMU in a device with ID 80h:\n"); 
  printf("                  \t    %s -i 80 --msc testimu\n", execName);   
  printf("                  \t 5. Set the current attitude angle output to zero in a device with ID 80h:\n"); 
  printf("                  \t    %s -i 80 --msc setatt\n", execName);   
  printf("                  \t 6. Cancel attitude preset zero in a device with ID 80h:\n"); 
  printf("                  \t    %s -i 80 --msc cancelatt\n", execName);   
  printf("\n");  
  printf("  -b, --cfg1      \tConfig #1 Command for a certain device(multiple params are allowed at a time)\n"); 
  printf("                  \t(*): Device must be in config mode\n"); 
  printf("                  \tParams: autostart:0-1, newid:0-fe, canrate:0-1, prirsp:0-7, prisout:0-7, souten:012457\n"); 
  printf("                  \tExamples:\n");  
  printf("                  \t 1. Enable/Disable (1/0) sampling mode (on power-on) in device with ID 80h:\n"); 
  printf("                  \t    %s -i 80 --cfg1 autostart:1\n", execName);   
  printf("                  \t 2. Set new ID (0-fd) for a device with ID 80:\n"); 
  printf("                  \t    %s -i 80 --cfg1 newid:a1\n", execName); 
  printf("                  \t 3. Set CAN bit rate (0: 250kbps, 1: 500kbps) in a device with ID 80h:\n"); 
  printf("                  \t    %s -i 80 --cfg1 canrate:0\n", execName);   
  printf("                  \t 4. Sets the priority of the “Response” message in a device with ID 80h:\n"); 
  printf("                  \t    %s -i 80 --cfg1 prirsp:6\n", execName); 
  printf("                  \t 5. Sets the priority of the “SOUT” message in a device with ID 80h:\n"); 
  printf("                  \t    %s -i 80 --cfg1 prisout:6\n", execName);   
  printf("                  \t 6. Enable listed (1,2,4,5) and disable others (7) SOUT messages in a device with ID 80h:\n"); 
  printf("                  \t    %s -i 80 --cfg1 souten:1245\n", execName);
  printf("\n"); 
  printf("  -c, --cfg2      \tConfig #2 Command for a certain device (multiple params are allowed at a time)\n"); 
  printf("                  \t(*): Device must be in config mode\n"); 
  printf("                  \tParams: baseatt:0-17h, euleren:0-1, atten:0-1, attmod:0-2, srate:0-Ah, filter:1-13h\n"); 
  printf("                  \tExamples:\n");  
  printf("                  \t 1. Select the reference attitude (b) in device with ID 80h:\n"); 
  printf("                  \t    %s -i 80 --cfg2 baseatt:1\n", execName);   
  printf("                  \t 2. Selects attitude angle output mode (0: Inclination, 1: Euler) in a device with ID 80h:\n"); 
  printf("                  \t    %s -i 80 --cfg2 euleren:1\n", execName); 
  printf("                  \t 3. Set the data sample rate (500sps) in a device with ID 80h:\n"); 
  printf("                  \t    %s -i 80 --cfg2 srate:2\n", execName);   
  printf("                  \t 4. Set filter (Moving average filter TAP=128) in a device with ID 80h:\n"); 
  printf("                  \t    %s -i 80 --cfg2 filter:7\n", execName);
  printf("                  \t 5. Enable/Disable (1/0) attitude angle output in a device with ID 80h:\n");   
  printf("                  \t    %s -i 80 --cfg2 atten:1\n", execName);   
  printf("                  \t    (*): This can be used only for certain devices and sample rate 200, 100, 50\n");
  printf("                  \t 6. Select the attitude motion profile (c) in a device with ID 80h:\n");  
  printf("                  \t    %s -i 80 --cfg2 attmod:2\n", execName); 
  printf("                  \t    (*): This can be used only for certain devices\n"); 
  printf("\n");
  printf("  -m, --mode      \tMode Control Command (one mode param at a time)\n"); 
  printf("                  \tParams: smpl, cfg, rst\n"); 
  printf("                  \tExamples:\n");  
  printf("                  \t 1. Go to the sampling mode a device with ID 80h:\n"); 
  printf("                  \t    %s -i 80 --mode smpl\n", execName);   
  printf("                  \t 2. Go to the configuration mode a device with ID 80h:\n"); 
  printf("                  \t    %s -i 80 --mode cfg\n", execName);   
  printf("                  \t 3. Reset a device with ID 80h:\n"); 
  printf("                  \t    %s -i 80 --mode rst\n", execName);   
  printf("\n");
  printf("  -d, --dump      \tDump register values(no argument)\n"); 
  printf("                  \tExamples:\n");
  printf("                  \t 1. Dump register values from a device with ID 80h:\n"); 
  printf("                  \t    %s -i 80 --dump\n", execName);
  printf("                  \t 2. Dump register values from the devices with ID 80h and 81h:\n"); 
  printf("                  \t    %s -i 80 81 --dump\n", execName);
  printf("\n");   
  printf("  -x, --samplemax \tQuit after N specified samples(default:-1<no limit>, max:4294967294<dec>)\n"); 
  printf("                  \tExamples:\n");
  printf("                  \t 1. 200 samples output from a device with ID 80h:\n");
  printf("                  \t    %s -i 80 -x 200\n", execName);   
  printf("                  \t 2. Unlimited output to log files from the devices with ID 80h and 81h:\n");
  printf("                  \t    %s -i 80 81 --log\n", execName); 
  printf("                  \t***Ctrl+c to quit\n"); 
  printf("\n");      
  printf("  -t, --time      \tInterval in seconds to update the internal time(default:0 <update once at gets samples>)\n"); 
  printf("                  \tExample, update internal time every 10 second:\n");  
  printf("                  \t    %s -i 80 --time 10\n", execName);   
  printf("\n"); 
  printf("  -T, --human     \tShow sample timestamps in human-readable format(default: Unix timestamp)\n"); 
  printf("                  \tExample:\n");  
  printf("                  \t    %s -i 80 --human\n", execName);   
  printf("\n"); 
  printf("  -r, --raw       \tOutput samples in raw format (no argument, default: no)\n"); 
  printf("                  \tExample:\n");  
  printf("                  \t    %s -i 80 -x 10 --raw\n", execName);  
  printf("\n");  
  printf("  -l, --log       \tWrite sensor samples to CSV file. Each device ID has its own log file(no argument, default:no)\n"); 
  printf("                  \tExample:\n"); 
  printf("                  \t    %s -i 80 81 -x 10 --log\n", execName); 
  printf("                  \t***LogFile name is based on model name, node id and time\n"); 
  printf("\n");   
  printf("  -e, --err       \tRead errors if any(no argument)\n"); 
  printf("                  \tExamples:\n"); 
  printf("                  \t    %s -i 80 --err\n", execName); 
  printf("\n");   
  printf("  -s, --socan     \tSocketcan channel(default:'%s')\n", can_ch);
  printf("                  \tExample:\n");  
  printf("                  \t    %s -c can0 -i 80 -x 10\n", execName); 
  printf("\n");   
  printf("  -v, --version   \tShow version of %s\n", execName+2); 
  printf("\n");  
  printf("  -?, --help      \tThis info\n"); 
  printf("\n");   

}

void txpremod( uint8_t reg )
{
  if (tx_data[0] != 0x80 || tx_data[1] != reg) {
    memset(tx_data, 0, 8);
    memset(tx_mask, 0, 8);
    tx_data[0] = 0x80; //< Write
    tx_data[1] = reg;  //< REG    
  }    
  todo_recv = 1;
  todo_send = 1;      
  command = CMD_REGMODIFY;
}

void fnExit( void )
{
  for ( int i=0; i<0xFE; i++ ) {
    if (eps_dev[i].pLogFile)
      fclose(eps_dev[i].pLogFile);
      
    if (eps_dev[i].pErrFile)
      fclose(eps_dev[i].pErrFile);      
  }
}

/* main */
int main(int argc, char *argv[])
{
  int error, opt;
  int option_index = 0;
  uint32_t timeMsgCnt = 0; 
  time_t app_start_time;
  struct tm timeinfo;
  struct timeval tv;

  atexit(fnExit);
  
  gettimeofday(&tv,NULL);
  app_start_time = (time_t)tv.tv_sec;
  timeinfo = *localtime((time_t*) &app_start_time);
  strftime(strAppStartTime, sizeof(strAppStartTime), "%Y-%m-%d %H:%M:%S", &timeinfo); 
  sprintf(&strAppStartTime[strlen(strAppStartTime)], ".%lu", tv.tv_usec/1000);

  static struct option long_options[] =
  { 
      {   "verbose",        no_argument, &verbose, 1},
      {       "msc",  required_argument, 0, 'a'},
      {      "cfg1",  required_argument, 0, 'b'},
      {      "cfg2",  required_argument, 0, 'c'}, 
      {      "dump",        no_argument, 0, 'd'},
      {       "err",        no_argument, 0, 'e'}, 
      {        "id",  required_argument, 0, 'i'}, 
      {       "log",        no_argument, 0, 'l'}, 
      {      "mode",  required_argument, 0, 'm'},
      {      "info",        no_argument, 0, 'o'},
      {       "raw",        no_argument, 0, 'r'},
      {     "socan",  required_argument, 0, 's'},
      {      "time",  required_argument, 0, 't'},
      {   "version",        no_argument, 0, 'v'},
      { "samplemax",  required_argument, 0, 'x'},
      {    "hostid",  required_argument, 0, 'I'},
      {      "read",  required_argument, 0, 'R'},
      {     "human",        no_argument, 0, 'T'},
      {     "write",  required_argument, 0, 'W'},
      {      "help",        no_argument, 0, '?'},
      {0, 0, 0, 0}
  };  

  while ((opt = getopt_long(argc, argv, "a:b:c:dei:lm:ors:t:vx:I:R:TW:?", long_options, &option_index)) != -1) {
    switch (opt) {
      case 0:
        /* If this option set a flag, do nothing else now. */
        if (long_options[option_index].flag != 0)
          break;
        printf ("option %s", long_options[option_index].name);
        if (optarg)
          printf (" with arg %s", optarg);
        printf ("\n");
        break;
      case 'd':
        todo_recv = 1;
        todo_send = 1;
        command = CMD_REGDUMP;
        break;
      case 'e':
        tx_data[0] = 0x00; //< Read
        tx_data[1] = 0x01; //< Error Reg       
        todo_recv = 1;
        todo_send = 1;
        command = CMD_READERROR;
        break;
      case 'o':
        command = CMD_MODELVERS; 
        break;
      case 's':
          strcpy(can_ch, optarg);
        break;
      case 'r':
        RawOutput = 1;
        break;
      case 'i':
        for (int i=optind-1; i<=argc && argv[i] && *(argv[i]) != '-'; i++) {
          node_addr = strtoul(argv[i], NULL, 16);
          if (node_addr > 0xfd) {
            goto INPUT_ERR;
          } else {
            eps_dev[node_addr].pass = 1;
            id_cnt++;
            if (id_cnt == 0xff) 
              goto INPUT_ERR;
          }
        }
        break;
      case 'v':
        puts(LOGGER_SW_VERSION);
        exit(0);
        break;
      case 't':
        timeUpdInt = strtoul(optarg, NULL, 10);
        break;
      case 'x':
        limit_cnt = strtoul(optarg, NULL, 10);
        command = CMD_SAMPLES;
        break;
      /* MODE (REG 0x00) */
      case 'm': 
        str2upcase(optarg);
        tx_data[0] = 0x80; //< Write
        tx_data[1] = 0x00; //< Reg MODE     
        if      (strcmp(optarg, "SMPL") == 0) tx_data[2] = 0x01;  
        else if (strcmp(optarg,  "CFG") == 0) tx_data[2] = 0x02; 
        else if (strcmp(optarg,  "RST") == 0) tx_data[2] = 0x03;
        else goto INPUT_ERR;
        todo_recv = 0;
        todo_send = 1;      
        command = CMD_REGWRITE;
        break; 
      case 'l': 
        LogFile = 1;
        break;      
      /* MSC_CMD (REG 0x04) */
      case 'a': //< restore
        if (tx_data[0] != 0x80 || tx_data[1] != 0x04) {
          memset(tx_data, 0, 8);
        }    
        tx_data[0] = 0x80; //< Write
        tx_data[1] = 0x04; //< MSC_CMD Reg
        str2upcase(optarg);
        if      (strcmp(optarg,      "SAVE")==0) tx_data[2] = 0x01; //< Save parameter
        else if (strcmp(optarg,   "RESTORE")==0) tx_data[2] = 0x02; //< Restore default parameter
        else if (strcmp(optarg,      "TEST")==0) tx_data[2] = 0x03; //< Self-test (excludes self-test of internal IMU)
        else if (strcmp(optarg,   "TESTIMU")==0) tx_data[2] = 0x04; //< Self-test of internal IMU (when device is in a static or stationary state)
        else if (strcmp(optarg,    "SETATT")==0) tx_data[2] = 0x05; //< Set the current attitude angle output to zero
        else if (strcmp(optarg, "CANCELATT")==0) tx_data[2] = 0x06; //< cancel attitude preset zero
        else goto INPUT_ERR;
        todo_recv = 0;
        todo_send = 1;      
        command = CMD_REGWRITE;
        break;
      /* CONFIG1 (REG 0x05) */
      case 'b':  
        txpremod(0x05);
        for (int i=optind-1; i<=argc && argv[i] && *(argv[i]) != '-'; i++) {
          str2upcase(argv[i]);
          if (strncmp(argv[i], "AUTOSTART:", 10)==0) {
            tx_data[2] = strtoul(argv[i]+10, NULL, 16)&1;//< AUTO_START bit0
            tx_mask[2] = 1; //< Byte2 to be modified
          } else if (strncmp(argv[i], "NEWID:", 6)==0) {
            tx_data[3] = (uint8_t)strtoul(argv[i]+6, NULL, 16); //< CAN_ADDR
            tx_mask[3] = 0xfe; //< Byte3 to be modified         
          } else if (strncmp(argv[i], "CANRATE:", 8)==0) {
            tx_data[4] = (uint8_t)strtoul(argv[i]+8, NULL, 16)&1; //< CAN_RATE
            tx_mask[4] = 1; //< Byte4 to be modified        
          } else if (strncmp(argv[i], "PRIRSP:", 7)==0) {
            tx_data[5] |= (uint8_t)(strtoul(argv[i]+7, NULL, 16)&7)<<4; //< CAN_PRI
            tx_mask[5] |= 0x70; //< Byte5 to be modified (current max: 0x77)      
          } else if (strncmp(argv[i], "PRISOUT:", 8)==0) {
            tx_data[5] |= (uint8_t)(strtoul(argv[i]+8, NULL, 16)&7)<<0; //< CAN_PRI
            tx_mask[5] |= 0x7; //< Byte5 to be modified (current max: 0x77)           
          } else if (strncmp(argv[i], "SOUTEN:", 7)==0) {
            if (strchr(argv[i]+7, '1')) tx_data[6] |= 1<<0; //< enable SOUT 1
            if (strchr(argv[i]+7, '2')) tx_data[6] |= 1<<1; //< enable SOUT 2
            if (strchr(argv[i]+7, '4')) tx_data[6] |= 1<<3; //< enable SOUT 4
            if (strchr(argv[i]+7, '5')) tx_data[6] |= 1<<4; //< enable SOUT 5
            if (strchr(argv[i]+7, '7')) tx_data[6] |= 1<<6; //< enable SOUT 7
            if (strchr(argv[i]+7, '0')) tx_data[6]  = 0;    //< disable all SOUT 
            tx_mask[6] = 0xff; //< Byte6 to be modified (current max: 0x5b)     
          } else goto INPUT_ERR;
        }
        break; 
      /* CONFIG2 (REG 0x06) */   
      case 'c':
        txpremod(0x06);
        for (int i=optind-1; i<=argc && argv[i] && *(argv[i]) != '-'; i++) {
          str2upcase(argv[i]);
          if (strncmp(argv[i], "BASEATT:", 8)==0) { /*0-1f*/
            tx_data[3] |= (uint8_t)(strtoul(argv[i]+8, NULL, 16)&0x1f) << 3;
            tx_mask[3] |= 0xf8; //< Bits in Byte3 to be modified
          } else if (strncmp(argv[i], "EULEREN:", 8)==0) {
            tx_data[3] |= (uint8_t)(strtoul(argv[i]+8, NULL, 16)&1) << 1;
            tx_mask[3] |= 2; //< Bits in Byte3 to be modified     
          } else if (strncmp(argv[i], "ATTEN:", 6)==0) {
            tx_data[3] |= (uint8_t)strtoul(argv[i]+6, NULL, 16)&1;
            tx_mask[3] |= 1; //< Bits in Byte3 to be modified     
          } else if (strncmp(argv[i], "ATTMOT:", 7)==0) {
            tx_data[4] = (uint8_t)strtoul(argv[i]+7, NULL, 16)&3; 
            tx_mask[4] = 3; //< Bits in Byte4 to be modified 
          } else if (strncmp(argv[i], "SRATE:", 6)==0) {
            tx_data[6] = (uint8_t)strtoul(argv[i]+6, NULL, 16)&0x0f;
            tx_mask[6] = 0x0f; //< Byte6 to be modified
          } else if (strncmp(argv[i], "FILTER:", 7)==0) {
            tx_data[7] = (uint8_t)strtoul(argv[i]+7, NULL, 16)&0x1f;
            tx_mask[7] = 0x1f; //< Byte7 to be modified  
          } else goto INPUT_ERR;
        }
        break;   
      case 'I':
        host_addr = strtoul(optarg, NULL, 16);
        if (host_addr > 0xfd) {
          host_addr = HOST_ADDR;
        }             
        break;
      case 'R':
        tx_data[0] = 0x00; //< Read
        tx_data[1] = strtoul(optarg, NULL, 16);  
        todo_recv = 1;
        todo_send = 1;      
        command = CMD_REGREAD;    
        break;
      case 'W':
        tx_data[0] = 0x80; //< Write
        tx_data[1] = strtoul(optarg, NULL, 16);     
        for (int i=0; i<6; i++) {
          if ( argc - optind - i > 0 ) {
            if ( *(argv[optind+i]) != '-') {
                tx_data[2+i] = strtoul(argv[optind+i], NULL, 16);
            } else {
              break;
            }
          } 
        }
        todo_recv = 0;
        todo_send = 1;      
        command = CMD_REGWRITE;
        break;
      case 'T':
        timeStampInHumanForm = 1;
        break;
      default:
  INPUT_ERR:
        usage(argv[0]);
        exit(1);
        break;
    }
  } //< while() end

  if ((command & CMD_REGMODIFY) && (id_cnt != 1)) {
    /* Modify Reg only for one device at a time */
    usage(argv[0]);
    exit(1);      
  }
  
  if (id_cnt == 0) {
    usage(argv[0]);
    exit(1);      
  }
  if (id_cnt > 1) {
    node_addr = J1939_NO_ADDR;
  }

  for ( int i=0; i<0xFE; i++ ) {
    eps_dev[i].dev_name  = 0;
    eps_dev[i].version   = 0;
    eps_dev[i].regs_cnt  = 0;
    eps_dev[i].smplHwCnt = 0;
    eps_dev[i].smplSwCnt = 0;    
    eps_dev[i].func_ptr_sout1 = ignore;
    eps_dev[i].func_ptr_sout2 = ignore;
    eps_dev[i].func_ptr_sout4 = ignore;
    eps_dev[i].func_ptr_sout5 = ignore;
    eps_dev[i].func_ptr_sout7 = ignore;
    memset(eps_dev[i].infoString, 0, STRL);
    memset(eps_dev[i].smplString, 0, STRL);    
    eps_dev[i].pLogFile = NULL;
    eps_dev[i].pErrFile = NULL;
    eps_dev[i].regs     = NULL;
  }

  if ( todo_recv ) {
    error = pthread_create(&idrx, NULL, &receive_thread, NULL);
    
    if (error != 0) {
        fprintf(stderr, "\n- can't create receive_thread :[%s]", strerror(error));
        return 0;
    } else if (verbose){
        fprintf(stderr, "\n- receive_thread created successfully\n");
    }
  }

  if ( todo_send ) {
    uint8_t sent_to_node_id;
    for ( uint8_t i=0; i<0xfd; i++ ) {
      if (eps_dev[i].pass == 0) continue;
      sent_to_node_id = i;
      todo_send = 1;
      if (verbose) printf("ID: %02xh\n", sent_to_node_id);
      error = pthread_create(&idtx, NULL, &send_thread, &sent_to_node_id);
      
      if (error != 0) {
          fprintf(stderr, "\n- can't create send_thread :[%s]", strerror(error));
          return 0;
      } else if (verbose){
          fprintf(stderr, "\n- send_thread created successfully\n");
      }
      pthread_join(idtx, NULL);
    }
  } 
  
  command |= CMD_TIMESTAMP_ONLY;
  
  while ( todo_send || todo_recv ) 
  {
    if (timeUpdInt) {
      if (timeMsgCnt==0) {
        uint8_t sent_to_node_id;
        timeMsgCnt = timeUpdInt;  
        
        for ( uint8_t i=0; i<0xfd; i++ ) {
          if (eps_dev[i].pass == 0) continue;
          sent_to_node_id = i;
          error = pthread_create(&idtx, NULL, &send_thread, &sent_to_node_id);          
          if (error != 0) {
              fprintf(stderr, "\n- can't create send_thread :[%s]", strerror(error));
              return 0;
          } else if (verbose){
              fprintf(stderr, "\n- send_thread created successfully\n");
          }
          pthread_join(idtx, NULL); 
        }
      } else {
        timeMsgCnt--;
      }
    }

    if (rx_cnt-- <= 0) {  
      if (verbose) puts("exit by time out");   
      exit(0);
    }
    sleep(1);
  } 
    
  return 0;
}

