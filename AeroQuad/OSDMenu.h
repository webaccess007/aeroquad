/*
  AeroQuad v3.0 - Nov 2011
 www.AeroQuad.com
 Copyright (c) 2011 Ted Carancho.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/* Menu system implementation, usable with OSD */

#ifndef _AQ_OSD_MENU_
#define _AQ_OSD_MENU_

//#define MENU_GOPRO // enable GoPro controls... not usable atm.

struct MenuItem {
  const byte l;         // menu level the item is on
  const char *t;        // text to show
  void (*f)(byte,byte); // handler func on leaf level
  const byte d;         // data to give for handler
};
extern const struct MenuItem menuData[];

#define MENU_INIT     0
#define MENU_UP       1
#define MENU_DOWN     2
#define MENU_SELECT   3
#define MENU_EXIT     4
#define MENU_CALLBACK 5

#define MENU_NOFUNC   0

#define MENU_SYM_BOTH '\015'
#define MENU_SYM_UP   '\016'
#define MENU_SYM_DOWN '\017'


byte menuInfunc = 0;   // tells if a handler func is active
// 0 - we're in base menu
// 1 - call handler on stick actions
// 2-255 countdown and callback when it gets to 1 - no sticks
byte menuEntry  = 255; // Active menu entry
byte menuAtExit = 0;   // are we at the exit at the top
byte stickWaitNeutral = 1;

// DATA that menu functions can freely use to store state
byte menuFuncData[10];    // 10 bytes of data for funcs to use as they wish...
float menuFuncDataFloat; // float for menufuncs use

#ifdef CameraControl
void menuHandleCam(byte mode, byte action) {

  switch (action) {
  case MENU_INIT:
    menuFuncData[0]=0;
    menuFuncData[1]=0;
    break;
    
  case MENU_EXIT:
    if (menuFuncData[0]==0) {
      menuInfunc=0;
      return;
    }
    else {
      menuFuncData[0]--;
    }
    break;
    
  case MENU_SELECT:
    if (menuFuncData[0]<1) {
      menuFuncData[0]++;
    }
    break;
    
  case MENU_UP:
  case MENU_DOWN:
    if (menuFuncData[0]==0) {
      if (action==MENU_UP) {
        if (menuFuncData[1] < 3) {
          menuFuncData[1]++;
        }
      } 
      else {
        if (menuFuncData[1]>0) {
          menuFuncData[1]--;
        }
      }
    }
    else if (menuFuncData[0]==1) {
      int val=(action==MENU_UP)?10:-10;
      switch (menuFuncData[1]) {
      case 0:
        setMode(action==MENU_UP?1:0);
        break;
      case 1:
        setCenterPitch(constrain(getCenterPitch() + val, getServoMinPitch(), getServoMaxPitch()));
        break;
      case 2:
        setCenterRoll(constrain(getCenterRoll() + val, getServoMinRoll(), getServoMaxRoll()));
        break;
      case 3:
        setCenterYaw(constrain(getCenterYaw() + val, getServoMinYaw(), getServoMaxYaw()));
        break;
      }
    }
    break;
  }

  if (menuFuncData[1] == 0) {
    notifyOSDmenu(OSD_NOCLEAR | OSD_CURSOR, menuFuncData[0] ? 18 : 1, menuFuncData[0] ? 18 : 16, "%cStabilizer mode: %1d", MENU_SYM_BOTH, getMode());
  }
  else {
    notifyOSDmenu(OSD_NOCLEAR|OSD_CURSOR,
      menuFuncData[0]?15:8,menuFuncData[0]?18:12,
      "%cCenter %s: %04d",MENU_SYM_BOTH,
      (menuFuncData[1]==1)?"Pitch":
      (menuFuncData[1]==2)?"Roll ":
      "Yaw  ",
      (menuFuncData[1]==1)?getCenterPitch():
      (menuFuncData[1]==2)?getCenterRoll():
      getCenterYaw());
  }
}
#endif

void menuHandleOsd(byte mode, byte action) {

  switch (mode) {
  case 0: 
    armedTime=0;
    break;
  }
  menuInfunc = 0;
}

#ifdef MENU_GOPRO
const char *gopro_b_txt[3] = { "Shutter", "Mode", "Power" };

void menuHandleGopro(byte mode, byte action) {
  
  switch (action) {
  case MENU_CALLBACK:
    // depress I/O line...
    menuInfunc = 0; // exit to menu
    break;
    
  case MENU_INIT:
    // activate I/O
    memset(buf, 0, MENU_BUFSIZE);
    snprintf(buf, MENU_BUFSIZE, "%s pressed", gopro_b_txt[mode]);
    menuRefresh();
    if (mode==2) {
      menuInfunc = 35; //3.5 sec
    }
    else {
      menuInfunc = 10; // 1 sec
    }
    break;
    
  default:
    break;
  }
}
#endif

// edit digit  with of form [+/-][i*#].[d*#]
// pos: 0=sign, 1-(i)=intpart, (i+1)-(i+d+1)=decimals
void menuEditFloat(float *f, byte i, byte d, byte pos, byte action, float min, float max) {

  if (pos > (1 + i + d)) {
    return;
  }
  // if ((action!=MENU_UP)&&(action!=MENU_DOWN)) return;
  if (pos == 0 && (min*max < 0)) { // change sign only if min/max are different sign
    *f=-*f;
  }
  else {
    *f += (action==MENU_UP?1.0:-1.0)*pow(10.0,i-pos);
  }
  *f = constrain(*f,min,max);
}

char *pidnames[10] = {
  "RRoll", "RPitc", "RYaw ", "ARoll", "APitc",
  "Headi", "AGRol", "AGPit", "Altit", "ZDamp"};

void menuHandlePidTune(byte mode, byte action) {

  switch (action) {
  case MENU_INIT:
    menuFuncData[0] = 0; //level 0-select PID;1-select P/I/D;>=2-edit value
    menuFuncData[1] = 0; // PIDno
    menuFuncData[2] = 0; // 0=P/1=I/2=D
    break;
    
  case MENU_EXIT:
    if (menuFuncData[0]>0) {
      menuFuncData[0]--;
    }
    else {
      menuInfunc=0;
      return;
    }
    menuFuncData[3]=0;
    break;
    
  case MENU_SELECT:
    if (menuFuncData[0]<9) {
      menuFuncData[0]++;
    }
    else {
      if (menuFuncData[3]) {
        switch (menuFuncData[2]) {
        case 0: 
          PID[menuFuncData[1]].P=menuFuncDataFloat;
          break;
        case 1: 
          PID[menuFuncData[1]].I=menuFuncDataFloat;
          break;
        case 2: 
          PID[menuFuncData[1]].D=menuFuncDataFloat;
          break;
        }
        notifyOSD(OSD_NOCLEAR,"PID value saved!!");
      }
      else {
        notifyOSD(OSD_NOCLEAR,"PID value not saved!!");
      }
      menuInfunc=10;
      menuFuncData[0]=1; //return to P/I/D selection
      return;
    }
    break;
    
  case MENU_UP:
    if (menuFuncData[0] == 0) {
      if (menuFuncData[1] < 9) {
        menuFuncData[1]++;
      }
    }
    else if (menuFuncData[0] == 1) {
      if (menuFuncData[2] < 2) {
        menuFuncData[2]++;
      }
    }
    else if (menuFuncData[0] == 9) {
      menuFuncData[3] = 1;
    }
    else {
      menuEditFloat(&menuFuncDataFloat,4,2,menuFuncData[0]-2,MENU_UP,-1000,1000);
    }
    break;
    
  case MENU_DOWN:
    if (menuFuncData[0]<2) {
      if (menuFuncData[menuFuncData[0]+1] > 0) {
        menuFuncData[menuFuncData[0]+1]--;
      }
    }
    else if (menuFuncData[0]==9) {
      menuFuncData[3]=0;
    }
    else {
      menuEditFloat(&menuFuncDataFloat,4,2,menuFuncData[0]-2,MENU_DOWN,-1000,1000);
    }
    break;
  }

  if (menuFuncData[0]<2) {
    menuFuncDataFloat = 
       (menuFuncData[2]==0?PID[menuFuncData[1]].P:
          menuFuncData[2]==1?PID[menuFuncData[1]].I:
          PID[menuFuncData[1]].D);
  }

  if (menuFuncData[0] < 9) {
    byte cl,cr;
    byte updn = MENU_SYM_BOTH;
    if (menuFuncData[0]==0) {
      // selecting PID
      cl = 5;
      cr = 9;
      if (menuFuncData[1] == 0) {
        updn = MENU_SYM_UP;
      }
      if (menuFuncData[1] == 9) {
        updn = MENU_SYM_DOWN;
      }
    }
    else if (menuFuncData[0]==1) {
      // selecting P/I/D
      cl = cr = 11;
      if (menuFuncData[2] == 0) {
        updn = MENU_SYM_UP;
      }
      if (menuFuncData[2] == 2) {
        updn=MENU_SYM_DOWN;
      }
    }
    else if (menuFuncData[0] < 9) {
      cl = cr = ((menuFuncData[0] > 6) ? 12 : 11) + menuFuncData[0];
    }
    notifyOSDmenu(OSD_CURSOR | OSD_NOCLEAR, cl, cr, "%cPID %s:%c=%c%04d.%02d",
      updn, pidnames[menuFuncData[1]],
      menuFuncData[2]==0?'P':menuFuncData[2]==1?'I':'D',
      (menuFuncDataFloat>=0)?'+':'-',(int)abs(menuFuncDataFloat),(int)abs((menuFuncDataFloat-(int)menuFuncDataFloat)*100.0));

  }
  else {
    float old=(menuFuncData[2]==0?PID[menuFuncData[1]].P:
    menuFuncData[2]==1?PID[menuFuncData[1]].I:
    PID[menuFuncData[1]].D);
    notifyOSDmenu(OSD_CURSOR|OSD_NOCLEAR,21,21,"C %c%04d.%02d->%c%04d.%02d?%c",
      (old>=0)?'+':'-',(int)abs(old),(int)abs((old-(int)old)*100.0),
      (menuFuncDataFloat>=0)?'+':'-',(int)abs(menuFuncDataFloat),(int)abs((menuFuncDataFloat-(int)menuFuncDataFloat)*100.0),
      menuFuncData[3]?'Y':'N');
  }
}

void writeEEPROM();
void initializeEEPROM();

void menuEeprom(byte mode, byte action) {

  switch (action) {
    case MENU_CALLBACK:
      menuInfunc = 0; // exit to menu
      break;
      
    case MENU_INIT:
    case MENU_UP:
    case MENU_DOWN:
      {
        if (action==MENU_INIT) {
          menuFuncData[0] = 0;
        }
        if (action==MENU_UP) {
          menuFuncData[0] = 1;
        }
        if (action==MENU_DOWN) {
          menuFuncData[0] = 0;
        }
        byte cpos = strlen(menuData[menuEntry].t)+4;
          notifyOSDmenu(OSD_CURSOR|OSD_NOCLEAR, cpos, cpos, "%c%s? %c",
          menuFuncData[0]?MENU_SYM_DOWN:MENU_SYM_UP,
          menuData[menuEntry].t,menuFuncData[0]?'Y':'N');
      }
      break;
    case MENU_EXIT:
      menuInfunc = 0;
      return;
      
    case MENU_SELECT:
      if (menuFuncData[0] == 1) {
        switch (mode) {
          case 0:
            writeEEPROM(); // defined in DataStorage.h
            zeroIntegralError();
            notifyOSD(OSD_NOCLEAR,"EEPROM data saved");
            break;
            
          case 1:
            // Initialize EEPROM with default values
            initializeEEPROM(); // defined in DataStorage.h
            calibrateGyro();
            calibrateAccel();
            zeroIntegralError();
            #ifdef HeadingMagHold
              initializeMagnetometer();
            #endif
            #ifdef AltitudeHold
              initializeBaro();
            #endif
            notifyOSD(OSD_NOCLEAR,"EEPROM reinitialized");
            break;
        }
        menuInfunc = 10; // callback after 1s
      }
      else {
        menuInfunc = 0;
      }
  }
}

const struct MenuItem menuData[] = {
#if 0
  {
    0, "Waypoints",        MENU_NOFUNC   ,0  }
  ,
  {
    1, "Select dest.",     menuHandleWpt ,0  }
  ,
  {
    1, "Add wpt",          MENU_NOFUNC   ,0  }
  ,
  {
    2, "Add wpt here",     menuHandleWpt ,1  }
  ,
  {
    2, "Add wpt by coord", menuHandleWpt ,2  }
  ,
  {
    1, "Delete wpt",       menuHandleWpt ,3  }
  ,
#endif
#ifdef MENU_GOPRO
  {
    0, "GoPro",            MENU_NOFUNC     ,0  }
  ,
  {
    1, "Shutter",          menuHandleGopro ,0  }
  ,
  {
    1, "Mode",             menuHandleGopro ,1  }
  ,
  {
    1, "Pwr",              menuHandleGopro ,2  }
  ,
#endif
  {
    0, "Config",           MENU_NOFUNC     ,0  }
  ,
#ifdef CameraControl
  {
    1, "Camera stabilizer",menuHandleCam ,0  }
  ,
#endif
  {
    1, "OSD",              MENU_NOFUNC   ,0  }
  ,
  {
    2, "Reset flightime",  menuHandleOsd ,0  }
  ,
  {
    0, "Setup",            MENU_NOFUNC   ,0  }
  ,
  {
    1, "Edit PIDs",        menuHandlePidTune,0  }
  ,
  {
    1, "Save to EEPROM",   menuEeprom  ,0  }
  ,
  {
    1, "Reinit EEPROM",    menuEeprom  ,1  }
  ,
};

#define MENU_ENTRIES (sizeof(menuData) / sizeof(MenuItem))

byte menu_islast(byte x) {

  for (byte i=x+1; i<MENU_ENTRIES; i++) {
    if (menuData[i].l==menuData[x].l) {
      return 0;
    }
    if (menuData[i].l<menuData[x].l) {
      return 1;
    }
  }
  return 1;
}

#define MENU_LEVEL(x)      (menuData[x].l&0x7f)
#define MENU_CALLFUNC(x,y) menuData[x].f(menuData[x].d,y);

#define MENU_STICK_CENTER  1500  // center value
#define MENU_STICK_NEUTRAL 100   // less than this from center is neutral
#define MENU_STICK_ACTIVE  200   // over this is select
#define MENU_STICK_REPEAT  400   // autorepeat at extreme values

void menuShow(byte entry) {

  if (255 != menuEntry) {
    if (menuAtExit) {
      notifyOSD(OSD_NOCLEAR, "%cEXIT", MENU_SYM_DOWN);
    }
    else {
      if (menuData[entry].t) {
        notifyOSD(OSD_NOCLEAR,"%c%s",
          menu_islast(entry) ? MENU_SYM_UP : MENU_SYM_BOTH,
          menuData[entry].t);
      }
      else {
        notifyOSD(OSD_NOCLEAR,"BUG!!");
      }
    }
  } 
  else {
    notifyOSD(OSD_NOCLEAR,NULL);
  }
}

void menu_up() {

  if (255 == menuEntry) {
    return;
  }

  if (menuInfunc) {
    MENU_CALLFUNC(menuEntry,MENU_UP)
      return;
  }

  if ((0 == menuEntry) || (MENU_LEVEL(menuEntry)>MENU_LEVEL(menuEntry-1))) {
    menuAtExit = 1;
  }
  else {
    for (byte i=menuEntry-1; i>=0; i--) {
      if (MENU_LEVEL(menuEntry)==MENU_LEVEL(i)) {
        menuEntry=i;
        break;
      }
    }
  }
  menuShow(menuEntry);
}

void menu_down() {

  if (255 == menuEntry) {
    return;
  }
  if (menuInfunc) {
    MENU_CALLFUNC(menuEntry,MENU_DOWN)
      return;
  }
  if (menuAtExit) {
    menuAtExit=0;
  }
  else {
    if (menu_islast(menuEntry)) {
      return;
    }
    for (byte i=menuEntry+1;i<MENU_ENTRIES;i++) {
      if (MENU_LEVEL(menuEntry)==MENU_LEVEL(i)) {
        menuEntry=i;
        break;
      }
    }
  }
  menuShow(menuEntry);
}

void menu_select() {
  if (255==menuEntry) {
    // enable menu
    menuAtExit=0;
    menuEntry=0;
  }
  else if (menuInfunc) {
    MENU_CALLFUNC(menuEntry,MENU_SELECT)
      if (menuInfunc) {
        return; // redisplay menu if we exited from handler
      }
  }
  else if (menuAtExit) {
    if (0==menuEntry) {
      menuEntry=255;
    }
    else {
      // leave submenu
      menuEntry--;
      menuAtExit=0;
    }
  }
  else if (menuData[menuEntry].f != MENU_NOFUNC) {
    menuInfunc = 1;
    MENU_CALLFUNC(menuEntry,MENU_INIT)
      return;
  }
  else if (MENU_LEVEL(menuEntry)<MENU_LEVEL(menuEntry+1)) {
    // enter submenu
    menuEntry++;
  }
  menuShow(menuEntry);
}

void menuExit() {

  if (255==menuEntry) {
    return;
  }

  if ((0 == menuEntry) || ( 0 == MENU_LEVEL(menuEntry))) {
    menuEntry = 255;
  }
  else if (menuInfunc) {
    MENU_CALLFUNC(menuEntry,MENU_EXIT)
      if (menuInfunc)
        return;
  }
  else {
    // leave submenu
    for (byte i=menuEntry-1; i>=0; i--) {
      if (MENU_LEVEL(i)<MENU_LEVEL(menuEntry)) {
        menuEntry = i;
        menuAtExit = 0;
        break;
      }
    }
  }
  menuShow(menuEntry);
}

void updateMenu(void) {

  if (armed) {
    if (menuEntry != 255) {
      // BAIL OUT!!!
      notifyOSD(0,NULL); // clear menuline
      menuInfunc = 0;
      menuEntry = 255;
    }
    return;
  }

  if (menuInfunc > 1) {
    menuInfunc--;
    if (menuInfunc == 1) {
      // call the callback when counter hits 1
      MENU_CALLFUNC(menuEntry,MENU_CALLBACK)
        // show the menu entry if we exited handler
        if (menuInfunc==0) {
          menuShow(menuEntry);
        }
    }
    return;
  }

  const short roll  = receiverCommand[ROLL]-MENU_STICK_CENTER;  // pitch/roll should be -500 - +500
  const short pitch = receiverCommand[PITCH]-MENU_STICK_CENTER;
  if (abs(roll) < MENU_STICK_NEUTRAL) {
    if (abs(pitch) < MENU_STICK_NEUTRAL) {
      // stick at center
      if (stickWaitNeutral) {
        stickWaitNeutral = 0;
      }
      return;
    }
    else {
      // roll at neutral, pitch not
      if (abs(pitch) > MENU_STICK_REPEAT) {
        stickWaitNeutral=0;
      }
      if (stickWaitNeutral) {
        return;
      }
      if (abs(pitch) > MENU_STICK_ACTIVE) {
        if (pitch>0) {
          menu_up();
        }
        else {
          menu_down();
        }
        stickWaitNeutral=1;
      }
    }
  }
  else {
    if (abs(pitch)>=MENU_STICK_NEUTRAL) {
      return; // both ways active
    }
    if (abs(roll) > MENU_STICK_ACTIVE) {
      if (roll>0) {
        if (!stickWaitNeutral) {
          menu_select();
        }
      }
      else {
        if ((!stickWaitNeutral) || (abs(roll)>=MENU_STICK_REPEAT)) {
          menuExit();
        }
      }
      stickWaitNeutral=1;
    }
  }
  return;
}

#endif // Menu_h

