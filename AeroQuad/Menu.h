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

struct menuitem {
  const byte l;         // menu level the item is on
  const char *t;        // text to show
  void (*f)(byte,byte); // handler func on leaf level
  const byte d;         // data to give for handler
};
extern const struct menuitem menudata[];

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


byte  menu_infunc = 0;   // tells if a handler func is active
                         // 0 - we're in base menu
                         // 1 - call handler on stick actions
                         // 2-255 countdown and callback when it gets to 1 - no sticks
byte  menu_entry  = 255; // Active menu entry
byte  menu_atexit = 0;   // are we at the exit at the top
short stick_r_old = 0;
short stick_p_old = 0;
byte  stick_waitneutral = 1;

// DATA that menu functions can freely use to store state
byte menu_func_data[10];    // 10 bytes of data for funcs to use as they wish...
float menu_func_data_float; // float for menufuncs use

#ifdef CameraControl
void menu_handle_cam(byte mode, byte action) {

  switch (action) {
    case MENU_INIT:
      menu_func_data[0]=0;
      menu_func_data[1]=0;
      break;
    case MENU_EXIT:
      if (menu_func_data[0]==0) {
        menu_infunc=0;
        return;
      }
      else {
        menu_func_data[0]--;
      }
      break;
    case MENU_SELECT:
      if (menu_func_data[0]<1) {
        menu_func_data[0]++;
      }
      break;
    case MENU_UP:
    case MENU_DOWN:
      if (menu_func_data[0]==0) {
        if (action==MENU_UP) {
          if (menu_func_data[1]<3) menu_func_data[1]++;
        } else {
          if (menu_func_data[1]>0) menu_func_data[1]--;
        }
      }
      else if (menu_func_data[0]==1) {
        int val=(action==MENU_UP)?10:-10;
        switch (menu_func_data[1]) {
          case 0:
            setMode(action==MENU_UP?1:0);
            break;
          case 1:
            setCenterPitch(constrain(getCenterPitch()+val,getServoMinPitch(),getServoMaxPitch()));
            break;
          case 2:
            setCenterRoll(constrain(getCenterRoll()+val,getServoMinRoll(),getServoMaxRoll()));
            break;
          case 3:
            setCenterYaw(constrain(getCenterYaw()+val,getServoMinYaw(),getServoMaxYaw()));
            break;
        }
      }
      break;
  }

  if (menu_func_data[1]==0) {
    notifyOSDmenu(OSD_NOCLEAR|OSD_CURSOR,
      menu_func_data[0]?18:1,menu_func_data[0]?18:16,
      "%cStabilizer mode: %1d",MENU_SYM_BOTH,
      getMode());
  }
  else {
    notifyOSDmenu(OSD_NOCLEAR|OSD_CURSOR,
      menu_func_data[0]?15:8,menu_func_data[0]?18:12,
      "%cCenter %s: %04d",MENU_SYM_BOTH,
      (menu_func_data[1]==1)?"Pitch":
      (menu_func_data[1]==2)?"Roll ":
      "Yaw  ",
      (menu_func_data[1]==1)?getCenterPitch():
      (menu_func_data[1]==2)?getCenterRoll():
      getCenterYaw());

  }
}
#endif

void menu_handle_osd(byte mode, byte action) {

  switch (mode) {
    case 0: armedTime=0;
            break;
  }
  menu_infunc=0;
}

#ifdef MENU_GOPRO
const char *gopro_b_txt[3] = { "Shutter", "Mode", "Power" };

void menu_handle_gopro(byte mode, byte action) {
  switch (action) {
    case MENU_CALLBACK:
      // depress I/O line...
      menu_infunc=0; // exit to menu
      break;
    case MENU_INIT:
      // activate I/O
      memset(buf,0,MENU_BUFSIZE);
      snprintf(buf,MENU_BUFSIZE,"%s pressed",gopro_b_txt[mode]);
      menu_refresh();
      if (mode==2) {
        menu_infunc=35; //3.5 sec
      }
      else {
        menu_infunc=10; // 1 sec
      }
      break;
    default:
      break;
  }
}
#endif

// edit digit  with of form [+/-][i*#].[d*#]
// pos: 0=sign, 1-(i)=intpart, (i+1)-(i+d+1)=decimals
void menu_edit_float(float *f, byte i, byte d, byte pos, byte action, float min, float max) {

  if (pos > (1+i+d)) {
    return;
  }
  // if ((action!=MENU_UP)&&(action!=MENU_DOWN)) return;
  if (pos==0 && (min*max<0)) { // change sign only if min/max are different sign
    *f=-*f;
  }
  else {
    *f += (action==MENU_UP?1.0:-1.0)*pow(10.0,i-pos);
  }
  *f=constrain(*f,min,max);
}

char *pidnames[10] = {
  "RRoll", "RPitc", "RYaw ", "ARoll", "APitc",
  "Headi", "AGRol", "AGPit", "Altit", "ZDamp"};

void menu_handle_pidtune(byte mode, byte action) {

  switch (action) {
    case MENU_INIT:
      menu_func_data[0]=0; //level 0-select PID;1-select P/I/D;>=2-edit value
      menu_func_data[1]=0; // PIDno
      menu_func_data[2]=0; // 0=P/1=I/2=D
      break;
    case MENU_EXIT:
      if (menu_func_data[0]>0) {
        menu_func_data[0]--;
      }
      else {
        menu_infunc=0;
        return;
      }
      menu_func_data[3]=0;
      break;
    case MENU_SELECT:
      if (menu_func_data[0]<9) {
        menu_func_data[0]++;
      }
      else {
        if (menu_func_data[3]) {
          switch (menu_func_data[2]) {
            case 0: PID[menu_func_data[1]].P=menu_func_data_float;
                    break;
            case 1: PID[menu_func_data[1]].I=menu_func_data_float;
                    break;
            case 2: PID[menu_func_data[1]].D=menu_func_data_float;
                    break;
          }
          notifyOSD(OSD_NOCLEAR,"PID value saved!!");
        }
        else {
          notifyOSD(OSD_NOCLEAR,"PID value not saved!!");
        }
        menu_infunc=10;
        menu_func_data[0]=1; //return to P/I/D selection
        return;
      }
      break;
    case MENU_UP:
      if (menu_func_data[0]==0) {
        if (menu_func_data[1]<9) menu_func_data[1]++;
      }
      else if (menu_func_data[0]==1) {
        if (menu_func_data[2]<2) menu_func_data[2]++;
      }
      else if (menu_func_data[0]==9) {
        menu_func_data[3]=1;
      }
      else {
        menu_edit_float(&menu_func_data_float,4,2,menu_func_data[0]-2,MENU_UP,-1000,1000);
      }
      break;
    case MENU_DOWN:
      if (menu_func_data[0]<2) {
        if (menu_func_data[menu_func_data[0]+1]>0) menu_func_data[menu_func_data[0]+1]--;
      }
      else if (menu_func_data[0]==9) {
        menu_func_data[3]=0;
      }
      else {
        menu_edit_float(&menu_func_data_float,4,2,menu_func_data[0]-2,MENU_DOWN,-1000,1000);
      }
      break;
  }

  if (menu_func_data[0]<2) {
    menu_func_data_float =
      (menu_func_data[2]==0?PID[menu_func_data[1]].P:
       menu_func_data[2]==1?PID[menu_func_data[1]].I:
       PID[menu_func_data[1]].D);
  }

  if (menu_func_data[0]<9) {
    byte cl,cr;
    byte updn=MENU_SYM_BOTH;
    if (menu_func_data[0]==0) {
      // selecting PID
      cl=5;
      cr=9;
      if (menu_func_data[1]==0) updn=MENU_SYM_UP;
      if (menu_func_data[1]==9) updn=MENU_SYM_DOWN;
    }
    else if (menu_func_data[0]==1) {
      // selecting P/I/D
      cl=cr=11;
      if (menu_func_data[2]==0) updn=MENU_SYM_UP;
      if (menu_func_data[2]==2) updn=MENU_SYM_DOWN;
    }
    else if (menu_func_data[0]<9) {
      cl=cr=((menu_func_data[0]>6)?12:11)+menu_func_data[0];
    }
    notifyOSDmenu(OSD_CURSOR|OSD_NOCLEAR,cl,cr,"%cPID %s:%c=%c%04d.%02d",
      updn, pidnames[menu_func_data[1]],
      menu_func_data[2]==0?'P':menu_func_data[2]==1?'I':'D',
      (menu_func_data_float>=0)?'+':'-',(int)abs(menu_func_data_float),(int)abs((menu_func_data_float-(int)menu_func_data_float)*100.0));

  }
  else {
    float old=(menu_func_data[2]==0?PID[menu_func_data[1]].P:
       menu_func_data[2]==1?PID[menu_func_data[1]].I:
       PID[menu_func_data[1]].D);
    notifyOSDmenu(OSD_CURSOR|OSD_NOCLEAR,21,21,"C %c%04d.%02d->%c%04d.%02d?%c",
      (old>=0)?'+':'-',(int)abs(old),(int)abs((old-(int)old)*100.0),
      (menu_func_data_float>=0)?'+':'-',(int)abs(menu_func_data_float),(int)abs((menu_func_data_float-(int)menu_func_data_float)*100.0),
       menu_func_data[3]?'Y':'N');
  }
}

void writeEEPROM();
void initializeEEPROM();

void menu_eeprom(byte mode, byte action) {

  switch (action) {
    case MENU_CALLBACK:
      menu_infunc=0; // exit to menu
      break;
    case MENU_INIT:
    case MENU_UP:
    case MENU_DOWN:
      {
        if (action==MENU_INIT) menu_func_data[0]=0;
        if (action==MENU_UP) menu_func_data[0]=1;
        if (action==MENU_DOWN) menu_func_data[0]=0;
        byte cpos=strlen(menudata[menu_entry].t)+4;
          notifyOSDmenu(OSD_CURSOR|OSD_NOCLEAR, cpos, cpos, "%c%s? %c",
          menu_func_data[0]?MENU_SYM_DOWN:MENU_SYM_UP,
          menudata[menu_entry].t,menu_func_data[0]?'Y':'N');
      }
      break;
    case MENU_EXIT:
      menu_infunc=0;
      return;
    case MENU_SELECT:
      if (menu_func_data[0]==1) {
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
        menu_infunc=10; // callback after 1s
      }
      else {
        menu_infunc=0;
      }
  }
}

const struct menuitem menudata[] = {
#if 0
  {0, "Waypoints",        MENU_NOFUNC,0},
  {1, "Select dest.",     menu_handle_wpt,0},
  {1, "Add wpt",          MENU_NOFUNC,0},
  {2, "Add wpt here",     menu_handle_wpt,1},
  {2, "Add wpt by coord", menu_handle_wpt,2},
  {1, "Delete wpt",       menu_handle_wpt,3},
#endif
#ifdef MENU_GOPRO
  {0, "GoPro",            MENU_NOFUNC,0},
  {1, "Shutter",          menu_handle_gopro,0},
  {1, "Mode",             menu_handle_gopro,1},
  {1, "Pwr",              menu_handle_gopro,2},
#endif
  {0, "Config",           MENU_NOFUNC,0},
#ifdef CameraControl
  {1, "Camera stabilizer",menu_handle_cam,0},
#endif
  {1, "OSD",              MENU_NOFUNC,0},
  {2, "Reset flightime",  menu_handle_osd,0},
  {0, "Setup",            MENU_NOFUNC,0},
  {1, "Edit PIDs",        menu_handle_pidtune,0},
  {1, "Save to EEPROM",   menu_eeprom,0},
  {1, "Reinit EEPROM",    menu_eeprom,1},
  };

#define MENU_ENTRIES (sizeof(menudata)/sizeof(menuitem))

byte  menu_islast(byte x) {

  for (byte i=x+1; i<MENU_ENTRIES; i++) {
    if (menudata[i].l==menudata[x].l)
      return 0;
    if (menudata[i].l<menudata[x].l)
      return 1;
  }
  return 1;
}

#define MENU_LEVEL(x)      (menudata[x].l&0x7f)
#define MENU_CALLFUNC(x,y) menudata[x].f(menudata[x].d,y);

#define MENU_STICK_CENTER  1500  // center value
#define MENU_STICK_NEUTRAL 100   // less than this from center is neutral
#define MENU_STICK_ACTIVE  200   // over this is select
#define MENU_STICK_REPEAT  400   // autorepeat at extreme values

void menu_show(byte entry) {

  if (255!=menu_entry) {
    if (menu_atexit) {
      notifyOSD(OSD_NOCLEAR,"%cEXIT",MENU_SYM_DOWN);
    }
    else {
      if (menudata[entry].t) {
        notifyOSD(OSD_NOCLEAR,"%c%s",
          menu_islast(entry)?MENU_SYM_UP:MENU_SYM_BOTH,
          menudata[entry].t);
      }
      else {
        notifyOSD(OSD_NOCLEAR,"BUG!!");
      }
    }
  } else {
    notifyOSD(OSD_NOCLEAR,NULL);
  }
}

void menu_up() {

  if (255==menu_entry) return;

  if (menu_infunc) {
    MENU_CALLFUNC(menu_entry,MENU_UP)
    return;
  }

  if ((0==menu_entry) || (MENU_LEVEL(menu_entry)>MENU_LEVEL(menu_entry-1))) {
    menu_atexit=1;
  }
  else {
    for (byte i=menu_entry-1; i>=0; i--) {
      if (MENU_LEVEL(menu_entry)==MENU_LEVEL(i)) {
        menu_entry=i;
        break;
      }
    }
  }
  menu_show(menu_entry);
}

void menu_down() {

  if (255==menu_entry)
    return;
  if (menu_infunc) {
    MENU_CALLFUNC(menu_entry,MENU_DOWN)
    return;
  }
  if (menu_atexit) {
    menu_atexit=0;
  }
  else {
    if (menu_islast(menu_entry)) return;
    for (byte i=menu_entry+1;i<MENU_ENTRIES;i++) {
      if (MENU_LEVEL(menu_entry)==MENU_LEVEL(i)) {
        menu_entry=i;
        break;
      }
    }
  }
  menu_show(menu_entry);
}

void menu_select() {
  if (255==menu_entry) {
    // enable menu
    menu_atexit=0;
    menu_entry=0;
  }
  else if (menu_infunc) {
    MENU_CALLFUNC(menu_entry,MENU_SELECT)
    if (menu_infunc) return; // redisplay menu if we exited from handler
  }
  else if (menu_atexit) {
    if (0==menu_entry) {
      menu_entry=255;
    }
    else {
      // leave submenu
      menu_entry--;
      menu_atexit=0;
    }
  }
  else if (menudata[menu_entry].f!=MENU_NOFUNC) {
    menu_infunc=1;
    MENU_CALLFUNC(menu_entry,MENU_INIT)
    return;
  }
  else if (MENU_LEVEL(menu_entry)<MENU_LEVEL(menu_entry+1)) {
    // enter submenu
    menu_entry++;
  }
  menu_show(menu_entry);
}

void menu_exit() {

  if (255==menu_entry)
    return;

  if ((0==menu_entry)||(0==MENU_LEVEL(menu_entry))) {
    menu_entry=255;
  }
  else if (menu_infunc) {
    MENU_CALLFUNC(menu_entry,MENU_EXIT)
    if (menu_infunc)
      return;
  }
  else {
    // leave submenu
    for (byte i=menu_entry-1; i>=0; i--) {
       if (MENU_LEVEL(i)<MENU_LEVEL(menu_entry)) {
         menu_entry=i;
         menu_atexit=0;
         break;
       }
    }
  }
  menu_show(menu_entry);
}

void updateMenu(void) {

  if (armed==true) {
    if (menu_entry!=255) {
      // BAIL OUT!!!
      notifyOSD(0,NULL); // clear menuline
      menu_infunc=0;
      menu_entry=255;
    }
    return;
  }

  if (menu_infunc>1) {
    menu_infunc--;
    if (menu_infunc==1) {
      // call the callback when counter hits 1
      MENU_CALLFUNC(menu_entry,MENU_CALLBACK)
      // show the menu entry if we exited handler
      if (menu_infunc==0) menu_show(menu_entry);
    }
    return;
  }

  const short roll  = receiverCommand[ROLL]-MENU_STICK_CENTER;  // pitch/roll should be -500 - +500
  const short pitch = receiverCommand[PITCH]-MENU_STICK_CENTER;
  if (abs(roll)<MENU_STICK_NEUTRAL) {
    if (abs(pitch)<MENU_STICK_NEUTRAL) {
      // stick at center
      if (stick_waitneutral) {
        stick_waitneutral=0;
      }
      return;
    }
    else {
      // roll at neutral, pitch not
      if (abs(pitch) > MENU_STICK_REPEAT) stick_waitneutral=0;
      if (stick_waitneutral)
        return;
      if (abs(pitch) > MENU_STICK_ACTIVE) {
        if (pitch>0) {
          menu_up();
        }
        else {
          menu_down();
        }
        stick_waitneutral=1;
      }
    }
  }
  else {
    if (abs(pitch)>=MENU_STICK_NEUTRAL) return; // both ways active
    if (abs(roll) > MENU_STICK_ACTIVE) {
        if (roll>0) {
          if (!stick_waitneutral) {
            menu_select();
          }
        }
        else {
          if ((!stick_waitneutral) || (abs(roll)>=MENU_STICK_REPEAT)) {
            menu_exit();
          }
        }
        stick_waitneutral=1;
      }
  }
  return;
}

#endif // Menu_h
