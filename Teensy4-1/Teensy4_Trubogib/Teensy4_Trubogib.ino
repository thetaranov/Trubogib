/*
 * Teensy4_Trubogib.ino — Прошивка Teensy 4.1, контроллер движения трубогиба v2.0
 *
 * Архитектура:
 *   EXE TubeBender ──USB-Serial──► Teensy 4.1 ──Step/Dir──► MR-JE-200A (×2)
 *                                      │
 *                               RS-485 Modbus RTU
 *                                      │
 *                               Owen PLC110-30 (Master)
 *
 * Teensy = Modbus Slave (ID=1, 115200 8N1)
 * ПЛК = Modbus Master (пишет рег.0-6 по FC 0x10, читает рег.9-10 по FC 0x03)
 *
 * Пины макетной платы (4× MAX490 + 1× MAX485):
 *   0 (RX1)  — MAX485 RO (через делитель 1кОм+2кОм, 5V→3.3V)
 *   1 (TX1)  — MAX485 DI
 *   24       — MAX485 DE+RE
 *   2        — Step Z  (MAX490 #1 DI → Y/Z → CN1 PP/PG серво Z)
 *   3        — Dir Z   (MAX490 #2 DI → Y/Z → CN1 NP/NG серво Z)
 *   4        — Step C  (MAX490 #3 DI → Y/Z → CN1 PP/PG серво C)
 *   5        — Dir C   (MAX490 #4 DI → Y/Z → CN1 NP/NG серво C)
 *   6        — ЗАПАС   (SON Z теперь через ПЛК DO7 → синий провод CN1 pin 15)
 *   7        — ЗАПАС   (SON C теперь через ПЛК DO8 → синий провод CN1 pin 15)
 *   8        — ЗАПАС   (Home Z — при необходимости)
 *   9        — ЗАПАС   (Home C — при необходимости)
 *
 * Механика:
 *   Серво: MR-JE-200A + HG-SN152J-S100 (1.5 кВт), Pr.PA13=0011
 *   10000 имп/об (заводская настройка MR-JE-A)
 *   Ось Z: PF115L1-008 (i=1:8), шестерня M2 Z35 (d_дел=70мм) + рейка M2
 *          80000 имп/оборот вых. вала, π×70 мм/оборот → 363.78 шаг/мм
 *   Ось C: PF115L2-020 (i=1:20), Z35→Z105 (3:1), итого 60:1
 *          600000 имп/оборот патрона, 360° → 1666.67 шаг/°
 */

/* ======================== КОНФИГУРАЦИЯ ======================== */

#define FW_VERSION "2.1"

/* -- Modbus -- */
#define MODBUS_BAUD       115200
#define MODBUS_ADDR       1
#define RS485_DE          24        /* DE+RE MAX485 */

/* -- Step/Dir -- */
#define PIN_STEP_Z        2
#define PIN_DIR_Z         3
#define PIN_STEP_C        4
#define PIN_DIR_C         5
/* Пины 6-9: ЗАПАС (SON теперь через ПЛК DO7/DO8, Home — при необходимости) */
// #define PIN_SON_Z      6         /* запас */
// #define PIN_SON_C      7         /* запас */
// #define PIN_HOME_Z     8         /* запас */
// #define PIN_HOME_C     9         /* запас */

/* -- Безопасность -- */
#define WD_TIMEOUT_MS     2000      /* >2 с без Modbus → E-Stop */
#define DEBOUNCE_MS       20        /* антидребезг Home */

/* -- Доставка Ext CMD в ПЛК -- */
#define EXT_CMD_PREZERO_MS      120   /* для edge-команд (START/STOP/ZERO/AUTO) */
#define EXT_CMD_HOLD_MS         450
#define EXT_CMD_POSTZERO_MS     250
#define EXT_CMD_FAST_HOLD_MS    350   /* для level-команд SON/E-STOP */
#define EXT_CMD_FAST_POSTZERO_MS 300

/* -- Движение -- */
#define FREQ_MIN          500.0f    /* Гц, стартовая/финальная частота */
#define FREQ_MAX_Z        100000.0f /* Гц, макс. Step Z */
#define FREQ_MAX_C        50000.0f  /* Гц, макс. Step C */
#define ACCEL_HZ_S_Z      200000.0f /* Гц/с, ускорение Z */
#define ACCEL_HZ_S_C      100000.0f /* Гц/с, ускорение C */
#define HOME_FREQ         2000.0f   /* Гц, грубый поиск Home */
#define HOME_FREQ_SLOW    500.0f    /* Гц, точный подход */
#define HOME_BACKOFF      500       /* шагов отъезда от датчика */
#define PULSE_US          2         /* мкс, ширина импульса Step */
#define MIN_INTERVAL_US   7         /* мкс, мин. период таймера */

/* -- Механика -- */
#define STEPS_MM_Z        363.78f   /* шаг/мм  (10000×8 / (π×70)) */
#define STEPS_DEG_C       1666.67f  /* шаг/°   (10000×60 / 360) */

/* ======================== КАРТА РЕГИСТРОВ MODBUS ======================== */

/*  Регистры 0-6: ПЛК → Teensy (FC 0x10, запись каждые 100 мс) */
#define R_CUR_ANGLE       0   /* gCurrentAngle — текущий угол гибки (°) */
#define R_TGT_ANGLE       1   /* gTargetAngle  — целевой угол (°) */
#define R_CYCLE           2   /* gCycleNum     — номер цикла (1/2) */
#define R_MODE            3   /* gMode         — 0=ручной, 1=авто */
#define R_DONE            4   /* gDone         — цель достигнута 0/1 */
#define R_PEDAL_FWD       5   /* gPedalFwd     — педаль вперёд 0/1 */
#define R_PEDAL_REV       6   /* gPedalRev     — педаль назад 0/1 */
#define R_ALARMS          7   /* Feedback from PLC: b0=almZ,b1=almC,b2=DO7,b3=DO8,b4=DO9,b5=DO12 */

/*  Регистры 8-10: Teensy → ПЛК (FC 0x03, чтение каждые 100 мс)
 *  Совместимость:
 *    - старые проекты ПЛК: gExtCommand в рег.8
 *    - актуальный TRUBOGIB_CLEAN.exp: gExtCommand в рег.9, gExtTarget в рег.10
 */
#define R_EXT_CMD_COMPAT  8   /* compat: gExtCommand (legacy map 0..9) */
#define R_EXT_CMD         9   /* gExtCommand (actual PLC map) */
#define R_EXT_TGT         10  /* gExtTarget  (actual PLC map) */

/*  Регистры 11-20: EXE ↔ Teensy (управление осями) */
#define R_AXIS_CMD        11  /* Команда: 1=homeZ,2=homeC,3=moveZ,4=moveC,5=stop,6=SON on,7=SON off,10=E-Stop */
#define R_Z_TGT           12  /* Цель Z (мм, int16) */
#define R_Z_SPD           13  /* Скорость Z (мм/с) */
#define R_C_TGT           14  /* Цель C (°×10, int16) */
#define R_C_SPD           15  /* Скорость C (°/с) */
#define R_Z_POS           16  /* Позиция Z (мм, r/o) */
#define R_Z_ST            17  /* Статус Z (r/o) */
#define R_C_POS           18  /* Позиция C (°×10, r/o) */
#define R_C_ST            19  /* Статус C (r/o) */
#define R_FLAGS           20  /* Флаги (r/o): b0=SON_Z,b1=SON_C,b2=homeZ,b3=homeC,b4=linkOK */

#define NUM_REGS          32
uint16_t regs[NUM_REGS];

/* ======================== ТИПЫ ======================== */

enum State : uint8_t { ST_IDLE=0, ST_MOVING=1, ST_DONE=2, ST_ERROR=3, ST_HOMING=4 };
enum HPhase : uint8_t { HP_NONE=0, HP_SEEK=1, HP_BACK=2, HP_REFINE=3 };

struct Axis {
  uint8_t pStep, pDir, pSon, pHome;
  float spu;            /* steps per unit (mm или °) */
  float fMax, acc;      /* макс. частота, ускорение */

  volatile long pos;    /* текущая позиция (шаги) */
  long tgt;             /* целевая позиция (шаги) */

  /* профиль движения */
  volatile float freq;  /* текущая частота (Гц) */
  float freqTgt;        /* целевая частота */
  volatile long decelN; /* кол-во шагов разгона = кол-во шагов торможения */
  volatile long done;   /* шагов сделано */
  long total;           /* всего шагов в текущем движении */
  int8_t dir;           /* +1 / -1 */

  volatile State  st;
  volatile HPhase hp;
  volatile long   backTgt; /* цель отъезда при Home */
  bool son;                /* Servo ON */

  /* debounce Home */
  unsigned long dbT;
  bool dbRaw, sensor;
};

Axis az, ac;
IntervalTimer tmZ, tmC;

/* ======================== ГЛОБАЛЬНЫЕ ======================== */

unsigned long mbOkTime = 0;
bool wdTrip = false;
unsigned long mbRx = 0, mbTx = 0, mbErr = 0;
char usbBuf[128];
int usbN = 0;
unsigned long extCmdT = 0, extTgtT = 0;
uint16_t extCmdPulseVal = 0;
unsigned long extCmdPulseStart = 0;
bool extCmdFastMode = false;

/* ======================== CRC16 MODBUS ======================== */

uint16_t crc16(const uint8_t *buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= buf[i];
    for (int j = 0; j < 8; j++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

/* ======================== RS-485 ======================== */

inline void txEn(bool on) { digitalWriteFast(RS485_DE, on); }

void mbSend(const uint8_t *buf, int len) {
  txEn(true);
  delayMicroseconds(50);
  Serial1.write(buf, len);
  Serial1.flush();
  delayMicroseconds(100);
  txEn(false);
  mbTx++;
}

/* ======================== MODBUS SLAVE ======================== */

void mbHandle(const uint8_t *f, int len) {
  if (len < 4 || f[0] != MODBUS_ADDR) return;

  uint16_t cc = crc16(f, len - 2);
  uint16_t cr = f[len-2] | (f[len-1] << 8);
  if (cc != cr) { mbErr++; return; }

  mbRx++;
  mbOkTime = millis();
  uint8_t fc = f[1];

  if (fc == 0x03 && len == 8) {
    /* FC 03: Read Holding Registers */
    uint16_t s = (f[2]<<8)|f[3], n = (f[4]<<8)|f[5];
    if (s + n > NUM_REGS || n > 32) return;
    uint8_t r[5 + 64];
    r[0] = MODBUS_ADDR; r[1] = 0x03; r[2] = n * 2;
    for (int i = 0; i < n; i++) {
      r[3+i*2]   = regs[s+i] >> 8;
      r[3+i*2+1] = regs[s+i] & 0xFF;
    }
    int rl = 3 + n*2;
    uint16_t c = crc16(r, rl);
    r[rl++] = c & 0xFF; r[rl++] = c >> 8;
    mbSend(r, rl);
  }
  else if (fc == 0x10 && len >= 9) {
    /* FC 10: Write Multiple Registers */
    uint16_t s = (f[2]<<8)|f[3], n = (f[4]<<8)|f[5];
    uint8_t bc = f[6];
    if (s+n > NUM_REGS || bc != n*2 || len < 9+(int)bc) return;
    for (int i = 0; i < n; i++) {
      int reg = s + i;
      /* защита r/o регистров */
      if (reg == R_EXT_CMD || reg == R_EXT_TGT) continue;
      if (reg >= R_Z_POS) continue;
      regs[reg] = (f[7+i*2]<<8) | f[8+i*2];
    }
    uint8_t r[8] = { MODBUS_ADDR, 0x10, f[2],f[3], f[4],f[5], 0,0 };
    uint16_t c = crc16(r, 6);
    r[6] = c & 0xFF; r[7] = c >> 8;
    mbSend(r, 8);
  }
  else {
    /* Exception: Illegal Function */
    uint8_t r[5] = { MODBUS_ADDR, (uint8_t)(fc|0x80), 0x01, 0,0 };
    uint16_t c = crc16(r, 3);
    r[3] = c & 0xFF; r[4] = c >> 8;
    mbSend(r, 5);
  }
}

/* RS-485 приём */
static uint8_t rxB[256];
static int rxN = 0;
static unsigned long rxT = 0;

void pollMb() {
  while (Serial1.available() && rxN < 255) {
    rxB[rxN++] = Serial1.read();
    rxT = micros();
  }
  if (rxN > 0 && (micros() - rxT > 1750)) {   /* t3.5 gap: фикс. 1750 мкс для скоростей >19200 (в т.ч. 115200) */
    if (rxN >= 4) mbHandle(rxB, rxN);
    rxN = 0;
  }
  if (rxN > 0 && (micros() - rxT > 50000))     /* таймаут мусора */
    rxN = 0;
}

/* ======================== STEP ISR — ТРАПЕЦЕИДАЛЬНЫЙ ПРОФИЛЬ ======================== */

/*
 * Алгоритм: f += acc/f (разгон), f -= acc/f (торможение)
 * decelN отслеживает кол-во шагов разгона — столько же нужно на торможение.
 * Когда remaining <= decelN → переход на торможение.
 * Треугольный профиль (короткие перемещения) обрабатывается автоматически.
 */

void isrZ() {
  /* --- Homing --- */
  if (az.st == ST_HOMING) {
    if (az.hp == HP_BACK) {
      digitalWriteFast(PIN_DIR_Z, HIGH);
      digitalWriteFast(PIN_STEP_Z, HIGH);
      delayMicroseconds(PULSE_US);
      digitalWriteFast(PIN_STEP_Z, LOW);
      az.pos++;
      if (az.pos >= az.backTgt) {
        az.hp = HP_REFINE;
        tmZ.update((unsigned long)(1000000.0f / HOME_FREQ_SLOW));
      }
    } else { /* HP_SEEK или HP_REFINE */
      digitalWriteFast(PIN_DIR_Z, LOW);
      digitalWriteFast(PIN_STEP_Z, HIGH);
      delayMicroseconds(PULSE_US);
      digitalWriteFast(PIN_STEP_Z, LOW);
      az.pos--;
    }
    return;
  }

  /* --- Обычное движение --- */
  if (az.st != ST_MOVING) { tmZ.end(); return; }

  long rem = az.total - az.done;
  if (rem <= 0) { az.st = ST_DONE; tmZ.end(); return; }

  /* Импульс Step */
  digitalWriteFast(PIN_STEP_Z, HIGH);

  az.pos += az.dir;
  az.done++;
  rem--;

  /* Обновление частоты */
  float f = az.freq;
  if (rem <= az.decelN) {
    f -= ACCEL_HZ_S_Z / f;                  /* торможение */
    if (f < FREQ_MIN) f = FREQ_MIN;
  } else if (f < az.freqTgt) {
    f += ACCEL_HZ_S_Z / f;                  /* разгон */
    if (f > az.freqTgt) f = az.freqTgt;
    az.decelN = az.done;                     /* зеркало: торм. = разг. */
  }
  az.freq = f;

  delayMicroseconds(PULSE_US);
  digitalWriteFast(PIN_STEP_Z, LOW);

  unsigned long iv = (unsigned long)(1000000.0f / f);
  if (iv < MIN_INTERVAL_US) iv = MIN_INTERVAL_US;
  tmZ.update(iv);
}

void isrC() {
  if (ac.st == ST_HOMING) {
    if (ac.hp == HP_BACK) {
      digitalWriteFast(PIN_DIR_C, HIGH);
      digitalWriteFast(PIN_STEP_C, HIGH);
      delayMicroseconds(PULSE_US);
      digitalWriteFast(PIN_STEP_C, LOW);
      ac.pos++;
      if (ac.pos >= ac.backTgt) {
        ac.hp = HP_REFINE;
        tmC.update((unsigned long)(1000000.0f / HOME_FREQ_SLOW));
      }
    } else {
      digitalWriteFast(PIN_DIR_C, LOW);
      digitalWriteFast(PIN_STEP_C, HIGH);
      delayMicroseconds(PULSE_US);
      digitalWriteFast(PIN_STEP_C, LOW);
      ac.pos--;
    }
    return;
  }

  if (ac.st != ST_MOVING) { tmC.end(); return; }

  long rem = ac.total - ac.done;
  if (rem <= 0) { ac.st = ST_DONE; tmC.end(); return; }

  digitalWriteFast(PIN_STEP_C, HIGH);

  ac.pos += ac.dir;
  ac.done++;
  rem--;

  float f = ac.freq;
  if (rem <= ac.decelN) {
    f -= ACCEL_HZ_S_C / f;
    if (f < FREQ_MIN) f = FREQ_MIN;
  } else if (f < ac.freqTgt) {
    f += ACCEL_HZ_S_C / f;
    if (f > ac.freqTgt) f = ac.freqTgt;
    ac.decelN = ac.done;
  }
  ac.freq = f;

  delayMicroseconds(PULSE_US);
  digitalWriteFast(PIN_STEP_C, LOW);

  unsigned long iv = (unsigned long)(1000000.0f / f);
  if (iv < MIN_INTERVAL_US) iv = MIN_INTERVAL_US;
  tmC.update(iv);
}

/* ======================== УПРАВЛЕНИЕ ОСЯМИ ======================== */

void sonSet(Axis &a, bool on) {
  /* SON управляется ПЛК (DO9/DO12), здесь только внутренний трекинг */
  a.son = on;
}

void queueExtCmdPulse(uint16_t cmd) {
  /* SON/E-STOP (6/7/10) обрабатываются в ПЛК level-логикой: отправляем без pre-zero.
     Остальные команды остаются pulse/edge-совместимыми. */
  extCmdPulseVal = cmd;
  extCmdPulseStart = millis();
  extCmdFastMode = (cmd == 6 || cmd == 7 || cmd == 10);
  if (extCmdFastMode) {
    /* Level-latched режим: держим команду постоянно до следующей команды.
       Это исключает пропуски при редком/неровном опросе ПЛК. */
    regs[R_EXT_CMD_COMPAT] = cmd;
    regs[R_EXT_CMD] = cmd;
  } else {
    regs[R_EXT_CMD_COMPAT] = 0;
    regs[R_EXT_CMD] = 0;
  }
  extCmdT = millis();
}

void serviceExtCmdPulse() {
  if (extCmdPulseVal == 0) return;

  unsigned long now = millis();
  unsigned long elapsed = now - extCmdPulseStart;

  if (extCmdFastMode) {
    /* Для level-команд 6/7/10 НЕ делаем автосброс в 0.
       ПЛК сам трактует уровень mb_in_Command и держит SON-состояние стабильно. */
    regs[R_EXT_CMD_COMPAT] = extCmdPulseVal;
    regs[R_EXT_CMD] = extCmdPulseVal;
    return;
  }

  if (elapsed < EXT_CMD_PREZERO_MS) {
    /* pre-zero: сбросить предыдущее значение в ПЛК */
    regs[R_EXT_CMD_COMPAT] = 0;
    regs[R_EXT_CMD] = 0;
    return;
  }

  if (elapsed < (EXT_CMD_PREZERO_MS + EXT_CMD_HOLD_MS)) {
    /* удерживать команду достаточно долго (как в ESP: ~1с+) */
    regs[R_EXT_CMD_COMPAT] = extCmdPulseVal;
    regs[R_EXT_CMD] = extCmdPulseVal;
    return;
  }

  if (elapsed < (EXT_CMD_PREZERO_MS + EXT_CMD_HOLD_MS + EXT_CMD_POSTZERO_MS)) {
    /* post-zero: дать ПЛК сбросить prev_mb_in_command */
    regs[R_EXT_CMD_COMPAT] = 0;
    regs[R_EXT_CMD] = 0;
    return;
  }

  {
    regs[R_EXT_CMD_COMPAT] = 0;
    regs[R_EXT_CMD] = 0;
    extCmdPulseVal = 0;
    extCmdPulseStart = 0;
    extCmdFastMode = false;
    extCmdT = 0;
  }
}

void axStop(Axis &a, IntervalTimer &t) {
  t.end();
  a.st = ST_IDLE;
  a.hp = HP_NONE;
}

void eStop() {
  tmZ.end(); tmC.end();
  az.st = ST_IDLE; az.hp = HP_NONE;
  ac.st = ST_IDLE; ac.hp = HP_NONE;
  az.son = false; ac.son = false;
  /* Отправить E-Stop в ПЛК (CMD 10) → SON OFF на DO7/DO8 */
  queueExtCmdPulse(10);
}

void startMove(Axis &a, IntervalTimer &t, void(*isr)(), long tgtSteps, float maxFreq) {
  if (!a.son) { a.st = ST_ERROR; return; }

  t.end();

  long delta = tgtSteps - a.pos;
  if (delta == 0) { a.st = ST_DONE; return; }

  a.tgt     = tgtSteps;
  a.dir     = (delta > 0) ? 1 : -1;
  a.total   = labs(delta);
  a.done    = 0;
  a.decelN  = 0;
  a.freq    = FREQ_MIN;
  a.freqTgt = maxFreq;
  a.st      = ST_MOVING;

  digitalWriteFast(a.pDir, delta > 0 ? HIGH : LOW);

  unsigned long iv = (unsigned long)(1000000.0f / FREQ_MIN);
  t.begin(isr, iv);
}

void startHome(Axis &a, IntervalTimer &t, void(*isr)()) {
  if (!a.son) { sonSet(a, true); delay(200); }
  t.end();
  a.st = ST_HOMING;
  a.hp = HP_SEEK;
  t.begin(isr, (unsigned long)(1000000.0f / HOME_FREQ));
}

/* Home debounce */
bool readHome(Axis &a) {
  bool raw = (digitalReadFast(a.pHome) == LOW);
  if (raw != a.dbRaw) { a.dbT = millis(); a.dbRaw = raw; }
  if ((millis() - a.dbT) >= DEBOUNCE_MS) a.sensor = a.dbRaw;
  return a.sensor;
}

/* Home FSM (вызывается из loop) */
void homeUpdate(Axis &a, IntervalTimer &t, void(*isr)()) {
  if (a.st != ST_HOMING) return;
  bool hit = readHome(a);

  if (a.hp == HP_SEEK && hit) {
    t.end();
    a.hp = HP_BACK;
    a.backTgt = a.pos + HOME_BACKOFF;
    t.begin(isr, (unsigned long)(1000000.0f / HOME_FREQ));
  }
  else if (a.hp == HP_REFINE && hit) {
    t.end();
    a.pos = 0;
    a.tgt = 0;
    a.st  = ST_DONE;
    a.hp  = HP_NONE;
  }
}

/* Команды осей из регистра R_AXIS_CMD */
void procAxisCmd() {
  uint16_t cmd = regs[R_AXIS_CMD];
  if (cmd == 0) return;
  regs[R_AXIS_CMD] = 0;

  switch (cmd) {
    case 1: startHome(az, tmZ, isrZ); break;
    case 2: startHome(ac, tmC, isrC); break;
    case 3: {
      int16_t mm = (int16_t)regs[R_Z_TGT];
      uint16_t spd = regs[R_Z_SPD];
      if (spd == 0) spd = 100;
      float freq = spd * STEPS_MM_Z;
      if (freq > FREQ_MAX_Z) freq = FREQ_MAX_Z;
      startMove(az, tmZ, isrZ, (long)(mm * STEPS_MM_Z), freq);
      break;
    }
    case 4: {
      int16_t d10 = (int16_t)regs[R_C_TGT];
      uint16_t spd = regs[R_C_SPD];
      if (spd == 0) spd = 30;
      float freq = spd * STEPS_DEG_C;
      if (freq > FREQ_MAX_C) freq = FREQ_MAX_C;
      startMove(ac, tmC, isrC, (long)(d10 / 10.0f * STEPS_DEG_C), freq);
      break;
    }
    case 5:  axStop(az, tmZ); axStop(ac, tmC); break;
    case 6:  az.son = true;  ac.son = true;
             queueExtCmdPulse(6); break;  /* SON ON → ПЛК */
    case 7:  axStop(az, tmZ); axStop(ac, tmC);
             az.son = false; ac.son = false;
             queueExtCmdPulse(7); break;  /* SON OFF → ПЛК */
    case 10: eStop(); break;
  }
}

/* Обновление read-only регистров */
void updateRegs() {
  noInterrupts();
  long zp = az.pos, cp = ac.pos;
  uint8_t zs = az.st, cs = ac.st;
  interrupts();

  regs[R_Z_POS] = (uint16_t)(int16_t)(zp / STEPS_MM_Z);
  regs[R_Z_ST]  = zs;
  regs[R_C_POS] = (uint16_t)(int16_t)(cp * 10.0f / STEPS_DEG_C);
  regs[R_C_ST]  = cs;

  uint16_t fl = 0;
  if (az.son)    fl |= 0x01;
  if (ac.son)    fl |= 0x02;
  if (az.sensor) fl |= 0x04;
  if (ac.sensor) fl |= 0x08;
  if (!wdTrip)   fl |= 0x10;
  regs[R_FLAGS] = fl;
}

/* ======================== WATCHDOG ======================== */

void checkWD() {
  if (mbOkTime == 0) return;
  bool lost = (millis() - mbOkTime) > WD_TIMEOUT_MS;
  if (lost && !wdTrip) {
    wdTrip = true;
    eStop();
    Serial.println("!WD: Modbus lost >2s, E-Stop!");
  } else if (!lost) {
    wdTrip = false;
  }
}

/* Проверка аларм серво (от ПЛК через Modbus рег.7) */
void checkAlarms() {
  uint16_t alm = regs[R_ALARMS];

  /* ФАКТИЧЕСКИЙ статус SON из ПЛК: gDO_7/gDO_8 */
  az.son = (alm & 0x04) != 0;  /* bit2 = DO7 */
  ac.son = (alm & 0x08) != 0;  /* bit3 = DO8 */

  /* Если аларм — остановить соответствующую ось и снять SON */
  if (alm & 1) { /* almZ */
    if (az.son) { axStop(az, tmZ); az.son = false; }
  }
  if (alm & 2) { /* almC */
    if (ac.son) { axStop(ac, tmC); ac.son = false; }
  }
}

/* Авто-сброс команд ПЛК */
void clearPlcRegs() {
  if (regs[R_EXT_TGT] && extTgtT && (millis()-extTgtT > 2000))
    { regs[R_EXT_TGT] = 0; extTgtT = 0; }
}

/* ======================== USB-SERIAL ПРОТОКОЛ ======================== */

/*
 * Команды (завершаются \n):
 *   ?           — JSON-статус
 *   HELP        — список команд
 *   REGS        — дамп всех регистров
 *   MZ pos,spd  — движение Z (мм, мм/с)
 *   MC pos,spd  — движение C (°, °/с)
 *   HZ / HC     — Home ось Z / C
 *   STOP        — остановить обе оси
 *   ESTOP       — аварийная остановка + SON off
 *   SON 1/0     — Servo ON/OFF обе оси
 *   CMD n       — записать gExtCommand (рег.9)
 *   TGT n       — записать gExtTarget (рег.10)
 *   WR reg val  — записать произвольный регистр
 */

void printStatus() {
  unsigned long lnk = (mbOkTime > 0) ? (millis() - mbOkTime) : 99999;
  noInterrupts();
  long zp = az.pos, cp = ac.pos;
  uint8_t zs = az.st, cs = ac.st;
  interrupts();

  Serial.print("{\"v\":\"" FW_VERSION "\",\"mb\":{\"rx\":");
  Serial.print(mbRx);   Serial.print(",\"tx\":");
  Serial.print(mbTx);   Serial.print(",\"err\":");
  Serial.print(mbErr);  Serial.print(",\"link\":");
  Serial.print(lnk);
  Serial.print("},\"z\":{\"mm\":");
  Serial.print(zp / STEPS_MM_Z, 2);
  Serial.print(",\"st\":"); Serial.print(zs);
  Serial.print(",\"son\":"); Serial.print(az.son);
  Serial.print("},\"c\":{\"deg\":");
  Serial.print(cp / STEPS_DEG_C, 2);
  Serial.print(",\"st\":"); Serial.print(cs);
  Serial.print(",\"son\":"); Serial.print(ac.son);
  Serial.print("},\"bend\":{\"cur\":");
  Serial.print((int16_t)regs[R_CUR_ANGLE]);
  Serial.print(",\"tgt\":"); Serial.print((int16_t)regs[R_TGT_ANGLE]);
  Serial.print(",\"cyc\":"); Serial.print(regs[R_CYCLE]);
  Serial.print(",\"mode\":"); Serial.print(regs[R_MODE]);
  Serial.print(",\"done\":"); Serial.print(regs[R_DONE]);
  Serial.print("},\"wd\":"); Serial.print(wdTrip);
  Serial.print(",\"alm\":{\"z\":"); Serial.print((regs[R_ALARMS] & 1) ? 1 : 0);
  Serial.print(",\"c\":"); Serial.print((regs[R_ALARMS] & 2) ? 1 : 0);
  Serial.print("},\"ext\":{\"cmd8\":"); Serial.print((int16_t)regs[R_EXT_CMD_COMPAT]);
  Serial.print(",\"cmd9\":"); Serial.print((int16_t)regs[R_EXT_CMD]);
  Serial.print(",\"tgt10\":"); Serial.print((int16_t)regs[R_EXT_TGT]);
  Serial.print(",\"pulse\":"); Serial.print(extCmdPulseVal ? 1 : 0);
  Serial.print(",\"fast\":"); Serial.print(extCmdFastMode ? 1 : 0);
  Serial.print(",\"do7\":"); Serial.print((regs[R_ALARMS] & 4) ? 1 : 0);
  Serial.print(",\"do8\":"); Serial.print((regs[R_ALARMS] & 8) ? 1 : 0);
  Serial.print("}");
  Serial.println("}");
}

void processUsb(const char *cmd) {
  /* Статус */
  if (cmd[0] == '?') { printStatus(); return; }

  /* Дамп регистров */
  if (strncmp(cmd, "REGS", 4) == 0) {
    for (int i = 0; i < NUM_REGS; i++) {
      Serial.print("R"); Serial.print(i); Serial.print("=");
      Serial.print((int16_t)regs[i]);
      Serial.print(i < NUM_REGS-1 ? " " : "\n");
    }
    return;
  }

  /* Помощь */
  if (strncmp(cmd, "HELP", 4) == 0) {
    Serial.println(F(
      "?         JSON status\n"
      "REGS      dump registers\n"
      "MZ mm,spd move Z (mm, mm/s)\n"
      "MC deg,spd move C (deg, deg/s)\n"
      "HZ / HC   home axis\n"
      "STOP      stop axes\n"
      "ESTOP     e-stop + SON off\n"
      "SON 1/0   servo on/off\n"
      "CMD n     set gExtCommand\n"
      "TGT n     set gExtTarget\n"
      "WR r v    write register"
    ));
    return;
  }

  /* Move Z */
  if (cmd[0] == 'M' && cmd[1] == 'Z') {
    float pos = 0, spd = 100;
    if (sscanf(cmd+2, "%f,%f", &pos, &spd) >= 1) {
      float freq = fabsf(spd) * STEPS_MM_Z;
      if (freq > FREQ_MAX_Z) freq = FREQ_MAX_Z;
      if (freq < FREQ_MIN)   freq = FREQ_MIN;
      startMove(az, tmZ, isrZ, (long)(pos * STEPS_MM_Z), freq);
      Serial.println("OK MZ");
    } else Serial.println("ERR MZ");
    return;
  }

  /* Move C */
  if (cmd[0] == 'M' && cmd[1] == 'C') {
    float pos = 0, spd = 30;
    if (sscanf(cmd+2, "%f,%f", &pos, &spd) >= 1) {
      float freq = fabsf(spd) * STEPS_DEG_C;
      if (freq > FREQ_MAX_C) freq = FREQ_MAX_C;
      if (freq < FREQ_MIN)   freq = FREQ_MIN;
      startMove(ac, tmC, isrC, (long)(pos * STEPS_DEG_C), freq);
      Serial.println("OK MC");
    } else Serial.println("ERR MC");
    return;
  }

  /* Home */
  if (cmd[0]=='H' && cmd[1]=='Z') { startHome(az, tmZ, isrZ); Serial.println("OK HZ"); return; }
  if (cmd[0]=='H' && cmd[1]=='C') { startHome(ac, tmC, isrC); Serial.println("OK HC"); return; }

  /* Stop */
  if (strncmp(cmd,"STOP",4)==0) { axStop(az,tmZ); axStop(ac,tmC); Serial.println("OK STOP"); return; }
  if (strncmp(cmd,"ESTOP",5)==0) { eStop(); Serial.println("OK ESTOP"); return; }

  /* Servo ON/OFF → forwarded to PLC via Modbus CMD */
  if (strncmp(cmd,"SON",3)==0) {
    int v = atoi(cmd+3);
    az.son = (v != 0); ac.son = (v != 0);
    queueExtCmdPulse(v ? 6 : 7);  /* CMD 6=SON ON, CMD 7=SON OFF → ПЛК DO9/DO12 */
    Serial.print("OK SON "); Serial.println(v);
    return;
  }

  /* Команда ПЛК */
  if (strncmp(cmd,"CMD",3)==0) {
    queueExtCmdPulse((uint16_t)atoi(cmd+3));
    Serial.println("OK CMD");
    return;
  }

  /* Целевой угол для ПЛК */
  if (strncmp(cmd,"TGT",3)==0) {
    regs[R_EXT_TGT] = atoi(cmd+3);
    extTgtT = millis();
    Serial.println("OK TGT");
    return;
  }

  /* Запись в регистр */
  if (cmd[0]=='W' && cmd[1]=='R') {
    int r = 0, v = 0;
    if (sscanf(cmd+2, "%d %d", &r, &v)==2 && r>=0 && r<NUM_REGS) {
      regs[r] = (uint16_t)v;
      Serial.print("OK WR R"); Serial.print(r); Serial.print("="); Serial.println(v);
    } else Serial.println("ERR WR");
    return;
  }

  Serial.println("ERR ?");
}

void pollUsb() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (usbN > 0) { usbBuf[usbN] = 0; processUsb(usbBuf); usbN = 0; }
    } else if (usbN < 126) {
      usbBuf[usbN++] = c;
    }
  }
}

/* ======================== SETUP ======================== */

void setup() {
  /* RS-485 */
  pinMode(RS485_DE, OUTPUT);
  txEn(false);
  Serial1.begin(MODBUS_BAUD, SERIAL_8N1);

  /* USB-Serial (EXE + debug) */
  Serial.begin(115200);
  delay(100);
  Serial.println("=== Teensy 4.1 Trubogib v" FW_VERSION " ===");
  Serial.print("CPU: "); Serial.print(F_CPU / 1000000); Serial.println(" MHz");
  Serial.println("Modbus Slave ID=1, 115200 8N1");
  Serial.println("Type ? for status, HELP for commands");

  /* Step/Dir */
  pinMode(PIN_STEP_Z, OUTPUT); digitalWriteFast(PIN_STEP_Z, LOW);
  pinMode(PIN_DIR_Z,  OUTPUT); digitalWriteFast(PIN_DIR_Z,  LOW);
  pinMode(PIN_STEP_C, OUTPUT); digitalWriteFast(PIN_STEP_C, LOW);
  pinMode(PIN_DIR_C,  OUTPUT); digitalWriteFast(PIN_DIR_C,  LOW);

  /* Пины 6-9: ЗАПАС (SON через ПЛК DO7/DO8, Home — при необходимости) */
  /* pinMode(6, OUTPUT); — не используется */
  /* pinMode(7, OUTPUT); — не используется */
  /* pinMode(8, INPUT_PULLUP); — не используется */
  /* pinMode(9, INPUT_PULLUP); — не используется */

  /* Инициализация осей */
  memset(&az, 0, sizeof(az));
  az.pStep = PIN_STEP_Z; az.pDir = PIN_DIR_Z;
  az.pSon  = 0;  az.pHome = 0;   /* пины 6-9 запас, SON через ПЛК */
  az.spu   = STEPS_MM_Z; az.fMax  = FREQ_MAX_Z; az.acc = ACCEL_HZ_S_Z;

  memset(&ac, 0, sizeof(ac));
  ac.pStep = PIN_STEP_C; ac.pDir = PIN_DIR_C;
  ac.pSon  = 0;  ac.pHome = 0;
  ac.spu   = STEPS_DEG_C; ac.fMax = FREQ_MAX_C; ac.acc = ACCEL_HZ_S_C;

  /* Регистры */
  memset(regs, 0, sizeof(regs));
}

/* ======================== MAIN LOOP ======================== */

void loop() {
  pollMb();                         /* 1. Modbus RS-485 (приоритет) */
  pollUsb();                        /* 2. USB-Serial (EXE + отладка) */
  procAxisCmd();                    /* 3. Команды осей из регистра */
  serviceExtCmdPulse();             /* 3.1 Надежная доставка CMD в ПЛК (8/9) */
  homeUpdate(az, tmZ, isrZ);       /* 4. FSM Home Z (debounce) */
  homeUpdate(ac, tmC, isrC);       /* 5. FSM Home C (debounce) */
  updateRegs();                     /* 6. Обновление read-only регистров */
  checkAlarms();                    /* 7. Проверка аларм серво (от ПЛК D17/D18) */
  checkWD();                        /* 8. Watchdog (>2с → E-Stop) */
  clearPlcRegs();                   /* 9. Авто-сброс команд ПЛК */
}
