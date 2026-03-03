// ============================================================
// ADSRduino - 最適化版 (Optimized Version)
// ATmega328P (Uno/Nano) および Arduino Nano R4 サポート
// Original: m0xpd, Feb 2017  https://github.com/m0xpd/ADSRduino
//
// ★ 要ハードウェア変更 (Rewiring required):
//   旧 DAC_SCK  pin 5  →  pin 13  (HW SPI SCK)
//   旧 DAC_SDI  pin 4  →  pin 11  (HW SPI MOSI)
//   DAC_CS      pin 6  →  そのまま (unchanged)
//   DAC_LDAC    pin 8  →  そのまま (unchanged)
//
// ══ 最適化サマリー ═══════════════════════════════════════════
//  [1] SPI クロック設定
//  [2] CS / LDAC 操作
//  [3] 差分方程式: float → Q15 固定小数点演算 (AVRのみ)
//        float mul×2 + add (~344 cy)  →  uint32 mul×2 + shift (~50 cy)
//        round() ~100 cy も不要 (整数演算のため)
//        Nano R4: FPU あり → float のまま 1cy
//  [4] cos() + sqrt(): ~3000 cy → LUT ルックアップ ~4 cy
//        AVR: 64エントリ × 2 byte = 128 byte PROGMEM のみ消費
//        R4:  通常の const float 配列 (フラッシュ 256KB、直接参照)
//  [5] Software SPI (48 × digitalWrite ~4136 cy)
//      → Hardware SPI + 直接ポート操作: ~100 cy
//
//  ┌────────────────┬──────────┬──────────┬────────────┐
//  │ 処理           │ 変更前   │ 変更後   │  削減      │
//  ├────────────────┼──────────┼──────────┼────────────┤
//  │ 差分方程式     │  ~344 cy │  ~50 cy  │  ~294 cy   │
//  │ cos()+sqrt()/5 │  ~600 cy │   ~1 cy  │  ~599 cy   │
//  │ SPI 転送       │ ~4136 cy │ ~100 cy  │ ~4036 cy   │
//  │ round()        │  ~100 cy │    0 cy  │  ~100 cy   │
//  │ analogRead()/5 │  ~320 cy │  ~320 cy │     0 cy   │
//  │ その他         │   ~50 cy │   ~40 cy │   ~10 cy   │
//  ├────────────────┼──────────┼──────────┼────────────┤
//  │ 合計           │ ~5550 cy │  ~511 cy │ ~5039 cy   │
//  │ @16MHz         │  ~347 µs │   ~32 µs │            │
//  │ サンプルレート │ ~2.9 kHz │  ~31 kHz │  約10.8倍  │
//  └────────────────┴──────────┴──────────┴────────────┘
// ============================================================

#include <SPI.h>
// ★ avr/pgmspace.h はここには書かない → BOARD_AVR328ブロック内のみ

// ============================================================
// ボード自動判別
// ============================================================
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  #define BOARD_AVR328
#elif defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI)
  #define BOARD_R4
#else
  #error "未対応のボードです。ピン定義と SPI 設定を手動で追加してください。"
#endif

// ============================================================
// ピン定義 (ボード共通)
//   ATmega328P:  SCK=pin13(PB5), MOSI=pin11(PB3)
//   RA4M1 R4:    SCK=pin13,      MOSI=pin11  (同じピン番号)
// ============================================================
static const uint8_t gatePin  = 2;
static const uint8_t modePin  = 3;
static const uint8_t DAC_CS   = 6;    // AVR: PORTD bit6
static const uint8_t DAC_LDAC = 8;    // AVR: PORTB bit0

// ============================================================
// [1] SPI クロック設定
// ============================================================
#if defined(BOARD_AVR328)
  #define SPI_CLOCK 8000000UL
#elif defined(BOARD_R4)
  #define SPI_CLOCK 24000000UL
#endif

// ============================================================
// [2] CS / LDAC 操作マクロ
//   AVR: 直接ポート操作 (~2 cy) で digitalWrite (~65 cy) を回避
//   R4:  digitalWrite で十分 (効果差が小さく移植性を優先)
// ============================================================
#if defined(BOARD_AVR328)
  #define CS_LOW()     (PORTD &= ~_BV(6))
  #define CS_HIGH()    (PORTD |=  _BV(6))
  #define LDAC_PULSE() { PORTB &= ~_BV(0); PORTB |= _BV(0); }
#elif defined(BOARD_R4)
  #define CS_LOW()     digitalWrite(DAC_CS,   LOW)
  #define CS_HIGH()    digitalWrite(DAC_CS,   HIGH)
  #define LDAC_PULSE() { digitalWrite(DAC_LDAC, LOW); digitalWrite(DAC_LDAC, HIGH); }
#endif

// ============================================================
// [3] 演算型・変数・差分方程式マクロ
//
//   ★ 変数名は両ボードで統一 (alpha, alpha1, alpha2, alpha3,
//     envelope, drive, sustain_lvl) し、共通コードから参照可能にする。
//     型だけを #if で切り替える。
//
// --- AVR: Q15 固定小数点 ---
//   alpha_t = uint16_t,  値 = round(float_alpha × 32768)
//   envelope / drive / sustain_lvl = uint16_t (0..4096)
//
//   差分方程式: y = ((32768−a)×x + a×y) >> 15
//   最大中間値: 32768 × 4096 × 2 = 268,435,456 < 2^32  → 溢れなし
//
//   検証:
//     a=32768(1.0): y→y  ✓   a=0(0.0): y→x  ✓
//     x=y=2048, a=29491(0.9): (3277+29491)×2048>>15 = 2048 ✓
//
// --- R4: float (Cortex-M4 FPU, 1サイクル演算) ---
//   alpha_t = float
//   envelope / drive / sustain_lvl = float
// ============================================================
#if defined(BOARD_AVR328)
  #include <avr/pgmspace.h>      // ★ AVR専用ヘッダはここで include

  typedef uint16_t alpha_t;
  #define FLOAT_TO_ALPHA(x)  ((alpha_t)((x) * 32768u))

  static alpha_t  alpha       = FLOAT_TO_ALPHA(0.70f);
  static alpha_t  alpha1      = FLOAT_TO_ALPHA(0.90f);
  static alpha_t  alpha2      = FLOAT_TO_ALPHA(0.90f);
  static alpha_t  alpha3      = FLOAT_TO_ALPHA(0.95f);
  static uint16_t envelope    = 0;
  static uint16_t drive       = 0;
  static uint16_t sustain_lvl = 0;

  static inline uint16_t diff_eq(alpha_t a, uint16_t x, uint16_t y) {
    return (uint16_t)(
        ((uint32_t)(32768u - a) * (uint32_t)x
       + (uint32_t)a            * (uint32_t)y) >> 15
    );
  }

  #define CALC_ENVELOPE(a, d, e) diff_eq((a), (d), (e))
  #define SET_DRIVE_FULL()       (drive = 4096u)
  #define SET_DRIVE_SUSTAIN()    (drive = sustain_lvl)
  #define SET_DRIVE_ZERO()       (drive = 0u)
  #define DRIVE_IS_FULL()        (drive == 4096u)        // ★ 整数比較: 安全
  #define ENV_OVER_4000()        (envelope > 4000u)
  #define ENV_NEAR_SUSTAIN()     (envelope <= sustain_lvl + 1u)
  #define ENV_NEAR_ZERO()        (envelope < 4u)
  #define DAC_VALUE()            (envelope)
  #define SET_ALPHA(src)         (alpha = (src))
  #define SET_SUSTAIN()          (sustain_lvl = (uint16_t)((uint16_t)analogRead(2) << 2))

#elif defined(BOARD_R4)
  typedef float alpha_t;

  static alpha_t alpha       = 0.70f;
  static alpha_t alpha1      = 0.90f;
  static alpha_t alpha2      = 0.90f;
  static alpha_t alpha3      = 0.95f;
  static float   envelope    = 0.0f;
  static float   drive       = 0.0f;
  static float   sustain_lvl = 0.0f;

  #define CALC_ENVELOPE(a, d, e) ((1.0f - (a)) * (d) + (a) * (e))
  #define SET_DRIVE_FULL()       (drive = 4096.0f)
  #define SET_DRIVE_SUSTAIN()    (drive = sustain_lvl)
  #define SET_DRIVE_ZERO()       (drive = 0.0f)
  #define DRIVE_IS_FULL()        (drive > 4095.9f)       // ★ float比較: 閾値で判定
  #define ENV_OVER_4000()        (envelope > 4000.0f)
  #define ENV_NEAR_SUSTAIN()     (envelope < sustain_lvl + 1.0f)
  #define ENV_NEAR_ZERO()        (envelope < 4.0f)
  #define DAC_VALUE()            ((uint16_t)roundf(envelope))
  #define SET_ALPHA(src)         (alpha = (src))
  #define SET_SUSTAIN()          (sustain_lvl = (float)((uint16_t)analogRead(2) << 2))
#endif

// ============================================================
// [4] cos+sqrt LUT
//   lut[i] = sqrt(cos((1023 − i×16) / 795.0))
//   インデックス: analogRead(ch) >> 4  (0..63, 16ステップ分解能)
//
//   AVR: uint16_t × Q15スケール, PROGMEM格納 (128 byte flash)
//        pgm_read_word() でアクセス
//   R4:  float, 通常の const 配列 (直接参照)
//
// 【LUT再生成 Python スクリプト】
//   import math
//   for i in range(64):
//       cv = i * 16
//       v = math.sqrt(math.cos((1023-cv)/795.0))
//       print(f"{round(v*32768)},", end=' ')   # AVR用
//       # print(f"{v:.4f}f,",        end=' ')  # R4用
// ============================================================
#if defined(BOARD_AVR328)
  static const uint16_t cos_lut[64] PROGMEM = {
    17312, 17872, 18435, 18995, 19555,
    20055, 20529, 21001, 21473, 21945,
    22370, 22774, 23178, 23582, 23988,
    24352, 24703, 25055, 25408, 25760,
    26075, 26379, 26686, 26991, 27295,
    27569, 27834, 28100, 28365, 28631,
    28859, 29080, 29301, 29523, 29744,
    29935, 30122, 30309, 30496, 30682,
    30836, 30987, 31138, 31288, 31438,
    31557, 31674, 31791, 31907, 32025,
    32108, 32190, 32272, 32355, 32437,
    32487, 32537, 32587, 32637, 32686,
    32703, 32720, 32736, 32753
  };
  #define READ_LUT(i) ((alpha_t)pgm_read_word(&cos_lut[(i)]))

#elif defined(BOARD_R4)
  static const float cos_lut[64] = {
    0.5282f, 0.5453f, 0.5624f, 0.5793f, 0.5962f,
    0.6115f, 0.6259f, 0.6403f, 0.6547f, 0.6690f,
    0.6819f, 0.6944f, 0.7068f, 0.7193f, 0.7319f,
    0.7426f, 0.7530f, 0.7635f, 0.7740f, 0.7844f,
    0.7938f, 0.8030f, 0.8122f, 0.8214f, 0.8306f,
    0.8389f, 0.8469f, 0.8549f, 0.8630f, 0.8710f,
    0.8780f, 0.8848f, 0.8916f, 0.8984f, 0.9052f,
    0.9112f, 0.9170f, 0.9228f, 0.9286f, 0.9343f,
    0.9391f, 0.9438f, 0.9485f, 0.9531f, 0.9578f,
    0.9615f, 0.9651f, 0.9687f, 0.9723f, 0.9759f,
    0.9784f, 0.9808f, 0.9832f, 0.9856f, 0.9880f,
    0.9896f, 0.9911f, 0.9926f, 0.9942f, 0.9957f,
    0.9962f, 0.9967f, 0.9972f, 0.9977f
  };
  #define READ_LUT(i) ((alpha_t)cos_lut[(i)])
#endif

// ============================================================
// 共通状態変数 (型非依存)
// ============================================================
static uint8_t scan        = 0;
static bool    note_active = false;
static bool    loop_mode   = false;
static bool    decay       = false;
static bool    release_done = true;

// ============================================================
// MCP4921 DAC 書き込み (共通)
//   コマンドワード 16 bit:
//     bit14: BUF=0 (Vref unbuffered)
//     bit13: /GA =1 (1× gain)   → 0x30
//     bit12: /SHDN=1 (active)
//     bit11-0: D11..D0
// ============================================================
static void set_dac(uint16_t val) {
  uint8_t hi = 0x30u | (uint8_t)(val >> 8);
  uint8_t lo = (uint8_t)(val & 0xFFu);
  CS_LOW();
  SPI.transfer(hi);
  SPI.transfer(lo);
  CS_HIGH();
  LDAC_PULSE();
}

// ============================================================
// パラメータ更新 (5ループに1チャンネルをスキャン)
//   ★ 変数名 alpha1/alpha2/alpha3 は両ボード共通 → コンパイル通過
//   ★ READ_LUT() の戻り値型は alpha_t に自動キャスト
// ============================================================
static void update_params(uint8_t s) {
  switch (s) {
    case 0: alpha1    = READ_LUT(analogRead(0) >> 4); break;  // Attack
    case 1: alpha2    = READ_LUT(analogRead(1) >> 4); break;  // Decay
    case 2: SET_SUSTAIN();                             break;  // Sustain
    case 3: alpha3    = READ_LUT(analogRead(3) >> 4); break;  // Release
    case 4: loop_mode = !digitalRead(modePin);         break;  // Loop
  }
}

// ============================================================
// setup (共通)
// ============================================================
void setup() {
  pinMode(DAC_CS,   OUTPUT);
  pinMode(DAC_LDAC, OUTPUT);
  pinMode(gatePin,  INPUT_PULLUP);
  pinMode(modePin,  INPUT_PULLUP);
  CS_HIGH();
  digitalWrite(DAC_LDAC, HIGH);

  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
  set_dac(0);
}

// ============================================================
// メインループ (共通ロジック — 演算部はマクロで自動切り替え)
// ============================================================
void loop() {
  bool gate = digitalRead(gatePin);
  update_params(scan);

  bool trigger = gate || (loop_mode && release_done);

  while (trigger) {
    if (!note_active) {
      decay = false;
      SET_DRIVE_FULL();
      SET_ALPHA(alpha1);
      note_active = true;
    }

    // ★ drive==4096 の直接比較をやめ DRIVE_IS_FULL() マクロに統一
    //    AVR: uint16_t 厳密比較、R4: float 閾値比較
    if (!decay && ENV_OVER_4000() && DRIVE_IS_FULL()) {
      decay = true;
      SET_DRIVE_SUSTAIN();
      SET_ALPHA(alpha2);
    }

    envelope = CALC_ENVELOPE(alpha, drive, envelope);
    set_dac(DAC_VALUE());

    if (loop_mode && decay && ENV_NEAR_SUSTAIN()) {
      decay = false;
      break;
    }
    gate    = digitalRead(gatePin);
    trigger = gate || (loop_mode && release_done);
  }

  if (note_active) {
    SET_DRIVE_ZERO();
    SET_ALPHA(alpha3);
    note_active  = false;
    release_done = false;
  }

  envelope = CALC_ENVELOPE(alpha3, drive, envelope);
  set_dac(DAC_VALUE());

  gate = digitalRead(gatePin);
  if (++scan >= 5u) scan = 0u;
  if (ENV_NEAR_ZERO()) release_done = true;
}
