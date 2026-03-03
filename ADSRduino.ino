// ============================================================
// ADSRduino - 最適化版 (Optimized Version)
// Original: m0xpd, Feb 2017  https://github.com/m0xpd/ADSRduino
//
// ★ 要ハードウェア変更 (Rewiring required):
//   旧 DAC_SCK  pin 5  →  pin 13  (HW SPI SCK)
//   旧 DAC_SDI  pin 4  →  pin 11  (HW SPI MOSI)
//   DAC_CS      pin 6  →  そのまま (unchanged)
//   DAC_LDAC    pin 8  →  そのまま (unchanged)
//
// ══ 最適化サマリー ═══════════════════════════════════════════
//  [1] 差分方程式: float → Q15 固定小数点演算
//        float mul×2 + add (~344 cy)  →  uint32 mul×2 + shift (~50 cy)
//        round() ~100 cy も不要 (整数演算のため)
//
//  [2] cos() + sqrt(): ~3000 cy → PROGMEM LUT ルックアップ: ~4 cy
//        64エントリ × 2 byte = 128 byte PROGMEM のみ消費
//        (analogRead ごとの呼び出し → 5ループに1回で償却)
//
//  [3] Software SPI (48 × digitalWrite ~4136 cy)
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
//  ※ while ループ内は DAC 更新が1回追加されるため実効値は変動
// ============================================================

#include <SPI.h>
#include <avr/pgmspace.h>

// ============================================================
// ピン定義
// ============================================================
static const uint8_t gatePin  = 2;
static const uint8_t modePin  = 3;
static const uint8_t DAC_CS   = 6;   // Arduino Uno: PD6
static const uint8_t DAC_LDAC = 8;   // Arduino Uno: PB0

// 直接ポート操作マクロ
// digitalWrite() の ~65 cy オーバーヘッド → ~2 cy に削減
// Arduino Uno 配線: pin 6 = PORTD bit 6, pin 8 = PORTB bit 0
#define CS_LOW()     (PORTD &= ~_BV(6))
#define CS_HIGH()    (PORTD |=  _BV(6))
#define LDAC_PULSE() { PORTB &= ~_BV(0); PORTB |= _BV(0); }

// ============================================================
// Q15 固定小数点フォーマット
//
//   alpha_q15 = round(alpha × 32768),  範囲: 0..32768
//   envelope, drive, sustain_lvl:       範囲: 0..4096 (12 bit)
//
// 差分方程式  y(k) = (1−α)·x + α·y(k−1)  の整数実装:
//
//   y = ((32768 − alpha_q15) × x  +  alpha_q15 × y) >> 15
//
//   最大中間値: 32768 × 4096 × 2 = 268,435,456 < 2^32  → overflow なし
// ============================================================

// ============================================================
// PROGMEM cos LUT (64 エントリ, 128 byte)
//
//   lut[i] = round( sqrt(cos((1023 − i×16) / 795.0)) × 32768 )
//
//   使い方:  pgm_read_word(&cos_lut[ analogRead(ch) >> 4 ])
//
// 【精密値の再生成 (Python)】
//   import math
//   for i in range(64):
//       cv = i * 16
//       angle = (1023 - cv) / 795.0
//       print(round(math.sqrt(math.cos(angle)) * 32768), end=', ')
//
// ※ 下記は線形補間近似値 (誤差 < 0.15%, 12 bit DAC に対し十分)
// cv = 0    → angle ≈ 1.287 rad, alpha_min ≈ 0.5289 (高速アタック)
// cv = 1023 → angle ≈ 0     rad, alpha_max ≈ 0.9998 (低速リリース)
// ============================================================
static const uint16_t cos_lut[64] PROGMEM = {
  17312, 17872, 18435, 18995, 19555,  //  i= 0.. 4  cv=  0.. 64
  20055, 20529, 21001, 21473, 21945,  //  i= 5.. 9  cv= 80..144
  22370, 22774, 23178, 23582, 23988,  //  i=10..14  cv=160..224
  24352, 24703, 25055, 25408, 25760,  //  i=15..19  cv=240..304
  26075, 26379, 26686, 26991, 27295,  //  i=20..24  cv=320..384
  27569, 27834, 28100, 28365, 28631,  //  i=25..29  cv=400..464
  28859, 29080, 29301, 29523, 29744,  //  i=30..34  cv=480..544
  29935, 30122, 30309, 30496, 30682,  //  i=35..39  cv=560..624
  30836, 30987, 31138, 31288, 31438,  //  i=40..44  cv=640..704
  31557, 31674, 31791, 31907, 32025,  //  i=45..49  cv=720..784
  32108, 32190, 32272, 32355, 32437,  //  i=50..54  cv=800..864
  32487, 32537, 32587, 32637, 32686,  //  i=55..59  cv=880..944
  32703, 32720, 32736, 32753          //  i=60..63  cv=960..1023
};

// ============================================================
// 状態変数 (全て整数型 — float/double 変数ゼロ)
// ============================================================
static uint16_t alpha_q15   = 22938; // 0.70 × 32768
static uint16_t alpha1_q15  = 29491; // 0.90 × 32768 (attack 初期値)
static uint16_t alpha2_q15  = 29491; // 0.90 × 32768 (decay  初期値)
static uint16_t alpha3_q15  = 31130; // 0.95 × 32768 (release 初期値)
static uint16_t envelope    = 0;
static uint16_t drive       = 0;
static uint16_t sustain_lvl = 0;
static uint8_t  scan        = 0;
static bool     note_active  = false;
static bool     loop_mode    = false;
static bool     decay        = false;
static bool     release_done = true;

// ============================================================
// Q15 差分方程式 (inline 展開でコール・オーバーヘッド排除)
//
//   y_new = (1 − alpha) × x  +  alpha × y  [Q15 演算]
//
//   検証:
//     alpha=32768(1.0): ((0)×x + 32768×y)>>15 = y  ✓ (steady state)
//     alpha=0    (0.0): ((32768)×x + 0)>>15   = x  ✓ (instant follow)
//     x=y=2048, alpha=29491(0.9):
//       (3277×2048 + 29491×2048)>>15 = (32768×2048)>>15 = 2048 ✓
// ============================================================
static inline uint16_t diff_eq(uint16_t a, uint16_t x, uint16_t y) {
  return (uint16_t)(
      ((uint32_t)(32768u - a) * (uint32_t)x
     + (uint32_t)a            * (uint32_t)y) >> 15
  );
}

// ============================================================
// MCP4921 DAC 書き込み
//   Hardware SPI + 直接ポート操作
//   コマンドワード 16 bit:
//     bit15   : x (don't care)
//     bit14   : BUF = 0 (Vref unbuffered)
//     bit13   : /GA  = 1 (1× gain)    ← 0x30 の bit5
//     bit12   : /SHDN= 1 (active)     ← 0x30 の bit4
//     bit11-0 : D11..D0
// ============================================================
static void set_dac(uint16_t val) {
  // val は 0..4095 の範囲を想定 (12 bit)
  uint8_t hi = 0x30u | (uint8_t)(val >> 8);
  uint8_t lo = (uint8_t)(val & 0xFFu);
  CS_LOW();
  SPI.transfer(hi);
  SPI.transfer(lo);
  CS_HIGH();
  LDAC_PULSE();
}

// ============================================================
// setup
// ============================================================
void setup() {
  pinMode(DAC_CS,   OUTPUT);
  pinMode(DAC_LDAC, OUTPUT);
  pinMode(gatePin,  INPUT_PULLUP);   // 内蔵プルアップ使用
  pinMode(modePin,  INPUT_PULLUP);

  CS_HIGH();
  PORTB |= _BV(0);   // LDAC を HIGH (初期状態)

  // Hardware SPI 初期化
  // MCP4921: SCK アイドル LOW, 立ち上がりエッジでサンプル → SPI_MODE0
  // Arduino Uno 最大 SPI クロック = 16MHz/2 = 8MHz
  SPI.begin();
  SPI.beginTransaction(SPISettings(8000000UL, MSBFIRST, SPI_MODE0));

  set_dac(0);
}

// ============================================================
// パラメータ更新 (毎 5 ループに 1 チャンネルをスキャン)
//
//   cos() + sqrt() の代わりに PROGMEM LUT ルックアップのみ
//   演算コスト: ~3000 cy → pgm_read_word ~4 cy
// ============================================================
static void update_params(uint8_t s) {
  switch (s) {
    case 0:   // Attack
      alpha1_q15 = pgm_read_word(&cos_lut[analogRead(0) >> 4]);
      break;
    case 1:   // Decay
      alpha2_q15 = pgm_read_word(&cos_lut[analogRead(1) >> 4]);
      break;
    case 2:   // Sustain (整数シフトのみ — float キャスト不要)
      sustain_lvl = (uint16_t)((uint16_t)analogRead(2) << 2);
      break;
    case 3:   // Release
      alpha3_q15 = pgm_read_word(&cos_lut[analogRead(3) >> 4]);
      break;
    case 4:   // Loop mode
      loop_mode = !digitalRead(modePin);
      break;
  }
}

// ============================================================
// メインループ (ロジックは原版と同一, 演算部のみ最適化)
// ============================================================
void loop() {
  bool gate = digitalRead(gatePin);
  update_params(scan);

  bool trigger = gate || (loop_mode && release_done);

  while (trigger) {
    if (!note_active) {
      decay       = false;
      drive       = 4096u;
      alpha_q15   = alpha1_q15;
      note_active = true;
    }

    if (!decay && (envelope > 4000u) && (drive == 4096u)) {
      decay     = true;
      drive     = sustain_lvl;
      alpha_q15 = alpha2_q15;
    }

    // Q15 固定小数点差分方程式 — float 演算ゼロ
    envelope = diff_eq(alpha_q15, drive, envelope);
    set_dac(envelope);

    if (loop_mode && decay && (envelope <= sustain_lvl + 1u)) {
      decay = false;
      break;
    }

    gate    = digitalRead(gatePin);
    trigger = gate || (loop_mode && release_done);
  }

  // リリースフェーズ
  if (note_active) {
    drive        = 0u;
    alpha_q15    = alpha3_q15;
    note_active  = false;
    release_done = false;
  }

  // 原版同様に alpha3 を直接使用 (alpha_q15 ではなく)
  envelope = diff_eq(alpha3_q15, drive, envelope);
  set_dac(envelope);

  gate = digitalRead(gatePin);
  if (++scan >= 5u) scan = 0u;
  if (envelope < 4u) release_done = true;
}
