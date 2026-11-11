#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>

#define PIN_IR A0

// ---------------- 고정 상수 ----------------
const uint8_t NUM_POINTS = 7;      // 보정 위치 개수 (0, 5, 10, ..., 30 cm)
const uint8_t MAX_ORDER  = 5;      // 선택 가능한 최대 다항식 차수
const uint8_t BUF_SIZE   = 100;    // 한 번에 저장할 수 있는 최대 샘플 수
const uint8_t LINE_LEN   = 24;     // 시리얼 한 줄 입력 버퍼 크기
const float   STEP_MM    = 50.0f;  // 보정 위치 사이 간격 (5 cm = 50 mm)

// ---------------- 사용자가 고르는 설정 ----------------
uint8_t modelType    = 1;    // 1: ADC 다항식, 2: 1/ADC 다항식
uint8_t polyOrder    = 2;    // 다항식 차수
uint8_t medianWindow = 100;  // 위치 하나당 읽는 샘플 수

// ---------------- 보정 결과 ----------------
unsigned int calibAdc[NUM_POINTS];   // 각 위치에서 얻은 필터링된 ADC 값
unsigned int sampleBuf[BUF_SIZE];    // 중앙값 계산용 임시 버퍼
float coef[MAX_ORDER + 1];           // coef[0]이 상수항, coef[k]가 x^k 계수
float xMid  = 0.0f;                  // 정규화 중심
float xHalf = 1.0f;                  // 정규화 반폭

// 스케치 전처리기가 자동으로 만들어 주는 원형에 기대지 않도록 직접 선언해 둔다.
bool readSerialLine(char *buf, uint8_t cap);
uint8_t askNumber(uint8_t lo, uint8_t hi, uint8_t def);
int cmpUint(const void *p, const void *q);
unsigned int ir_sensor_filtered(unsigned int n, float position, int verbose);
float toX(unsigned int adc);
bool solveLeastSquares();
float volt_to_distance(unsigned int a_value);
bool withinCalibRange(unsigned int adc);
void printFunction();
void printFitSummary();
void runCalibration();

void setup()
{
  Serial.begin(1000000);  // 예제 코드와 같은 통신 속도

  // Leonardo 같은 Native USB 보드를 위해 포트가 열리기를 최대 3초까지만 기다린다.
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000UL) {
    delay(1);
  }

  runCalibration();
}

void loop()
{
  char cmd[LINE_LEN];
  Serial.println(F("[Enter] 거리 측정   [R]+[Enter] 보정 다시 하기"));

  if (!readSerialLine(cmd, sizeof(cmd))) {
    Serial.println(F("입력이 너무 깁니다. 다시 입력해 주세요."));
    return;
  }
  if (cmd[0] == 'R' || cmd[0] == 'r') {
    runCalibration();
    return;
  }
  if (cmd[0] != '\0') {
    Serial.println(F("측정하려면 아무것도 입력하지 말고 Enter만 누르세요."));
    return;
  }

  unsigned int adc = ir_sensor_filtered(medianWindow, 0.5f, 0);
  if (modelType == 2 && adc == 0) {
    Serial.println(F("ADC 값이 0이라 1/ADC를 계산할 수 없습니다. 측정을 건너뜁니다."));
    return;
  }

  Serial.print(F("필터값:"));
  Serial.print(adc);
  Serial.print(F(" ==> 거리:"));
  Serial.print(volt_to_distance(adc), 2);
  Serial.println(F(" mm"));

  if (!withinCalibRange(adc)) {
    Serial.println(F("주의: 보정한 ADC 범위를 벗어났습니다 (외삽된 값)."));
  }
}

// 보정 때 얻은 ADC 최솟값~최댓값 안에 들어오는지 확인한다.
bool withinCalibRange(unsigned int adc)
{
  unsigned int lo = calibAdc[0];
  unsigned int hi = calibAdc[0];
  for (uint8_t i = 1; i < NUM_POINTS; ++i) {
    if (calibAdc[i] < lo) lo = calibAdc[i];
    if (calibAdc[i] > hi) hi = calibAdc[i];
  }
  return adc >= lo && adc <= hi;
}

// 줄 끝이 CR, LF, CRLF 중 어느 것이든 한 줄로 읽는다.
// CR 바로 뒤에 오는 LF는 버리므로 CRLF가 빈 줄 두 개로 잡히지 않는다.
// String 대신 호출한 쪽의 고정 버퍼를 써서 Uno의 힙 단편화를 피한다.
// 버퍼보다 긴 줄은 끝까지 읽어서 버리고 false를 돌려준다.
bool readSerialLine(char *buf, uint8_t cap)
{
  static bool dropNextLF = false;
  uint8_t len = 0;
  bool tooLong = false;

  for (;;) {
    if (!Serial.available()) {
      delay(1);
      continue;
    }
    char ch = (char)Serial.read();

    if (dropNextLF) {
      dropNextLF = false;
      if (ch == '\n') continue;
    }
    if (ch == '\r' || ch == '\n') {
      dropNextLF = (ch == '\r');
      buf[len] = '\0';
      return !tooLong;
    }

    if (len + 1 < cap) {
      buf[len++] = ch;
    } else {
      tooLong = true;
    }
  }
}

// lo~hi 범위의 정수를 입력받는다. 빈 줄이면 def를 돌려준다.
uint8_t askNumber(uint8_t lo, uint8_t hi, uint8_t def)
{
  char buf[LINE_LEN];
  for (;;) {
    Serial.print(F("입력 범위 "));
    Serial.print(lo);
    Serial.print('~');
    Serial.print(hi);
    Serial.print(F(" (기본값 "));
    Serial.print(def);
    Serial.println(F("). 숫자를 입력하고 Enter를 누르세요."));

    if (readSerialLine(buf, sizeof(buf))) {
      if (buf[0] == '\0') return def;

      char *rest;
      long val = strtol(buf, &rest, 10);
      bool parsed = (rest != buf);
      while (isspace((unsigned char)*rest)) ++rest;  // 숫자 뒤 공백은 허용
      if (parsed && *rest == '\0' && val >= lo && val <= hi) {
        return (uint8_t)val;
      }
    }
    Serial.println(F("올바르지 않은 값입니다. 다시 입력해 주세요."));
  }
}

// qsort용 비교 함수 (오버플로 없는 오름차순 비교)
int cmpUint(const void *p, const void *q)
{
  unsigned int a = *(const unsigned int *)p;
  unsigned int b = *(const unsigned int *)q;
  return (a > b) - (a < b);
}

// 예제 1, 2와 인터페이스가 같은 필터. malloc 대신 전역 버퍼를 쓴다.
// 정렬된 샘플에서 n * position 번째 값을 고른다(position = 0.5면 중앙값).
// 인자가 잘못되면 0을 돌려준다.
unsigned int ir_sensor_filtered(unsigned int n, float position, int verbose)
{
  if (n == 0 || n > BUF_SIZE || !(position >= 0.0f && position <= 1.0f)) {
    return 0;
  }

  unsigned long t0 = millis();
  for (unsigned int i = 0; i < n; ++i) {
    sampleBuf[i] = analogRead(PIN_IR);
    if (verbose == 1) {
      Serial.print(sampleBuf[i]);
      Serial.print(' ');
    }
  }
  qsort(sampleBuf, n, sizeof(sampleBuf[0]), cmpUint);

  unsigned int idx = (unsigned int)(n * position);
  if (idx >= n) idx = n - 1;
  unsigned int picked = sampleBuf[idx];

  if (verbose == 1) Serial.println();
  if (verbose >= 2) {
    Serial.print(F("소요 시간: "));
    Serial.print(millis() - t0);
    Serial.println(F(" ms"));
  }
  return picked;
}

// 선택한 모델에 맞춰 ADC 값을 다항식 변수로 바꾼다.
float toX(unsigned int adc)
{
  if (modelType == 2) return 1.0f / (float)adc;
  return (float)adc;
}

// 보정 데이터로 다항식 계수를 구한다. 실패하면 false.
bool solveLeastSquares()
{
  // --- 1단계: 변환된 x의 범위를 구해 [-1, 1] 정규화 계수를 정한다 ---
  float xLo = 0.0f;
  float xHi = 0.0f;
  for (uint8_t i = 0; i < NUM_POINTS; ++i) {
    if (modelType == 2 && calibAdc[i] == 0) return false;  // 0으로 나누기 방지
    float x = toX(calibAdc[i]);
    if (i == 0 || x < xLo) xLo = x;
    if (i == 0 || x > xHi) xHi = x;
  }
  xMid  = (xLo + xHi) * 0.5f;
  xHalf = (xHi - xLo) * 0.5f;
  if (!(xHalf > 0.0f)) return false;  // 모든 ADC가 같으면 맞출 수 없음

  // --- 2단계: 설계 행렬 A와 목표 벡터 y 구성 ---
  // A의 r번째 행 = [1, x, x^2, ..., x^order], y = 실제 거리(mm)
  float A[NUM_POINTS][MAX_ORDER + 1];
  float y[NUM_POINTS];
  float h[NUM_POINTS];  // Householder 반사 벡터
  const uint8_t cols = polyOrder + 1;

  for (uint8_t r = 0; r < NUM_POINTS; ++r) {
    float x = (toX(calibAdc[r]) - xMid) / xHalf;
    A[r][0] = 1.0f;
    for (uint8_t c = 1; c < cols; ++c) A[r][c] = A[r][c - 1] * x;
    y[r] = STEP_MM * r;  // 0, 50, 100, ..., 300 mm
  }

  // --- 3단계: Householder QR. A를 위삼각 R로, y를 Q^T y로 바꾼다 ---
  for (uint8_t k = 0; k < cols; ++k) {
    float sq = 0.0f;
    for (uint8_t r = k; r < NUM_POINTS; ++r) sq += A[r][k] * A[r][k];
    float len = sqrtf(sq);
    if (!(len > 0.00001f)) return false;  // 열이 사실상 0 (데이터 중복 등)

    // 상쇄 오차를 줄이려고 대각 원소와 반대 부호를 고른다.
    float alpha = (A[k][k] >= 0.0f) ? -len : len;
    for (uint8_t r = k; r < NUM_POINTS; ++r) h[r] = A[r][k];
    h[k] -= alpha;

    float hh = 0.0f;
    for (uint8_t r = k; r < NUM_POINTS; ++r) hh += h[r] * h[r];
    float beta = 2.0f / hh;

    // 남은 열에 반사 적용
    for (uint8_t c = k + 1; c < cols; ++c) {
      float dot = 0.0f;
      for (uint8_t r = k; r < NUM_POINTS; ++r) dot += h[r] * A[r][c];
      for (uint8_t r = k; r < NUM_POINTS; ++r) A[r][c] -= beta * h[r] * dot;
    }

    // 목표 벡터에도 같은 반사 적용
    float dotY = 0.0f;
    for (uint8_t r = k; r < NUM_POINTS; ++r) dotY += h[r] * y[r];
    for (uint8_t r = k; r < NUM_POINTS; ++r) y[r] -= beta * h[r] * dotY;

    A[k][k] = alpha;
    for (uint8_t r = k + 1; r < NUM_POINTS; ++r) A[r][k] = 0.0f;
  }

  // --- 4단계: 후진 대입으로 R * coef = Q^T y 풀기 ---
  for (int k = cols - 1; k >= 0; --k) {
    float acc = y[k];
    for (uint8_t c = k + 1; c < cols; ++c) acc -= A[k][c] * coef[c];
    coef[k] = acc / A[k][k];
    if (!isfinite(coef[k])) return false;
  }
  return true;
}

// 현재 보정 결과로 ADC 값을 mm 거리로 바꾼다.
float volt_to_distance(unsigned int a_value)
{
  if (modelType == 2 && a_value == 0) return -1.0f;

  float x = (toX(a_value) - xMid) / xHalf;
  // Horner 방식: ((c_n * x + c_{n-1}) * x + ...) + c_0
  float d = coef[polyOrder];
  for (int i = polyOrder - 1; i >= 0; --i) {
    d = d * x + coef[i];
  }
  return d;
}

// 예제 2에 그대로 붙여 넣을 수 있는 함수 코드를 출력한다.
void printFunction()
{
  Serial.println();
  Serial.println(F("----- 아래 함수를 예제 2에 붙여 넣으세요 -----"));
  Serial.println(F("// 반환 단위: mm. 보정한 ADC 범위 안에서 사용하세요."));
  Serial.println(F("float volt_to_distance(unsigned int a_value)"));
  Serial.println(F("{"));
  if (modelType == 2) {
    Serial.println(F("  if (a_value == 0) return -1.0f; // 잘못된 ADC 값"));
  }

  Serial.print(F("  const float x = ("));
  if (modelType == 2) {
    Serial.print(F("1.0f / (float)a_value"));
  } else {
    Serial.print(F("(float)a_value"));
  }
  Serial.print(F(" - ("));
  Serial.print(xMid, 10);
  Serial.print(F("f)) / "));
  Serial.print(xHalf, 10);
  Serial.println(F("f;"));

  Serial.print(F("  return "));
  for (uint8_t i = 0; i < polyOrder; ++i) Serial.print('(');
  Serial.print(coef[polyOrder], 8);
  Serial.print('f');
  for (int i = polyOrder - 1; i >= 0; --i) {
    Serial.print(F(" * x + ("));
    Serial.print(coef[i], 8);
    Serial.print(F("f))"));
  }
  Serial.println(';');
  Serial.println('}');
  Serial.println(F("----- 함수 끝 -----"));
}

// 보정 위치별 오차와 R^2, RMSE를 표로 보여 준다.
void printFitSummary()
{
  const float meanMm = STEP_MM * (NUM_POINTS - 1) * 0.5f;  // 실제 거리 평균 = 150 mm
  float sse = 0.0f;  // 잔차 제곱합
  float sst = 0.0f;  // 전체 제곱합
  bool rising = true;
  bool falling = true;

  Serial.println();
  Serial.println(F("ADC\t실제(mm)\t계산(mm)\t오차(mm)"));
  for (uint8_t i = 0; i < NUM_POINTS; ++i) {
    float truth = STEP_MM * i;
    float model = volt_to_distance(calibAdc[i]);
    float diff = model - truth;
    sse += diff * diff;
    sst += (truth - meanMm) * (truth - meanMm);

    Serial.print(calibAdc[i]);
    Serial.print('\t');
    Serial.print(truth, 0);
    Serial.print(F("\t\t"));
    Serial.print(model, 2);
    Serial.print(F("\t\t"));
    Serial.println(diff, 2);

    if (i > 0) {
      if (calibAdc[i] <= calibAdc[i - 1]) rising = false;
      if (calibAdc[i] >= calibAdc[i - 1]) falling = false;
    }
  }

  Serial.print(F("결정계수 R^2 = "));
  Serial.println(1.0f - sse / sst, 6);
  Serial.print(F("보정 지점 RMSE = "));
  Serial.print(sqrtf(sse / NUM_POINTS), 3);
  Serial.println(F(" mm"));
  Serial.println(F("팁: 보정 지점 사이 위치(예: 55, 155, 255 mm)에서도 정확도를 확인해 보세요."));

  if (!rising && !falling) {
    Serial.println(F("경고: ADC 값이 한 방향으로 변하지 않습니다. 공의 위치를 확인하세요."));
  }
  if (polyOrder > 3) {
    Serial.println(F("경고: 차수가 3보다 크면 7개 점에 과적합될 수 있습니다."));
  }
}

// 설정 입력 -> 7개 위치 측정 -> 곡선 맞춤. 실패하면 처음부터 다시 진행한다.
void runCalibration()
{
  char buf[LINE_LEN];

  for (;;) {
    Serial.println();
    Serial.println(F("##### 20C10 : 적외선 거리 센서 곡선 맞춤 #####"));
    Serial.println(F("시리얼 모니터 설정: 1000000 baud, 줄 끝은 Newline 또는 Both NL & CR."));
    Serial.println(F("방식 1 = ADC에 대한 다항식"));
    Serial.println(F("방식 2 = 1/ADC에 대한 다항식"));
    modelType = askNumber(1, 2, 1);

    Serial.println(F("다항식 차수 (2 또는 3부터 시작해 보세요):"));
    polyOrder = askNumber(1, MAX_ORDER, 2);

    Serial.println(F("위치마다 읽을 샘플 수 (중앙값 필터):"));
    medianWindow = askNumber(1, BUF_SIZE, 100);

    for (uint8_t i = 0; i < NUM_POINTS; ++i) {
      for (;;) {
        Serial.print(F("공을 "));
        Serial.print(i * 5);
        Serial.print(F(" cm ("));
        Serial.print(i * 50);
        Serial.println(F(" mm) 위치에 놓고, 멈추면 아무것도 입력하지 말고 Enter를 누르세요."));

        if (!readSerialLine(buf, sizeof(buf)) || buf[0] != '\0') {
          Serial.println(F("빈 줄(Enter만)을 보내야 측정이 시작됩니다."));
          continue;
        }

        calibAdc[i] = ir_sensor_filtered(medianWindow, 0.5f, 0);
        if (modelType == 2 && calibAdc[i] == 0) {
          Serial.println(F("ADC 값이 0이라 역수를 구할 수 없습니다. 배선을 확인하고 다시 측정하세요."));
          continue;
        }

        Serial.print(F("저장된 ADC = "));
        Serial.println(calibAdc[i]);
        if (calibAdc[i] == 0 || calibAdc[i] == 1023) {
          Serial.println(F("경고: ADC 값이 한계에 도달했습니다. 배선이나 측정 범위를 확인하세요."));
        }
        break;
      }
    }

    if (solveLeastSquares()) {
      printFunction();
      printFitSummary();
      return;
    }
    Serial.println(F("곡선 맞춤 실패: 서로 다른 ADC 값이 부족하거나 데이터가 불안정합니다."));
    Serial.println(F("센서와 공 위치를 확인하거나 차수를 낮춰 보세요. 처음부터 다시 시작합니다."));
  }
}
