#include <USBSerial.h>
#include <usbd_cdc_if.h>
#include <math.h>
#include <string.h>
#include <stm32f1xx_hal.h>

extern "C" {
extern uint8_t _sidata, _sdata, _edata;
}

namespace {
constexpr uint8_t RX_LINE_CAPACITY = 80;
constexpr uint8_t STATUS_LED_PIN = PC13;
constexpr uint8_t MOTOR_CW_PIN = PB10;
constexpr uint8_t MOTOR_CCW_PIN = PB11;
constexpr uint8_t ENABLE_SENSORS_PIN = PB12;
constexpr uint8_t MANUAL_TRIGGER_PIN = PB13;
constexpr char IDENTIFY[] = "!IDENTIFY*40CF";
constexpr uint32_t CAL_PAGE_ADDRESS = 0x0801EC00UL;  // Two pages reserved for calibration.
constexpr uint32_t BM_PAGE_ADDRESS = 0x0801F400UL;  // Two pages reserved for BM.
constexpr uint32_t BM_PAGE_SIZE = 1024UL;
constexpr uint32_t SID_PAGE_ADDRESS = 0x0801FC00UL;  // Reserved final 1 KB of CB flash.
constexpr uint16_t CAL_RECORD_WORD_COUNT = 16;
constexpr uint16_t CAL_RECORD_SIZE = CAL_RECORD_WORD_COUNT * sizeof(uint16_t);
constexpr uint16_t CAL_RECORD_MAGIC = 0xCA1B;
constexpr uint16_t CAL_RECORD_VERSION = 2;
constexpr uint16_t CAL_RECORD_COMMIT = 0xC3A5;
constexpr uint16_t SID_SLOT_COUNT = 512;
constexpr uint16_t BM_RECORD_WORD_COUNT = 8;
constexpr uint16_t BM_RECORD_SIZE = BM_RECORD_WORD_COUNT * sizeof(uint16_t);
constexpr uint16_t BM_RECORD_MAGIC = 0xB04D;
constexpr uint16_t BM_RECORD_COMMIT = 0xA55A;
constexpr uint8_t BM_NOTIFICATION_QUEUE_CAPACITY = 2;
constexpr uint8_t BM_NOTIFICATION_FRAME_CAPACITY =
    sizeof("!BM,00000000,CLOSED*0000\n");
constexpr uint16_t HEAD_DEGREES = 10;
constexpr uint16_t TAIL_DEGREES = 40;
constexpr char CAL_PHASE_TEMPLATE[] =
    "!CAL-PHASE,00000000,00000000-0000-0000-0000-000000000000,HEAD,CW*0000\n";
constexpr char CAL_OPERATOR_TEMPLATE[] =
    "!CAL-OPERATOR,00000000,00000000-0000-0000-0000-000000000000,HEAD*0000\n";
constexpr char CAL_CONFIRM_TEMPLATE[] =
    "!CAL-CONFIRM,00000000,00000000-0000-0000-0000-000000000000,HEAD*0000";
constexpr char CAL_DONE_TEMPLATE[] =
    "!CAL-DONE,00000000,00000000-0000-0000-0000-000000000000,0000000000,0000000.000000*0000\n";
constexpr char CAL_CANCELED_TEMPLATE[] =
    "!CAL-CANCELED,00000000,00000000-0000-0000-0000-000000000000*0000\n";
constexpr char CAL_CANCEL_REQUEST_TEMPLATE[] =
    "!CAL-CANCEL,00000000,00000000-0000-0000-0000-000000000000*0000";
char sessionBody[] = "!SESSION,00000000";
char sid[] = "00000000";
constexpr char SYNC_PREFIX[] = "!SYNC,";
constexpr uint8_t CID_LENGTH = 36;
char rxLine[RX_LINE_CAPACITY];
uint8_t rxLength = 0;
bool rxOverflow = false;
bool sessionActive = false;
bool masterReady = false;
bool calibrationRequired = false;
bool calibrationOperationActive = false;
bool calibrationPhasePending = false;
bool calibrationOperatorPromptSent = false;
bool calibrationHeadReferenceValid = false;
bool calibrationTailPhase = false;
char calibrationCommandCid[CID_LENGTH] = {};
int32_t calibrationHeadCount = 0;
int32_t calibrationTailCount = 0;
bool calibrationScaleValid = false;
double calibrationPulsesPerDegree = 0.0;
int32_t pendingCalibrationPulseDelta = 0;
double pendingCalibrationPulsesPerDegree = 0.0;
bool calibrationReadyAfterDone = false;
bool calibrationManualSenseSeen = false;
bool calibrationHeadMotionObserved = false;
int32_t calibrationHeadLastCount = 0;
uint32_t calibrationHeadLastMotionAt = 0;
bool calibrationBondClosePending = false;
uint32_t calibrationBondLastMotionAt = 0;
bool calibrationBondClosurePendingReady = false;
bool calibrationReadyWaitsForBondNotifications = false;
bool calibrationCancelIsExplicit = false;
bool calibrationCanceledNotReadyPending = false;
constexpr uint8_t ENCODER_COUNTS_PER_STEP = 4;
constexpr uint32_t ENCODER_DETENT_STABLE_MS = 2;
constexpr uint32_t PUSH_DEBOUNCE_MS = 40;
constexpr uint32_t PREVIEW_MIN_INTERVAL_MS = 50;
constexpr uint32_t ENCODER_EXT_DIAG_INTERVAL_MS = 1000;
constexpr uint32_t ENCODER_EXT_AZIMUTH_MIN_INTERVAL_MS = 100;
constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 3000;
constexpr int32_t ENCODER_EXT_COUNTS_PER_TURN = 144;
constexpr float ENCODER_EXT_DEGREES_PER_COUNT = 2.5f;
constexpr float ENCODER_EXT_TEST_REFERENCE_DEGREES = 220.0f;
TIM_HandleTypeDef encoderTimer = {};
TIM_HandleTypeDef encoderExtTimer = {};
bool encoderEnabled = false;
bool encoderExtEnabled = false;
uint16_t encoderPreviousCount = 0;
uint16_t encoderExtPreviousCount = 0;
int32_t encoderCountAccumulator = 0;
int32_t encoderExtPositionCounts = 0;
volatile float azimuthExtMeasured = ENCODER_EXT_TEST_REFERENCE_DEGREES;
uint16_t encoderExtLastRoundedAzimuth = 220;
uint16_t encoderExtAzimuthToReport = 220;
bool encoderExtAzimuthReportPending = false;
bool manualAzimuthReportPending = false;
uint32_t lastEncoderExtAzimuthReportAt = 0;
uint8_t encoderDetentPhase = 0;
uint8_t encoderPhase = 0;
uint32_t encoderPhaseSince = 0;
uint32_t lastEncoderExtDiagnosticAt = 0;
uint16_t azimuthCurrent = 220;
uint16_t azimuthRequest = 220;
struct MemorySlot {
  uint16_t azimuth;
  bool valid;
};
MemorySlot memories[6] = {};
uint8_t pendingMemoryIndex = 0;
uint16_t pendingMemoryAzimuth = 0;
char pendingMemoryCid[CID_LENGTH];
char pendingMemoryResponse[sizeof("!NACK,00000000,00000000-0000-0000-0000-000000000000,MEMORY_EMPTY*0000\n")];
uint8_t memoryResponseStage = 0;
uint8_t memoryResponseOffset = 0;
uint8_t memoryResponseLength = 0;
bool recallMemoryPending = false;
bool recallMemoryFromTelegram = false;
bool recallAwaitingEncoder = false;
bool memoryRecallCancelPending = false;
bool memoryReadyAfterNack = false;
bool manualTriggerWasHigh = false;
bool heartbeatReceived = false;
uint32_t lastValidHeartbeatAt = 0;
char heartbeatResponse[] = "!HEARTBEAT,00000000*0000\n";
bool heartbeatResponsePending = false;
uint16_t lastPreviewAzimuth = 220;
uint32_t lastPreviewSentAt = 0;
bool previewPending = false;
bool previewSentOnce = false;
char pendingAzimuthResponse[
    sizeof(CAL_DONE_TEMPLATE) > sizeof(
        "!NACK,00000000,00000000-0000-0000-0000-000000000000,NOT_READY*0000\n")
        ? sizeof(CAL_DONE_TEMPLATE)
        : sizeof("!NACK,00000000,00000000-0000-0000-0000-000000000000,NOT_READY*0000\n")];
static_assert(sizeof(pendingAzimuthResponse) >= sizeof(CAL_PHASE_TEMPLATE),
    "TX buffer must hold a complete CAL-PHASE frame");
static_assert(sizeof(pendingAzimuthResponse) >= sizeof(CAL_OPERATOR_TEMPLATE),
    "TX buffer must hold a complete CAL-OPERATOR frame");
static_assert(sizeof(pendingAzimuthResponse) >= sizeof(CAL_PHASE_TEMPLATE) + 1,
    "TX buffer must hold a complete TAIL CAL-PHASE frame");
static_assert(sizeof(pendingAzimuthResponse) >= sizeof(CAL_DONE_TEMPLATE),
    "TX buffer must hold a complete CAL-DONE frame");
static_assert(sizeof(pendingAzimuthResponse) >= sizeof(CAL_CANCELED_TEMPLATE),
    "TX buffer must hold a complete CAL-CANCELED frame");
char pendingReadyResponse[] = "!READY,00000000,1,1,1*0000\n";
volatile uint16_t pendingAzimuthTarget = 220;
volatile char pendingAzimuthCid[5] = "0000";
bool azimuthCommandPending = false;
bool pendingAzimuthIsLocalPush = false;
bool pendingAzimuthIsMemoryRecall = false;
bool azimuthCompletionQueued = false;
bool azimuthTargetReached = false;
bool sensorStabilizationPending = false;
bool sensorStabilized = false;
uint32_t sensorStabilizationStartedAt = 0;
bool azimuthResponsePending = false;
bool readyResponsePending = false;
uint8_t azimuthResponseOffset = 0;
uint8_t readyResponseOffset = 0;
uint8_t azimuthResponseLength = sizeof(pendingAzimuthResponse) - 1;
bool pushRawPressed = false;
bool pushStablePressed = false;
bool pushPending = false;
bool localTargetSelected = false;
bool manualReadyPending = false;
uint16_t pushPendingAzimuth = 220;
uint32_t pushChangedAt = 0;
enum class MovementBondState : uint8_t { Unknown, Closed, Open };
MovementBondState movementBondState = MovementBondState::Unknown;
uint32_t movementBondSequence = 0;
int32_t movementBondStablePosition = 0;
char movementBondNotifications[BM_NOTIFICATION_QUEUE_CAPACITY]
    [BM_NOTIFICATION_FRAME_CAPACITY] = {};
uint8_t movementBondNotificationLengths[BM_NOTIFICATION_QUEUE_CAPACITY] = {};
uint8_t movementBondNotificationHead = 0;
uint8_t movementBondNotificationCount = 0;
bool movementBondClosingPending = false;
bool movementBondMotionStarted = false;
uint32_t movementBondLastExtMotionAt = 0;
bool manualMovementBondOwned = false;
bool manualMovementBondClosingPending = false;
uint32_t manualMovementBondLastMotionAt = 0;

void stopSensorStabilization();
void cancelCalibrationOperation();
bool heartbeatIsRecent();
bool isAzimuthAllowed(uint16_t azimuth);
bool isCurrentAzimuthMechanicallyAllowed();
void prepareCalibrationReadyResponse();
void beginMovementBondCloseWait();
void serviceMovementBondCloseWait();
void serviceManualMovementBondCloseWait();
void serviceCalibrationBondCloseWait();
void serviceCalibrationReadyAfterBondNotifications();

uint16_t calculateCrc16(const char* body, uint8_t bodyLength)
{
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < bodyLength; ++i) {
    crc ^= static_cast<uint16_t>(body[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
  }
  return crc;
}

struct MovementBondRecord {
  uint32_t sequence;
  MovementBondState state;
  int32_t stablePosition;
};

struct MovementBondScan {
  bool found;
  MovementBondRecord latest;
  uint8_t latestPage;
  bool invalidPage[2];
};

struct CalibrationRecord {
  uint32_t sequence;
  int32_t pulseDelta;
  double pulsesPerDegree;
  int32_t headCount;
};

struct CalibrationRecordScan {
  bool found;
  CalibrationRecord latest;
  uint8_t latestPage;
};

static_assert(CAL_PAGE_ADDRESS + 2 * BM_PAGE_SIZE == BM_PAGE_ADDRESS,
    "Calibration pages must not overlap BM pages");
static_assert(BM_PAGE_ADDRESS + 2 * BM_PAGE_SIZE == SID_PAGE_ADDRESS,
    "The two BM pages and SID page must be separate and contiguous");
static_assert(CAL_RECORD_SIZE <= BM_PAGE_SIZE && sizeof(double) == sizeof(uint64_t),
    "Calibration record must fit a page and store an eight-byte double");
static_assert(BM_RECORD_SIZE <= BM_PAGE_SIZE,
    "A BM record must fit in one Flash page");

uint16_t movementBondRecordCrc(const uint16_t* words)
{
  uint8_t bytes[BM_RECORD_SIZE - 2 * sizeof(uint16_t)];
  for (uint8_t word = 0; word < 6; ++word) {
    bytes[word * 2] = static_cast<uint8_t>(words[word]);
    bytes[word * 2 + 1] = static_cast<uint8_t>(words[word] >> 8);
  }
  return calculateCrc16(reinterpret_cast<const char*>(bytes), sizeof(bytes));
}

bool readMovementBondRecord(uint32_t address, MovementBondRecord& record)
{
  const volatile uint16_t* flash = reinterpret_cast<volatile uint16_t*>(address);
  uint16_t words[BM_RECORD_WORD_COUNT];
  for (uint8_t i = 0; i < BM_RECORD_WORD_COUNT; ++i) {
    words[i] = flash[i];
  }
  if (words[0] != BM_RECORD_MAGIC || words[7] != BM_RECORD_COMMIT
      || (words[3] != 0 && words[3] != 1)
      || words[6] != movementBondRecordCrc(words)) {
    return false;
  }
  record.sequence = static_cast<uint32_t>(words[1])
      | (static_cast<uint32_t>(words[2]) << 16);
  record.state = words[3] == 1
      ? MovementBondState::Open : MovementBondState::Closed;
  const uint32_t position = static_cast<uint32_t>(words[4])
      | (static_cast<uint32_t>(words[5]) << 16);
  record.stablePosition = static_cast<int32_t>(position);
  return true;
}

bool movementBondPageErased(uint32_t address)
{
  const volatile uint16_t* flash = reinterpret_cast<volatile uint16_t*>(address);
  for (uint16_t i = 0; i < BM_PAGE_SIZE / sizeof(uint16_t); ++i) {
    if (flash[i] != 0xFFFF) {
      return false;
    }
  }
  return true;
}

MovementBondScan scanMovementBondFlash()
{
  MovementBondScan scan = {};
  const uint32_t pageAddresses[2] = {BM_PAGE_ADDRESS, BM_PAGE_ADDRESS + BM_PAGE_SIZE};
  for (uint8_t page = 0; page < 2; ++page) {
    MovementBondRecord candidate = {};
    const bool recordValid = readMovementBondRecord(pageAddresses[page], candidate);
    const bool pageErased = movementBondPageErased(pageAddresses[page]);
    bool tailErased = true;
    const volatile uint16_t* tail = reinterpret_cast<volatile uint16_t*>(
        pageAddresses[page] + BM_RECORD_SIZE);
    for (uint16_t word = BM_RECORD_SIZE / sizeof(uint16_t);
        word < BM_PAGE_SIZE / sizeof(uint16_t); ++word) {
      if (tail[word - BM_RECORD_SIZE / sizeof(uint16_t)] != 0xFFFF) {
        tailErased = false;
        break;
      }
    }
    if (pageErased) {
      continue;
    }
    if (!recordValid || !tailErased) {
      scan.invalidPage[page] = true;
    } else {
      if (!scan.found
          || static_cast<int32_t>(candidate.sequence - scan.latest.sequence) > 0) {
        scan.found = true;
        scan.latest = candidate;
        scan.latestPage = page;
      }
    }
  }
  return scan;
}

bool eraseMovementBondPage(uint32_t address)
{
  FLASH_EraseInitTypeDef erase = {};
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = address;
  erase.NbPages = 1;
  uint32_t pageError = 0;
  return HAL_FLASHEx_Erase(&erase, &pageError) == HAL_OK
      && pageError == 0xFFFFFFFFUL;
}

bool writeMovementBondRecord(MovementBondState state, int32_t stablePosition)
{
  MovementBondScan scan = scanMovementBondFlash();
  const uint32_t pageAddresses[2] = {BM_PAGE_ADDRESS, BM_PAGE_ADDRESS + BM_PAGE_SIZE};
  const bool bothPagesInvalid = scan.invalidPage[0] && scan.invalidPage[1];
  uint8_t targetPage = scan.found ? 1 - scan.latestPage
      : (scan.invalidPage[0] ? 0 : (scan.invalidPage[1] ? 1 : 0));
  const bool eraseBothPages = bothPagesInvalid;
  const bool eraseTargetPage = eraseBothPages
      || !movementBondPageErased(pageAddresses[targetPage]);

  uint16_t words[BM_RECORD_WORD_COUNT] = {};
  const uint32_t sequence = scan.found ? scan.latest.sequence + 1U : 1U;
  const uint32_t position = static_cast<uint32_t>(stablePosition);
  words[0] = BM_RECORD_MAGIC;
  words[1] = static_cast<uint16_t>(sequence);
  words[2] = static_cast<uint16_t>(sequence >> 16);
  words[3] = state == MovementBondState::Open ? 1 : 0;
  words[4] = static_cast<uint16_t>(position);
  words[5] = static_cast<uint16_t>(position >> 16);
  words[6] = movementBondRecordCrc(words);
  words[7] = BM_RECORD_COMMIT;

  if (HAL_FLASH_Unlock() != HAL_OK) {
    return false;
  }
  bool success = true;
  if (eraseBothPages && !eraseMovementBondPage(pageAddresses[1 - targetPage])) {
    success = false;
  }
  if (success && eraseTargetPage && !eraseMovementBondPage(pageAddresses[targetPage])) {
    success = false;
  }
  const uint32_t address = pageAddresses[targetPage];
  for (uint8_t i = 0; success && i < BM_RECORD_WORD_COUNT - 1; ++i) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
        address + static_cast<uint32_t>(i) * sizeof(uint16_t), words[i]) != HAL_OK
        || *reinterpret_cast<volatile uint16_t*>(address
            + static_cast<uint32_t>(i) * sizeof(uint16_t)) != words[i]) {
      success = false;
    }
  }
  if (success && (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
      address + static_cast<uint32_t>(BM_RECORD_WORD_COUNT - 1) * sizeof(uint16_t),
      words[BM_RECORD_WORD_COUNT - 1]) != HAL_OK
      || *reinterpret_cast<volatile uint16_t*>(address
          + static_cast<uint32_t>(BM_RECORD_WORD_COUNT - 1) * sizeof(uint16_t))
          != words[BM_RECORD_WORD_COUNT - 1])) {
    success = false;
  }
  HAL_FLASH_Lock();

  MovementBondRecord verify = {};
  if (!success || !readMovementBondRecord(address, verify)
      || verify.sequence != sequence || verify.state != state
      || verify.stablePosition != stablePosition) {
    return false;
  }
  movementBondState = state;
  movementBondSequence = sequence;
  movementBondStablePosition = stablePosition;
  return true;
}

uint16_t calibrationRecordCrc(const uint16_t* words)
{
  uint8_t bytes[CAL_RECORD_SIZE - 2 * sizeof(uint16_t)];
  for (uint8_t word = 0; word < CAL_RECORD_WORD_COUNT - 2; ++word) {
    bytes[word * 2] = static_cast<uint8_t>(words[word]);
    bytes[word * 2 + 1] = static_cast<uint8_t>(words[word] >> 8);
  }
  return calculateCrc16(reinterpret_cast<const char*>(bytes), sizeof(bytes));
}

bool readCalibrationRecord(uint32_t address, CalibrationRecord& record)
{
  const volatile uint16_t* flash = reinterpret_cast<volatile uint16_t*>(address);
  uint16_t words[CAL_RECORD_WORD_COUNT];
  for (uint8_t i = 0; i < CAL_RECORD_WORD_COUNT; ++i) {
    words[i] = flash[i];
  }
  if (words[0] != CAL_RECORD_MAGIC || words[3] != CAL_RECORD_VERSION
      || words[4] != HEAD_DEGREES || words[5] != TAIL_DEGREES
      || words[14] != calibrationRecordCrc(words)
      || words[15] != CAL_RECORD_COMMIT) {
    return false;
  }
  const uint32_t rawDelta = static_cast<uint32_t>(words[6])
      | (static_cast<uint32_t>(words[7]) << 16);
  if (rawDelta == 0 || rawDelta > INT32_MAX) {
    return false;
  }
  uint64_t factorBits = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    factorBits |= static_cast<uint64_t>(words[8 + i]) << (16 * i);
  }
  double factor = 0.0;
  memcpy(&factor, &factorBits, sizeof(factor));
  const double expected = static_cast<double>(rawDelta) / 330.0;
  if (!(factor > 0.0) || factor != expected) {
    return false;
  }
  const int32_t headCount = static_cast<int32_t>(static_cast<uint32_t>(words[12])
      | (static_cast<uint32_t>(words[13]) << 16));
  const int64_t tailCount = static_cast<int64_t>(headCount) - rawDelta;
  if (tailCount < INT32_MIN || tailCount > INT32_MAX) {
    return false;
  }
  record.sequence = static_cast<uint32_t>(words[1])
      | (static_cast<uint32_t>(words[2]) << 16);
  record.pulseDelta = static_cast<int32_t>(rawDelta);
  record.pulsesPerDegree = factor;
  record.headCount = headCount;
  return true;
}

bool calibrationRecordTailErased(uint32_t address)
{
  const volatile uint16_t* flash = reinterpret_cast<volatile uint16_t*>(address);
  for (uint16_t i = CAL_RECORD_WORD_COUNT; i < BM_PAGE_SIZE / sizeof(uint16_t); ++i) {
    if (flash[i] != 0xFFFF) {
      return false;
    }
  }
  return true;
}

CalibrationRecordScan scanCalibrationFlash()
{
  CalibrationRecordScan scan = {};
  for (uint8_t page = 0; page < 2; ++page) {
    const uint32_t address = CAL_PAGE_ADDRESS + static_cast<uint32_t>(page) * BM_PAGE_SIZE;
    CalibrationRecord candidate = {};
    if (readCalibrationRecord(address, candidate)
        && calibrationRecordTailErased(address)
        && (!scan.found
            || static_cast<int32_t>(candidate.sequence - scan.latest.sequence) > 0)) {
      scan.found = true;
      scan.latest = candidate;
      scan.latestPage = page;
    }
  }
  return scan;
}

bool writeCalibrationRecord(int32_t pulseDelta, double pulsesPerDegree,
    int32_t headCount)
{
  if (pulseDelta <= 0
      || pulsesPerDegree != static_cast<double>(pulseDelta) / 330.0
      || static_cast<int64_t>(headCount) - pulseDelta < INT32_MIN) {
    return false;
  }
  const CalibrationRecordScan scan = scanCalibrationFlash();
  const uint8_t targetPage = scan.found ? 1 - scan.latestPage : 0;
  const uint32_t address = CAL_PAGE_ADDRESS
      + static_cast<uint32_t>(targetPage) * BM_PAGE_SIZE;
  const uint32_t sequence = scan.found ? scan.latest.sequence + 1U : 1U;
  uint16_t words[CAL_RECORD_WORD_COUNT] = {};
  words[0] = CAL_RECORD_MAGIC;
  words[1] = static_cast<uint16_t>(sequence);
  words[2] = static_cast<uint16_t>(sequence >> 16);
  words[3] = CAL_RECORD_VERSION;
  words[4] = HEAD_DEGREES;
  words[5] = TAIL_DEGREES;
  words[6] = static_cast<uint16_t>(pulseDelta);
  words[7] = static_cast<uint16_t>(static_cast<uint32_t>(pulseDelta) >> 16);
  uint64_t factorBits = 0;
  memcpy(&factorBits, &pulsesPerDegree, sizeof(factorBits));
  for (uint8_t i = 0; i < 4; ++i) {
    words[8 + i] = static_cast<uint16_t>(factorBits >> (16 * i));
  }
  const uint32_t rawHeadCount = static_cast<uint32_t>(headCount);
  words[12] = static_cast<uint16_t>(rawHeadCount);
  words[13] = static_cast<uint16_t>(rawHeadCount >> 16);
  words[14] = calibrationRecordCrc(words);
  words[15] = CAL_RECORD_COMMIT;

  if (HAL_FLASH_Unlock() != HAL_OK) {
    return false;
  }
  bool success = movementBondPageErased(address) || eraseMovementBondPage(address);
  for (uint8_t i = 0; success && i < CAL_RECORD_WORD_COUNT; ++i) {
    const uint32_t wordAddress = address + static_cast<uint32_t>(i) * sizeof(uint16_t);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, wordAddress, words[i]) != HAL_OK
        || *reinterpret_cast<volatile uint16_t*>(wordAddress) != words[i]) {
      success = false;
    }
  }
  HAL_FLASH_Lock();

  CalibrationRecord verify = {};
  return success && readCalibrationRecord(address, verify)
      && calibrationRecordTailErased(address)
      && verify.sequence == sequence && verify.pulseDelta == pulseDelta
      && verify.pulsesPerDegree == pulsesPerDegree
      && verify.headCount == headCount;
}

void restoreCalibrationAtBoot()
{
  const CalibrationRecordScan scan = scanCalibrationFlash();
  calibrationScaleValid = scan.found;
  calibrationPulsesPerDegree = scan.found ? scan.latest.pulsesPerDegree : 0.0;
  calibrationHeadReferenceValid = false;
  calibrationRequired = true;
  masterReady = false;
  if (!encoderExtEnabled || !scan.found
      || movementBondState != MovementBondState::Closed) {
    return;
  }
  const int64_t relativeCounts = static_cast<int64_t>(movementBondStablePosition)
      - scan.latest.headCount;
  if (relativeCounts > 0 || relativeCounts < -static_cast<int64_t>(scan.latest.pulseDelta)) {
    return;
  }
  double measured = HEAD_DEGREES
      + static_cast<double>(relativeCounts) / scan.latest.pulsesPerDegree;
  if (measured < 0.0) {
    measured += 360.0;
  }
  uint16_t rounded = static_cast<uint16_t>(measured + 0.5);
  if (rounded == 360) {
    rounded = 0;
  }
  if (!isAzimuthAllowed(rounded)) {
    return;
  }
  calibrationHeadCount = scan.latest.headCount;
  calibrationTailCount = static_cast<int32_t>(
      static_cast<int64_t>(scan.latest.headCount) - scan.latest.pulseDelta);
  encoderExtPositionCounts = movementBondStablePosition;
  calibrationHeadReferenceValid = true;
  azimuthExtMeasured = static_cast<float>(measured);
  encoderExtLastRoundedAzimuth = rounded;
  encoderExtAzimuthToReport = rounded;
  azimuthCurrent = rounded;
  azimuthRequest = rounded;
  lastPreviewAzimuth = rounded;
  pushPendingAzimuth = rounded;
  calibrationRequired = false;
}

void queueMovementBondNotification(MovementBondState state)
{
  if (!sessionActive || !heartbeatIsRecent()
      || movementBondNotificationCount >= BM_NOTIFICATION_QUEUE_CAPACITY) {
    return;
  }
  const uint8_t tail = (movementBondNotificationHead + movementBondNotificationCount)
      % BM_NOTIFICATION_QUEUE_CAPACITY;
  char* frame = movementBondNotifications[tail];
  constexpr char PREFIX[] = "!BM,";
  const char* label = state == MovementBondState::Open ? "OPEN" : "CLOSED";
  const uint8_t labelLength = state == MovementBondState::Open
      ? sizeof("OPEN") - 1 : sizeof("CLOSED") - 1;
  memcpy(frame, PREFIX, sizeof(PREFIX) - 1);
  memcpy(frame + sizeof(PREFIX) - 1, sid, sizeof(sid) - 1);
  uint8_t bodyLength = sizeof(PREFIX) - 1 + sizeof(sid) - 1;
  frame[bodyLength++] = ',';
  memcpy(frame + bodyLength, label, labelLength);
  bodyLength += labelLength;
  frame[bodyLength] = '*';
  const uint16_t crc = calculateCrc16(frame, bodyLength);
  constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
  for (uint8_t i = 0; i < 4; ++i) {
    frame[bodyLength + 1 + i] = HEX_DIGITS[(crc >> (12 - 4 * i)) & 0x0F];
  }
  frame[bodyLength + 5] = '\n';
  movementBondNotificationLengths[tail] = bodyLength + 6;
  ++movementBondNotificationCount;
}

// SARC event frames use !BM,SID,OPEN*CRC4 and !BM,SID,CLOSED*CRC4.
bool openMovementBond()
{
  if (movementBondState == MovementBondState::Open) {
    return true;
  }
  if (!writeMovementBondRecord(MovementBondState::Open,
      movementBondStablePosition)) {
    return false;
  }
  queueMovementBondNotification(MovementBondState::Open);
  return true;
}

bool closeMovementBond(bool movementCompletedSuccessfully, int32_t stablePosition)
{
  if (!movementCompletedSuccessfully || movementBondState != MovementBondState::Open) {
    return false;
  }
  if (!writeMovementBondRecord(MovementBondState::Closed, stablePosition)) {
    return false;
  }
  queueMovementBondNotification(MovementBondState::Closed);
  return true;
}

void restoreMovementBondAtBoot()
{
  const MovementBondScan scan = scanMovementBondFlash();
  const bool flashCorrupt = scan.invalidPage[0] || scan.invalidPage[1];
  if (scan.found && (!flashCorrupt
      || scan.latest.state == MovementBondState::Open)) {
    movementBondSequence = scan.latest.sequence;
    movementBondState = scan.latest.state;
    movementBondStablePosition = scan.latest.stablePosition;
  } else {
    movementBondState = MovementBondState::Unknown;
    movementBondStablePosition = 0;
    if (!scan.found) {
      movementBondSequence = 0;
    }
  }

  calibrationRequired = true;
  masterReady = false;
}

void serviceMovementBondNotifications()
{
  if (!heartbeatIsRecent()) {
    movementBondNotificationHead = 0;
    movementBondNotificationCount = 0;
    return;
  }
  if (movementBondNotificationCount == 0) {
    return;
  }
  if (azimuthResponsePending || readyResponsePending || memoryResponseStage
      || heartbeatResponsePending) {
    return;
  }
  const uint8_t length =
      movementBondNotificationLengths[movementBondNotificationHead];
  if (SerialUSB.availableForWrite() < length
      || SerialUSB.write(movementBondNotifications[movementBondNotificationHead],
          length) != length) {
    return;
  }
  movementBondNotificationHead = (movementBondNotificationHead + 1)
      % BM_NOTIFICATION_QUEUE_CAPACITY;
  --movementBondNotificationCount;
}

void serviceCalibrationBondCloseWait()
{
  if (!calibrationBondClosePending) {
    return;
  }
  if (!calibrationOperationActive || !heartbeatIsRecent() || !encoderExtEnabled
      || digitalRead(MOTOR_CW_PIN) == HIGH || digitalRead(MOTOR_CCW_PIN) == HIGH
      || !calibrationScaleValid || calibrationPulsesPerDegree <= 0.0
      || pendingCalibrationPulseDelta <= 0
      || movementBondState != MovementBondState::Open
      || !isCurrentAzimuthMechanicallyAllowed()
      || !isAzimuthAllowed(encoderExtLastRoundedAzimuth)) {
    cancelCalibrationOperation();
    return;
  }
  if (digitalRead(MANUAL_TRIGGER_PIN) == HIGH) {
    calibrationBondLastMotionAt = millis();
    return;
  }
  if (millis() - calibrationBondLastMotionAt < 2000) {
    return;
  }
  if (!writeCalibrationRecord(pendingCalibrationPulseDelta,
      pendingCalibrationPulsesPerDegree, calibrationHeadCount)) {
    cancelCalibrationOperation();
    return;
  }
  if (!heartbeatIsRecent()
      || digitalRead(MOTOR_CW_PIN) == HIGH || digitalRead(MOTOR_CCW_PIN) == HIGH
      || static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(&encoderExtTimer))
          != encoderExtPreviousCount) {
    cancelCalibrationOperation();
    return;
  }
  if (digitalRead(MANUAL_TRIGGER_PIN) == HIGH) {
    calibrationBondLastMotionAt = millis();
    return;
  }
  if (!closeMovementBond(true, encoderExtPositionCounts)) {
    cancelCalibrationOperation();
    return;
  }
  calibrationBondClosePending = false;
  calibrationBondClosurePendingReady = true;
  calibrationReadyWaitsForBondNotifications = true;
  calibrationOperationActive = false;
  calibrationOperatorPromptSent = false;
  calibrationTailPhase = false;
  calibrationRequired = true;
  masterReady = false;
  digitalWrite(ENABLE_SENSORS_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void serviceCalibrationReadyAfterBondNotifications()
{
  if (!calibrationReadyWaitsForBondNotifications) {
    return;
  }
  if (!calibrationBondClosurePendingReady
      || movementBondState != MovementBondState::Closed
      || !heartbeatIsRecent() || !encoderExtEnabled
      || digitalRead(MANUAL_TRIGGER_PIN) == HIGH
      || digitalRead(MOTOR_CW_PIN) == HIGH || digitalRead(MOTOR_CCW_PIN) == HIGH) {
    cancelCalibrationOperation();
    return;
  }
  if (encoderExtPositionCounts != movementBondStablePosition
      || static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(&encoderExtTimer))
          != encoderExtPreviousCount) {
    cancelCalibrationOperation();
    return;
  }
  if (movementBondNotificationCount != 0) {
    return;
  }
  calibrationReadyWaitsForBondNotifications = false;
  prepareCalibrationReadyResponse();
}

void prepareCalibrationCanceledResponse(const char* commandCid)
{
  memcpy(pendingAzimuthResponse, CAL_CANCELED_TEMPLATE,
      sizeof(CAL_CANCELED_TEMPLATE));
  memcpy(pendingAzimuthResponse + sizeof("!CAL-CANCELED,") - 1,
      sid, sizeof(sid) - 1);
  memcpy(pendingAzimuthResponse + sizeof("!CAL-CANCELED,00000000,") - 1,
      commandCid, CID_LENGTH);
  const uint8_t bodyLength = strchr(pendingAzimuthResponse, '*') - pendingAzimuthResponse;
  const uint16_t crc = calculateCrc16(pendingAzimuthResponse, bodyLength);
  constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
  for (uint8_t i = 0; i < 4; ++i) {
    pendingAzimuthResponse[bodyLength + 1 + i] = HEX_DIGITS[(crc >> (12 - 4 * i)) & 0x0F];
  }
  azimuthResponseLength = bodyLength + 6;
  azimuthResponseOffset = 0;
  azimuthResponsePending = true;
}

void queueCalibrationReply(const char* commandCid, bool accepted)
{
  char response[] =
      "!NACK,00000000,00000000-0000-0000-0000-000000000000,NOT_READY*0000\n";
  if (accepted) {
    strcpy(response, "!ACK,00000000,00000000-0000-0000-0000-000000000000*0000\n");
  }
  char* sidField = strchr(response, ',') + 1;
  memcpy(sidField, sid, sizeof(sid) - 1);
  char* cidField = strchr(sidField, ',') + 1;
  memcpy(cidField, commandCid, CID_LENGTH);
  const uint8_t bodyLength = strchr(response, '*') - response;
  const uint16_t crc = calculateCrc16(response, bodyLength);
  constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
  for (uint8_t i = 0; i < 4; ++i) {
    response[bodyLength + 1 + i] = HEX_DIGITS[(crc >> (12 - 4 * i)) & 0x0F];
  }
  const uint8_t responseLength = bodyLength + 6;
  memcpy(pendingAzimuthResponse, response, responseLength);
  azimuthResponseLength = responseLength;
  azimuthResponseOffset = 0;
  azimuthResponsePending = true;
  if (accepted) {
    calibrationPhasePending = true;
  }
}

void prepareCalibrationPhaseResponse()
{
  memcpy(pendingAzimuthResponse, CAL_PHASE_TEMPLATE, sizeof(CAL_PHASE_TEMPLATE));
  if (calibrationTailPhase) {
    constexpr uint8_t phaseOffset = sizeof(
        "!CAL-PHASE,00000000,00000000-0000-0000-0000-000000000000,") - 1;
    constexpr uint8_t directionOffset = sizeof(
        "!CAL-PHASE,00000000,00000000-0000-0000-0000-000000000000,HEAD,") - 1;
    memmove(pendingAzimuthResponse + directionOffset + 1,
        pendingAzimuthResponse + directionOffset,
        sizeof(CAL_PHASE_TEMPLATE) - directionOffset);
    memcpy(pendingAzimuthResponse + phaseOffset, "TAIL", sizeof("TAIL") - 1);
  }
  memcpy(pendingAzimuthResponse + sizeof("!CAL-PHASE,") - 1, sid, sizeof(sid) - 1);
  memcpy(pendingAzimuthResponse + sizeof("!CAL-PHASE,00000000,") - 1,
      calibrationCommandCid, CID_LENGTH);
  const uint8_t bodyLength = strchr(pendingAzimuthResponse, '*') - pendingAzimuthResponse;
  const uint16_t crc = calculateCrc16(pendingAzimuthResponse, bodyLength);
  constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
  for (uint8_t i = 0; i < 4; ++i) {
    pendingAzimuthResponse[bodyLength + 1 + i] = HEX_DIGITS[(crc >> (12 - 4 * i)) & 0x0F];
  }
  azimuthResponseLength = bodyLength + 6;
  azimuthResponseOffset = 0;
  azimuthResponsePending = true;
}

void prepareCalibrationOperatorResponse()
{
  memcpy(pendingAzimuthResponse, CAL_OPERATOR_TEMPLATE, sizeof(CAL_OPERATOR_TEMPLATE));
  if (calibrationTailPhase) {
    constexpr uint8_t phaseOffset = sizeof(
        "!CAL-OPERATOR,00000000,00000000-0000-0000-0000-000000000000,") - 1;
    memcpy(pendingAzimuthResponse + phaseOffset, "TAIL", sizeof("TAIL") - 1);
  }
  memcpy(pendingAzimuthResponse + sizeof("!CAL-OPERATOR,") - 1,
      sid, sizeof(sid) - 1);
  memcpy(pendingAzimuthResponse + sizeof("!CAL-OPERATOR,00000000,") - 1,
      calibrationCommandCid, CID_LENGTH);
  const uint8_t bodyLength = strchr(pendingAzimuthResponse, '*') - pendingAzimuthResponse;
  const uint16_t crc = calculateCrc16(pendingAzimuthResponse, bodyLength);
  constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
  for (uint8_t i = 0; i < 4; ++i) {
    pendingAzimuthResponse[bodyLength + 1 + i] = HEX_DIGITS[(crc >> (12 - 4 * i)) & 0x0F];
  }
  azimuthResponseLength = bodyLength + 6;
  azimuthResponseOffset = 0;
  azimuthResponsePending = true;
}

void prepareCalibrationDoneResponse(int32_t pulseDelta, double pulsesPerDegree)
{
  memcpy(pendingAzimuthResponse, CAL_DONE_TEMPLATE, sizeof(CAL_DONE_TEMPLATE));
  memcpy(pendingAzimuthResponse + sizeof("!CAL-DONE,") - 1,
      sid, sizeof(sid) - 1);
  constexpr uint8_t cidOffset = sizeof("!CAL-DONE,00000000,") - 1;
  memcpy(pendingAzimuthResponse + cidOffset, calibrationCommandCid, CID_LENGTH);
  constexpr uint8_t pulseDeltaOffset = sizeof(
      "!CAL-DONE,00000000,00000000-0000-0000-0000-000000000000,") - 1;
  uint32_t pulseDigits = static_cast<uint32_t>(pulseDelta);
  for (int8_t digit = 9; digit >= 0; --digit) {
    pendingAzimuthResponse[pulseDeltaOffset + digit] = '0' + pulseDigits % 10;
    pulseDigits /= 10;
  }
  constexpr uint8_t pulsesPerDegreeOffset = pulseDeltaOffset + 11;
  const uint64_t scaledPulsesPerDegree =
      (static_cast<uint64_t>(pulseDelta) * 1000000ULL + 165ULL) / 330ULL;
  uint32_t wholePulsesPerDegree = scaledPulsesPerDegree / 1000000ULL;
  for (int8_t digit = 6; digit >= 0; --digit) {
    pendingAzimuthResponse[pulsesPerDegreeOffset + digit] =
        '0' + wholePulsesPerDegree % 10;
    wholePulsesPerDegree /= 10;
  }
  pendingAzimuthResponse[pulsesPerDegreeOffset + 7] = '.';
  uint32_t fractionalPulsesPerDegree = scaledPulsesPerDegree % 1000000ULL;
  for (int8_t digit = 5; digit >= 0; --digit) {
    pendingAzimuthResponse[pulsesPerDegreeOffset + 8 + digit] =
        '0' + fractionalPulsesPerDegree % 10;
    fractionalPulsesPerDegree /= 10;
  }
  const uint8_t bodyLength = strchr(pendingAzimuthResponse, '*') - pendingAzimuthResponse;
  const uint16_t crc = calculateCrc16(pendingAzimuthResponse, bodyLength);
  constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
  for (uint8_t i = 0; i < 4; ++i) {
    pendingAzimuthResponse[bodyLength + 1 + i] = HEX_DIGITS[(crc >> (12 - 4 * i)) & 0x0F];
  }
  azimuthResponseLength = bodyLength + 6;
  azimuthResponseOffset = 0;
  azimuthResponsePending = true;
  pendingCalibrationPulseDelta = pulseDelta;
  pendingCalibrationPulsesPerDegree = pulsesPerDegree;
}

void prepareCalibrationReadyResponse()
{
  char ready[] = "!READY,00000000,1,1,1*0000\n";
  memcpy(ready + sizeof("!READY,") - 1, sid, sizeof(sid) - 1);
  const uint8_t bodyLength = sizeof("!READY,00000000,1,1,1") - 1;
  const uint16_t crc = calculateCrc16(ready, bodyLength);
  constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
  for (uint8_t i = 0; i < 4; ++i) {
    ready[bodyLength + 1 + i] = HEX_DIGITS[(crc >> (12 - 4 * i)) & 0x0F];
  }
  memcpy(pendingReadyResponse, ready, sizeof(ready));
  readyResponseOffset = 0;
  readyResponsePending = true;
  calibrationReadyAfterDone = true;
}

void prepareCalibrationNotReadyResponse()
{
  char ready[] = "!READY,00000000,0,1,0*0000\n";
  memcpy(ready + sizeof("!READY,") - 1, sid, sizeof(sid) - 1);
  const uint8_t bodyLength = sizeof("!READY,00000000,0,1,0") - 1;
  const uint16_t crc = calculateCrc16(ready, bodyLength);
  constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
  for (uint8_t i = 0; i < 4; ++i) {
    ready[bodyLength + 1 + i] = HEX_DIGITS[(crc >> (12 - 4 * i)) & 0x0F];
  }
  memcpy(pendingReadyResponse, ready, sizeof(ready));
  readyResponseOffset = 0;
  readyResponsePending = true;
  calibrationReadyAfterDone = false;
}

void cancelPendingCalibrationPhase()
{
  calibrationPhasePending = false;
  if (memcmp(pendingAzimuthResponse, "!CAL-PHASE,", sizeof("!CAL-PHASE,") - 1) == 0) {
    azimuthResponsePending = false;
    azimuthResponseOffset = 0;
  }
}

bool heartbeatIsRecent()
{
  return sessionActive && CDC_connected() && heartbeatReceived
      && millis() - lastValidHeartbeatAt < HEARTBEAT_TIMEOUT_MS;
}

void refreshReadyFrameAfterCalibrationRequired(char* response)
{
  response[sizeof("!READY,00000000,") - 1] = '0';
  response[sizeof("!READY,00000000,1,1,") - 1] = '0';
  const uint8_t bodyLength = strchr(response, '*') - response;
  const uint16_t crc = calculateCrc16(response, bodyLength);
  constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
  for (uint8_t i = 0; i < 4; ++i) {
    response[bodyLength + 1 + i] = HEX_DIGITS[(crc >> (12 - 4 * i)) & 0x0F];
  }
}

void requireCalibration()
{
  if (calibrationRequired) {
    return;
  }
  calibrationRequired = true;
  masterReady = false;
  digitalWrite(STATUS_LED_PIN, HIGH);
  if (readyResponsePending) {
    refreshReadyFrameAfterCalibrationRequired(pendingReadyResponse);
    readyResponseOffset = 0;
  }
  if (memoryResponseStage == 4) {
    refreshReadyFrameAfterCalibrationRequired(pendingMemoryResponse);
    memoryResponseOffset = 0;
  }
}

bool isAzimuthAllowed(uint16_t azimuth)
{
  return azimuth <= 359 && !(azimuth > HEAD_DEGREES && azimuth < TAIL_DEGREES);
}

double currentAzimuthUnrounded()
{
  if (!calibrationHeadReferenceValid || !calibrationScaleValid
      || calibrationPulsesPerDegree <= 0.0) {
    return azimuthExtMeasured;
  }
  const int64_t relativeCounts = static_cast<int64_t>(encoderExtPositionCounts)
      - static_cast<int64_t>(calibrationHeadCount);
  double actualAngle = HEAD_DEGREES
      + static_cast<double>(relativeCounts) / calibrationPulsesPerDegree;
  actualAngle = fmod(actualAngle, 360.0);
  if (actualAngle < 0.0) {
    actualAngle += 360.0;
  }
  return actualAngle;
}

bool isCurrentAzimuthMechanicallyAllowed()
{
  const double actualAngle = currentAzimuthUnrounded();
  return !(actualAngle > HEAD_DEGREES && actualAngle < TAIL_DEGREES);
}

bool isAzimuthTargetReached(uint16_t target)
{
  const double actualAngle = currentAzimuthUnrounded();
  if (actualAngle > HEAD_DEGREES && actualAngle < TAIL_DEGREES) {
    return false;
  }
  if (azimuthTargetReached) {
    return true;
  }

  const double actualArcPosition = actualAngle >= TAIL_DEGREES
      ? actualAngle - TAIL_DEGREES : actualAngle + 360.0 - TAIL_DEGREES;
  const double targetArcPosition = target >= TAIL_DEGREES
      ? target - TAIL_DEGREES : target + 360.0 - TAIL_DEGREES;
  const bool movingCw = digitalRead(MOTOR_CW_PIN) == HIGH;
  const bool movingCcw = digitalRead(MOTOR_CCW_PIN) == HIGH;
  if (!movingCw && !movingCcw) {
    constexpr double NUMERIC_MARGIN_DEGREES = 0.000001;
    azimuthTargetReached = fabs(actualArcPosition - targetArcPosition)
        <= 5.0 + NUMERIC_MARGIN_DEGREES;
  } else if ((movingCw && actualArcPosition >= targetArcPosition)
      || (movingCcw && actualArcPosition <= targetArcPosition)) {
    digitalWrite(MOTOR_CW_PIN, LOW);
    digitalWrite(MOTOR_CCW_PIN, LOW);
    azimuthTargetReached = true;
  }
  return azimuthTargetReached;
}

bool allocateSid()
{
  const uint32_t sketchEnd = reinterpret_cast<uint32_t>(&_sidata)
      + reinterpret_cast<uint32_t>(&_edata) - reinterpret_cast<uint32_t>(&_sdata);
  if (sketchEnd > SID_PAGE_ADDRESS) {
    return false;
  }
  const volatile uint16_t* slots = reinterpret_cast<volatile uint16_t*>(SID_PAGE_ADDRESS);
  uint16_t slot = 0;
  while (slot < SID_SLOT_COUNT && slots[slot] != 0xFFFF) {
    ++slot;
  }
  if (slot == SID_SLOT_COUNT || HAL_FLASH_Unlock() != HAL_OK) {
    return false;
  }
  const HAL_StatusTypeDef written = HAL_FLASH_Program(
      FLASH_TYPEPROGRAM_HALFWORD, SID_PAGE_ADDRESS + slot * 2U, 0U);
  HAL_FLASH_Lock();
  if (written != HAL_OK || slots[slot] != 0U) {
    return false;
  }
  const uint32_t value = 0xA1B20000UL | (static_cast<uint32_t>(slot) + 1U);
  constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
  for (uint8_t i = 0; i < 8; ++i) {
    sid[i] = HEX_DIGITS[(value >> (28 - 4 * i)) & 0x0F];
  }
  memcpy(sessionBody + sizeof("!SESSION,") - 1, sid, 8);
  return true;
}

void prepareMemoryResponse()
{
  const char* frame;
  uint8_t cidOffset = 0;
  if (memoryResponseStage == 1) {
    frame = "!NACK,00000000,00000000-0000-0000-0000-000000000000,NOT_READY*0000\n";
    cidOffset = sizeof("!NACK,00000000,") - 1;
  } else if (memoryResponseStage == 5) {
    frame = "!NACK,00000000,00000000-0000-0000-0000-000000000000,MEMORY_EMPTY*0000\n";
    cidOffset = sizeof("!NACK,00000000,") - 1;
  } else if (memoryResponseStage == 2) {
    frame = "!ACK,00000000,00000000-0000-0000-0000-000000000000*0000\n";
    cidOffset = sizeof("!ACK,00000000,") - 1;
  } else if (memoryResponseStage == 3) {
    frame = "!DONE,00000000,00000000-0000-0000-0000-000000000000,M1,000*0000\n";
    cidOffset = sizeof("!DONE,00000000,") - 1;
  } else {
    frame = "!READY,00000000,1,1,1,000*0000\n";
  }
  strcpy(pendingMemoryResponse, frame);
  memcpy(strchr(pendingMemoryResponse, ',') + 1, sid, 8);
  if (cidOffset) {
    memcpy(pendingMemoryResponse + cidOffset, pendingMemoryCid, CID_LENGTH);
  }
  if (memoryResponseStage == 3 || memoryResponseStage == 4) {
    const uint8_t azimuthOffset = memoryResponseStage == 3
        ? sizeof("!DONE,00000000,00000000-0000-0000-0000-000000000000,M1,") - 1
        : sizeof("!READY,00000000,1,1,1,") - 1;
    const uint16_t azimuth = memoryResponseStage == 3
        ? memories[pendingMemoryIndex].azimuth : azimuthCurrent;
    pendingMemoryResponse[azimuthOffset] = '0' + azimuth / 100;
    pendingMemoryResponse[azimuthOffset + 1] = '0' + (azimuth / 10) % 10;
    pendingMemoryResponse[azimuthOffset + 2] = '0' + azimuth % 10;
    if (memoryResponseStage == 3) {
      const uint8_t memoryDigitOffset = sizeof(
          "!DONE,00000000,00000000-0000-0000-0000-000000000000,M") - 1;
      pendingMemoryResponse[memoryDigitOffset] = '1' + pendingMemoryIndex;
    } else if (calibrationRequired) {
      pendingMemoryResponse[sizeof("!READY,00000000,") - 1] = '0';
      pendingMemoryResponse[sizeof("!READY,00000000,1,1,") - 1] = '0';
    }
  }
  const uint8_t bodyLength = strchr(pendingMemoryResponse, '*') - pendingMemoryResponse;
  const uint16_t crc = calculateCrc16(pendingMemoryResponse, bodyLength);
  constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
  for (uint8_t i = 0; i < 4; ++i) {
    pendingMemoryResponse[bodyLength + 1 + i] = HEX_DIGITS[(crc >> (12 - 4 * i)) & 0x0F];
  }
  memoryResponseLength = bodyLength + 6;
  memoryResponseOffset = 0;
}

void serviceMemoryResponse()
{
  if (!memoryResponseStage) {
    return;
  }
  if (memoryResponseStage == 3 && recallAwaitingEncoder && memoryRecallCancelPending) {
    recallAwaitingEncoder = false;
    memoryResponseStage = 1;
    memoryReadyAfterNack = false;
    prepareMemoryResponse();
  }
  if (memoryResponseStage == 3 && recallAwaitingEncoder) {
    if (!encoderExtEnabled || encoderExtLastRoundedAzimuth != pendingAzimuthTarget) {
      return;
    }
    recallAwaitingEncoder = false;
  }
  const int available = SerialUSB.availableForWrite();
  if (available > 0) {
    const uint8_t remaining = memoryResponseLength - memoryResponseOffset;
    const uint8_t chunk = available < remaining ? available : remaining;
    memoryResponseOffset += static_cast<uint8_t>(SerialUSB.write(
        reinterpret_cast<const uint8_t*>(pendingMemoryResponse) + memoryResponseOffset, chunk));
  }
  if (memoryResponseOffset != memoryResponseLength) {
    return;
  }
  if (memoryResponseStage == 1 && memoryRecallCancelPending) {
    pendingAzimuthIsMemoryRecall = false;
    memoryRecallCancelPending = false;
    recallMemoryPending = false;
    recallMemoryFromTelegram = false;
    recallAwaitingEncoder = false;
    azimuthCommandPending = false;
    masterReady = false;
    digitalWrite(STATUS_LED_PIN, HIGH);
    memoryResponseStage = 0;
    return;
  }
  if (memoryResponseStage == 1 && memoryReadyAfterNack) {
    memoryReadyAfterNack = false;
    memoryResponseStage = 4;
    prepareMemoryResponse();
    return;
  }
  if (memoryResponseStage == 1 || memoryResponseStage == 4 || memoryResponseStage == 5) {
    if (memoryResponseStage == 4 && pendingAzimuthIsMemoryRecall) {
      pendingAzimuthIsMemoryRecall = false;
      memoryRecallCancelPending = false;
      azimuthCommandPending = false;
      azimuthCompletionQueued = false;
      masterReady = !calibrationRequired;
      digitalWrite(STATUS_LED_PIN, masterReady ? LOW : HIGH);
    }
    memoryResponseStage = 0;
    return;
  }
  if (memoryResponseStage == 2) {
    if (recallMemoryPending) {
      if (memoryRecallCancelPending) {
        recallMemoryPending = false;
        recallMemoryFromTelegram = false;
        recallAwaitingEncoder = false;
        memoryResponseStage = 1;
        memoryReadyAfterNack = false;
        prepareMemoryResponse();
        return;
      }
      if (recallMemoryFromTelegram) {
        memories[pendingMemoryIndex].azimuth = pendingMemoryAzimuth;
        memories[pendingMemoryIndex].valid = true;
        recallMemoryFromTelegram = false;
      }
      pendingAzimuthTarget = pendingMemoryAzimuth;
      azimuthRequest = pendingAzimuthTarget;
      recallMemoryPending = false;
      recallAwaitingEncoder = true;
    } else {
      memories[pendingMemoryIndex].azimuth = pendingMemoryAzimuth;
      memories[pendingMemoryIndex].valid = true;
    }
  }
  ++memoryResponseStage;
  prepareMemoryResponse();
}

void handleReceivedLine()
{
  uint8_t lineLength = rxLength;
  if (lineLength > 0 && rxLine[lineLength - 1] == '\r') {
    --lineLength;
  }
  if (memoryResponseStage == 3 && recallAwaitingEncoder
      && !(lineLength >= sizeof("!HEARTBEAT,") - 1
          && memcmp(rxLine, "!HEARTBEAT,", sizeof("!HEARTBEAT,") - 1) == 0)) {
    rxLength = 0;
    rxOverflow = false;
    return;
  }

  if (!rxOverflow && lineLength == sizeof(IDENTIFY) - 1
      && memcmp(rxLine, IDENTIFY, sizeof(IDENTIFY) - 1) == 0) {
    movementBondNotificationHead = 0;
    movementBondNotificationCount = 0;
    cancelCalibrationOperation();
    calibrationCanceledNotReadyPending = false;
    azimuthResponsePending = false;
    azimuthResponseOffset = 0;
    calibrationReadyAfterDone = false;
    heartbeatReceived = false;
    heartbeatResponsePending = false;
    masterReady = false;
    digitalWrite(STATUS_LED_PIN, HIGH);
    sessionActive = false;
    azimuthCommandPending = false;
    pendingAzimuthIsLocalPush = false;
    pendingAzimuthIsMemoryRecall = false;
    azimuthCompletionQueued = false;
    memoryRecallCancelPending = false;
    pushPending = false;
    manualReadyPending = false;
    if (!allocateSid()) {
      rxLength = 0;
      rxOverflow = false;
      return;
    }
    const uint16_t crc = calculateCrc16(sessionBody, sizeof(sessionBody) - 1);
    constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
    char response[] = "!SESSION,00000000*0000\n";
    memcpy(response, sessionBody, sizeof(sessionBody) - 1);
    for (uint8_t i = 0; i < 4; ++i) {
      response[sizeof(sessionBody) + i] = HEX_DIGITS[(crc >> (12 - 4 * i)) & 0x0F];
    }
    SerialUSB.write(response);
    sessionActive = true;
    lastValidHeartbeatAt = millis();
  } else if (!rxOverflow && sessionActive
      && lineLength == sizeof("!HEARTBEAT,00000000*0000") - 1
      && memcmp(rxLine, "!HEARTBEAT,", sizeof("!HEARTBEAT,") - 1) == 0
      && memcmp(rxLine + sizeof("!HEARTBEAT,") - 1, sid, sizeof(sid) - 1) == 0) {
    constexpr uint8_t bodyLength = sizeof("!HEARTBEAT,00000000") - 1;
    bool valid = rxLine[bodyLength] == '*';
    uint16_t receivedCrc = 0;
    for (uint8_t i = 0; i < 4; ++i) {
      const char c = rxLine[bodyLength + 1 + i];
      if (c >= '0' && c <= '9') {
        receivedCrc = (receivedCrc << 4) | (c - '0');
      } else if (c >= 'A' && c <= 'F') {
        receivedCrc = (receivedCrc << 4) | (c - 'A' + 10);
      } else {
        valid = false;
      }
    }
    const uint16_t crc = calculateCrc16(rxLine, bodyLength);
    if (valid && crc == receivedCrc && CDC_connected()) {
      heartbeatReceived = true;
      lastValidHeartbeatAt = millis();
      memcpy(heartbeatResponse, rxLine, lineLength);
      heartbeatResponse[lineLength] = '\n';
      heartbeatResponsePending = true;
    }
  } else if (!rxOverflow && sessionActive
      && lineLength == sizeof(
          "!CAL-START,00000000,00000000-0000-0000-0000-000000000000*0000") - 1
      && memcmp(rxLine, "!CAL-START,", sizeof("!CAL-START,") - 1) == 0
      && memcmp(rxLine + sizeof("!CAL-START,") - 1, sid, sizeof(sid) - 1) == 0) {
    constexpr uint8_t cidOffset = sizeof("!CAL-START,00000000,") - 1;
    constexpr uint8_t bodyLength = sizeof(
        "!CAL-START,00000000,00000000-0000-0000-0000-000000000000") - 1;
    const char* cid = rxLine + cidOffset;
    bool valid = rxLine[cidOffset - 1] == ',' && rxLine[bodyLength] == '*';
    for (uint8_t i = 0; i < CID_LENGTH; ++i) {
      const char c = cid[i];
      if (i == 8 || i == 13 || i == 18 || i == 23) {
        valid = valid && c == '-';
      } else {
        valid = valid && ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'));
      }
    }
    uint16_t receivedCrc = 0;
    for (uint8_t i = 0; i < 4; ++i) {
      const char c = rxLine[bodyLength + 1 + i];
      if (c >= '0' && c <= '9') {
        receivedCrc = (receivedCrc << 4) | (c - '0');
      } else if (c >= 'A' && c <= 'F') {
        receivedCrc = (receivedCrc << 4) | (c - 'A' + 10);
      } else {
        valid = false;
      }
    }
    const uint16_t crc = calculateCrc16(rxLine, bodyLength);
    if (valid && crc == receivedCrc) {
      if (calibrationOperationActive) {
        if (memcmp(cid, calibrationCommandCid, CID_LENGTH) != 0) {
          queueCalibrationReply(cid, false);
        }
      } else {
        const bool automaticOperationInProgress = azimuthCommandPending || pushPending
            || memoryResponseStage || azimuthResponsePending || readyResponsePending
            || sensorStabilizationPending || movementBondClosingPending
            || manualMovementBondClosingPending || calibrationBondClosePending
            || calibrationBondClosurePendingReady
            || calibrationReadyWaitsForBondNotifications
            || digitalRead(MOTOR_CW_PIN) == HIGH
            || digitalRead(MOTOR_CCW_PIN) == HIGH;
        if (!heartbeatIsRecent() || digitalRead(MANUAL_TRIGGER_PIN) == HIGH
            || automaticOperationInProgress || (!masterReady && !calibrationRequired)) {
          queueCalibrationReply(cid, false);
        } else if (!openMovementBond()) {
          calibrationRequired = true;
          masterReady = false;
          digitalWrite(STATUS_LED_PIN, HIGH);
          queueCalibrationReply(cid, false);
        } else {
          calibrationCanceledNotReadyPending = false;
          memcpy(calibrationCommandCid, cid, CID_LENGTH);
          calibrationOperationActive = true;
          calibrationOperatorPromptSent = false;
          calibrationManualSenseSeen = false;
          calibrationHeadMotionObserved = false;
          calibrationHeadLastCount = encoderExtPositionCounts;
          calibrationHeadLastMotionAt = millis();
          calibrationHeadReferenceValid = false;
          calibrationTailPhase = false;
          calibrationScaleValid = false;
          calibrationPulsesPerDegree = 0.0;
          calibrationReadyAfterDone = false;
          calibrationBondClosePending = false;
          calibrationBondClosurePendingReady = false;
          calibrationReadyWaitsForBondNotifications = false;
          calibrationRequired = true;
          masterReady = false;
          digitalWrite(STATUS_LED_PIN, HIGH);
          queueCalibrationReply(cid, true);
        }
      }
    }
  } else if (!rxOverflow && sessionActive
      && lineLength == sizeof(CAL_CANCEL_REQUEST_TEMPLATE) - 1
      && memcmp(rxLine, "!CAL-CANCEL,", sizeof("!CAL-CANCEL,") - 1) == 0
      && memcmp(rxLine + sizeof("!CAL-CANCEL,") - 1, sid, sizeof(sid) - 1) == 0) {
    constexpr uint8_t cidOffset = sizeof("!CAL-CANCEL,00000000,") - 1;
    constexpr uint8_t bodyLength = sizeof(
        "!CAL-CANCEL,00000000,00000000-0000-0000-0000-000000000000") - 1;
    const char* cid = rxLine + cidOffset;
    bool valid = rxLine[cidOffset - 1] == ',' && rxLine[bodyLength] == '*';
    for (uint8_t i = 0; i < CID_LENGTH; ++i) {
      const char c = cid[i];
      if (i == 8 || i == 13 || i == 18 || i == 23) {
        valid = valid && c == '-';
      } else {
        valid = valid && ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'));
      }
    }
    uint16_t receivedCrc = 0;
    for (uint8_t i = 0; i < 4; ++i) {
      const char c = rxLine[bodyLength + 1 + i];
      if (c >= '0' && c <= '9') {
        receivedCrc = (receivedCrc << 4) | (c - '0');
      } else if (c >= 'A' && c <= 'F') {
        receivedCrc = (receivedCrc << 4) | (c - 'A' + 10);
      } else {
        valid = false;
      }
    }
    const uint16_t crc = calculateCrc16(rxLine, bodyLength);
    if (valid && crc == receivedCrc && CDC_connected() && heartbeatIsRecent()
        && calibrationOperationActive
        && memcmp(cid, calibrationCommandCid, CID_LENGTH) == 0) {
      char canceledCid[CID_LENGTH];
      memcpy(canceledCid, cid, CID_LENGTH);
      calibrationCancelIsExplicit = true;
      cancelCalibrationOperation();
      calibrationCancelIsExplicit = false;
      calibrationRequired = true;
      masterReady = false;
      calibrationCanceledNotReadyPending = true;
      prepareCalibrationCanceledResponse(canceledCid);
    }
  } else if (!rxOverflow && sessionActive
      && lineLength == sizeof(CAL_CONFIRM_TEMPLATE) - 1
      && memcmp(rxLine, "!CAL-CONFIRM,", sizeof("!CAL-CONFIRM,") - 1) == 0
      && memcmp(rxLine + sizeof("!CAL-CONFIRM,") - 1, sid, sizeof(sid) - 1) == 0) {
    constexpr uint8_t cidOffset = sizeof("!CAL-CONFIRM,00000000,") - 1;
    constexpr uint8_t phaseOffset = sizeof(
        "!CAL-CONFIRM,00000000,00000000-0000-0000-0000-000000000000,") - 1;
    constexpr uint8_t bodyLength = sizeof(
        "!CAL-CONFIRM,00000000,00000000-0000-0000-0000-000000000000,HEAD") - 1;
    const char* cid = rxLine + cidOffset;
    const bool confirmHead = memcmp(rxLine + phaseOffset, "HEAD", sizeof("HEAD") - 1) == 0;
    const bool confirmTail = memcmp(rxLine + phaseOffset, "TAIL", sizeof("TAIL") - 1) == 0;
    bool valid = rxLine[cidOffset - 1] == ','
        && (confirmHead || confirmTail)
        && rxLine[bodyLength] == '*';
    for (uint8_t i = 0; i < CID_LENGTH; ++i) {
      const char c = cid[i];
      if (i == 8 || i == 13 || i == 18 || i == 23) {
        valid = valid && c == '-';
      } else {
        valid = valid && ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'));
      }
    }
    uint16_t receivedCrc = 0;
    for (uint8_t i = 0; i < 4; ++i) {
      const char c = rxLine[bodyLength + 1 + i];
      if (c >= '0' && c <= '9') {
        receivedCrc = (receivedCrc << 4) | (c - '0');
      } else if (c >= 'A' && c <= 'F') {
        receivedCrc = (receivedCrc << 4) | (c - 'A' + 10);
      } else {
        valid = false;
      }
    }
    const uint16_t crc = calculateCrc16(rxLine, bodyLength);
    const bool commonConditions = valid && crc == receivedCrc && calibrationOperationActive
        && calibrationOperatorPromptSent && calibrationManualSenseSeen
        && calibrationHeadMotionObserved
        && sensorStabilized && !calibrationPhasePending
        && !azimuthResponsePending && !readyResponsePending && !memoryResponseStage
        && encoderExtEnabled && memcmp(cid, calibrationCommandCid, CID_LENGTH) == 0
        && digitalRead(MOTOR_CW_PIN) == LOW && digitalRead(MOTOR_CCW_PIN) == LOW
        && digitalRead(MANUAL_TRIGGER_PIN) == LOW && heartbeatIsRecent()
        && millis() - calibrationHeadLastMotionAt >= 3000
        && static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(&encoderExtTimer))
            == encoderExtPreviousCount;
    if (commonConditions && confirmHead && !calibrationTailPhase) {
      calibrationHeadCount = encoderExtPositionCounts;
      calibrationHeadReferenceValid = true;
      calibrationOperatorPromptSent = false;
      azimuthExtMeasured = HEAD_DEGREES;
      encoderExtLastRoundedAzimuth = HEAD_DEGREES;
      azimuthCurrent = HEAD_DEGREES;
      encoderExtAzimuthToReport = HEAD_DEGREES;
      encoderExtAzimuthReportPending = true;
      calibrationTailPhase = true;
      calibrationManualSenseSeen = false;
      calibrationHeadMotionObserved = false;
      calibrationHeadLastCount = encoderExtPositionCounts;
      calibrationHeadLastMotionAt = millis();
      calibrationPhasePending = true;
      prepareCalibrationPhaseResponse();
    } else if (commonConditions && confirmTail && calibrationTailPhase
        && calibrationHeadReferenceValid) {
      calibrationTailCount = encoderExtPositionCounts;
      const int64_t pulseDelta = static_cast<int64_t>(calibrationHeadCount)
          - static_cast<int64_t>(calibrationTailCount);
      if (pulseDelta > 0 && pulseDelta <= INT32_MAX) {
        const double pulsesPerDegree = static_cast<double>(pulseDelta) / 330.0;
        if (pulsesPerDegree > 0.0) {
          calibrationOperatorPromptSent = false;
          sensorStabilized = false;
          prepareCalibrationDoneResponse(static_cast<int32_t>(pulseDelta), pulsesPerDegree);
        } else {
          cancelCalibrationOperation();
        }
      } else {
        cancelCalibrationOperation();
      }
    }
  } else if (!rxOverflow && sessionActive
      && lineLength == sizeof(SYNC_PREFIX) - 1 + sizeof(sid) - 1 + 1 + CID_LENGTH + 1 + 4
      && memcmp(rxLine, SYNC_PREFIX, sizeof(SYNC_PREFIX) - 1) == 0
      && memcmp(rxLine + sizeof(SYNC_PREFIX) - 1, sid, sizeof(sid) - 1) == 0
      && rxLine[sizeof(SYNC_PREFIX) + sizeof(sid) - 2] == ',') {
    const char* cid = rxLine + sizeof(SYNC_PREFIX) + sizeof(sid) - 1;
    bool valid = true;
    for (uint8_t i = 0; i < CID_LENGTH; ++i) {
      const char c = cid[i];
      if (i == 8 || i == 13 || i == 18 || i == 23) {
        valid = valid && c == '-';
      } else {
        valid = valid && ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'));
      }
    }
    const uint8_t bodyLength = sizeof(SYNC_PREFIX) - 1 + sizeof(sid) - 1 + 1 + CID_LENGTH;
    valid = valid && rxLine[bodyLength] == '*';
    uint16_t receivedCrc = 0;
    for (uint8_t i = 0; i < 4; ++i) {
      const char c = rxLine[bodyLength + 1 + i];
      if (c >= '0' && c <= '9') {
        receivedCrc = (receivedCrc << 4) | (c - '0');
      } else if (c >= 'A' && c <= 'F') {
        receivedCrc = (receivedCrc << 4) | (c - 'A' + 10);
      } else {
        valid = false;
      }
    }
    uint16_t crc = calculateCrc16(rxLine, bodyLength);
    if (valid && crc == receivedCrc) {
      const bool canReady = !calibrationRequired
          && movementBondState == MovementBondState::Closed
          && digitalRead(MANUAL_TRIGGER_PIN) == LOW
          && !manualMovementBondClosingPending
          && encoderExtEnabled && isCurrentAzimuthMechanicallyAllowed()
          && isAzimuthAllowed(encoderExtLastRoundedAzimuth);
      masterReady = false;
      digitalWrite(STATUS_LED_PIN, HIGH);
      char response[] = "!SYNC_RSP,00000000,00000000-0000-0000-0000-000000000000,000,NORMAL,1,1*0000\n";
      memcpy(response + sizeof("!SYNC_RSP,") - 1, sid, sizeof(sid) - 1);
      memcpy(response + sizeof("!SYNC_RSP,00000000,") - 1, cid, CID_LENGTH);
      const uint8_t azimuthOffset = sizeof(
          "!SYNC_RSP,00000000,00000000-0000-0000-0000-000000000000,") - 1;
      response[azimuthOffset] = '0' + azimuthCurrent / 100;
      response[azimuthOffset + 1] = '0' + (azimuthCurrent / 10) % 10;
      response[azimuthOffset + 2] = '0' + azimuthCurrent % 10;
      const uint8_t syncValidOffset = sizeof(
          "!SYNC_RSP,00000000,00000000-0000-0000-0000-000000000000,000,NORMAL,1,") - 1;
      response[syncValidOffset] = calibrationRequired ? '0' : '1';
      const uint8_t responseBodyLength = sizeof("!SYNC_RSP,00000000,") - 1
          + CID_LENGTH + sizeof(",000,NORMAL,1,1") - 1;
      crc = calculateCrc16(response, responseBodyLength);
      constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
      for (uint8_t i = 0; i < 4; ++i) {
        response[responseBodyLength + 1 + i] = HEX_DIGITS[(crc >> (12 - 4 * i)) & 0x0F];
      }
      if (SerialUSB.write(response) == sizeof(response) - 1) {
        char ready[] = "!READY,00000000,1,1,1*0000\n";
        memcpy(ready + sizeof("!READY,") - 1, sid, sizeof(sid) - 1);
        ready[sizeof("!READY,00000000,") - 1] = canReady ? '1' : '0';
        ready[sizeof("!READY,00000000,1,1,") - 1] = calibrationRequired ? '0' : '1';
        const uint8_t readyBodyLength = sizeof("!READY,00000000,1,1,1") - 1;
        crc = calculateCrc16(ready, readyBodyLength);
        for (uint8_t i = 0; i < 4; ++i) {
          ready[readyBodyLength + 1 + i] = HEX_DIGITS[(crc >> (12 - 4 * i)) & 0x0F];
        }
        if (SerialUSB.write(ready) == sizeof(ready) - 1) {
          masterReady = canReady;
          digitalWrite(STATUS_LED_PIN, masterReady ? LOW : HIGH);
        }
      }
    }
  } else if (!rxOverflow && sessionActive
      && lineLength == sizeof("!AZIMUTH_PREVIEW,00000000,000,NORMAL*0000") - 1
      && memcmp(rxLine, "!AZIMUTH_PREVIEW,", sizeof("!AZIMUTH_PREVIEW,") - 1) == 0
      && memcmp(rxLine + sizeof("!AZIMUTH_PREVIEW,") - 1,
          sid, sizeof(sid) - 1) == 0) {
    constexpr uint8_t azimuthOffset = sizeof("!AZIMUTH_PREVIEW,00000000,") - 1;
    constexpr uint8_t bodyLength = sizeof(
        "!AZIMUTH_PREVIEW,00000000,000,NORMAL") - 1;
    bool valid = rxLine[azimuthOffset - 1] == ','
        && memcmp(rxLine + azimuthOffset + 3,
            ",NORMAL", sizeof(",NORMAL") - 1) == 0
        && rxLine[bodyLength] == '*';
    uint16_t requestedAzimuth = 0;
    for (uint8_t i = 0; i < 3; ++i) {
      const char c = rxLine[azimuthOffset + i];
      valid = valid && c >= '0' && c <= '9';
      if (c >= '0' && c <= '9') {
        requestedAzimuth = requestedAzimuth * 10 + c - '0';
      }
    }
    uint16_t receivedCrc = 0;
    for (uint8_t i = 0; i < 4; ++i) {
      const char c = rxLine[bodyLength + 1 + i];
      if (c >= '0' && c <= '9') {
        receivedCrc = (receivedCrc << 4) | (c - '0');
      } else if (c >= 'A' && c <= 'F') {
        receivedCrc = (receivedCrc << 4) | (c - 'A' + 10);
      } else {
        valid = false;
      }
    }
    if (valid && requestedAzimuth <= 359
        && calculateCrc16(rxLine, bodyLength) == receivedCrc
        && masterReady && encoderExtEnabled && !calibrationRequired
        && movementBondState == MovementBondState::Closed
        && !azimuthCommandPending && !calibrationOperationActive
        && digitalRead(MANUAL_TRIGGER_PIN) == LOW
        && isAzimuthAllowed(requestedAzimuth)) {
      azimuthRequest = requestedAzimuth;
      localTargetSelected = requestedAzimuth != encoderExtLastRoundedAzimuth;
      lastPreviewAzimuth = requestedAzimuth;
      previewPending = false;
    }
  } else if (!rxOverflow && sessionActive
      && !azimuthResponsePending && !readyResponsePending
      && lineLength == sizeof("!AZIMUTH,00000000,0000,000,NORMAL*0000") - 1
      && memcmp(rxLine, "!AZIMUTH,", sizeof("!AZIMUTH,") - 1) == 0
      && memcmp(rxLine + sizeof("!AZIMUTH,") - 1, sid, sizeof(sid) - 1) == 0) {
    constexpr uint8_t cidOffset = sizeof("!AZIMUTH,00000000,") - 1;
    constexpr uint8_t azimuthOffset = sizeof("!AZIMUTH,00000000,0000,") - 1;
    constexpr uint8_t bodyLength = sizeof("!AZIMUTH,00000000,0000,000,NORMAL") - 1;
    bool valid = rxLine[cidOffset - 1] == ',' && rxLine[azimuthOffset - 1] == ','
        && memcmp(rxLine + azimuthOffset + 3, ",NORMAL", sizeof(",NORMAL") - 1) == 0
        && rxLine[bodyLength] == '*';
    bool nonzeroCid = false;
    for (uint8_t i = 0; i < 4; ++i) {
      const char c = rxLine[cidOffset + i];
      valid = valid && ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F'));
      nonzeroCid = nonzeroCid || c != '0';
    }
    valid = valid && nonzeroCid;
    uint16_t commandedAzimuth = 0;
    for (uint8_t i = 0; i < 3; ++i) {
      const char c = rxLine[azimuthOffset + i];
      valid = valid && c >= '0' && c <= '9';
      if (c >= '0' && c <= '9') {
        commandedAzimuth = commandedAzimuth * 10 + c - '0';
      }
    }
    valid = valid && commandedAzimuth <= 359;
    uint16_t receivedCrc = 0;
    for (uint8_t i = 0; i < 4; ++i) {
      const char c = rxLine[bodyLength + 1 + i];
      if (c >= '0' && c <= '9') {
        receivedCrc = (receivedCrc << 4) | (c - '0');
      } else if (c >= 'A' && c <= 'F') {
        receivedCrc = (receivedCrc << 4) | (c - 'A' + 10);
      } else {
        valid = false;
      }
    }
    uint16_t crc = calculateCrc16(rxLine, bodyLength);
    if (valid && crc == receivedCrc) {
      bool repeatedPendingCommand = azimuthCommandPending
          && commandedAzimuth == pendingAzimuthTarget;
      for (uint8_t i = 0; i < 4; ++i) {
        repeatedPendingCommand = repeatedPendingCommand
            && rxLine[cidOffset + i] == pendingAzimuthCid[i];
      }
      if (repeatedPendingCommand) {
        rxLength = 0;
        rxOverflow = false;
        return;
      }
      if (!masterReady || !encoderExtEnabled || azimuthCommandPending
          || calibrationOperationActive
          || digitalRead(MANUAL_TRIGGER_PIN) == HIGH
          || !isAzimuthAllowed(commandedAzimuth)) {
        char response[] = "!NACK,00000000,0000,NOT_READY*0000\n";
        memcpy(response + sizeof("!NACK,") - 1, sid, sizeof(sid) - 1);
        memcpy(response + sizeof("!NACK,00000000,") - 1, rxLine + cidOffset, 4);
        const uint8_t nackBodyLength = sizeof("!NACK,00000000,0000,NOT_READY") - 1;
        crc = calculateCrc16(response, nackBodyLength);
        constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
        for (uint8_t i = 0; i < 4; ++i) {
          response[nackBodyLength + 1 + i] = HEX_DIGITS[(crc >> (12 - 4 * i)) & 0x0F];
        }
        memcpy(pendingAzimuthResponse, response, sizeof(response));
        azimuthResponseLength = sizeof(response) - 1;
        azimuthResponseOffset = 0;
        azimuthResponsePending = true;
      } else {
        pendingAzimuthTarget = commandedAzimuth;
        for (uint8_t i = 0; i < 4; ++i) {
          pendingAzimuthCid[i] = rxLine[cidOffset + i];
        }
        azimuthCommandPending = true;
        azimuthTargetReached = false;
        movementBondMotionStarted = false;
        pendingAzimuthIsLocalPush = false;
        azimuthCompletionQueued = false;
        azimuthRequest = commandedAzimuth;
        localTargetSelected = false;
        previewPending = false;
        masterReady = false;
        digitalWrite(STATUS_LED_PIN, HIGH);
      }
    }
  } else if (!rxOverflow && sessionActive
      && lineLength == sizeof("!SAVE,00000000,00000000-0000-0000-0000-000000000000,M1*0000") - 1
      && memcmp(rxLine, "!SAVE,", sizeof("!SAVE,") - 1) == 0
      && memcmp(rxLine + sizeof("!SAVE,") - 1, sid, sizeof(sid) - 1) == 0) {
    constexpr uint8_t cidOffset = sizeof("!SAVE,00000000,") - 1;
    constexpr uint8_t memoryOffset = sizeof("!SAVE,00000000,00000000-0000-0000-0000-000000000000,") - 1;
    constexpr uint8_t bodyLength = sizeof("!SAVE,00000000,00000000-0000-0000-0000-000000000000,M1") - 1;
    bool valid = rxLine[cidOffset - 1] == ',' && rxLine[memoryOffset - 1] == ','
        && rxLine[memoryOffset] == 'M'
        && rxLine[memoryOffset + 1] >= '1' && rxLine[memoryOffset + 1] <= '6'
        && rxLine[bodyLength] == '*';
    const char* cid = rxLine + cidOffset;
    for (uint8_t i = 0; i < CID_LENGTH; ++i) {
      const char c = cid[i];
      if (i == 8 || i == 13 || i == 18 || i == 23) {
        valid = valid && c == '-';
      } else {
        valid = valid && ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'));
      }
    }
    uint16_t receivedCrc = 0;
    for (uint8_t i = 0; i < 4; ++i) {
      const char c = rxLine[bodyLength + 1 + i];
      if (c >= '0' && c <= '9') {
        receivedCrc = (receivedCrc << 4) | (c - '0');
      } else if (c >= 'A' && c <= 'F') {
        receivedCrc = (receivedCrc << 4) | (c - 'A' + 10);
      } else {
        valid = false;
      }
    }
    const uint16_t crc = calculateCrc16(rxLine, bodyLength);
    if (valid && crc == receivedCrc) {
      pendingMemoryIndex = rxLine[memoryOffset + 1] - '1';
      memcpy(pendingMemoryCid, cid, CID_LENGTH);
      recallMemoryPending = false;
      if (masterReady && !calibrationOperationActive && isAzimuthAllowed(azimuthCurrent)) {
        pendingMemoryAzimuth = azimuthCurrent;
        memoryResponseStage = 2;
      } else {
        memoryResponseStage = 1;
        memoryReadyAfterNack = masterReady && !calibrationOperationActive;
      }
      prepareMemoryResponse();
    }
  } else if (!rxOverflow && sessionActive
      && (lineLength == sizeof("!MEM,00000000,00000000-0000-0000-0000-000000000000,M1*0000") - 1
          || lineLength == sizeof("!MEM,00000000,00000000-0000-0000-0000-000000000000,M1,000*0000") - 1)
      && memcmp(rxLine, "!MEM,", sizeof("!MEM,") - 1) == 0
      && memcmp(rxLine + sizeof("!MEM,") - 1, sid, sizeof(sid) - 1) == 0) {
    constexpr uint8_t cidOffset = sizeof("!MEM,00000000,") - 1;
    constexpr uint8_t memoryOffset = sizeof("!MEM,00000000,00000000-0000-0000-0000-000000000000,") - 1;
    const bool hasAzimuth = lineLength == sizeof(
        "!MEM,00000000,00000000-0000-0000-0000-000000000000,M1,000*0000") - 1;
    const uint8_t bodyLength = hasAzimuth
        ? sizeof("!MEM,00000000,00000000-0000-0000-0000-000000000000,M1,000") - 1
        : sizeof("!MEM,00000000,00000000-0000-0000-0000-000000000000,M1") - 1;
    bool valid = rxLine[cidOffset - 1] == ',' && rxLine[memoryOffset - 1] == ','
        && rxLine[memoryOffset] == 'M'
        && rxLine[memoryOffset + 1] >= '1' && rxLine[memoryOffset + 1] <= '6'
        && rxLine[bodyLength] == '*';
    uint16_t requestedAzimuth = 0;
    if (hasAzimuth) {
      valid = valid && rxLine[memoryOffset + 2] == ',';
      for (uint8_t i = 0; i < 3; ++i) {
        const char c = rxLine[memoryOffset + 3 + i];
        valid = valid && c >= '0' && c <= '9';
        if (c >= '0' && c <= '9') {
          requestedAzimuth = requestedAzimuth * 10 + c - '0';
        }
      }
      valid = valid && requestedAzimuth <= 359;
    }
    const char* cid = rxLine + cidOffset;
    for (uint8_t i = 0; i < CID_LENGTH; ++i) {
      const char c = cid[i];
      if (i == 8 || i == 13 || i == 18 || i == 23) {
        valid = valid && c == '-';
      } else {
        valid = valid && ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'));
      }
    }
    uint16_t receivedCrc = 0;
    for (uint8_t i = 0; i < 4; ++i) {
      const char c = rxLine[bodyLength + 1 + i];
      if (c >= '0' && c <= '9') {
        receivedCrc = (receivedCrc << 4) | (c - '0');
      } else if (c >= 'A' && c <= 'F') {
        receivedCrc = (receivedCrc << 4) | (c - 'A' + 10);
      } else {
        valid = false;
      }
    }
    const uint16_t crc = calculateCrc16(rxLine, bodyLength);
    if (valid && crc == receivedCrc) {
      pendingMemoryIndex = rxLine[memoryOffset + 1] - '1';
      memcpy(pendingMemoryCid, cid, CID_LENGTH);
      memoryReadyAfterNack = false;
      if (hasAzimuth) {
        if (masterReady && !calibrationOperationActive && encoderExtEnabled
            && digitalRead(MANUAL_TRIGGER_PIN) == LOW
            && isAzimuthAllowed(requestedAzimuth)) {
          pendingMemoryAzimuth = requestedAzimuth;
          pendingAzimuthTarget = requestedAzimuth;
          azimuthRequest = requestedAzimuth;
          recallMemoryPending = true;
          recallMemoryFromTelegram = true;
          recallAwaitingEncoder = false;
          azimuthCommandPending = true;
          azimuthTargetReached = false;
          movementBondMotionStarted = false;
          pendingAzimuthIsLocalPush = false;
          pendingAzimuthIsMemoryRecall = true;
          azimuthCompletionQueued = false;
          for (uint8_t i = 0; i < 4; ++i) {
            pendingAzimuthCid[i] = cid[i];
          }
          masterReady = false;
          digitalWrite(STATUS_LED_PIN, HIGH);
          memoryResponseStage = 2;
        } else {
          recallMemoryPending = false;
          recallMemoryFromTelegram = false;
          memoryResponseStage = 1;
          memoryReadyAfterNack = masterReady && !calibrationOperationActive;
        }
      } else if (memories[pendingMemoryIndex].valid) {
        if (masterReady && !calibrationOperationActive && encoderExtEnabled
            && digitalRead(MANUAL_TRIGGER_PIN) == LOW
            && isAzimuthAllowed(memories[pendingMemoryIndex].azimuth)) {
          pendingMemoryAzimuth = memories[pendingMemoryIndex].azimuth;
          pendingAzimuthTarget = pendingMemoryAzimuth;
          azimuthRequest = pendingAzimuthTarget;
          recallMemoryPending = true;
          recallMemoryFromTelegram = false;
          recallAwaitingEncoder = false;
          azimuthCommandPending = true;
          azimuthTargetReached = false;
          movementBondMotionStarted = false;
          pendingAzimuthIsLocalPush = false;
          pendingAzimuthIsMemoryRecall = true;
          azimuthCompletionQueued = false;
          for (uint8_t i = 0; i < 4; ++i) {
            pendingAzimuthCid[i] = cid[i];
          }
          masterReady = false;
          digitalWrite(STATUS_LED_PIN, HIGH);
          memoryResponseStage = 2;
        } else {
          recallMemoryPending = false;
          recallMemoryFromTelegram = false;
          memoryResponseStage = 1;
          memoryReadyAfterNack = masterReady && !calibrationOperationActive;
        }
      } else {
        recallMemoryPending = false;
        recallMemoryFromTelegram = false;
        memoryResponseStage = 5;
      }
      prepareMemoryResponse();
    }
  }
  rxLength = 0;
  rxOverflow = false;
}

void servicePendingAzimuthResponses()
{
  if (calibrationBondClosurePendingReady && encoderExtEnabled
      && (encoderExtPositionCounts != movementBondStablePosition
          || static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(&encoderExtTimer))
              != encoderExtPreviousCount)) {
    cancelCalibrationOperation();
  }
  if (calibrationPhasePending && (!sessionActive || !heartbeatIsRecent())) {
    cancelPendingCalibrationPhase();
  }
  if (azimuthResponsePending
      && memcmp(pendingAzimuthResponse, "!CAL-DONE,", sizeof("!CAL-DONE,") - 1) == 0
      && (!calibrationOperationActive || !heartbeatIsRecent()
          || digitalRead(MOTOR_CW_PIN) == HIGH || digitalRead(MOTOR_CCW_PIN) == HIGH)) {
    cancelCalibrationOperation();
  }
  if (azimuthResponsePending) {
    const uint8_t responseLength = azimuthResponseLength;
    const int available = SerialUSB.availableForWrite();
    if (available > 0) {
      const uint8_t remaining = responseLength - azimuthResponseOffset;
      const uint8_t chunk = available < remaining ? available : remaining;
      azimuthResponseOffset += static_cast<uint8_t>(SerialUSB.write(
          reinterpret_cast<const uint8_t*>(pendingAzimuthResponse) + azimuthResponseOffset,
          chunk));
    }
    if (azimuthResponseOffset < responseLength) {
      return;
    }
    azimuthResponsePending = false;
    if (memcmp(pendingAzimuthResponse, "!CAL-CANCELED,",
        sizeof("!CAL-CANCELED,") - 1) == 0) {
      if (calibrationCanceledNotReadyPending) {
        calibrationCanceledNotReadyPending = false;
        prepareCalibrationNotReadyResponse();
      }
    }
    if (memcmp(pendingAzimuthResponse, "!CAL-DONE,",
        sizeof("!CAL-DONE,") - 1) == 0) {
      if (calibrationOperationActive && pendingCalibrationPulseDelta > 0
          && heartbeatIsRecent() && encoderExtEnabled
          && digitalRead(MOTOR_CW_PIN) == LOW && digitalRead(MOTOR_CCW_PIN) == LOW) {
        calibrationPulsesPerDegree = pendingCalibrationPulsesPerDegree;
        calibrationScaleValid = true;
        calibrationHeadReferenceValid = true;
        calibrationOperatorPromptSent = false;
        calibrationTailPhase = false;
        calibrationRequired = true;
        masterReady = false;
        digitalWrite(STATUS_LED_PIN, HIGH);
        stopSensorStabilization();
        digitalWrite(MOTOR_CW_PIN, LOW);
        digitalWrite(MOTOR_CCW_PIN, LOW);
        calibrationBondLastMotionAt = millis();
        calibrationBondClosePending = true;

        const int64_t relativeCounts = static_cast<int64_t>(encoderExtPositionCounts)
            - static_cast<int64_t>(calibrationHeadCount);
        double measured = HEAD_DEGREES
            + static_cast<double>(relativeCounts) / calibrationPulsesPerDegree;
        measured = fmod(measured, 360.0);
        if (measured < 0.0) {
          measured += 360.0;
        }
        azimuthExtMeasured = static_cast<float>(measured);
        uint16_t rounded = static_cast<uint16_t>(measured + 0.5);
        if (rounded == 360) {
          rounded = 0;
        }
        encoderExtLastRoundedAzimuth = rounded;
        azimuthCurrent = rounded;
        encoderExtAzimuthToReport = rounded;
        encoderExtAzimuthReportPending = true;
      } else {
        cancelCalibrationOperation();
      }
      return;
    }
    if (memcmp(pendingAzimuthResponse, "!CAL-OPERATOR,",
        sizeof("!CAL-OPERATOR,") - 1) == 0 && calibrationOperationActive
        && sensorStabilized && calibrationManualSenseSeen
        && calibrationHeadMotionObserved
        && digitalRead(MANUAL_TRIGGER_PIN) == LOW
        && millis() - calibrationHeadLastMotionAt >= 3000
        && static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(&encoderExtTimer))
            == encoderExtPreviousCount) {
      calibrationOperatorPromptSent = true;
    }
    if (calibrationPhasePending) {
      if (memcmp(pendingAzimuthResponse, "!CAL-PHASE,",
          sizeof("!CAL-PHASE,") - 1) == 0) {
        calibrationPhasePending = false;
        if (calibrationOperationActive) {
          if (!calibrationTailPhase || !sensorStabilized) {
            sensorStabilizationPending = true;
            sensorStabilized = false;
            sensorStabilizationStartedAt = millis();
            digitalWrite(ENABLE_SENSORS_PIN, HIGH);
          }
        }
      } else {
        prepareCalibrationPhaseResponse();
        return;
      }
    }
  }

  if (readyResponsePending && calibrationReadyAfterDone
      && (!heartbeatIsRecent() || !encoderExtEnabled || calibrationOperationActive
          || (calibrationRequired && !calibrationBondClosurePendingReady)
          || (calibrationBondClosurePendingReady
              && (movementBondState != MovementBondState::Closed
                  || calibrationReadyWaitsForBondNotifications))
          || digitalRead(MANUAL_TRIGGER_PIN) == HIGH
          || digitalRead(MOTOR_CW_PIN) == HIGH || digitalRead(MOTOR_CCW_PIN) == HIGH)) {
    cancelCalibrationOperation();
  }
  if (readyResponsePending) {
    const uint8_t responseLength = sizeof(pendingReadyResponse) - 1;
    const int available = SerialUSB.availableForWrite();
    if (available > 0) {
      const uint8_t remaining = responseLength - readyResponseOffset;
      const uint8_t chunk = available < remaining ? available : remaining;
      readyResponseOffset += static_cast<uint8_t>(SerialUSB.write(
          reinterpret_cast<const uint8_t*>(pendingReadyResponse) + readyResponseOffset,
          chunk));
    }
    if (readyResponseOffset == responseLength) {
      readyResponsePending = false;
      if (manualReadyPending
          && pendingReadyResponse[sizeof("!READY,00000000,") - 1] == '1') {
        manualReadyPending = false;
        masterReady = heartbeatIsRecent() && !calibrationRequired
            && movementBondState == MovementBondState::Closed
            && digitalRead(MANUAL_TRIGGER_PIN) == LOW;
        digitalWrite(STATUS_LED_PIN, masterReady ? LOW : HIGH);
      }
      if (calibrationReadyAfterDone) {
        if (calibrationBondClosurePendingReady) {
          if (!heartbeatIsRecent() || !encoderExtEnabled
              || calibrationOperationActive || !calibrationRequired
              || movementBondState != MovementBondState::Closed
              || digitalRead(MANUAL_TRIGGER_PIN) == HIGH
              || digitalRead(MOTOR_CW_PIN) == HIGH
              || digitalRead(MOTOR_CCW_PIN) == HIGH) {
            cancelCalibrationOperation();
            return;
          }
          calibrationBondClosurePendingReady = false;
          calibrationRequired = false;
        }
        calibrationReadyAfterDone = false;
        masterReady = heartbeatIsRecent() && encoderExtEnabled
            && !calibrationOperationActive && !calibrationRequired
            && digitalRead(MANUAL_TRIGGER_PIN) == LOW
            && digitalRead(MOTOR_CW_PIN) == LOW && digitalRead(MOTOR_CCW_PIN) == LOW;
        digitalWrite(STATUS_LED_PIN, masterReady ? LOW : HIGH);
      }
    }
  }
}

void stopSensorStabilization()
{
  sensorStabilizationPending = false;
  sensorStabilized = false;
  digitalWrite(MOTOR_CW_PIN, LOW);
  digitalWrite(MOTOR_CCW_PIN, LOW);
  digitalWrite(ENABLE_SENSORS_PIN,
      digitalRead(MANUAL_TRIGGER_PIN) == HIGH ? HIGH : LOW);
}

void cancelCalibrationOperation()
{
  const bool calibrationWasActive = calibrationOperationActive;
  const bool calibrationFinalizationPending = calibrationBondClosePending
      || calibrationBondClosurePendingReady || calibrationReadyWaitsForBondNotifications
      || calibrationReadyAfterDone;
  const bool calibrationWasCanceled = calibrationWasActive || calibrationFinalizationPending;
  char canceledCid[CID_LENGTH] = {};
  if (calibrationWasCanceled) {
    memcpy(canceledCid, calibrationCommandCid, CID_LENGTH);
  }
  if (calibrationWasCanceled && movementBondState != MovementBondState::Open
      && !openMovementBond()) {
    movementBondState = MovementBondState::Unknown;
  }
  cancelPendingCalibrationPhase();
  if (memcmp(pendingAzimuthResponse, "!CAL-OPERATOR,",
      sizeof("!CAL-OPERATOR,") - 1) == 0) {
    azimuthResponsePending = false;
    azimuthResponseOffset = 0;
  }
  if (memcmp(pendingAzimuthResponse, "!CAL-DONE,",
      sizeof("!CAL-DONE,") - 1) == 0) {
    azimuthResponsePending = false;
    azimuthResponseOffset = 0;
  }
  if (calibrationWasCanceled) {
    readyResponsePending = false;
    readyResponseOffset = 0;
    calibrationReadyAfterDone = false;
    calibrationRequired = true;
    masterReady = false;
    digitalWrite(STATUS_LED_PIN, HIGH);
    calibrationCanceledNotReadyPending = !calibrationCancelIsExplicit;
  }
  calibrationOperationActive = false;
  calibrationOperatorPromptSent = false;
  calibrationTailPhase = false;
  if (calibrationWasCanceled) {
    calibrationHeadReferenceValid = false;
    calibrationScaleValid = false;
    calibrationPulsesPerDegree = 0.0;
  }
  calibrationBondClosePending = false;
  calibrationBondLastMotionAt = 0;
  calibrationBondClosurePendingReady = false;
  calibrationReadyWaitsForBondNotifications = false;
  calibrationManualSenseSeen = false;
  calibrationHeadMotionObserved = false;
  calibrationHeadLastCount = encoderExtPositionCounts;
  calibrationHeadLastMotionAt = 0;
  pendingCalibrationPulseDelta = 0;
  pendingCalibrationPulsesPerDegree = 0.0;
  memset(calibrationCommandCid, 0, sizeof(calibrationCommandCid));
  stopSensorStabilization();
  if (calibrationWasCanceled && !calibrationCancelIsExplicit
      && sessionActive && CDC_connected() && heartbeatIsRecent()) {
    calibrationCanceledNotReadyPending = true;
    prepareCalibrationCanceledResponse(canceledCid);
  } else if (!sessionActive || !CDC_connected() || !heartbeatIsRecent()) {
    calibrationCanceledNotReadyPending = false;
  }
}

void serviceCalibrationHeadEncoderPosition()
{
  if (!calibrationOperationActive || calibrationBondClosePending
      || encoderExtPositionCounts == calibrationHeadLastCount) {
    return;
  }
  const int32_t movement = encoderExtPositionCounts - calibrationHeadLastCount;
  calibrationHeadLastCount = encoderExtPositionCounts;
  calibrationHeadLastMotionAt = millis();
  calibrationOperatorPromptSent = false;
  if (azimuthResponsePending && azimuthResponseOffset == 0
      && memcmp(pendingAzimuthResponse, "!CAL-OPERATOR,",
          sizeof("!CAL-OPERATOR,") - 1) == 0) {
    azimuthResponsePending = false;
  }
  if (calibrationManualSenseSeen
      && (calibrationTailPhase ? movement < 0 : movement > 0)) {
    calibrationHeadMotionObserved = true;
  }
}

void queueMemoryRecallCancellation()
{
  stopSensorStabilization();
  memoryRecallCancelPending = true;
  memoryReadyAfterNack = false;
}

void queuePendingAzimuthAbort()
{
  stopSensorStabilization();
  char response[] = "!NACK,00000000,0000,NOT_READY*0000\n";
  memcpy(response + sizeof("!NACK,") - 1, sid, sizeof(sid) - 1);
  const uint8_t cidOffset = sizeof("!NACK,00000000,") - 1;
  for (uint8_t i = 0; i < 4; ++i) {
    response[cidOffset + i] = pendingAzimuthCid[i];
  }
  constexpr uint8_t nackBodyLength = sizeof("!NACK,00000000,0000,NOT_READY") - 1;
  const uint16_t crc = calculateCrc16(response, nackBodyLength);
  constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
  for (uint8_t i = 0; i < 4; ++i) {
    response[nackBodyLength + 1 + i] = HEX_DIGITS[(crc >> (12 - 4 * i)) & 0x0F];
  }
  memcpy(pendingAzimuthResponse, response, sizeof(response));
  azimuthResponseLength = sizeof(response) - 1;
  azimuthResponseOffset = 0;
  readyResponsePending = false;
  readyResponseOffset = 0;
  azimuthResponsePending = true;
  azimuthCommandPending = false;
  pendingAzimuthIsLocalPush = false;
  azimuthCompletionQueued = false;
  masterReady = false;
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void beginMovementBondCloseWait()
{
  digitalWrite(MOTOR_CW_PIN, LOW);
  digitalWrite(MOTOR_CCW_PIN, LOW);
  movementBondClosingPending = true;
  movementBondLastExtMotionAt = millis();
}

void serviceMovementBondCloseWait()
{
  if (!movementBondClosingPending) {
    return;
  }
  if (!sessionActive || !heartbeatIsRecent() || !CDC_connected()
      || digitalRead(MANUAL_TRIGGER_PIN) == HIGH || calibrationOperationActive
      || !azimuthCommandPending || pendingAzimuthIsMemoryRecall
      || !azimuthTargetReached) {
    movementBondClosingPending = false;
    return;
  }
  if (!encoderExtEnabled || !isAzimuthAllowed(encoderExtLastRoundedAzimuth)
      || !isCurrentAzimuthMechanicallyAllowed()) {
    movementBondClosingPending = false;
    requireCalibration();
    queuePendingAzimuthAbort();
    return;
  }
  if (digitalRead(MOTOR_CW_PIN) == HIGH || digitalRead(MOTOR_CCW_PIN) == HIGH) {
    digitalWrite(MOTOR_CW_PIN, LOW);
    digitalWrite(MOTOR_CCW_PIN, LOW);
    movementBondClosingPending = false;
    requireCalibration();
    queuePendingAzimuthAbort();
    return;
  }
  if (millis() - movementBondLastExtMotionAt < 2000) {
    return;
  }
  if (!closeMovementBond(true, encoderExtPositionCounts)) {
    movementBondClosingPending = false;
    requireCalibration();
    queuePendingAzimuthAbort();
    return;
  }
  movementBondClosingPending = false;
  movementBondMotionStarted = false;
}

void queueLocalPushCancellation()
{
  stopSensorStabilization();
  azimuthCommandPending = false;
  pendingAzimuthIsLocalPush = false;
  azimuthCompletionQueued = false;
  azimuthResponsePending = false;
  azimuthResponseOffset = 0;
  readyResponseOffset = 0;
  readyResponsePending = false;
  masterReady = false;
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void serviceHeartbeatMonitor()
{
  if (heartbeatIsRecent()) {
    return;
  }
  if (!sessionActive) {
    heartbeatReceived = false;
    heartbeatResponsePending = false;
    return;
  }
  if (azimuthCommandPending && !azimuthCompletionQueued
      && !memoryRecallCancelPending && CDC_connected()) {
    if (pendingAzimuthIsMemoryRecall) {
      queueMemoryRecallCancellation();
    } else if (pendingAzimuthIsLocalPush) {
      queueLocalPushCancellation();
    } else {
      queuePendingAzimuthAbort();
    }
  }
  if (!CDC_connected()
      || millis() - lastValidHeartbeatAt >= HEARTBEAT_TIMEOUT_MS) {
    cancelCalibrationOperation();
    sessionActive = false;
    masterReady = false;
    manualReadyPending = false;
    digitalWrite(STATUS_LED_PIN, HIGH);
    azimuthCommandPending = false;
    pendingAzimuthIsLocalPush = false;
    pendingAzimuthIsMemoryRecall = false;
    azimuthCompletionQueued = false;
    azimuthResponsePending = false;
    readyResponsePending = false;
    calibrationReadyAfterDone = false;
    memoryResponseStage = 0;
    recallMemoryPending = false;
    recallAwaitingEncoder = false;
    memoryRecallCancelPending = false;
    pushPending = false;
    heartbeatReceived = false;
    heartbeatResponsePending = false;
    calibrationCanceledNotReadyPending = false;
    rxLength = 0;
    rxOverflow = false;
  }
}

void serviceHeartbeatResponse()
{
  if (heartbeatResponsePending && sessionActive && CDC_connected()
      && !azimuthResponsePending && !readyResponsePending
      && (!memoryResponseStage || (memoryResponseStage == 3 && recallAwaitingEncoder))
      && SerialUSB.availableForWrite() >= static_cast<int>(sizeof(heartbeatResponse) - 1)
      && SerialUSB.write(heartbeatResponse) == sizeof(heartbeatResponse) - 1) {
    heartbeatResponsePending = false;
  }
}

void queueManualReadyResponse(bool readyState)
{
  if (!heartbeatIsRecent()) {
    return;
  }
  char ready[] = "!READY,00000000,0,1,1*0000\n";
  memcpy(ready + sizeof("!READY,") - 1, sid, sizeof(sid) - 1);
  ready[sizeof("!READY,00000000,") - 1] = readyState ? '1' : '0';
  ready[sizeof("!READY,00000000,0,1,") - 1] = calibrationRequired ? '0' : '1';
  const uint8_t bodyLength = sizeof("!READY,00000000,0,1,1") - 1;
  const uint16_t crc = calculateCrc16(ready, bodyLength);
  constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
  for (uint8_t i = 0; i < 4; ++i) {
    ready[bodyLength + 1 + i] = HEX_DIGITS[(crc >> (12 - 4 * i)) & 0x0F];
  }
  memcpy(pendingReadyResponse, ready, sizeof(ready));
  readyResponseOffset = 0;
  readyResponsePending = true;
}

void serviceManualTrigger()
{
  const bool manualActive = digitalRead(MANUAL_TRIGGER_PIN) == HIGH;
  if (!sessionActive && !manualActive) {
    manualAzimuthReportPending = false;
  }
  if (calibrationOperationActive) {
    if (manualActive) {
      calibrationManualSenseSeen = true;
      calibrationOperatorPromptSent = false;
      calibrationHeadLastMotionAt = millis();
      if (calibrationBondClosePending) {
        calibrationBondLastMotionAt = millis();
      }
      if (azimuthResponsePending && azimuthResponseOffset == 0
          && memcmp(pendingAzimuthResponse, "!CAL-OPERATOR,",
              sizeof("!CAL-OPERATOR,") - 1) == 0) {
        azimuthResponsePending = false;
      }
      if (!manualTriggerWasHigh) {
        encoderExtAzimuthToReport = encoderExtLastRoundedAzimuth;
        encoderExtAzimuthReportPending = true;
      }
    } else if (manualTriggerWasHigh) {
      calibrationHeadLastMotionAt = millis();
      if (calibrationBondClosePending) {
        calibrationBondLastMotionAt = millis();
      }
      encoderExtAzimuthToReport = encoderExtLastRoundedAzimuth;
      encoderExtAzimuthReportPending = true;
    }
    manualTriggerWasHigh = manualActive;
    digitalWrite(MOTOR_CW_PIN, LOW);
    digitalWrite(MOTOR_CCW_PIN, LOW);
    if (manualActive || sensorStabilized || sensorStabilizationPending) {
      digitalWrite(ENABLE_SENSORS_PIN, HIGH);
    }
    return;
  }
  if (manualActive && sessionActive && !heartbeatIsRecent()) {
    requireCalibration();
  }
  if (manualActive && !manualTriggerWasHigh) {
    manualTriggerWasHigh = true;
    manualAzimuthReportPending = true;
    digitalWrite(MOTOR_CW_PIN, LOW);
    digitalWrite(MOTOR_CCW_PIN, LOW);
    masterReady = false;
    digitalWrite(STATUS_LED_PIN, HIGH);
    pushPending = false;
    localTargetSelected = false;
    previewPending = false;
    manualReadyPending = false;
    readyResponsePending = false;
    readyResponseOffset = 0;
    encoderCountAccumulator = 0;
    manualMovementBondClosingPending = false;
    if (!manualMovementBondOwned) {
      if (movementBondState == MovementBondState::Open
          && azimuthCommandPending && !pendingAzimuthIsMemoryRecall
          && movementBondMotionStarted && !calibrationRequired) {
        movementBondClosingPending = false;
        movementBondMotionStarted = false;
        manualMovementBondOwned = true;
      } else if (movementBondState == MovementBondState::Open) {
        requireCalibration();
      } else if (openMovementBond()) {
        manualMovementBondOwned = true;
      } else {
        requireCalibration();
      }
    }
    cancelCalibrationOperation();
    encoderExtAzimuthToReport = encoderExtLastRoundedAzimuth;
    encoderExtAzimuthReportPending = true;
    if (azimuthCommandPending && !memoryRecallCancelPending) {
      if (pendingAzimuthIsMemoryRecall) {
        queueMemoryRecallCancellation();
      } else if (pendingAzimuthIsLocalPush) {
        queueLocalPushCancellation();
      } else {
        queuePendingAzimuthAbort();
      }
    }
    queueManualReadyResponse(false);
  } else if (!manualActive && manualTriggerWasHigh) {
    manualTriggerWasHigh = false;
    pushPending = false;
    localTargetSelected = false;
    encoderCountAccumulator = 0;
    if (manualMovementBondOwned && movementBondState == MovementBondState::Open) {
      manualMovementBondClosingPending = true;
      manualMovementBondLastMotionAt = millis();
    }
    if (!sessionActive) {
      manualAzimuthReportPending = false;
    }
    encoderExtAzimuthToReport = encoderExtLastRoundedAzimuth;
    encoderExtAzimuthReportPending = true;
    digitalWrite(ENABLE_SENSORS_PIN, LOW);
  }

  if (manualActive) {
    digitalWrite(MOTOR_CW_PIN, LOW);
    digitalWrite(MOTOR_CCW_PIN, LOW);
    digitalWrite(ENABLE_SENSORS_PIN, HIGH);
  }
}

void serviceManualMovementBondCloseWait()
{
  if (!manualMovementBondClosingPending) {
    return;
  }
  if (digitalRead(MANUAL_TRIGGER_PIN) == HIGH) {
    manualMovementBondClosingPending = false;
    return;
  }
  if (!manualMovementBondOwned || movementBondState != MovementBondState::Open) {
    manualMovementBondClosingPending = false;
    manualMovementBondOwned = false;
    return;
  }
  if (!encoderExtEnabled || !calibrationHeadReferenceValid || !calibrationScaleValid
      || calibrationPulsesPerDegree <= 0.0 || calibrationRequired
      || calibrationOperationActive || !isCurrentAzimuthMechanicallyAllowed()
      || !isAzimuthAllowed(encoderExtLastRoundedAzimuth)
      || digitalRead(MOTOR_CW_PIN) == HIGH || digitalRead(MOTOR_CCW_PIN) == HIGH) {
    manualMovementBondClosingPending = false;
    manualMovementBondOwned = false;
    requireCalibration();
    return;
  }
  if (millis() - manualMovementBondLastMotionAt < 2000) {
    return;
  }
  if (!closeMovementBond(true, encoderExtPositionCounts)) {
    manualMovementBondClosingPending = false;
    manualMovementBondOwned = false;
    requireCalibration();
    return;
  }
  azimuthCurrent = encoderExtLastRoundedAzimuth;
  azimuthRequest = azimuthCurrent;
  pushPendingAzimuth = azimuthCurrent;
  lastPreviewAzimuth = azimuthCurrent;
  previewPending = false;
  pushPending = false;
  localTargetSelected = false;
  encoderExtAzimuthToReport = azimuthCurrent;
  encoderExtAzimuthReportPending = true;
  manualReadyPending = sessionActive;
  manualMovementBondClosingPending = false;
  manualMovementBondOwned = false;
}

void serviceManualReadyAfterBond()
{
  if (!manualReadyPending || manualMovementBondClosingPending
      || movementBondNotificationCount != 0 || encoderExtAzimuthReportPending
      || readyResponsePending || azimuthResponsePending || memoryResponseStage) {
    return;
  }
  if (!heartbeatIsRecent() || !encoderExtEnabled || calibrationRequired
      || movementBondState != MovementBondState::Closed
      || digitalRead(MANUAL_TRIGGER_PIN) == HIGH
      || !isCurrentAzimuthMechanicallyAllowed()
      || !isAzimuthAllowed(encoderExtLastRoundedAzimuth)) {
    manualReadyPending = false;
    return;
  }
  queueManualReadyResponse(true);
}

void servicePendingSensorStabilization()
{
  if (calibrationOperationActive) {
    if (sensorStabilizationPending
        && millis() - sensorStabilizationStartedAt >= 1000) {
      if (!encoderExtEnabled) {
        cancelCalibrationOperation();
        return;
      }
      sensorStabilizationPending = false;
      sensorStabilized = true;
    }
    if (sensorStabilized && calibrationManualSenseSeen
        && calibrationHeadMotionObserved && !calibrationOperatorPromptSent
        && !azimuthResponsePending && digitalRead(MANUAL_TRIGGER_PIN) == LOW
        && millis() - calibrationHeadLastMotionAt >= 3000) {
      prepareCalibrationOperatorResponse();
    }
    return;
  }

  if (!azimuthCommandPending || memoryRecallCancelPending || azimuthCompletionQueued
      || !sessionActive || !CDC_connected()) {
    stopSensorStabilization();
    return;
  }

  if (movementBondClosingPending || manualMovementBondClosingPending) {
    return;
  }

  if (digitalRead(MANUAL_TRIGGER_PIN) == HIGH) {
    if (pendingAzimuthIsLocalPush) {
      queueLocalPushCancellation();
    } else if (pendingAzimuthIsMemoryRecall) {
      queueMemoryRecallCancellation();
    } else {
      queuePendingAzimuthAbort();
    }
    return;
  }

  const uint16_t actual = encoderExtLastRoundedAzimuth;
  if (!isCurrentAzimuthMechanicallyAllowed()
      || (!pendingAzimuthIsLocalPush && !isAzimuthAllowed(actual))) {
    if (pendingAzimuthIsMemoryRecall) {
      queueMemoryRecallCancellation();
    } else {
      queuePendingAzimuthAbort();
    }
    return;
  }
  if (isAzimuthTargetReached(pendingAzimuthTarget)) {
    stopSensorStabilization();
    if (!pendingAzimuthIsMemoryRecall
        && movementBondState == MovementBondState::Open
        && movementBondMotionStarted) {
      beginMovementBondCloseWait();
    }
    return;
  }

  if (!sensorStabilized) {
    digitalWrite(MOTOR_CW_PIN, LOW);
    digitalWrite(MOTOR_CCW_PIN, LOW);
    if (!sensorStabilizationPending) {
      digitalWrite(ENABLE_SENSORS_PIN, HIGH);
      sensorStabilizationStartedAt = millis();
      sensorStabilizationPending = true;
      return;
    }
    if (millis() - sensorStabilizationStartedAt < 1000) {
      return;
    }
    sensorStabilizationPending = false;
    sensorStabilized = true;
  }

  if (!pendingAzimuthIsMemoryRecall && !openMovementBond()) {
    requireCalibration();
    queuePendingAzimuthAbort();
    return;
  }

  const uint16_t currentArcPosition = actual >= TAIL_DEGREES
      ? actual - TAIL_DEGREES : actual + 360 - TAIL_DEGREES;
  const uint16_t targetArcPosition = pendingAzimuthTarget >= TAIL_DEGREES
      ? pendingAzimuthTarget - TAIL_DEGREES
      : pendingAzimuthTarget + 360 - TAIL_DEGREES;
  const uint8_t motorPin = targetArcPosition > currentArcPosition
      ? MOTOR_CW_PIN : MOTOR_CCW_PIN;
  const uint8_t inactiveMotorPin = motorPin == MOTOR_CW_PIN
      ? MOTOR_CCW_PIN : MOTOR_CW_PIN;
  digitalWrite(inactiveMotorPin, LOW);
  digitalWrite(motorPin, HIGH);
  if (!pendingAzimuthIsMemoryRecall) {
    movementBondMotionStarted = true;
  }
}

void handlePush()
{
  const bool pressed = digitalRead(PB0) == LOW;
  const uint32_t now = millis();
  if (pressed != pushRawPressed) {
    pushRawPressed = pressed;
    pushChangedAt = now;
  }
  if (pressed != pushStablePressed && now - pushChangedAt >= PUSH_DEBOUNCE_MS) {
    pushStablePressed = pressed;
    if (pressed && sessionActive && masterReady && CDC_connected()
        && !calibrationOperationActive
        && digitalRead(MANUAL_TRIGGER_PIN) == LOW
        && localTargetSelected && !azimuthCommandPending && !azimuthResponsePending
        && !readyResponsePending && !memoryResponseStage) {
      pushPending = true;
      pushPendingAzimuth = ((azimuthRequest + 5U) / 10U * 10U) % 360U;
    }
  }
  if (!pushPending) {
    return;
  }
  if (digitalRead(MANUAL_TRIGGER_PIN) == HIGH) {
    pushPending = false;
    return;
  }
  if (!sessionActive || !masterReady || !CDC_connected() || calibrationOperationActive) {
    pushPending = false;
    return;
  }
  if (azimuthCommandPending || azimuthResponsePending
      || readyResponsePending || memoryResponseStage) {
    pushPending = false;
    return;
  }

  if (!isAzimuthAllowed(pushPendingAzimuth)) {
    pushPending = false;
    return;
  }
  pendingAzimuthTarget = pushPendingAzimuth;
  for (uint8_t i = 0; i < 4; ++i) {
    pendingAzimuthCid[i] = '0';
  }
  azimuthCommandPending = true;
  azimuthTargetReached = false;
  movementBondMotionStarted = false;
  pendingAzimuthIsLocalPush = true;
  localTargetSelected = false;
  azimuthCompletionQueued = false;
  masterReady = false;
  digitalWrite(STATUS_LED_PIN, HIGH);
  pushPending = false;
}
}  // namespace

// The STMicroelectronics core starts USB CDC before setup().
void setup()
{
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);
  pinMode(MOTOR_CW_PIN, OUTPUT);
  pinMode(MOTOR_CCW_PIN, OUTPUT);
  pinMode(ENABLE_SENSORS_PIN, OUTPUT);
  pinMode(MANUAL_TRIGGER_PIN, INPUT_PULLDOWN);
  pinMode(PA0, INPUT_PULLUP);
  pinMode(PA1, INPUT_PULLUP);
  pinMode(PA6, INPUT_PULLUP);
  pinMode(PA7, INPUT_PULLUP);
  pinMode(PB0, INPUT_PULLUP);
  pushRawPressed = digitalRead(PB0) == LOW;
  pushStablePressed = pushRawPressed;
  pushChangedAt = millis();
  __HAL_RCC_TIM2_CLK_ENABLE();
  encoderTimer.Instance = TIM2;
  encoderTimer.Init.Prescaler = 0;
  encoderTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
  encoderTimer.Init.Period = 0xFFFF;
  encoderTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM_Encoder_InitTypeDef encoderConfig = {};
  encoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  encoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  encoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  encoderConfig.IC1Filter = 15;
  encoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  encoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  encoderConfig.IC2Filter = 15;
  encoderEnabled = HAL_TIM_Encoder_Init(&encoderTimer, &encoderConfig) == HAL_OK
      && HAL_TIM_Encoder_Start(&encoderTimer, TIM_CHANNEL_ALL) == HAL_OK;
  encoderPreviousCount = static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(&encoderTimer));

  __HAL_RCC_TIM3_CLK_ENABLE();
  encoderExtTimer.Instance = TIM3;
  encoderExtTimer.Init.Prescaler = 0;
  encoderExtTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
  encoderExtTimer.Init.Period = 0xFFFF;
  encoderExtTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM_Encoder_InitTypeDef encoderExtConfig = {};
  encoderExtConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  encoderExtConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  encoderExtConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  encoderExtConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  encoderExtConfig.IC1Filter = 15;
  encoderExtConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  encoderExtConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  encoderExtConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  encoderExtConfig.IC2Filter = 15;
  encoderExtEnabled = HAL_TIM_Encoder_Init(&encoderExtTimer, &encoderExtConfig) == HAL_OK
      && HAL_TIM_Encoder_Start(&encoderExtTimer, TIM_CHANNEL_ALL) == HAL_OK;
  encoderExtPreviousCount = static_cast<uint16_t>(
      __HAL_TIM_GET_COUNTER(&encoderExtTimer));
  encoderExtPositionCounts = 0;
  lastEncoderExtDiagnosticAt = millis();
  encoderDetentPhase = (digitalRead(PA0) << 1) | digitalRead(PA1);
  encoderPhase = encoderDetentPhase;
  encoderPhaseSince = millis();
  restoreMovementBondAtBoot();
  restoreCalibrationAtBoot();
}

void loop()
{
  serviceManualTrigger();
  if (memoryResponseStage && !CDC_connected()) {
    if (recallMemoryPending || recallAwaitingEncoder || pendingAzimuthIsMemoryRecall) {
      azimuthCommandPending = false;
      pendingAzimuthIsMemoryRecall = false;
      recallAwaitingEncoder = false;
      sessionActive = false;
      masterReady = false;
      digitalWrite(STATUS_LED_PIN, HIGH);
      stopSensorStabilization();
    }
    memoryResponseStage = 0;
    recallMemoryPending = false;
    recallMemoryFromTelegram = false;
    memoryRecallCancelPending = false;
    memoryReadyAfterNack = false;
  }
  if (azimuthCommandPending && !CDC_connected()) {
    azimuthCommandPending = false;
    pendingAzimuthIsLocalPush = false;
    pendingAzimuthIsMemoryRecall = false;
    azimuthCompletionQueued = false;
    memoryRecallCancelPending = false;
    memoryReadyAfterNack = false;
    azimuthResponsePending = false;
    readyResponsePending = false;
    calibrationReadyAfterDone = false;
    pushPending = false;
    sessionActive = false;
    masterReady = false;
    rxLength = 0;
    rxOverflow = false;
    digitalWrite(STATUS_LED_PIN, HIGH);
    stopSensorStabilization();
  }
  if (masterReady && !CDC_connected()) {
    masterReady = false;
    digitalWrite(STATUS_LED_PIN, HIGH);
    stopSensorStabilization();
  }
  while ((!memoryResponseStage || (memoryResponseStage == 3 && recallAwaitingEncoder))
      && !azimuthResponsePending && !readyResponsePending
      && SerialUSB.available() > 0) {
    const int received = SerialUSB.read();
    if (received == '\n') {
      handleReceivedLine();
    } else if (!rxOverflow) {
      if (rxLength < RX_LINE_CAPACITY) {
        rxLine[rxLength++] = static_cast<char>(received);
      } else {
        rxOverflow = true;
      }
    }
  }
  handlePush();
  serviceHeartbeatMonitor();
  servicePendingAzimuthResponses();
  if (azimuthCommandPending && azimuthCompletionQueued
      && !azimuthResponsePending && !readyResponsePending) {
    azimuthCommandPending = false;
    pendingAzimuthIsLocalPush = false;
    azimuthCompletionQueued = false;
    masterReady = !calibrationRequired;
    digitalWrite(STATUS_LED_PIN, masterReady ? LOW : HIGH);
  }
  serviceMemoryResponse();
  serviceHeartbeatResponse();
  serviceMovementBondNotifications();
  serviceCalibrationReadyAfterBondNotifications();
  if (encoderExtEnabled) {
    const uint16_t count = static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(&encoderExtTimer));
    const int16_t delta = static_cast<int16_t>(
        static_cast<uint16_t>(count - encoderExtPreviousCount));
    encoderExtPreviousCount = count;
    encoderExtPositionCounts += delta;
    if (delta != 0 && movementBondClosingPending) {
      movementBondLastExtMotionAt = millis();
    }
    if (delta != 0 && manualMovementBondClosingPending) {
      manualMovementBondLastMotionAt = millis();
    }
    if (delta != 0 && calibrationBondClosePending) {
      calibrationBondLastMotionAt = millis();
    }
      serviceCalibrationHeadEncoderPosition();
    if (delta != 0) {
      const int64_t positionCounts = calibrationHeadReferenceValid
          ? static_cast<int64_t>(encoderExtPositionCounts) - calibrationHeadCount
          : encoderExtPositionCounts;
      float measured;
      if (calibrationHeadReferenceValid && calibrationScaleValid) {
        double calibrated = HEAD_DEGREES
            + static_cast<double>(positionCounts) / calibrationPulsesPerDegree;
        calibrated = fmod(calibrated, 360.0);
        if (calibrated < 0.0) {
          calibrated += 360.0;
        }
        measured = static_cast<float>(calibrated);
      } else {
        const int32_t remainder = static_cast<int32_t>(
            positionCounts % ENCODER_EXT_COUNTS_PER_TURN);
        measured = calibrationHeadReferenceValid
            ? HEAD_DEGREES + remainder * ENCODER_EXT_DEGREES_PER_COUNT
            : ENCODER_EXT_TEST_REFERENCE_DEGREES
                + remainder * ENCODER_EXT_DEGREES_PER_COUNT;
        if (measured < 0.0f) {
          measured += 360.0f;
        } else if (measured >= 360.0f) {
          measured -= 360.0f;
        }
      }
      azimuthExtMeasured = measured;
      uint16_t rounded = static_cast<uint16_t>(measured + 0.5f);
      if (rounded == 360) {
        rounded = 0;
      }
      if (rounded != encoderExtLastRoundedAzimuth) {
        encoderExtLastRoundedAzimuth = rounded;
        azimuthCurrent = rounded;
        encoderExtAzimuthToReport = rounded;
        encoderExtAzimuthReportPending = true;
      }
    }

    const uint32_t now = millis();
    if (encoderExtAzimuthReportPending && sessionActive
        && (masterReady || azimuthCommandPending || manualTriggerWasHigh
            || manualReadyPending || manualAzimuthReportPending
            || (calibrationOperationActive && heartbeatIsRecent())) && CDC_connected()
        && (!manualMovementBondClosingPending || manualAzimuthReportPending)
        && !azimuthResponsePending && !readyResponsePending
        && (!memoryResponseStage || (memoryResponseStage == 3 && recallAwaitingEncoder))
        && !(azimuthCommandPending && pendingAzimuthIsLocalPush
            && encoderExtLastRoundedAzimuth == pendingAzimuthTarget)
        && now - lastEncoderExtAzimuthReportAt >= ENCODER_EXT_AZIMUTH_MIN_INTERVAL_MS) {
      char update[] = "!AZIMUTH,00000000,0000,000,NORMAL*0000\n";
      constexpr uint8_t updateLength = sizeof(update) - 1;
      if (SerialUSB.availableForWrite() >= updateLength) {
        memcpy(update + sizeof("!AZIMUTH,") - 1, sid, sizeof(sid) - 1);
        const uint8_t azimuthOffset = sizeof("!AZIMUTH,00000000,0000,") - 1;
        const uint16_t reported = encoderExtAzimuthToReport;
        update[azimuthOffset] = '0' + reported / 100;
        update[azimuthOffset + 1] = '0' + (reported / 10) % 10;
        update[azimuthOffset + 2] = '0' + reported % 10;
        const uint8_t modeOffset = sizeof("!AZIMUTH,00000000,0000,000,") - 1;
        const char* mode = manualTriggerWasHigh || manualAzimuthReportPending
            ? "MANUAL" : "NORMAL";
        memcpy(update + modeOffset, mode, sizeof("NORMAL") - 1);
        const uint8_t bodyLength = sizeof("!AZIMUTH,00000000,0000,000,NORMAL") - 1;
        const uint16_t crc = calculateCrc16(update, bodyLength);
        constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
        for (uint8_t i = 0; i < 4; ++i) {
          update[bodyLength + 1 + i] = HEX_DIGITS[(crc >> (12 - 4 * i)) & 0x0F];
        }
        if (SerialUSB.write(update) == updateLength) {
          encoderExtAzimuthReportPending = manualAzimuthReportPending
              && !manualTriggerWasHigh;
          manualAzimuthReportPending = false;
          lastEncoderExtAzimuthReportAt = now;
        }
      }
    }
    if (sessionActive && (masterReady || azimuthCommandPending
        || (calibrationOperationActive && heartbeatIsRecent())) && CDC_connected()
        && !azimuthResponsePending && !readyResponsePending && !memoryResponseStage
        && now - lastEncoderExtDiagnosticAt >= ENCODER_EXT_DIAG_INTERVAL_MS) {
      char diagnostic[] = "EXT_COUNT=+0000000000\n";
      uint32_t magnitude = static_cast<uint32_t>(encoderExtPositionCounts);
      if (encoderExtPositionCounts < 0) {
        diagnostic[10] = '-';
        magnitude = 0U - magnitude;
      }
      for (int8_t digit = 9; digit >= 0; --digit) {
        diagnostic[11 + digit] = '0' + magnitude % 10;
        magnitude /= 10;
      }
      const uint8_t diagnosticLength = sizeof(diagnostic) - 1;
      if (SerialUSB.availableForWrite() >= diagnosticLength) {
        SerialUSB.write(reinterpret_cast<const uint8_t*>(diagnostic), diagnosticLength);
        lastEncoderExtDiagnosticAt = now;
      }
    }
  }
  servicePendingSensorStabilization();
  serviceMovementBondCloseWait();
  serviceManualMovementBondCloseWait();
  serviceManualReadyAfterBond();
  serviceCalibrationBondCloseWait();
  if (azimuthCommandPending && !azimuthCompletionQueued && encoderExtEnabled
      && sessionActive && CDC_connected()
      && digitalRead(MANUAL_TRIGGER_PIN) == LOW
      && isAzimuthTargetReached(pendingAzimuthTarget)
      && !movementBondClosingPending
      && !azimuthResponsePending && !readyResponsePending && !memoryResponseStage) {
    azimuthCurrent = encoderExtLastRoundedAzimuth;
    char response[] = "!AZIMUTH,00000000,0000,000,NORMAL*0000\n";
    memcpy(response + sizeof("!AZIMUTH,") - 1, sid, sizeof(sid) - 1);
    if (!pendingAzimuthIsLocalPush) {
      const uint8_t cidOffset = sizeof("!AZIMUTH,00000000,") - 1;
      for (uint8_t i = 0; i < 4; ++i) {
        response[cidOffset + i] = pendingAzimuthCid[i];
      }
    } else {
      encoderExtAzimuthReportPending = false;
      lastEncoderExtAzimuthReportAt = millis();
    }
    const uint8_t azimuthOffset = sizeof("!AZIMUTH,00000000,0000,") - 1;
    response[azimuthOffset] = '0' + azimuthCurrent / 100;
    response[azimuthOffset + 1] = '0' + (azimuthCurrent / 10) % 10;
    response[azimuthOffset + 2] = '0' + azimuthCurrent % 10;
    const uint8_t bodyLength = sizeof("!AZIMUTH,00000000,0000,000,NORMAL") - 1;
    uint16_t crc = calculateCrc16(response, bodyLength);
    constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
    for (uint8_t i = 0; i < 4; ++i) {
      response[bodyLength + 1 + i] = HEX_DIGITS[(crc >> (12 - 4 * i)) & 0x0F];
    }
    memcpy(pendingAzimuthResponse, response, sizeof(response));
    azimuthResponseLength = sizeof(response) - 1;

    char ready[] = "!READY,00000000,1,1,1*0000\n";
    memcpy(ready + sizeof("!READY,") - 1, sid, sizeof(sid) - 1);
    ready[sizeof("!READY,00000000,") - 1] = calibrationRequired ? '0' : '1';
    ready[sizeof("!READY,00000000,1,1,") - 1] = calibrationRequired ? '0' : '1';
    const uint8_t readyBodyLength = sizeof("!READY,00000000,1,1,1") - 1;
    crc = calculateCrc16(ready, readyBodyLength);
    for (uint8_t i = 0; i < 4; ++i) {
      ready[readyBodyLength + 1 + i] = HEX_DIGITS[(crc >> (12 - 4 * i)) & 0x0F];
    }
    memcpy(pendingReadyResponse, ready, sizeof(ready));
    azimuthResponseOffset = 0;
    readyResponseOffset = 0;
    azimuthResponsePending = true;
    readyResponsePending = true;
    azimuthCompletionQueued = true;
  }
  if (!encoderEnabled) {
    return;
  }
  const uint16_t count = static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(&encoderTimer));
  const int16_t delta = static_cast<int16_t>(
      static_cast<uint16_t>(count - encoderPreviousCount));
  encoderPreviousCount = count;
  const uint8_t phase = (digitalRead(PA0) << 1) | digitalRead(PA1);
  if (phase != encoderPhase) {
    encoderPhase = phase;
    encoderPhaseSince = millis();
  }
  encoderCountAccumulator += delta;
  if (manualTriggerWasHigh || manualMovementBondClosingPending
      || !masterReady || movementBondState != MovementBondState::Closed) {
    encoderCountAccumulator = 0;
  } else if (phase == encoderDetentPhase
      && millis() - encoderPhaseSince >= ENCODER_DETENT_STABLE_MS) {
    while (encoderCountAccumulator >= ENCODER_COUNTS_PER_STEP
        || encoderCountAccumulator <= -ENCODER_COUNTS_PER_STEP) {
      const int direction = encoderCountAccumulator > 0 ? 1 : -1;
      encoderCountAccumulator -= direction * ENCODER_COUNTS_PER_STEP;
      const uint16_t nextRequest = direction > 0
          ? ((azimuthRequest / 10U + 1U) * 10U) % 360U
          : (azimuthRequest % 10U == 0U
              ? (azimuthRequest + 350U) % 360U
              : azimuthRequest / 10U * 10U);
      if (isAzimuthAllowed(nextRequest) && nextRequest != azimuthRequest) {
        azimuthRequest = nextRequest;
        localTargetSelected = true;
        previewPending = azimuthRequest != lastPreviewAzimuth;
      }
    }
    encoderCountAccumulator = 0;
  }
  if (azimuthResponsePending || readyResponsePending || memoryResponseStage || !previewPending
      || !sessionActive || !masterReady || !CDC_connected()) {
    return;
  }
  const uint32_t now = millis();
  if (previewSentOnce && now - lastPreviewSentAt < PREVIEW_MIN_INTERVAL_MS) {
    return;
  }
  char preview[] = "!AZIMUTH_PREVIEW,00000000,000,NORMAL*0000\n";
  if (SerialUSB.availableForWrite() < static_cast<int>(sizeof(preview) - 1)) {
    return;
  }
  memcpy(preview + sizeof("!AZIMUTH_PREVIEW,") - 1, sid, sizeof(sid) - 1);
  const uint8_t azimuthOffset = sizeof("!AZIMUTH_PREVIEW,00000000,") - 1;
  preview[azimuthOffset] = '0' + azimuthRequest / 100;
  preview[azimuthOffset + 1] = '0' + (azimuthRequest / 10) % 10;
  preview[azimuthOffset + 2] = '0' + azimuthRequest % 10;
  const uint8_t bodyLength = sizeof("!AZIMUTH_PREVIEW,00000000,000,NORMAL") - 1;
  const uint16_t crc = calculateCrc16(preview, bodyLength);
  constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
  for (uint8_t i = 0; i < 4; ++i) {
    preview[bodyLength + 1 + i] = HEX_DIGITS[(crc >> (12 - 4 * i)) & 0x0F];
  }
  if (SerialUSB.write(preview) == sizeof(preview) - 1) {
    lastPreviewAzimuth = azimuthRequest;
    lastPreviewSentAt = now;
    previewSentOnce = true;
    previewPending = false;
  }
}
