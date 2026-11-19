#include <string.h>

#include "codefree_comm.h"
#include "codefree_comm_config.h"
#include "mock_codefree_comm_external.h"
#include "unity.h"

/* ---------------------------------------------------------------------------
 * Fake port
 *
 * The port functions are stubbed with callbacks, so the library runs against
 * a simulated time base and a simulated bus:
 * - blocking port: a transfer completes inside the send call (like a polled
 *   HAL port)
 * - async port: a transfer completes later, when the test calls
 *   port_completeWrite() / port_completeRead() (like a driver interrupt)
 * - the next send calls can be rejected (port returns false)
 * - the fake chip reproduces the MLX80339 quirk: a read that starts in the
 *   same millisecond as the end of a write gets a response whose CRC is
 *   computed over the write address byte
 * ------------------------------------------------------------------------- */

#define SLAVE_ADDR 0x7Eu
#define MAX_LOGGED_WRITES 16u

/* Status frames captured from an MLX80339 at 0x7E (relative speed command) */
static const uint8_t FRAME_OK[8] = {0x34, 0x00, 0x22, 0xD2,
                                    0x63, 0x01, 0x01, 0x73};
/* Same command, CRC computed by the chip over the write address byte
 * (2*addr+0) because the read directly followed a write */
static const uint8_t FRAME_CRC_OVER_WRITE_ADDR[8] = {0x2B, 0x00, 0x05, 0xD2,
                                                     0x69, 0x01, 0x01, 0x94};

static uint32_t fakeTimeMs;
static bool initDriverResult;
static bool portBlocking;
static uint32_t rejectNextWrites;
static uint32_t rejectNextReads;
static const uint8_t *nextResponse;

static uint32_t nWrites;
static uint32_t nReads;
static uint32_t nWriteRejects;
static uint32_t nReadRejects;
static uint32_t nAborts;
static uint32_t writeTimeMs[MAX_LOGGED_WRITES];
static uint32_t lastReadTimeMs;
static bool writeEnded;
static uint32_t lastWriteEndMs;
static CodefreeCommWriteMessage_t writeLog[MAX_LOGGED_WRITES];
static CodefreeCommReadMessage_t *portReadBuffer;

static uint32_t nCbReadComplete;
static uint32_t nCbTimeout;
static uint32_t nCbCrcOk;
static CodefreeCommReadMessage_t *lastCbMsg;

static CodefreeCommReadMessage_t appReadMsg;

static uint32_t fake_getTimeMs(int cmock_num_calls) {
  (void)cmock_num_calls;
  return fakeTimeMs;
}

static bool fake_initDriver(int cmock_num_calls) {
  (void)cmock_num_calls;
  return initDriverResult;
}

static void fake_abortTransfer(int cmock_num_calls) {
  (void)cmock_num_calls;
  nAborts++;
}

static bool fake_sendWriteMessage(CodefreeCommWriteMessage_t *msg,
                                  int cmock_num_calls) {
  (void)cmock_num_calls;
  if (rejectNextWrites > 0u) {
    rejectNextWrites--;
    nWriteRejects++;
    return false;
  }
  if (nWrites < MAX_LOGGED_WRITES) {
    writeLog[nWrites] = *msg;
    writeTimeMs[nWrites] = fakeTimeMs;
  }
  nWrites++;
  if (portBlocking) {
    writeEnded = true;
    lastWriteEndMs = fakeTimeMs;
    codefree_comm_setStatus(CODEFREE_COMM_STATUS_IDLE);
  }
  return true;
}

static bool fake_sendReadRequest(CodefreeCommReadMessage_t *msg,
                                 int cmock_num_calls) {
  (void)cmock_num_calls;
  if (rejectNextReads > 0u) {
    rejectNextReads--;
    nReadRejects++;
    return false;
  }
  portReadBuffer = msg;
  lastReadTimeMs = fakeTimeMs;
  nReads++;
  if (portBlocking) {
    bool tooSoon = writeEnded && (fakeTimeMs == lastWriteEndMs);
    memcpy(msg->readResponse,
           tooSoon ? FRAME_CRC_OVER_WRITE_ADDR : nextResponse, 8);
    codefree_comm_handleRxComplete();
  }
  return true;
}

static void app_callback(CodefreeCommReadMessage_t *msg,
                         CodefreeCommEvent_t event) {
  lastCbMsg = msg;
  if (event == CODEFREE_COMM_EVENT_READ_COMPLETE) {
    nCbReadComplete++;
    if (msg->crcCorrect) {
      nCbCrcOk++;
    }
  } else if (event == CODEFREE_COMM_EVENT_TIMEOUT) {
    nCbTimeout++;
  }
}

/** Install the fake port and initialize the library. */
static void fakePort_init(bool blocking) {
  fakeTimeMs = 1000u;
  initDriverResult = true;
  portBlocking = blocking;
  rejectNextWrites = 0u;
  rejectNextReads = 0u;
  nextResponse = FRAME_OK;
  nWrites = nReads = nWriteRejects = nReadRejects = nAborts = 0u;
  writeEnded = false;
  nCbReadComplete = nCbTimeout = nCbCrcOk = 0u;
  lastCbMsg = NULL;
  portReadBuffer = NULL;
  memset(writeLog, 0, sizeof(writeLog));
  memset(&appReadMsg, 0, sizeof(appReadMsg));
  appReadMsg.slaveAddress = SLAVE_ADDR;

  codefree_comm_ext_getTimeMs_Stub(fake_getTimeMs);
  codefree_comm_ext_initDriver_Stub(fake_initDriver);
  codefree_comm_ext_abortTransfer_Stub(fake_abortTransfer);
  codefree_comm_ext_sendWriteMessage_Stub(fake_sendWriteMessage);
  codefree_comm_ext_sendReadRequest_Stub(fake_sendReadRequest);

  TEST_ASSERT_TRUE(codefree_comm_init(app_callback));
}

/** Async port: the write on the bus completes (driver interrupt). */
static void port_completeWrite(void) {
  writeEnded = true;
  lastWriteEndMs = fakeTimeMs;
  codefree_comm_setStatus(CODEFREE_COMM_STATUS_IDLE);
}

/** Async port: the read on the bus completes with the given response. */
static void port_completeRead(const uint8_t *response) {
  TEST_ASSERT_NOT_NULL(portReadBuffer);
  memcpy(portReadBuffer->readResponse, response, 8);
  codefree_comm_handleRxComplete();
}

/** Let time pass in 1 ms steps, calling the background handler each step. */
static void runMs(uint32_t ms) {
  for (uint32_t i = 0u; i < ms; i++) {
    fakeTimeMs++;
    codefree_comm_backgroundHandler();
  }
}

/** Reference CRC of a write frame: one's complement of the end-around-carry
 * sum of the address byte (2*addr+0), 0x00, command and the 4 data bytes. */
static uint8_t expectedWriteCrc(const CodefreeCommWriteMessage_t *msg) {
  uint32_t sum = ((uint32_t)msg->slaveAddress << 1) + msg->commandId +
                 msg->data[0] + msg->data[1] + msg->data[2] + msg->data[3];
  while (sum > 0xFFu) {
    sum = (sum >> 8) + (sum & 0xFFu);
  }
  return (uint8_t)(0xFFu - sum);
}

void setUp(void) {}

void tearDown(void) {}

/* ---------------------------------------------------------------------------
 * Initialization
 * ------------------------------------------------------------------------- */

void test_codefree_comm_init_should_ReturnTrue_WhenDriverInitSucceeds(void) {
  codefree_comm_ext_initDriver_ExpectAndReturn(true);
  bool result = codefree_comm_init(app_callback);

  TEST_ASSERT_TRUE(result);
  TEST_ASSERT_EQUAL(CODEFREE_COMM_STATUS_IDLE, codefree_comm_getStatus());
}

void test_codefree_comm_init_should_StayInactive_WhenDriverInitFails(void) {
  fakePort_init(true); /* a previous successful init */

  initDriverResult = false;
  TEST_ASSERT_FALSE(codefree_comm_init(app_callback));
  TEST_ASSERT_EQUAL(CODEFREE_COMM_STATUS_INACTIVE, codefree_comm_getStatus());

  codefree_comm_SetSpeedRelative(SLAVE_ADDR, 30u);
  TEST_ASSERT_FALSE(codefree_comm_executeRead(&appReadMsg));
  runMs(10u);
  TEST_ASSERT_EQUAL_UINT32(0u, nWrites);
  TEST_ASSERT_EQUAL_UINT32(0u, nReads);
}

/* ---------------------------------------------------------------------------
 * Write frames
 * ------------------------------------------------------------------------- */

void test_SetSpeedRelative_should_SendFrameWithCorrectCrc(void) {
  fakePort_init(true);

  codefree_comm_SetSpeedRelative(SLAVE_ADDR, 30u);

  TEST_ASSERT_EQUAL_UINT32(1u, nWrites);
  TEST_ASSERT_EQUAL_HEX8(SLAVE_ADDR, writeLog[0].slaveAddress);
  TEST_ASSERT_EQUAL_HEX8(CODEFREE_COMM_COMMAND_SETSPEEDRELATIVE,
                         writeLog[0].commandId);
  const uint8_t data[4] = {30u, 0u, 0u, 0u};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(data, writeLog[0].data, 4);
  /* 0xFC + 0x01 + 0x1E = 0x11B -> 0x1C -> CRC 0xE3 */
  TEST_ASSERT_EQUAL_HEX8(0xE3u, writeLog[0].crc);
}

void test_SetSpeedAbsolute_should_SendSpeedMsbFirst(void) {
  fakePort_init(true);

  codefree_comm_SetSpeedAbsolute(SLAVE_ADDR, 0x1234u);

  TEST_ASSERT_EQUAL_HEX8(CODEFREE_COMM_COMMAND_SETSPEEDABSOLUTE,
                         writeLog[0].commandId);
  const uint8_t data[4] = {0x12u, 0x34u, 0u, 0u};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(data, writeLog[0].data, 4);
  TEST_ASSERT_EQUAL_HEX8(expectedWriteCrc(&writeLog[0]), writeLog[0].crc);
}

void test_SetPositionRelative_should_SendPositionAndSpeed(void) {
  fakePort_init(true);

  codefree_comm_SetPositionRelative(SLAVE_ADDR, 0x40u, 100u);

  TEST_ASSERT_EQUAL_HEX8(CODEFREE_COMM_COMMAND_SETPOSITIONRELATIVE,
                         writeLog[0].commandId);
  const uint8_t data[4] = {0x40u, 0u, 100u, 0u};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(data, writeLog[0].data, 4);
  TEST_ASSERT_EQUAL_HEX8(expectedWriteCrc(&writeLog[0]), writeLog[0].crc);
}

void test_SetPositionAbsolute_should_SendPositionMsbFirstAndSpeed(void) {
  fakePort_init(true);

  codefree_comm_SetPositionAbsolute(SLAVE_ADDR, 0xABCDu, 50u);

  TEST_ASSERT_EQUAL_HEX8(CODEFREE_COMM_COMMAND_SETPOSITIONABSOLUTE,
                         writeLog[0].commandId);
  const uint8_t data[4] = {0xABu, 0xCDu, 50u, 0u};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(data, writeLog[0].data, 4);
  TEST_ASSERT_EQUAL_HEX8(expectedWriteCrc(&writeLog[0]), writeLog[0].crc);
}

void test_SetActiveCommand_should_SendKeyBytesAndCommandId(void) {
  fakePort_init(true);

  codefree_comm_SetActiveCommand(SLAVE_ADDR,
                                 CODEFREE_COMM_COMMAND_SETSPEEDABSOLUTE);

  TEST_ASSERT_EQUAL_HEX8(CODEFREE_COMM_COMMAND_SETACTIVECOMMAND,
                         writeLog[0].commandId);
  const uint8_t data[4] = {0x45u, 0x67u, 0x78u,
                           CODEFREE_COMM_COMMAND_SETSPEEDABSOLUTE};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(data, writeLog[0].data, 4);
  TEST_ASSERT_EQUAL_HEX8(expectedWriteCrc(&writeLog[0]), writeLog[0].crc);
}

/* ---------------------------------------------------------------------------
 * Read frames and CRC
 * ------------------------------------------------------------------------- */

void test_checkReadCrc_should_AcceptValidFrame(void) {
  CodefreeCommReadMessage_t msg = {0};
  msg.slaveAddress = SLAVE_ADDR;
  memcpy(msg.readResponse, FRAME_OK, 8);

  TEST_ASSERT_TRUE(codefree_comm_checkReadCrc(&msg));
  TEST_ASSERT_TRUE(msg.crcCorrect);
}

void test_checkReadCrc_should_RejectFrameWithCrcOverWriteAddress(void) {
  CodefreeCommReadMessage_t msg = {0};
  msg.slaveAddress = SLAVE_ADDR;
  memcpy(msg.readResponse, FRAME_CRC_OVER_WRITE_ADDR, 8);

  TEST_ASSERT_FALSE(codefree_comm_checkReadCrc(&msg));
  TEST_ASSERT_FALSE(msg.crcCorrect);
}

void test_executeRead_should_CopyResponseAndReportReadComplete(void) {
  fakePort_init(true);

  TEST_ASSERT_TRUE(codefree_comm_executeRead(&appReadMsg));

  TEST_ASSERT_EQUAL_UINT32(1u, nReads);
  TEST_ASSERT_EQUAL_UINT32(1u, nCbReadComplete);
  TEST_ASSERT_EQUAL_PTR(&appReadMsg, lastCbMsg);
  TEST_ASSERT_TRUE(appReadMsg.crcCorrect);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(FRAME_OK, appReadMsg.readResponse, 8);
  TEST_ASSERT_EQUAL_HEX8(0x34u, appReadMsg.payload.relativeSpeed.motorSpeed);
  TEST_ASSERT_EQUAL_HEX8(CODEFREE_COMM_COMMAND_SETSPEEDRELATIVE,
                         appReadMsg.commandId);
  TEST_ASSERT_EQUAL(CODEFREE_COMM_STATUS_IDLE, codefree_comm_getStatus());
}

void test_executeRead_should_ReportCrcError(void) {
  fakePort_init(true);
  nextResponse = FRAME_CRC_OVER_WRITE_ADDR;

  TEST_ASSERT_TRUE(codefree_comm_executeRead(&appReadMsg));

  TEST_ASSERT_EQUAL_UINT32(1u, nCbReadComplete);
  TEST_ASSERT_EQUAL_UINT32(0u, nCbCrcOk);
  TEST_ASSERT_FALSE(appReadMsg.crcCorrect);
}

void test_executeRead_should_RejectNullAndSecondRead(void) {
  fakePort_init(false);

  TEST_ASSERT_FALSE(codefree_comm_executeRead(NULL));

  TEST_ASSERT_TRUE(codefree_comm_executeRead(&appReadMsg)); /* on the bus */
  TEST_ASSERT_FALSE(codefree_comm_executeRead(&appReadMsg));

  port_completeRead(FRAME_OK);
  codefree_comm_SetSpeedRelative(SLAVE_ADDR, 1u);           /* pending (gap) */
  TEST_ASSERT_TRUE(codefree_comm_executeRead(&appReadMsg)); /* pending */
  TEST_ASSERT_FALSE(codefree_comm_executeRead(&appReadMsg));
}

/* ---------------------------------------------------------------------------
 * Minimum inter-frame gap
 * ------------------------------------------------------------------------- */

void test_readAfterWrite_should_WaitForMinFrameGap(void) {
  fakePort_init(true);

  codefree_comm_SetSpeedRelative(SLAVE_ADDR, 30u);
  TEST_ASSERT_TRUE(codefree_comm_executeRead(&appReadMsg)); /* accepted */
  TEST_ASSERT_EQUAL_UINT32(0u, nReads);                     /* not sent */

  runMs(CODEFREE_COMM_MIN_FRAME_GAP_MS - 1u);
  TEST_ASSERT_EQUAL_UINT32(0u, nReads);

  runMs(1u);
  TEST_ASSERT_EQUAL_UINT32(1u, nReads);
  TEST_ASSERT_EQUAL_UINT32(1u, nCbCrcOk);
}

void test_minFrameGap_should_BeMeasuredFromEndOfFrame(void) {
  fakePort_init(false);

  codefree_comm_SetSpeedRelative(SLAVE_ADDR, 30u); /* on the bus */
  TEST_ASSERT_TRUE(codefree_comm_executeRead(&appReadMsg));
  runMs(5u); /* long write transfer */
  TEST_ASSERT_EQUAL_UINT32(0u, nReads);

  port_completeWrite();
  uint32_t frameEndMs = fakeTimeMs;
  runMs(CODEFREE_COMM_MIN_FRAME_GAP_MS);

  TEST_ASSERT_EQUAL_UINT32(1u, nReads);
  TEST_ASSERT_EQUAL_UINT32(frameEndMs + CODEFREE_COMM_MIN_FRAME_GAP_MS,
                           lastReadTimeMs);
}

void test_setStatusIdleWhileIdle_should_NotRestartGap(void) {
  fakePort_init(true);

  codefree_comm_SetSpeedRelative(SLAVE_ADDR, 1u);
  runMs(CODEFREE_COMM_MIN_FRAME_GAP_MS);
  codefree_comm_setStatus(CODEFREE_COMM_STATUS_IDLE); /* no frame ended */

  codefree_comm_SetSpeedRelative(SLAVE_ADDR, 2u);
  TEST_ASSERT_EQUAL_UINT32(2u, nWrites); /* sent at once */
}

/* ---------------------------------------------------------------------------
 * Frame scheduling
 * ------------------------------------------------------------------------- */

void test_writeDuringRead_should_StayPending_AndLatestWins(void) {
  fakePort_init(false);

  TEST_ASSERT_TRUE(codefree_comm_executeRead(&appReadMsg)); /* on the bus */
  codefree_comm_SetSpeedRelative(SLAVE_ADDR, 10u);
  codefree_comm_SetSpeedRelative(SLAVE_ADDR, 20u); /* replaces 10 */
  TEST_ASSERT_TRUE(codefree_comm_isWritePending());
  TEST_ASSERT_EQUAL_UINT32(0u, nWrites);

  port_completeRead(FRAME_OK);
  runMs(CODEFREE_COMM_MIN_FRAME_GAP_MS);

  TEST_ASSERT_EQUAL_UINT32(1u, nWrites);
  TEST_ASSERT_EQUAL_HEX8(20u, writeLog[0].data[0]);
  TEST_ASSERT_EQUAL_HEX8(expectedWriteCrc(&writeLog[0]), writeLog[0].crc);
  TEST_ASSERT_FALSE(codefree_comm_isWritePending());
}

void test_pendingWriteAndRead_should_Alternate(void) {
  fakePort_init(false);

  /* Last frame was a read -> a pending write goes first */
  TEST_ASSERT_TRUE(codefree_comm_executeRead(&appReadMsg));
  port_completeRead(FRAME_OK);
  codefree_comm_SetSpeedRelative(SLAVE_ADDR, 5u);
  TEST_ASSERT_TRUE(codefree_comm_executeRead(&appReadMsg));
  runMs(CODEFREE_COMM_MIN_FRAME_GAP_MS);
  TEST_ASSERT_EQUAL_UINT32(1u, nWrites);
  TEST_ASSERT_EQUAL_UINT32(1u, nReads);

  /* Last frame was a write -> the pending read goes first */
  codefree_comm_SetSpeedRelative(SLAVE_ADDR, 6u);
  port_completeWrite();
  runMs(CODEFREE_COMM_MIN_FRAME_GAP_MS);
  TEST_ASSERT_EQUAL_UINT32(2u, nReads);
  TEST_ASSERT_EQUAL_UINT32(1u, nWrites);

  port_completeRead(FRAME_OK);
  runMs(CODEFREE_COMM_MIN_FRAME_GAP_MS);
  TEST_ASSERT_EQUAL_UINT32(2u, nWrites);
  TEST_ASSERT_EQUAL_HEX8(6u, writeLog[1].data[0]);
}

void test_reads_should_NotStarve_WhenWritesAreFasterThanGap(void) {
  fakePort_init(true);
  uint32_t requested = 0u;

  for (uint32_t t = 0u; t < 1000u; t++) {
    codefree_comm_SetSpeedRelative(SLAVE_ADDR, (uint8_t)t);
    if ((t % 100u) == 0u) {
      TEST_ASSERT_TRUE(codefree_comm_executeRead(&appReadMsg));
      requested++;
    }
    runMs(1u);
  }

  TEST_ASSERT_EQUAL_UINT32(10u, requested);
  TEST_ASSERT_EQUAL_UINT32(requested, nReads);
  TEST_ASSERT_EQUAL_UINT32(requested, nCbCrcOk);
  TEST_ASSERT_GREATER_THAN_UINT32(400u, nWrites);
}

void test_rampScenario_should_DeliverEveryReadWithValidCrc(void) {
  /* Write every 20 ms, read every 500 ms, both aligned: each read is
   * requested right after a write (the case that produced CRC errors). */
  fakePort_init(true);
  uint32_t requested = 0u;

  for (uint32_t t = 0u; t < 6000u; t++) {
    if ((t % 20u) == 0u) {
      codefree_comm_SetSpeedRelative(SLAVE_ADDR, (uint8_t)(t / 30u));
    }
    if ((t % 500u) == 0u) {
      TEST_ASSERT_TRUE(codefree_comm_executeRead(&appReadMsg));
      requested++;
    }
    runMs(1u);
  }

  TEST_ASSERT_EQUAL_UINT32(300u, nWrites);
  TEST_ASSERT_EQUAL_UINT32(requested, nReads);
  TEST_ASSERT_EQUAL_UINT32(requested, nCbCrcOk);
}

/* ---------------------------------------------------------------------------
 * Response timeout
 * ------------------------------------------------------------------------- */

void test_timeout_should_AbortAndReportTimeout(void) {
  fakePort_init(false);

  TEST_ASSERT_TRUE(codefree_comm_executeRead(&appReadMsg)); /* no answer */
  runMs(CODEFREE_COMM_RECEIVE_TIMEOUT_MS - 1u);
  TEST_ASSERT_EQUAL_UINT32(0u, nCbTimeout);
  TEST_ASSERT_EQUAL_UINT32(0u, nAborts);

  runMs(1u);
  TEST_ASSERT_EQUAL_UINT32(1u, nAborts);
  TEST_ASSERT_EQUAL_UINT32(1u, nCbTimeout);
  TEST_ASSERT_EQUAL_PTR(&appReadMsg, lastCbMsg);
  TEST_ASSERT_FALSE(appReadMsg.crcCorrect);
  TEST_ASSERT_EQUAL(CODEFREE_COMM_STATUS_IDLE, codefree_comm_getStatus());
}

void test_timeout_should_AlsoCoverReceivingState(void) {
  /* UART port: after the request byte is sent, the status is RECEIVING. */
  fakePort_init(false);

  TEST_ASSERT_TRUE(codefree_comm_executeRead(&appReadMsg));
  codefree_comm_setStatus(CODEFREE_COMM_STATUS_RECEIVING);
  runMs(CODEFREE_COMM_RECEIVE_TIMEOUT_MS);

  TEST_ASSERT_EQUAL_UINT32(1u, nAborts);
  TEST_ASSERT_EQUAL_UINT32(1u, nCbTimeout);
  TEST_ASSERT_EQUAL(CODEFREE_COMM_STATUS_IDLE, codefree_comm_getStatus());
}

void test_lateResponseAfterTimeout_should_BeIgnored(void) {
  fakePort_init(false);

  TEST_ASSERT_TRUE(codefree_comm_executeRead(&appReadMsg));
  runMs(CODEFREE_COMM_RECEIVE_TIMEOUT_MS);
  TEST_ASSERT_EQUAL_UINT32(1u, nCbTimeout);

  port_completeRead(FRAME_OK); /* arrives too late */
  TEST_ASSERT_EQUAL_UINT32(0u, nCbReadComplete);

  /* The library is usable again */
  runMs(CODEFREE_COMM_MIN_FRAME_GAP_MS);
  TEST_ASSERT_TRUE(codefree_comm_executeRead(&appReadMsg));
  TEST_ASSERT_EQUAL_UINT32(2u, nReads);
}

/* ---------------------------------------------------------------------------
 * Port rejects a frame (returns false)
 * ------------------------------------------------------------------------- */

void test_rejectedWrite_should_StayPending_AndBeRetriedAfterGap(void) {
  fakePort_init(true);
  rejectNextWrites = 1u;

  codefree_comm_SetSpeedRelative(SLAVE_ADDR, 42u);
  TEST_ASSERT_EQUAL_UINT32(1u, nWriteRejects);
  TEST_ASSERT_EQUAL_UINT32(0u, nWrites);
  TEST_ASSERT_TRUE(codefree_comm_isWritePending());

  runMs(CODEFREE_COMM_MIN_FRAME_GAP_MS);
  TEST_ASSERT_EQUAL_UINT32(1u, nWrites);
  TEST_ASSERT_EQUAL_HEX8(42u, writeLog[0].data[0]);
  TEST_ASSERT_FALSE(codefree_comm_isWritePending());
}

void test_rejectedRead_should_StayPending_AndComplete(void) {
  fakePort_init(true);
  rejectNextReads = 1u;

  TEST_ASSERT_TRUE(codefree_comm_executeRead(&appReadMsg));
  TEST_ASSERT_EQUAL_UINT32(1u, nReadRejects);
  TEST_ASSERT_EQUAL_UINT32(0u, nCbReadComplete);

  runMs(CODEFREE_COMM_MIN_FRAME_GAP_MS);
  TEST_ASSERT_EQUAL_UINT32(1u, nReads);
  TEST_ASSERT_EQUAL_UINT32(1u, nCbCrcOk);
  TEST_ASSERT_EQUAL_UINT32(0u, nCbTimeout);
}

/* ---------------------------------------------------------------------------
 * Known limitations (documented, not yet fixed)
 * ------------------------------------------------------------------------- */

void test_commandsToDifferentSlaves_should_AllBeSent(void) {
  TEST_IGNORE_MESSAGE("Known limitation: one pending write slot, a pending "
                      "command to another slave is replaced");

  fakePort_init(false);
  TEST_ASSERT_TRUE(codefree_comm_executeRead(&appReadMsg)); /* bus busy */
  codefree_comm_SetSpeedRelative(0x7Du, 10u);
  codefree_comm_SetSpeedRelative(0x7Eu, 20u);
  port_completeRead(FRAME_OK);
  runMs(10u * CODEFREE_COMM_MIN_FRAME_GAP_MS);
  TEST_ASSERT_EQUAL_UINT32(2u, nWrites);
}
