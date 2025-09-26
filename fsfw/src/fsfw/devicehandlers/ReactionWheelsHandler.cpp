#include "ReactionWheelsHandler.h"

#include <cmath>
#include <cstring>
#include <vector>

#include "RwEvents.h"
#include "RwProtocol.h"
#include "fsfw/action/ActionHelper.h"
#include "fsfw/datapoollocal/ProvidesDataPoolSubscriptionIF.h"
#include "fsfw/devicehandlers/DeviceHandlerIF.h"
#include "fsfw/ipc/MessageQueueIF.h"
#include "fsfw/ipc/MutexIF.h"
#include "fsfw/parameters/ParameterMessage.h"
#include "fsfw/returnvalues/returnvalue.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"
#include "fsfw/timemanager/Clock.h"


/*
 * ReactionWheelsHandler.cpp - Device handler for Reaction Wheels
 *
 *  Reaction Wheels Handler that uses the DeviceHandlerBase. Sends torque/speed
 *  commands and polls STATUS periodically. Publishes typed HK to the local
 *  data pool and emits events when triggered.
 *
 * Features:
 *  - Uses RwProtocol to pack/unpack messages (big-endian, CRC-16)
 *  - Implements DHB state machine: doStartUp()/doShutDown(), 
 *    buildNormalDeviceCommand(), scanForReply(), interpretDeviceReply(), 
 *  - Maps replies to LocalDataPoolManager and trigger RwEvent
 *  - Provide on-demand STATUS via TC
 *
 * Notes:
 *  - Timeouts/thresholds in RwConfig.h
 *  - Event IDs in RwEvents.h.
 *  - Uses RwProtocol.h building messages.
 *  - Commands: SET_MODE (ON/OFF/NORMAL), SET_SPEED [rpm], SET_TORQUE [mNm], REQ_STATUS.
 *  - HK: speed [rpm], torque [mNm], running [bool], flags [bitfield], error counters.
 *  - Parameter service: MAX_RPM [rpm], POLL_DIVIDER.
 *  - Debug via RW_VERBOSE)
 * 
 *  - Joest Homann
 */



// -------- Construction -------------------------------------------------------
// Set up RX ring buffer and enable receive-size FIFO
ReactionWheelsHandler::ReactionWheelsHandler(object_id_t objectId, object_id_t comIF,
                                             CookieIF* cookie)
    : DeviceHandlerBase(objectId, comIF, cookie) {
  rxRing.setToUseReceiveSizeFIFO(/*fifoDepth*/ 8);
  (void)rxRing.initialize();                        
}

// -------- Mode Handling: STARTUP --------------------------------
// Start startup sequence
void ReactionWheelsHandler::doStartUp() {
  if (!startingUp_) {
    startingUp_ = true;                 // Enter startup phase
    startupSent_ = false;               // No STATUS sent yet
    startupRetriesDone_ = 0;            // Init retry counter
    Clock::getUptime(&startupT0_);      // Get t0 for startup window
    startupLastTxMs_ = startupT0_;      // Last TX time = t0
    statusAwaitCnt = -1;                // not waiting for a reply
    (void)drainRxNow();                 // drain buffer before starting
  }
}

// -------- Mode Handling: SHUTDOWN -------------------------------
// Start shutdown sequence 
void ReactionWheelsHandler::doShutDown() {
  if (!shuttingDown_) {
    shuttingDown_ = true;               // Enter shutdown phase
    stopSent_ = false;                  // STOP not sent yet
    stopRetriesDone_ = 0;               // Init retry counter
    Clock::getUptime(&shutdownT0_);     // Get t0 for shutdown window
    lastStopTxMs_ = shutdownT0_;        // Last STOP TX time = t0
  }
}

// -------- Mode change ---------------------------------------------------
// React to mode transitions and adjust polling behavior
void ReactionWheelsHandler::modeChanged() {
  Mode_t m{};
  Submode_t s{};
  this->getMode(&m, &s);
  sif::info << "ReactionWheelsHandler: modeChanged -> " << static_cast<int>(m)
            << " (sub=" << static_cast<int>(s) << ")" << std::endl;

  if (m == MODE_OFF) {
    sif::info << "ReactionWheelsHandler: Mode OFF" << std::endl;
  }

  if (m == MODE_ON) {
    sif::info << "ReactionWheelsHandler: Mode ON" << std::endl;
    // No periodic polling in MODE_ON
  }

  if (m == MODE_NORMAL) {
    sif::info << "ReactionWheelsHandler: Mode NORMAL" << std::endl;
    (void)drainRxNow();                // clear RX before NORMAL
    statusPollCnt = statusPollDivider; // force immediate first poll
  }
}


// -------- HK subscription ----------------------------------------------------
// Toggle periodic HK via PUS 3/25 (0 = disabled)
#ifndef RW_ENABLE_SVC3_25
#define RW_ENABLE_SVC3_25 0
#endif

ReturnValue_t ReactionWheelsHandler::initializeAfterTaskCreation() {
  // Init base class - abort on error
  ReturnValue_t rv = DeviceHandlerBase::initializeAfterTaskCreation();
  if (rv != returnvalue::OK) {
    return rv;
  }

#if RW_ENABLE_SVC3_25
  // Enable periodic HK reports via PUS 3/25
  const sid_t hkSid{getObjectId(), DATASET_ID_HK};  // Data set identifier for HK
  subdp::RegularHkPeriodicParams params(hkSid, /*enable*/ true,
                                        /*period s*/ RwConfig::HK_PERIOD_S);  // Publishing freq
  auto* hkMgr = getHkManagerHandle();  // HK distributor handle
  if (hkMgr == nullptr) {
    return returnvalue::FAILED;  // No HK manager available
  }
  return hkMgr->subscribeForRegularPeriodicPacket(params);  // Start periodic HK emission
#else
  // Periodic HK disabled - nothing to do
  return returnvalue::OK;
#endif
}

// -------- RX helpers ---------------------------------------------------------
// Drain RX by issuing small reads and discarding data
ReturnValue_t ReactionWheelsHandler::drainRxNow() {
  if (communicationInterface == nullptr || comCookie == nullptr) {
    return returnvalue::OK;  // Nothing to drain
  }
  for (int i = 0; i < 3; ++i) {  // Try 3 pulls
    ReturnValue_t rvReq =
        communicationInterface->requestReceiveMessage(comCookie, RwProtocol::STATUS_LEN);
    if (rvReq != returnvalue::OK) break;  // No more data queued
    uint8_t* buf = nullptr;
    size_t sz = 0;
    ReturnValue_t rvRead = communicationInterface->readReceivedMessage(comCookie, &buf, &sz);
    if (rvRead != returnvalue::OK || buf == nullptr || sz == 0) break;  // done
    // Flush buffer, continue draining
  }
  return returnvalue::OK;
}

// Pull bytes from ComIF into ring buffer until full or no more data
ReturnValue_t ReactionWheelsHandler::drainRxIntoRing() {
  if (communicationInterface == nullptr || comCookie == nullptr) {
    return returnvalue::OK;  // No interface found
  }
  while (true) {
    (void)rxRing.lockRingBufferMutex(MutexIF::TimeoutType::WAITING,
                                     /*timeout ms*/ 2);     // Lock ringBuffer
    const size_t freeSpace = rxRing.availableWriteSpace();  // Remaining capacity
    if (freeSpace == 0) {
      (void)rxRing.unlockRingBufferMutex();  // RingBuffer is full
      break;
    }
    const size_t chunk = (freeSpace < 128) ? freeSpace : size_t(128);  // small bounded pull
    ReturnValue_t req = communicationInterface->requestReceiveMessage(comCookie, chunk);
    if (req != returnvalue::OK) {
      (void)rxRing.unlockRingBufferMutex();  // No more data to read
      break;
    }
    uint8_t* buf = nullptr;
    size_t sz = 0;
    ReturnValue_t rd = communicationInterface->readReceivedMessage(comCookie, &buf, &sz);
    if (rd != returnvalue::OK || buf == nullptr || sz == 0) {
      (void)rxRing.unlockRingBufferMutex();  // Empty read
      break;
    }
    const size_t wrote = rxRing.writeData(buf, sz);  // Write into RingBuffer
    if (wrote != sz) {
      sif::warning << "RW rxRing overflow: dropped " << (sz - wrote) << " bytes"
                   << std::endl;  // RingBuffer overflow -> dropped (sz - wrote) bytes
    }
    (void)rxRing.unlockRingBufferMutex();  // Unlock for next iteration
  }
  return returnvalue::OK;
}

// -------- NORMAL mode: periodic STATUS polls --------------------------------
// Build periodic STATUS command or report nothing to send
ReturnValue_t ReactionWheelsHandler::buildNormalDeviceCommand(DeviceCommandId_t* id) {
  // Avoid duplicate polls while waiting for a reply
  if (statusAwaitCnt >= 0) {
    if (++statusAwaitCnt > STATUS_TIMEOUT_CYCLES) {
      triggerEvent(RwEvents::TIMEOUT, 0, 0);  // Report timeout
      statusAwaitCnt = -1;                    // Stop waiting
    } else {
      *id = DeviceHandlerIF::NO_COMMAND_ID;   
      return NOTHING_TO_SEND;        // Nothing to send this cycle
    }
  }

  // Briefly block polls after external commands to avoid cmd overwriting and give device time to reply
  uint32_t nowMs = 0;
  Clock::getUptime(&nowMs);
  if (lastExtCmdMs != 0 && (nowMs - lastExtCmdMs) < POLL_BLOCK_MS) {
    *id = DeviceHandlerIF::NO_COMMAND_ID; // Skip periodic STATUS this cycle (within POLL_BLOCK_MS after external TC)
    return NOTHING_TO_SEND;
  }

  // Periodic STATUS poll
  if (++statusPollCnt >= statusPollDivider) {
    statusPollCnt = 0;                        // Reset poll counter
    const size_t total = RwProtocol::buildStatus(txBuf, sizeof(txBuf)); // Build STATUS command
    if (total == 0) {
      *id = DeviceHandlerIF::NO_COMMAND_ID;   // Build failed -> skipping
      return NOTHING_TO_SEND;
    }
    rawPacket = txBuf;                        // Set TX buffer
    rawPacketLen = total;                     // Set TX length
    *id = CMD_STATUS_POLL;                    // Command ID for poll
    statusAwaitCnt = 0;                       // Start reply wait time frame
    return returnvalue::OK;
  }

  *id = DeviceHandlerIF::NO_COMMAND_ID;       // Otherwise no command this cycle
  return NOTHING_TO_SEND;
}

// -------- Transition commands ------------------------------------------------
// Build commands for STARTUP/SHUTDOWN state transitions
ReturnValue_t ReactionWheelsHandler::buildTransitionDeviceCommand(DeviceCommandId_t* id) {
  uint32_t now = 0;
  Clock::getUptime(&now);

  // STARTUP: Send periodic STATUS to check device readiness
  if (startingUp_) {
    const bool retryWindow = (now - startupLastTxMs_) >= START_RETRY_MS;

    if (!startupSent_ || (retryWindow && startupRetriesDone_ < START_RETRIES)) {
      const size_t total = RwProtocol::buildStatus(txBuf, sizeof(txBuf));
      if (total != 0) {
        rawPacket = txBuf;                    // TX buffer for STATUS
        rawPacketLen = total;
        *id = CMD_STATUS_POLL;                // Expect STATUS reply
        statusAwaitCnt = 0;                   // Start reply wait window
        startupSent_ = true;                  // Mark as sent
        ++startupRetriesDone_;                // Count retries
        startupLastTxMs_ = now;               // Update last TX time
#if RW_VERBOSE
        sif::info << "RW: STARTUP STATUS sent (" << int(startupRetriesDone_) << "/"
                  << int(START_RETRIES) << ")" << std::endl;
#endif
        return returnvalue::OK;
      }
    }

    // Leave STARTUP if reply wait ended or min delay elapsed
    if (statusAwaitCnt < 0 || (now - startupT0_) >= RwConfig::DELAY_OFF_TO_ON_MS) {
      startingUp_ = false;                    // Done with startup
      setMode(MODE_ON);                       // Switch to mode ON
    }

    *id = DeviceHandlerIF::NO_COMMAND_ID;     // No command this cycle
    return NOTHING_TO_SEND;
  }

  // SHUTDOWN: Send STOP and wait for delay
  if (shuttingDown_) {
    const bool needRetryWindow = (now - lastStopTxMs_) >= STOP_RETRY_MS;
    if (!stopSent_ || (needRetryWindow && stopRetriesDone_ < STOP_RETRIES)) {
      const size_t total = RwProtocol::buildStop(txBuf, sizeof(txBuf));
      if (total != 0) {
        rawPacket = txBuf;                    // TX buffer for STOP
        rawPacketLen = total;
        *id = CMD_STOP;                       // STOP command
        stopSent_ = true;                     // Mark as sent
        ++stopRetriesDone_;                   // Count retries
        lastStopTxMs_ = now;                  // Update last TX time
#if RW_VERBOSE
        sif::info << "RW: STOP sent (" << int(stopRetriesDone_) << "/" << int(STOP_RETRIES) << ")"
                  << std::endl;
#endif
        return returnvalue::OK;
      }
    }
    if ((now - shutdownT0_) >= STOP_DELAY_MS) {
      shuttingDown_ = false;                  // Done with shutdown
      setMode(MODE_OFF);                      // Switch to mode OFF
    }
    *id = DeviceHandlerIF::NO_COMMAND_ID;     // No command this cycle
    return NOTHING_TO_SEND;
  }

  // No transition, therefore nothing to send here
  *id = DeviceHandlerIF::NO_COMMAND_ID;
  return NOTHING_TO_SEND;
}

// -------- External command builder ------------------------------------------
// Build device command from TC payload and enforce mode constraints and limits
ReturnValue_t ReactionWheelsHandler::buildCommandFromCommand(DeviceCommandId_t deviceCommand,
                                                             const uint8_t* data, size_t len) {
  // Check current mode: Allow only specific (safe) commands outside NORMAL
  Mode_t m = 0;
  Submode_t sm = 0;
  getMode(&m, &sm);
  const bool isNormal = (m == MODE_NORMAL);
  if (!isNormal) {
    switch (deviceCommand) {
      case CMD_STATUS:                      // Allowed outside NORMAL
      case CMD_STOP:                        // Allowed outside NORMAL
        break;
      default:
        return returnvalue::FAILED;         // Do not allow other commands
    }
  }

  // Build per-command payload
  switch (deviceCommand) {
    case CMD_SET_SPEED: {
      if (len < sizeof(int16_t)) {
        return returnvalue::FAILED;         // Payload too short
      }
      int16_t rpm = 0;
      std::memcpy(&rpm, data, sizeof(rpm)); // Read speed [rpm]

      // Clamp to configured limit
      int16_t lim = p_maxRpm > 0 ? p_maxRpm : RwConfig::MAX_RPM_DEFAULT;
      if (rpm > lim) rpm = lim;
      if (rpm < -lim) rpm = -lim;

      // Build SET_SPEED frame
      const size_t total = RwProtocol::buildSetSpeed(txBuf, sizeof(txBuf), rpm);
      if (total == 0) return returnvalue::FAILED;  // Build error
      rawPacket = txBuf;
      rawPacketLen = total;
      Clock::getUptime(&lastExtCmdMs);      // Block polls briefly
      return returnvalue::OK;
    }

    case CMD_SET_TORQUE: {
      if (len < sizeof(int16_t)) {
        return returnvalue::FAILED;         // Payload too short
      }
      int16_t tq = 0;
      std::memcpy(&tq, data, sizeof(tq));   // Read torque [mNm]

      // Clamp to configured limit
      const int16_t lim = RwConfig::MAX_TORQUE_MNM_DEFAULT;
      if (lim > 0) {
        if (tq > lim) tq = lim;
        if (tq < -lim) tq = -lim;
      }

      // Build SET_TORQUE frame
      const size_t total = RwProtocol::buildSetTorque(txBuf, sizeof(txBuf), tq);
      if (total == 0) return returnvalue::FAILED;  // Build error
      rawPacket = txBuf;
      rawPacketLen = total;
      Clock::getUptime(&lastExtCmdMs);      // Block polls briefly
      return returnvalue::OK;
    }

    case CMD_STOP: {
      // Build STOP frame
      const size_t total = RwProtocol::buildStop(txBuf, sizeof(txBuf));
      if (total == 0) return returnvalue::FAILED;  // Build error
      rawPacket = txBuf;
      rawPacketLen = total;
      Clock::getUptime(&lastExtCmdMs);      // Block polls briefly
      return returnvalue::OK;
    }

    case CMD_STATUS: {
      // Build STATUS frame
      (void)drainRxNow();                   // Clear RX before STATUS, so reply is fresh
      const size_t total = RwProtocol::buildStatus(txBuf, sizeof(txBuf));
      if (total == 0) return returnvalue::FAILED;  // build error
      rawPacket = txBuf;
      rawPacketLen = total;
      statusAwaitCnt = 0;                   // Start reply wait window
      return returnvalue::OK;
    }

    default:
      return returnvalue::FAILED;           // Unknown command
  }
}


// -------- Command/reply map --------------------------------------------------
// Map command IDs and expected replies
void ReactionWheelsHandler::fillCommandAndReplyMap() {
  insertInCommandMap(CMD_SET_SPEED, false, 0);   // "fire-and-forget"
  insertInCommandMap(CMD_SET_TORQUE, false, 0);  // "fire-and-forget"
  insertInCommandMap(CMD_STOP, false, 0);        // "fire-and-forget"

  // Periodic/onboard poll expecting a STATUS reply
  insertInCommandAndReplyMap(CMD_STATUS_POLL, 5, &replySet, RwProtocol::STATUS_LEN,
                             /*periodic*/ false, /*hasDifferentReplyId*/ true, REPLY_STATUS_POLL,
                             /*countdown*/ nullptr);

  // TC STATUS uses same reply path as periodic poll
  insertInCommandAndReplyMap(CMD_STATUS, 5, &replySet, RwProtocol::STATUS_LEN,
                             /*periodic*/ false, /*hasDifferentReplyId*/ true, REPLY_STATUS_POLL,
                             /*countdown*/ nullptr);
}

// -------- FDIR counters ------------------------------------------------------
// Count CRC errors and trigger event if threshold is reached
// INFO: CRC error = valid header/ID/length but bad CRC (bit errors)
void ReactionWheelsHandler::handleCrcError() {
  ++crcErrCnt;
  if (CRC_ERR_EVENT_THRESH != 0 && (crcErrCnt % CRC_ERR_EVENT_THRESH) == 0) {
    triggerEvent(RwEvents::CRC_ERROR, crcErrCnt, 0);
  }
}

// Count malformed frames and trigger event if threshold is reached
// INFO: Malformed = wrong ID, wrong length, incomplete frame
void ReactionWheelsHandler::handleMalformed() {
  ++malformedCnt;
  if (MALFORMED_EVENT_THRESH != 0 && (malformedCnt % MALFORMED_EVENT_THRESH) == 0) {
    triggerEvent(RwEvents::INVALID_REPLY, malformedCnt, 0);
  }
}

// -------- RX analysis helpers ------------------------------------------------
// Scan a raw RX window for wrong IDs/lengths/CRC and increment counters
void ReactionWheelsHandler::checkRxWindowForProtocolIssues(const uint8_t* buf, size_t n) {
  if (buf == nullptr || n < 2) return;
  for (size_t i = 0; i + 1 < n; ++i) {
    if (buf[i] != RwProtocol::START_REPLY) continue;             // Header mismatch
    uint8_t id = buf[i + 1];

    if (id != static_cast<uint8_t>(RwProtocol::RespId::STATUS)) {
      handleMalformed();                                         // Unexpected reply ID
      continue;
    }
    if (i + RwProtocol::STATUS_LEN > n) {
      handleMalformed();                                         // Incomplete frame
      continue;
    }
    if (!RwProtocol::verifyCrc16(&buf[i], RwProtocol::STATUS_LEN)) {
      handleCrcError();                                          // CRC mismatch
    }
  }
}

// Find the latest valid STATUS frame in a byte window
static inline bool findLatestValidStatus(const uint8_t* buf, size_t n, long& posOut) {
  if (buf == nullptr || n < RwProtocol::STATUS_LEN) return false;
  for (long i = static_cast<long>(n) - static_cast<long>(RwProtocol::STATUS_LEN); i >= 0; --i) {
    const uint8_t* p = &buf[static_cast<size_t>(i)];
    if (p[0] == RwProtocol::START_REPLY &&
        p[1] == static_cast<uint8_t>(RwProtocol::RespId::STATUS) &&
        RwProtocol::verifyCrc16(p, RwProtocol::STATUS_LEN)) {
      posOut = i;                                                // Latest valid frame
      return true;
    }
  }
  return false;
}

// -------- Reply scanning -----------------------------------------------------
// Extract a valid STATUS frame from the immediate window or from the RX ring
ReturnValue_t ReactionWheelsHandler::scanForReply(const uint8_t* start, size_t len,
                                                  DeviceCommandId_t* foundId, size_t* foundLen) {
  
  // Protocol check on the current RX window (header/ID/len/CRC checks)
  checkRxWindowForProtocolIssues(start, len);           // Increment counters if needed

  // Try to find the latest valid STATUS directly in the given window
  long pos = -1;
  if (findLatestValidStatus(start, len, pos)) {
    const uint8_t* src = start + static_cast<size_t>(pos);  // Point to frame start
    std::memcpy(replySet.raw.value, src, RwProtocol::STATUS_LEN);  // Copy frame
    replySet.raw.setValid(true);                               // Mark raw as valid
    replySet.setValidity(true, true);                          // Mark dataset as valid
    *foundId = REPLY_STATUS_POLL;                              // Map to reply ID
    *foundLen = RwProtocol::STATUS_LEN;                        // Report length
    return returnvalue::OK;                     
  }

  // Not in the window -> pull more data from ComIF into the ring
  (void)drainRxIntoRing();                                     // Drain into ringBuffer

  // Lock ring and check if we have enough bytes to hold a full frame
  (void)rxRing.lockRingBufferMutex(MutexIF::TimeoutType::WAITING, /*timeout ms*/ 2);
  size_t avail = rxRing.getAvailableReadData();
  if (avail < RwProtocol::STATUS_LEN) {
    (void)rxRing.unlockRingBufferMutex();                      // Not enough data
    return returnvalue::FAILED;
  }

  // Take a snapshot without consuming ring bytes (no read pointer advance)
  std::vector<uint8_t> snapshot(avail);
  size_t copied = 0;
  if (rxRing.readData(snapshot.data(), avail, /*incrementReadPtr=*/false,
                      /*readRemaining=*/true, &copied) != returnvalue::OK) {
    (void)rxRing.unlockRingBufferMutex();                      // Snapshot failed
    return returnvalue::FAILED;
  }
  snapshot.resize(copied);                                     // Shrink to actual size

  // Protocol check of the snapshot data (counts malformed/CRC errors)
  checkRxWindowForProtocolIssues(snapshot.data(), snapshot.size());

  // Search the snapshot data for the latest valid STATUS frame
  if (!findLatestValidStatus(snapshot.data(), snapshot.size(), pos)) {
    (void)rxRing.unlockRingBufferMutex();                      // None found
    return returnvalue::FAILED;
  }

  // Copy the valid frame to the RAW dataset and mark valid
  std::memcpy(replySet.raw.value, snapshot.data() + static_cast<size_t>(pos),
              RwProtocol::STATUS_LEN);
  replySet.raw.setValid(true);
  replySet.setValidity(true, true);

  // Consume ring bytes up to the end of the found frame (drop used bytes)
  (void)rxRing.deleteData(static_cast<size_t>(pos) + RwProtocol::STATUS_LEN,
                          /*deleteRemaining=*/false);

  // Unlock ring and report reply info to DHB
  (void)rxRing.unlockRingBufferMutex();
  *foundId = REPLY_STATUS_POLL;                                // Reply mapping
  *foundLen = RwProtocol::STATUS_LEN;                          // Frame length
  return returnvalue::OK;
}

// -------- Reply interpretation ----------------------------------------------
// Decode STATUS, update HK, set flags/events, and forward if needed
ReturnValue_t ReactionWheelsHandler::interpretDeviceReply(DeviceCommandId_t id,
                                                          const uint8_t* packet) {
  if (id != REPLY_STATUS_POLL) {
    return returnvalue::FAILED;                                   // Unexpected reply
  }
  statusAwaitCnt = -1;                                            // Stop waiting

  const uint8_t* pkt = replySet.raw.isValid() ? replySet.raw.value : packet;

  const int16_t speed = static_cast<int16_t>((pkt[2] << 8) | pkt[3]);  // [rpm]
  const int16_t torque = static_cast<int16_t>((pkt[4] << 8) | pkt[5]); // [mNm]
  const uint8_t running = pkt[6];                                     // [0/1]

  // Status log (only every Nth time to reduce log volume)
  static uint32_t logCtr = 0;
#if !defined(RW_STATUS_LOG_EVERY)
#define RW_STATUS_LOG_EVERY RwConfig::STATUS_LOG_EVERY
#endif
  if ((++logCtr % RW_STATUS_LOG_EVERY) == 0) {
    sif::info << "RW STATUS: speed=" << speed << " RPM, torque=" << torque
              << " mNm, running=" << int(running) << std::endl;
  }

  // Update HK dataset (write all, then validate and commit)
  hkSet.read();
  hkSet.setValidity(false, true);

  hkSet.speedRpm.value = speed;
  hkSet.torque_mNm.value = torque;
  hkSet.running.value = running;
  hkSet.flags.value = 0;
  hkSet.crcErrCnt.value = crcErrCnt;
  hkSet.malformedCnt.value = malformedCnt;

  uint32_t ms = 0;
  Clock::getUptime(&ms);
  hkSet.timestampMs.value = ms;

  // FDIR: Detect if stuck and increment counter if so
  const bool stuckCond = (running == 0) && (std::abs(static_cast<int>(speed)) > STUCK_RPM_THRESH);
  if (stuckCond) {
    if (++stuckRpmCnt >= STUCK_RPM_COUNT) {
      triggerEvent(RwEvents::STUCK, static_cast<uint32_t>(speed), static_cast<uint32_t>(running));
      stuckRpmCnt = 0;
      hkSet.flags.value |= FLAG_STUCK;
    }
  } else {
    stuckRpmCnt = 0;
  }

  // FDIR: Detect high torque and increment counter if so
  const bool torqueHigh = (std::abs(static_cast<int>(torque)) > HIGH_TORQUE_THRESH);
  if (torqueHigh) {
    if (++highTorqueCnt >= HIGH_TORQUE_COUNT) {
      triggerEvent(RwEvents::TORQUE_HIGH, static_cast<uint32_t>(torque), 0);
      highTorqueCnt = 0;
      hkSet.flags.value |= FLAG_TORQUE_HIGH;
    }
  } else {
    highTorqueCnt = 0;
  }

  // Set validity and commit HK
  hkSet.speedRpm.setValid(true);
  hkSet.torque_mNm.setValid(true);
  hkSet.running.setValid(true);
  hkSet.flags.setValid(true);
  hkSet.crcErrCnt.setValid(true);
  hkSet.malformedCnt.setValid(true);
  hkSet.timestampMs.setValid(true);
  hkSet.setValidity(true, true);
  hkSet.commit();

  // Forward STATUS to requester if the request was a TC
  if (pendingTcStatusTm) {
    if (pendingTcStatusReportedTo != MessageQueueIF::NO_QUEUE) {
      (void)actionHelper.reportData(pendingTcStatusReportedTo, CMD_STATUS, pkt,
                                    RwProtocol::STATUS_LEN, /*append*/ false);
    }
    pendingTcStatusTm = false;                                     // Reset echo latch
    pendingTcStatusReportedTo = MessageQueueIF::NO_QUEUE;           // Clear destination
  }

  return returnvalue::OK;
}

// -------- Transition delays --------------------------------------------------
// Provide delays for mode transitions for safe switching between modes
uint32_t ReactionWheelsHandler::getTransitionDelayMs(Mode_t from, Mode_t to) {
  if (from == MODE_OFF && to == MODE_ON) return RwConfig::DELAY_OFF_TO_ON_MS;
  if (from == MODE_ON && to == MODE_NORMAL) return RwConfig::DELAY_ON_TO_NORMAL_MS;
  if (to == MODE_OFF && from != MODE_OFF)
    return STOP_RETRIES * STOP_RETRY_MS + STOP_DELAY_MS + 200;      // STOP window + slack
  if (to != from) return 1000;                                      // Fallback
  return 0;
}

// -------- Local data pool registration ---------------------------------------
// Register local pool variables and sizes for HK and RAW
ReturnValue_t ReactionWheelsHandler::initializeLocalDataPool(localpool::DataPool& localDataPoolMap,
                                                             LocalDataPoolManager&) {
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::RAW_REPLY),
                           new PoolEntry<uint8_t>(RwProtocol::STATUS_LEN));
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::HK_SPEED_RPM), new PoolEntry<int16_t>(1));
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::HK_TORQUE_mNm), new PoolEntry<int16_t>(1));
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::HK_RUNNING), new PoolEntry<uint8_t>(1));
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::HK_FLAGS), new PoolEntry<uint16_t>(1));
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::HK_CRC_ERR_CNT),
                           new PoolEntry<uint32_t>(1));
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::HK_MALFORMED_CNT),
                           new PoolEntry<uint32_t>(1));
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::HK_TIMESTAMP_MS),
                           new PoolEntry<uint32_t>(1));
  return returnvalue::OK;
}

// -------- Action entry -------------------------------------------------------
// Entry point for action-based commands (e.g. TC STATUS echo)
ReturnValue_t ReactionWheelsHandler::executeAction(ActionId_t actionId,
                                                   MessageQueueId_t commandedBy,
                                                   const uint8_t* data, size_t size) {
  const auto cmd = static_cast<DeviceCommandId_t>(actionId);
  if (cmd == CMD_STATUS) {
    // Remember requester as interpretDeviceReply() will send payload back to requester
    pendingTcStatusTm = true;
    pendingTcStatusReportedTo = commandedBy;
    // INFO: Actual frame build happens in buildCommandFromCommand()
  }
  return DeviceHandlerBase::executeAction(actionId, commandedBy, data, size);
}

// -------- Parameter IF -------------------------------------------------------
// Expose/override parameters via PUS-20 (runtime changeable)
ReturnValue_t ReactionWheelsHandler::getParameter(uint8_t domainId, uint8_t parameterId,
                                                  ParameterWrapper* parameterWrapper,
                                                  const ParameterWrapper* newValues,
                                                  uint16_t /*startAtIndex*/) {
  if (domainId != PARAM_DOMAIN) {
    return returnvalue::FAILED;                       // Wrong domain
  }
  switch (static_cast<ParamId>(parameterId)) {
    case ParamId::MAX_RPM:
      if (newValues != nullptr) {
        int16_t v = 0;
        if (newValues->getElement(&v, 0) != returnvalue::OK) {
          return returnvalue::FAILED;                 // Bad write
        }
        p_maxRpm = v;                                 // Update limit
      }
      parameterWrapper->set(p_maxRpm);                // Readback
      return returnvalue::OK;

    case ParamId::POLL_DIVIDER:
      if (newValues != nullptr) {
        uint32_t v = 0;
        if (newValues->getElement(&v, 0) != returnvalue::OK) {
          return returnvalue::FAILED;                 // Bad write
        }
        statusPollDivider = v;                        // Update poll divider
      }
      parameterWrapper->set(statusPollDivider);       // Readback
      return returnvalue::OK;

    default:
      return returnvalue::FAILED;                     // Unknown parameter
  }
}

// -------- Dataset routing ----------------------------------------------------
// Route SIDs to local data sets
LocalPoolDataSetBase* ReactionWheelsHandler::getDataSetHandle(sid_t sid) {
  if (sid.ownerSetId == DATASET_ID_HK) return &hkSet;
  if (sid.ownerSetId == DATASET_ID_RAW) return &replySet;
  return DeviceHandlerBase::getDataSetHandle(sid);     // Fallback
}
