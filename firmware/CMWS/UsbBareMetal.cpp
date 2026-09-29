// =====================================================================
// USB HID joystick, backend B: bare-metal device stack on the raw
// STM32F1 USB peripheral. No core USB library, no HAL PCD.
//
// Compiled only when Tools > USB support = "None" (USBCON undefined).
// It exists for Blue Pills fitted with a Hangshun HK32F103 (non-A)
// instead of a genuine STM32F103: that chip's USB block departs from
// ST's in ways the core's stack cannot accommodate (see the PMA notes
// below). On genuine silicon it behaves like any other minimal stack.
//
// Scope: exactly what the joystick needs. EP0 control transfers for the
// standard and HID requests, one interrupt IN endpoint for reports.
// Nothing else.
// =====================================================================

#include "UsbHidJoystick.h"

#if !defined(USBCON)

#include <string.h>

#include "Config.h"
#include "UsbDescriptors.h"

extern "C" void USB_LP_CAN1_RX0_IRQHandler(void);

namespace {

// -------------------------------------------------------------------
// Packet memory access
//
// Local byte addresses are what the buffer descriptor table holds. On
// a genuine F103 the 512-byte PMA is 16-bit wide and every half-word
// sits in a 32-bit APB slot: local byte L lives at 0x40006000 + 2*L,
// read/written as 16 bits.
//
// On the HK32F103 (non-A), measured 2026-09-27:
//  - local 0..63 are not RAM but registers implementing the descriptor
//    table (ADDRx 16-bit, COUNTx_TX 10-bit, COUNTx_RX partly read-only);
//    packet buffers must live at local >= 64;
//  - local 64..511 is RAM organised as 32-bit words holding two
//    half-words; the "2*L, 16-bit" CPU view above still works for both
//    reading and writing (verified with a write/read-back sweep);
//  - the SIE reads and writes packets there exactly where the CPU
//    expects them, in both directions (verified by trying five TX
//    layouts during enumeration: the ST one is the one the host
//    accepted).
//
// What the core's stack gets wrong on this chip is therefore not the
// memory but the EP0 handshake: after a SETUP, the TX side must be
// forced to NAK before the answer is loaded (Hangshun documents this
// for its xC/D/E parts). Without it the data stage of the very first
// GET_DESCRIPTOR fails and enumeration stalls at ADDRESSED. This stack
// does that in handleSetup().
// -------------------------------------------------------------------

constexpr uint32_t kPma = 0x40006000UL;

inline volatile uint16_t& pmaHalfword(uint16_t index) {   // ST view: half-word i at +4i
  return *reinterpret_cast<volatile uint16_t*>(kPma + 4u * index);
}

// Descriptor table (BTABLE = 0): entry for endpoint e is 4 half-words at
// local 8e: ADDR_TX, COUNT_TX, ADDR_RX, COUNT_RX.
enum BdtField : uint8_t { kAddrTx = 0, kCountTx = 1, kAddrRx = 2, kCountRx = 3 };
inline volatile uint16_t& bdt(uint8_t ep, BdtField f) { return pmaHalfword(4u * ep + f); }

constexpr uint16_t kEp0OutBuf = 0x40;   // 64 bytes
constexpr uint16_t kEp0InBuf  = 0x80;   // 64 bytes
constexpr uint16_t kEp1InBuf  = 0xC0;   // 8 bytes
constexpr uint16_t kEp0Size   = 64;
constexpr uint16_t kCountRx64 = 0x8400; // BL_SIZE=1, NUM_BLOCK=1 -> 64 bytes

void pmaRead(uint16_t local, uint8_t* dst, uint16_t n) {
  uint16_t i = local / 2;
  while (n >= 2) {
    const uint16_t v = pmaHalfword(i++);
    *dst++ = static_cast<uint8_t>(v);
    *dst++ = static_cast<uint8_t>(v >> 8);
    n -= 2;
  }
  if (n) {
    *dst = static_cast<uint8_t>(pmaHalfword(i));
  }
}

void pmaWrite(uint16_t local, const uint8_t* src, uint16_t n) {
  uint16_t i = local / 2;
  while (n >= 2) {
    pmaHalfword(i++) = static_cast<uint16_t>(src[0] | (src[1] << 8));
    src += 2;
    n -= 2;
  }
  if (n) {
    pmaHalfword(i) = src[0];
  }
}

// -------------------------------------------------------------------
// Endpoint register helpers. STAT and DTOG bits toggle when written
// with 1; CTR bits clear when written with 0. Every write below keeps
// CTR bits at 1 (no effect) unless it deliberately clears one.
// -------------------------------------------------------------------

inline volatile uint16_t& epReg(uint8_t ep) {
  return *reinterpret_cast<volatile uint16_t*>(USB_BASE + 4u * ep);
}

constexpr uint16_t kEpNonToggle = USB_EPREG_MASK;   // CTR_RX|SETUP|T_FIELD|KIND|CTR_TX|EA

void epSetStatRx(uint8_t ep, uint16_t stat) {
  const uint16_t v = epReg(ep);
  epReg(ep) = static_cast<uint16_t>((v & kEpNonToggle) | USB_EP_CTR_RX | USB_EP_CTR_TX
                                    | ((v & USB_EPRX_STAT) ^ stat));
}

void epSetStatTx(uint8_t ep, uint16_t stat) {
  const uint16_t v = epReg(ep);
  epReg(ep) = static_cast<uint16_t>((v & kEpNonToggle) | USB_EP_CTR_RX | USB_EP_CTR_TX
                                    | ((v & USB_EPTX_STAT) ^ stat));
}

void epClearCtrRx(uint8_t ep) {
  epReg(ep) = static_cast<uint16_t>((epReg(ep) & kEpNonToggle & ~USB_EP_CTR_RX) | USB_EP_CTR_TX);
}

void epClearCtrTx(uint8_t ep) {
  epReg(ep) = static_cast<uint16_t>((epReg(ep) & kEpNonToggle & ~USB_EP_CTR_TX) | USB_EP_CTR_RX);
}

// Full (re)configuration of an endpoint: type, address, DTOG cleared,
// both directions in the given states.
void epConfigure(uint8_t ep, uint16_t type, uint16_t statRx, uint16_t statTx) {
  const uint16_t v = epReg(ep);
  epReg(ep) = static_cast<uint16_t>(type | ep | USB_EP_CTR_RX | USB_EP_CTR_TX
                                    | (v & USB_EP_DTOG_RX) | (v & USB_EP_DTOG_TX)   // toggle both to 0
                                    | ((v & USB_EPRX_STAT) ^ statRx)
                                    | ((v & USB_EPTX_STAT) ^ statTx));
}

// -------------------------------------------------------------------
// Device state
// -------------------------------------------------------------------

enum DevState : uint8_t { kDefault = 1, kAddressed = 2, kConfigured = 3 };

struct Setup {
  uint8_t  bmRequestType;
  uint8_t  bRequest;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
};

volatile uint8_t  s_devState      = kDefault;
volatile uint8_t  s_pendingAddr   = 0;       // applied after the SET_ADDRESS status stage
volatile uint8_t  s_config        = 0;
uint8_t           s_hidIdle       = 0;
uint8_t           s_hidProtocol   = 0;
volatile bool     s_busy          = false;   // report in flight on EP1

// EP0 IN data stage
const uint8_t*    s_txPtr         = nullptr;
uint16_t          s_txRemaining   = 0;
bool              s_txZlpNeeded   = false;   // short-packet rule: exact multiple of 64 and shorter than wLength
uint8_t           s_ctlBuf[UsbDesc::kStringMax];
uint8_t           s_report[UsbHidJoystick::REPORT_SIZE] = { 0 };

// Reset-storm guard (same policy as the HAL backend)
volatile uint16_t s_resetCount     = 0;
volatile uint32_t s_resetWindowAt  = 0;
volatile bool     s_stormDetected  = false;
bool              s_down           = false;
uint32_t          s_retryAt        = 0;

constexpr uint16_t kIntMask = USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_WKUPM
                            | USB_CNTR_ERRM | USB_CNTR_PMAOVRM;

// --- EP0 transfer machinery ------------------------------------------

void ep0Stall() {
  epSetStatRx(0, USB_EP_RX_STALL);
  epSetStatTx(0, USB_EP_TX_STALL);
}

void ep0SendChunk() {
  const uint16_t n = s_txRemaining > kEp0Size ? kEp0Size : s_txRemaining;
  pmaWrite(kEp0InBuf, s_txPtr, n);
  bdt(0, kCountTx) = n;
  s_txPtr       += n;
  s_txRemaining -= n;
  epSetStatTx(0, USB_EP_TX_VALID);
}

// Starts an IN data stage of `len` bytes (already clipped to wLength by
// the caller), followed by the host's OUT status stage.
void ep0SendData(const uint8_t* data, uint16_t len, uint16_t wLength) {
  s_txPtr       = data;
  s_txRemaining = len;
  s_txZlpNeeded = (len != 0) && (len % kEp0Size == 0) && (len < wLength);
  ep0SendChunk();
  epSetStatRx(0, USB_EP_RX_VALID);   // accept the status OUT (or a premature one)
}

void ep0SendStatus() {   // IN zero-length packet (status stage of a no-data / OUT request)
  s_txPtr       = nullptr;
  s_txRemaining = 0;
  s_txZlpNeeded = false;
  bdt(0, kCountTx) = 0;
  epSetStatTx(0, USB_EP_TX_VALID);
}

void resetEndpoints() {
  USB->BTABLE = 0;
  bdt(0, kAddrTx)  = kEp0InBuf;
  bdt(0, kCountTx) = 0;
  bdt(0, kAddrRx)  = kEp0OutBuf;
  bdt(0, kCountRx) = kCountRx64;
  epConfigure(0, USB_EP_CONTROL, USB_EP_RX_VALID, USB_EP_TX_NAK);

  bdt(1, kAddrTx)  = kEp1InBuf;
  bdt(1, kCountTx) = 0;
  bdt(1, kAddrRx)  = 0;
  bdt(1, kCountRx) = 0;
  epConfigure(1, USB_EP_INTERRUPT, USB_EP_RX_DIS, USB_EP_TX_NAK);

  USB->DADDR    = USB_DADDR_EF;   // address 0, function enabled
  s_devState    = kDefault;
  s_pendingAddr = 0;
  s_config      = 0;
  s_busy        = false;
  s_txRemaining = 0;
}

void noteBusReset() {
  const uint32_t now = HAL_GetTick();
  if (static_cast<uint32_t>(now - s_resetWindowAt) > cfg::USB_STORM_WINDOW_MS) {
    s_resetWindowAt = now;
    s_resetCount    = 0;
  }
  if (++s_resetCount >= cfg::USB_STORM_RESETS) {
    USB->CNTR       = static_cast<uint16_t>(USB->CNTR & 0x00FFu);
    s_stormDetected = true;
  }
}

// --- Request handling -------------------------------------------------

constexpr uint8_t kReqStandard = 0x00;
constexpr uint8_t kReqClass    = 0x20;
constexpr uint8_t kReqTypeMask = 0x60;
constexpr uint8_t kRecipMask   = 0x1F;

uint16_t descriptorFor(uint8_t type, uint8_t index, const uint8_t** out) {
  switch (type) {
    case 0x01: *out = UsbDesc::device;    return sizeof(UsbDesc::device);
    case 0x02: *out = UsbDesc::config;    return sizeof(UsbDesc::config);
    case 0x06: *out = UsbDesc::qualifier; return sizeof(UsbDesc::qualifier);
    case 0x03:
      *out = s_ctlBuf;
      switch (index) {
        case 0:  memcpy(s_ctlBuf, UsbDesc::langId, sizeof(UsbDesc::langId)); return sizeof(UsbDesc::langId);
        case 1:  return UsbDesc::manufacturerString(s_ctlBuf, sizeof(s_ctlBuf));
        case 2:  return UsbDesc::productString(s_ctlBuf, sizeof(s_ctlBuf));
        case 3:  return UsbDesc::serialString(s_ctlBuf, sizeof(s_ctlBuf));
        default: return 0;
      }
    case UsbDesc::kHidDescriptorType: *out = &UsbDesc::config[UsbDesc::kConfigHidOffset]; return UsbDesc::kHidDescSize;
    case UsbDesc::kHidReportDescType: *out = UsbDesc::report; return UsbDesc::kReportDescSize;
    default: return 0;
  }
}

void handleSetup() {
  uint8_t raw[8];
  pmaRead(kEp0OutBuf, raw, 8);
  Setup req;
  req.bmRequestType = raw[0];
  req.bRequest      = raw[1];
  req.wValue        = static_cast<uint16_t>(raw[2] | (raw[3] << 8));
  req.wIndex        = static_cast<uint16_t>(raw[4] | (raw[5] << 8));
  req.wLength       = static_cast<uint16_t>(raw[6] | (raw[7] << 8));

  // HK32 quirk (documented by Hangshun for the xC/D/E parts, confirmed
  // here on the non-A x8xB): after a SETUP the TX side may be left VALID
  // and an empty packet would go out before we load anything. Force NAK
  // first, always. This single line is what the core's stack lacks.
  epSetStatTx(0, USB_EP_TX_NAK);
  s_txRemaining = 0;

  const bool     dirIn  = (req.bmRequestType & 0x80) != 0;
  const uint8_t  type   = req.bmRequestType & kReqTypeMask;
  const uint8_t  recip  = req.bmRequestType & kRecipMask;
  const uint8_t* data   = nullptr;
  uint16_t       len    = 0;
  bool           ok     = true;
  bool           status = false;   // request without data stage: answer with an IN ZLP

  if (type == kReqStandard) {
    switch (req.bRequest) {
      case 0x06:   // GET_DESCRIPTOR
        len = descriptorFor(static_cast<uint8_t>(req.wValue >> 8), static_cast<uint8_t>(req.wValue), &data);
        ok  = len != 0;
        break;
      case 0x05:   // SET_ADDRESS: applied once the status stage has been sent
        s_pendingAddr = static_cast<uint8_t>(req.wValue & 0x7F);
        status = true;
        break;
      case 0x08:   // GET_CONFIGURATION
        s_ctlBuf[0] = s_config; data = s_ctlBuf; len = 1;
        break;
      case 0x09:   // SET_CONFIGURATION
        s_config = static_cast<uint8_t>(req.wValue);
        if (s_config == 1) {
          epConfigure(1, USB_EP_INTERRUPT, USB_EP_RX_DIS, USB_EP_TX_NAK);
          s_devState = kConfigured;
        } else {
          s_devState = kAddressed;
        }
        s_busy = false;
        status = true;
        break;
      case 0x00:   // GET_STATUS
        s_ctlBuf[0] = 0; s_ctlBuf[1] = 0; data = s_ctlBuf; len = 2;
        break;
      case 0x0A:   // GET_INTERFACE
        s_ctlBuf[0] = 0; data = s_ctlBuf; len = 1;
        break;
      case 0x0B:   // SET_INTERFACE
      case 0x01:   // CLEAR_FEATURE
      case 0x03:   // SET_FEATURE
        if (req.bRequest == 0x01 && recip == 0x02 && (req.wIndex & 0xFF) == UsbDesc::kEpInAddr) {
          epConfigure(1, USB_EP_INTERRUPT, USB_EP_RX_DIS, USB_EP_TX_NAK);   // clear halt + data toggle
          s_busy = false;
        }
        status = true;
        break;
      default:
        ok = false;
        break;
    }
  } else if (type == kReqClass && recip == 0x01) {   // HID class, interface recipient
    switch (req.bRequest) {
      case 0x01:   // GET_REPORT
        if ((req.wValue >> 8) == 0x01) { data = s_report; len = UsbHidJoystick::REPORT_SIZE; } else { ok = false; }
        break;
      case 0x02: s_ctlBuf[0] = s_hidIdle;     data = s_ctlBuf; len = 1; break;   // GET_IDLE
      case 0x03: s_ctlBuf[0] = s_hidProtocol; data = s_ctlBuf; len = 1; break;   // GET_PROTOCOL
      case 0x0A: s_hidIdle     = static_cast<uint8_t>(req.wValue >> 8); status = true; break;   // SET_IDLE
      case 0x0B: s_hidProtocol = static_cast<uint8_t>(req.wValue);      status = true; break;   // SET_PROTOCOL
      default:   ok = false; break;   // SET_REPORT carries OUT data we never arm: stall
    }
  } else {
    ok = false;
  }

  if (!ok) {
    ep0Stall();
    return;
  }
  if (status || !dirIn || req.wLength == 0) {
    ep0SendStatus();
    epSetStatRx(0, USB_EP_RX_VALID);
    return;
  }
  if (len > req.wLength) {
    len = req.wLength;
  }
  ep0SendData(data, len, req.wLength);
}

// EP0 IN transaction completed.
void handleEp0In() {
  if (s_txRemaining > 0) {
    ep0SendChunk();
    return;
  }
  if (s_txZlpNeeded) {
    s_txZlpNeeded = false;
    bdt(0, kCountTx) = 0;
    epSetStatTx(0, USB_EP_TX_VALID);
    return;
  }
  if (s_pendingAddr != 0) {
    // The SET_ADDRESS status stage has just gone out: switch address now.
    USB->DADDR    = static_cast<uint16_t>(USB_DADDR_EF | s_pendingAddr);
    s_devState    = kAddressed;
    s_pendingAddr = 0;
  }
  // Data stage done: the host's status OUT (ZLP) follows; RX is already VALID.
}

// EP0 OUT transaction completed (status stage of an IN transfer, or
// unexpected data): just re-arm reception.
void handleEp0Out() {
  bdt(0, kCountRx) = kCountRx64;
  epSetStatRx(0, USB_EP_RX_VALID);
}

}  // namespace

// -------------------------------------------------------------------
// Interrupt handler
// -------------------------------------------------------------------
extern "C" void USB_LP_CAN1_RX0_IRQHandler(void) {
  uint16_t istr = USB->ISTR;

  if (istr & USB_ISTR_RESET) {
    USB->ISTR = static_cast<uint16_t>(~USB_ISTR_RESET);
    noteBusReset();
    resetEndpoints();
  }
  if (istr & USB_ISTR_PMAOVR) {
    USB->ISTR = static_cast<uint16_t>(~USB_ISTR_PMAOVR);
  }
  if (istr & USB_ISTR_ERR) {
    USB->ISTR = static_cast<uint16_t>(~USB_ISTR_ERR);
  }
  if (istr & USB_ISTR_SUSP) {
    USB->ISTR = static_cast<uint16_t>(~USB_ISTR_SUSP);
  }
  if (istr & USB_ISTR_WKUP) {
    USB->ISTR = static_cast<uint16_t>(~USB_ISTR_WKUP);
  }

  while ((istr = USB->ISTR) & USB_ISTR_CTR) {
    const uint8_t ep = istr & USB_ISTR_EP_ID;
    const uint16_t v = epReg(ep);
    if (ep == 0) {
      if (v & USB_EP_CTR_RX) {
        epClearCtrRx(0);
        if (v & USB_EP_SETUP) {
          handleSetup();
        } else {
          handleEp0Out();
        }
      }
      if (v & USB_EP_CTR_TX) {
        epClearCtrTx(0);
        handleEp0In();
      }
    } else {
      if (v & USB_EP_CTR_TX) {
        epClearCtrTx(ep);
        s_busy = false;
      }
      if (v & USB_EP_CTR_RX) {
        epClearCtrRx(ep);
      }
    }
  }
}

namespace UsbHidJoystick {

void begin() {
  s_resetCount    = 0;
  s_resetWindowAt = HAL_GetTick();
  s_stormDetected = false;
  s_down          = false;

  __HAL_RCC_USB_CLK_ENABLE();

  // Ask the host to re-enumerate: pull D+ low through the fixed 1.5 k
  // pull-up for a moment, then let it go.
  pinMode(PA12, OUTPUT);
  digitalWrite(PA12, LOW);
  delay(5);
  pinMode(PA12, INPUT);
  pinMode(PA11, INPUT);

  USB->CNTR = USB_CNTR_FRES;
  delay(1);
  USB->CNTR = 0;
  delay(1);
  USB->ISTR = 0;
  resetEndpoints();          // also sets DADDR.EF, which keeps the HK32 from asserting RESET forever

  NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, cfg::USB_IRQ_PRIORITY);
  NVIC_ClearPendingIRQ(USB_LP_CAN1_RX0_IRQn);
  NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  USB->CNTR = kIntMask;
}

void service(uint32_t now) {
  if (s_stormDetected) {
    NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    USB->CNTR       = USB_CNTR_FRES | USB_CNTR_PDWN;
    s_resetCount    = 0;
    s_stormDetected = false;
    s_down          = true;
    s_devState      = kDefault;
    s_retryAt       = now + cfg::USB_RETRY_MS;
    return;
  }
  if (s_down && static_cast<int32_t>(now - s_retryAt) >= 0) {
    begin();
  }
}

bool configured() {
  return !s_down && s_devState == kConfigured;
}

bool send(const uint8_t* report) {
  if (!configured() || s_busy) {
    return false;
  }
  memcpy(s_report, report, REPORT_SIZE);
  s_busy = true;
  pmaWrite(kEp1InBuf, s_report, REPORT_SIZE);
  bdt(1, kCountTx) = REPORT_SIZE;
  epSetStatTx(1, USB_EP_TX_VALID);
  return true;
}

}  // namespace UsbHidJoystick

#endif  // !USBCON
