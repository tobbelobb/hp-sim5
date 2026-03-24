export function isKlipperRawBridgeMessage(msg) {
  if (!msg || typeof msg !== 'object') {
    return false;
  }
  return msg.action === 'klipper_parsed'
    || msg.action === 'klipper_clock'
    || msg.action === 'klipper_serial';
}
