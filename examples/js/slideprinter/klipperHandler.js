// KlipperHandler: Connects to a raw-bytes WebSocket and logs incoming frames.
// This is a first step to confirm we receive MCU bytes from Klipper.

export function connectKlipperRaw(url) {
  const ws = new WebSocket(url);
  ws.binaryType = 'arraybuffer';

  ws.onopen = () => {
    console.log(`KlipperHandler: connected to ${url}`);
  };

  ws.onmessage = (event) => {
    if (event.data instanceof ArrayBuffer) {
      const bytes = new Uint8Array(event.data);
      // Print as a compact hex string (truncate for readability)
      let hex = '';
      const maxLen = Math.min(bytes.length, 128);
      for (let i = 0; i < maxLen; i++) {
        hex += bytes[i].toString(16).padStart(2, '0');
      }
      if (bytes.length > maxLen) {
        hex += '…';
      }
      console.log(`KlipperHandler: ${bytes.length} bytes: ${hex}`);
    } else if (typeof event.data === 'string') {
      console.log(`KlipperHandler (text): ${event.data.slice(0, 200)}${event.data.length > 200 ? '…' : ''}`);
    } else {
      console.log('KlipperHandler: received frame of unknown type');
    }
  };

  ws.onerror = (err) => {
    console.error('KlipperHandler websocket error:', err);
  };

  ws.onclose = () => {
    console.log('KlipperHandler: connection closed');
  };

  return {
    ws,
    close: () => ws.close(),
  };
}

