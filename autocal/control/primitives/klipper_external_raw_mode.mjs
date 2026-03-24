export function createKlipperExternalRawModeHandlers({
  pushExternalCommands,
  scheduleReconnect,
} = {}) {
  const pushCommands = typeof pushExternalCommands === 'function' ? pushExternalCommands : null;
  const reconnect = typeof scheduleReconnect === 'function' ? scheduleReconnect : null;

  return {
    onCommand(command) {
      if (pushCommands) {
        pushCommands([command]);
      }
    },
    onClose() {
      if (reconnect) {
        reconnect('connection closed');
      }
    },
  };
}
