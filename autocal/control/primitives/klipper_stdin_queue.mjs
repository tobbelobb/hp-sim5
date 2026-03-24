export function createSequentialLineQueue({ onLine, onPrompt, onError } = {}) {
  const queue = [];
  let processing = false;

  const prompt = () => {
    if (typeof onPrompt === 'function') {
      onPrompt();
    }
  };

  async function processQueue() {
    if (processing) {
      return;
    }
    processing = true;
    try {
      while (queue.length > 0) {
        const next = queue.shift();
        // eslint-disable-next-line no-await-in-loop
        await next();
      }
    } finally {
      processing = false;
    }
  }

  function enqueue(line) {
    queue.push(async () => {
      if (typeof onLine === 'function') {
        await onLine(line);
      }
    });
    prompt();
    processQueue().catch((err) => {
      if (typeof onError === 'function') {
        onError(err);
      }
    });
  }

  return {
    enqueue,
    processQueue,
  };
}
