class MoveCommander {
    constructor(uri) {
        this.uri = uri;
        this.websocket = null;
        this.currentPosition = { A: 0, B: 0, C: 0 }; // Assuming spools are A, B, C
        this.feedRate = 1500; // mm/min
    }

    async connect() {
        if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
            return;
        }
        this.websocket = new WebSocket(this.uri);
        return new Promise((resolve, reject) => {
            this.websocket.onopen = () => {
                console.log("MoveCommander connected to websocket.");
                resolve();
            };
            this.websocket.onerror = (err) => {
                console.error("MoveCommander websocket error:", err);
                reject(err);
            };
            this.websocket.onmessage = (event) => {
                // The server sends world state, we can ignore it in the commander.
            };
            this.websocket.onclose = () => {
                console.log("MoveCommander websocket closed.");
            };
        });
    }

    parseGCode(gcode) {
        const lines = gcode.split('\n');
        const commands = [];
        for (const line of lines) {
            const trimmedLine = line.split(';')[0].trim();
            if (trimmedLine.length === 0) continue;

            const parts = trimmedLine.split(/\s+/);
            const command = parts[0].toUpperCase();

            if (command === 'G0' || command === 'G1') {
                const move = { type: 'Move' };
                let hasMove = false;
                for (let i = 1; i < parts.length; i++) {
                    const arg = parts[i];
                    const axis = arg.charAt(0).toUpperCase();
                    const value = parseFloat(arg.substring(1));

                    if (['A', 'B', 'C', 'E'].includes(axis)) {
                        move[axis] = value;
                        if (axis !== 'E') {
                            this.currentPosition[axis] = value;
                        }
                        hasMove = true;
                    } else if (axis === 'F') {
                        this.feedRate = value;
                    }
                }
                if (hasMove) {
                    commands.push(move);
                }
            } else if (command === 'G92') { // Set Position
                const move = { type: 'Add to reference' };
                let hasMove = false;
                for (let i = 1; i < parts.length; i++) {
                    const arg = parts[i];
                    const axis = arg.charAt(0).toUpperCase();
                    const value = parseFloat(arg.substring(1));
                    if (['A', 'B', 'C'].includes(axis)) {
                        move[axis] = this.currentPosition[axis] - value;
                        this.currentPosition[axis] = value;
                        hasMove = true;
                    }
                }
                if (hasMove) {
                    commands.push(move);
                }
            }
        }
        return commands;
    }

    async run(gcode) {
        try {
            await this.connect();
            const commands = this.parseGCode(gcode);
            for (const command of commands) {
                const message = {
                    action: 'gcode',
                    command: command
                };
                this.websocket.send(JSON.stringify(message));
                await new Promise(resolve => setTimeout(resolve, 10)); // 10ms delay between commands
            }
            postMessage({ type: 'done' });
        } catch (e) {
            console.error("MoveCommander failed to run:", e);
            postMessage({ type: 'error', message: e.message });
        } finally {
            if (this.websocket) {
                this.websocket.close();
            }
        }
    }
}

const uri = "ws://localhost:8766";
const commander = new MoveCommander(uri);

self.onmessage = function(e) {
    if (e.data.type === 'start' && e.data.gcode) {
        commander.run(e.data.gcode);
    }
};
