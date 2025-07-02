class MoveCommander {
    constructor() {
        this.currentPosition = { A: 0, B: 0, C: 0 }; // Assuming spools are A, B, C
        this.feedRate = 1500; // mm/min
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
        const commands = this.parseGCode(gcode);
        for (const command of commands) {
            // A real implementation would calculate move duration and wait.
            // For now, just post them. A small delay can simulate processing.
            postMessage(command);
            await new Promise(resolve => setTimeout(resolve, 10)); // 10ms delay between commands
        }
        postMessage({ type: 'done' });
    }
}

const commander = new MoveCommander();

self.onmessage = function(e) {
    if (e.data.type === 'start' && e.data.gcode) {
        commander.run(e.data.gcode);
    }
};
