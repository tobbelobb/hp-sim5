// --- Math Helpers ---
function norm(arr) {
    return Math.sqrt(arr.reduce((sum, val) => sum + val * val, 0));
}
function subtract(a, b) {
    return a.map((val, i) => val - b[i]);
}
function add(a, b) {
    return a.map((val, i) => val + b[i]);
}
function scale(a, s) {
    return a.map(val => val * s);
}

// --- Kinematics Placeholder ---
function pos_to_motor_pos_samples_deg(anchors_mm, pos_mm, low_axis_max_force, use_flex, spool_buildup_factor) {
    console.error("Kinematics (pos_to_motor_pos_samples_deg) not implemented in JS. G1 and G92 commands will not work correctly.");
    return [[0, 0, 0]]; // Return dummy data
}


class MoveCommander {
    constructor(uri) {
        this.uri = uri;
        this.websocket = null;
        this.last_speed_mm_per_min = 1000.0;
        this.commands = [];
        this.dt = 1.0 / 200.0; // Fallback from Python version
        this.current_angles_rad = [0.0, 0.0, 0.0];
        this.anchors_mm = []; // Requires data from guessed_data.py
        this.current_pos_mm = { 'X': 0.0, 'Y': 0.0, 'Z': 0.0 };
        this.spool_radius_mm = 10.0; // Placeholder, requires data from guessed_data.py
        this.low_axis_max_force = 20.0;
        this.use_flex = false;
        this.spool_buildup_factor = 0.0;
    }

    async connect() {
        if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
            return;
        }
        this.websocket = new WebSocket(this.uri);
        return new Promise((resolve, reject) => {
            this.websocket.onopen = () => {
                console.log("MoveCommander connected to websocket.");
                // Start a task to drain incoming messages, like in the Python version
                this.websocket.onmessage = (event) => { /* discard */ };
                resolve();
            };
            this.websocket.onerror = (err) => {
                console.error("MoveCommander websocket error:", err);
                reject(err);
            };
            this.websocket.onclose = () => {
                console.log("MoveCommander websocket closed.");
            };
        });
    }

    _parse_gcode(gcode) {
        const commands = [];
        const lines = gcode.split('\n');
        for (const line of lines) {
            if (line.startsWith('G1')) {
                const command = this._parse_g1_command(line);
                if (command) commands.push(command);
            } else if (line.startsWith('G6')) {
                const command = this._parse_g6_command(line);
                if (command) commands.push(command);
            } else if (line.startsWith('G92')) {
                const command = this._parse_g92_command(line);
                if (command) commands.push(command);
            }
        }
        return commands;
    }

    _parse_g1_command(line) {
        const parts = line.trim().split(/\s+/);
        const command = { type: 'G1' };
        for (const part of parts) {
            const code = part.charAt(0).toUpperCase();
            const value = parseFloat(part.substring(1));
            if (code === 'X' || code === 'Y' || code === 'Z' || code === 'E') {
                command[code] = value;
            } else if (code === 'F') {
                this.last_speed_mm_per_min = value;
            }
        }
        command['speed'] = this.last_speed_mm_per_min;
        return command;
    }

    _parse_g92_command(line) {
        const parts = line.trim().split(/\s+/);
        const command = { type: 'G92' };
        let has_coords = false;
        for (const part of parts) {
            const code = part.charAt(0).toUpperCase();
            if (code === 'X' || code === 'Y' || code === 'Z') {
                command[code] = parseFloat(part.substring(1));
                has_coords = true;
            }
        }
        return has_coords ? command : null;
    }

    _parse_g6_command(line) {
        const parts = line.trim().split(/\s+/);
        const command = { type: 'G6' };
        for (const part of parts) {
            const code = part.charAt(0).toUpperCase();
            const value = parseFloat(part.substring(1));
            if (code === 'A' || code === 'B' || code === 'C') {
                command[code] = value;
            } else if (code === 'F') {
                command['speed'] = value;
            }
        }
        return command;
    }

    async sendCommand(command) {
        const message = { action: 'gcode', command: command };
        this.websocket.send(JSON.stringify(message));
    }

    async run(gcode) {
        try {
            await this.connect();
            this.commands = this._parse_gcode(gcode);
            const axesABC = ['A', 'B', 'C'];
            const axesXYZ = ['X', 'Y', 'Z'];

            for (const command of this.commands) {
                if (command.type === 'G1') {
                    const target_pos_mm = { ...this.current_pos_mm };
                    let has_move = false;
                    axesXYZ.forEach(axis => {
                        if (axis in command) {
                            target_pos_mm[axis] = command[axis];
                            has_move = true;
                        }
                    });

                    const start_pos_mm_arr = axesXYZ.map(ax => this.current_pos_mm[ax]);
                    const target_pos_mm_arr = axesXYZ.map(ax => target_pos_mm[ax]);
                    const distance_mm = norm(subtract(target_pos_mm_arr, start_pos_mm_arr));
                    const speed_mm_per_s = command.speed / 60.0;
                    const duration_s = (distance_mm > 1e-6 && speed_mm_per_s > 1e-6) ? distance_mm / speed_mm_per_s : 0;
                    const num_steps = duration_s > 0 ? Math.ceil(duration_s / this.dt) : 0;

                    if (num_steps > 0) {
                        for (let i = 1; i <= num_steps; i++) {
                            const t = i / num_steps;
                            const interp_pos_mm_arr = add(start_pos_mm_arr, scale(subtract(target_pos_mm_arr, start_pos_mm_arr), t));
                            const motor_positions_deg = pos_to_motor_pos_samples_deg(this.anchors_mm, [interp_pos_mm_arr], this.low_axis_max_force, this.use_flex, this.spool_buildup_factor);
                            const interp_angles_rad = scale(motor_positions_deg[0], Math.PI / 180.0);

                            const interpolated_cmd = { type: 'Move' };
                            axesABC.forEach((axis, idx) => interpolated_cmd[axis] = interp_angles_rad[idx]);
                            await this.sendCommand(interpolated_cmd);
                            await new Promise(resolve => setTimeout(resolve, this.dt * 1000));
                        }
                    }
                    this.current_pos_mm = target_pos_mm;

                } else if (command.type === 'G92') {
                    const target_pos_mm = { ...this.current_pos_mm };
                    axesXYZ.forEach(axis => {
                        if (axis in command) target_pos_mm[axis] = command[axis];
                    });
                    const pos_mm = [axesXYZ.map(ax => target_pos_mm[ax])];
                    const target_angles_rad = scale(pos_to_motor_pos_samples_deg(this.anchors_mm, pos_mm, this.low_axis_max_force, this.use_flex, this.spool_buildup_factor)[0], Math.PI / 180.0);
                    const deltas_rad = subtract(target_angles_rad, this.current_angles_rad);
                    this.current_angles_rad = target_angles_rad;
                    this.current_pos_mm = target_pos_mm;

                    const cmd = { type: 'Add to reference' };
                    axesABC.forEach((axis, idx) => cmd[axis] = deltas_rad[idx]);
                    await this.sendCommand(cmd);

                } else if (command.type === 'G6') {
                    const line_deltas_mm = [command.A || 0, command.B || 0, command.C || 0];
                    const delta_angles_rad = scale(line_deltas_mm, 1.0 / this.spool_radius_mm);
                    const target_angles_rad = add(this.current_angles_rad, delta_angles_rad);
                    const line_distance_mm = norm(line_deltas_mm);
                    const speed_mm_per_s = (command.speed || 1000.0) / 60.0;
                    const duration_s = line_distance_mm / speed_mm_per_s;
                    const num_steps = Math.ceil(duration_s / this.dt);

                    if (num_steps > 0) {
                        const deltas_rad_step = subtract(target_angles_rad, this.current_angles_rad);
                        for (let i = 1; i <= num_steps; i++) {
                            const t = i / num_steps;
                            const interpolated_cmd = { type: 'Move' };
                            const interp_angles = add(this.current_angles_rad, scale(deltas_rad_step, t));
                            axesABC.forEach((axis, idx) => interpolated_cmd[axis] = interp_angles[idx]);
                            await this.sendCommand(interpolated_cmd);
                            await new Promise(resolve => setTimeout(resolve, this.dt * 1000));
                        }
                    }
                    this.current_angles_rad = target_angles_rad;
                }
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
