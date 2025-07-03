import { pos_to_motor_pos_samples_deg, spool_r_in_origin_first_guess, norm, subtract, add, scale } from './kinematics.js';
import { guessed_anchors } from './guessedData.js';


class MoveCommander {
    constructor({uri = null, commandHandler = null} = {}) {
        this.uri = uri;
        this.commandHandler = commandHandler;
        this.websocket = null;
        this.last_speed_mm_per_min = 1000.0;
        this.commands = [];
        this.dt = 1.0 / 200.0; // Fallback from Python version
        this.current_angles_rad = [0.0, 0.0, 0.0];
        this.anchors_mm = guessed_anchors;
        this.current_pos_mm = { 'X': 0.0, 'Y': 0.0, 'Z': 0.0 };
        this.spool_radius_mm = spool_r_in_origin_first_guess;
        this.low_axis_max_force = 20.0;
        this.use_flex = false;
        this.spool_buildup_factor = 0.0;

    }

    async connect() {
        if (this.commandHandler || !this.uri) {
            return;
        }
        if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
            return;
        }
        this.websocket = new WebSocket(this.uri);
        return new Promise((resolve, reject) => {
            this.websocket.onopen = () => {
                console.log("MoveCommander connected to websocket.");
                // Start a task to drain incoming messages, like in the Python version
                this.websocket.onmessage = () => { /* discard */ };
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
        if (this.websocket) {
            this.websocket.send(JSON.stringify(message));
        } else if (this.commandHandler) {
            this.commandHandler(message);
        }
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

                    const extrusion_delta_mm = command.E || 0.0;

                    if (!has_move && extrusion_delta_mm === 0.0) {
                        continue;
                    }

                    const start_pos_mm_arr = axesXYZ.map(ax => this.current_pos_mm[ax]);
                    const target_pos_mm_arr = axesXYZ.map(ax => target_pos_mm[ax]);
                    const distance_mm = norm(subtract(target_pos_mm_arr, start_pos_mm_arr));
                    const speed_mm_per_s = command.speed / 60.0;
                    const duration_s = (distance_mm > 1e-6 && speed_mm_per_s > 1e-6) ? distance_mm / speed_mm_per_s : 0;
                    const num_steps = duration_s > 0 ? Math.ceil(duration_s / this.dt) : 0;

                    if (num_steps > 0) {
                        console.log(`Executing G1 move to ${JSON.stringify(target_pos_mm)} (mm) over ${duration_s.toFixed(2)}s in ${num_steps} time steps.`);
                        const extrusion_per_step = extrusion_delta_mm / num_steps;
                        let final_angles_rad = null;

                        for (let i = 1; i <= num_steps; i++) {
                            const t = i / num_steps;
                            const interp_pos_mm_arr = add(start_pos_mm_arr, scale(subtract(target_pos_mm_arr, start_pos_mm_arr), t));
                            const motor_positions_deg = pos_to_motor_pos_samples_deg(this.anchors_mm, [interp_pos_mm_arr], this.low_axis_max_force, this.use_flex, this.spool_buildup_factor);
                            const interp_angles_rad = scale(motor_positions_deg[0], Math.PI / 180.0);
                            final_angles_rad = interp_angles_rad;

                            const interpolated_cmd = { type: 'Move' };
                            axesABC.forEach((axis, idx) => interpolated_cmd[axis] = interp_angles_rad[idx]);

                            if (extrusion_per_step > 0.0) {
                                interpolated_cmd['E'] = extrusion_per_step;
                            }

                            await this.sendCommand(interpolated_cmd);
                            await new Promise(resolve => setTimeout(resolve, this.dt * 1000));
                        }

                        if (final_angles_rad) {
                            this.current_angles_rad = final_angles_rad;
                        }
                    } else { // num_steps === 0
                        let target_angles_rad;
                        if (has_move) {
                            const pos_mm = [axesXYZ.map(ax => target_pos_mm[ax])];
                            const motor_positions_deg = pos_to_motor_pos_samples_deg(this.anchors_mm, pos_mm, this.low_axis_max_force, this.use_flex, this.spool_buildup_factor);
                            target_angles_rad = scale(motor_positions_deg[0], Math.PI / 180.0);
                            this.current_angles_rad = target_angles_rad;
                        } else {
                            target_angles_rad = this.current_angles_rad;
                        }

                        const cmd = { type: 'Move' };
                        axesABC.forEach((axis, idx) => cmd[axis] = target_angles_rad[idx]);

                        if (extrusion_delta_mm > 0.0) {
                            cmd['E'] = extrusion_delta_mm;
                        }

                        if (has_move || extrusion_delta_mm > 0.0) {
                            await this.sendCommand(cmd);
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
                    // In python, spool_radius_mm is an array, so this is element-wise division.
                    // Here, spool_radius_mm is an array, so we need to do it element-wise.
                    const delta_angles_rad = line_deltas_mm.map((delta, i) => delta / this.spool_radius_mm[i]);
                    const target_angles_rad = add(this.current_angles_rad, delta_angles_rad);
                    const line_distance_mm = norm(line_deltas_mm);
                    const speed_mm_per_s = (command.speed || 1000.0) / 60.0;
                    const duration_s = line_distance_mm / speed_mm_per_s;
                    const num_steps = Math.ceil(duration_s / this.dt);

                    console.log(`Executing G6 move with deltas ${line_deltas_mm} (mm) over ${duration_s.toFixed(2)}s in ${num_steps} time steps.`);

                    const deltas_rad = subtract(target_angles_rad, this.current_angles_rad);
                    if (num_steps > 0) {
                        for (let i = 1; i <= num_steps; i++) {
                            const t = i / num_steps;
                            const interpolated_cmd = { type: 'Move' };
                            const interp_angles = add(this.current_angles_rad, scale(deltas_rad, t));
                            axesABC.forEach((axis, idx) => interpolated_cmd[axis] = interp_angles[idx]);
                            await this.sendCommand(interpolated_cmd);
                            await new Promise(resolve => setTimeout(resolve, this.dt * 1000));
                        }
                    }

                    this.current_angles_rad = target_angles_rad;

                    const cmd = { type: 'Add to reference' };
                    axesABC.forEach((axis, idx) => cmd[axis] = deltas_rad[idx]);
                    await this.sendCommand(cmd);
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

const commander = new MoveCommander({
    uri: null,
    commandHandler: (msg) => postMessage(msg)
});

self.onmessage = function(e) {
    if (e.data.type === 'start' && e.data.gcode) {
        commander.run(e.data.gcode);
    } else if (e.data.type === 'set_dt' && e.data.dt) {
        commander.dt = e.data.dt;
    } else if (e.data.type === 'set_uri') {
        commander.uri = e.data.uri;
        if (commander.uri) {
            commander.commandHandler = null;
        }
    }
};

export default MoveCommander;
