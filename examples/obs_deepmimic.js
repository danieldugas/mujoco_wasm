import { MuJoCoDemo } from './mujocoWebDemo.js';
import { quat_to_rpy } from './transforms.js';


/** @param {MuJoCoDemo} mjd*/
export function getDeepmimicObs(mjd, policy, ENV, ACTOBS_LOG, ZEROACT, ONEACT) {
    let N_FRAMES = 100;
    // ONEACT = true; 
    // N_FRAMES = 5;
    // ACTOBS_LOG = true;

    let timestep = mjd.model.getOptions().timestep;
    let target_xyz = [14.0, 0.0, 1.3];
    let initial_z = 0.8; // from flagrun
    mjd.DBG_div_element.innerHTML = "";
    let torso_name = "torso";
    let lfoot_name = "left_foot";
    let rfoot_name = "right_foot";
    let floor_name = "floor";
    if (ENV == "DEEP_MIMIC_HUMANOID") {
        torso_name = "chest";
        lfoot_name = "left_ankle";
        rfoot_name = "right_ankle";
    } else if (ENV == "DEEP_MIMIC_UNITREE_G1") {
        torso_name = "pelvis";
        lfoot_name = "left_foot";
        rfoot_name = "right_foot";
    }

    // Get observation vector (see gym_forward_walker.py calc_state())
    // 8 Body pose, target values
    let mean_xyz = [0.0, 0.0, 0.0];
    let n = 0;
    let torso_xyz = [0.0, 0.0, 0.0];
    let torso_wxyz = [0.0, 0.0, 0.0, 0.0];
    let torso_rpy = [0.0, 0.0, 0.0];
    let torso_vel = [0.0, 0.0, 0.0];
    let torso_rotvel = [0.0, 0.0, 0.0];
    for (let b = 0; b < mjd.model.nbody; b++) {
        if (mjd.bodies[b]) {
        n += 1;
        mean_xyz[0] += mjd.simulation.xpos[b*3+0];
        mean_xyz[1] += mjd.simulation.xpos[b*3+1];
        mean_xyz[2] += mjd.simulation.xpos[b*3+2];
        if (mjd.bodies[b].name == torso_name) {
            torso_xyz[0] = mjd.simulation.xpos[b*3+0];
            torso_xyz[1] = mjd.simulation.xpos[b*3+1];
            torso_xyz[2] = mjd.simulation.xpos[b*3+2];
            torso_vel[0] = mjd.simulation.cvel[b*6+3];
            torso_vel[1] = mjd.simulation.cvel[b*6+4];
            torso_vel[2] = mjd.simulation.cvel[b*6+5];
            torso_rotvel[0] = mjd.simulation.cvel[b*6+0];
            torso_rotvel[1] = mjd.simulation.cvel[b*6+1];
            torso_rotvel[2] = mjd.simulation.cvel[b*6+2];
            torso_wxyz[0] = mjd.simulation.xquat[b*4+0];
            torso_wxyz[1] = mjd.simulation.xquat[b*4+1];
            torso_wxyz[2] = mjd.simulation.xquat[b*4+2];
            torso_wxyz[3] = mjd.simulation.xquat[b*4+3];
            // let quat = new THREE.Quaternion(torso_wxyz[1], torso_wxyz[2], torso_wxyz[3], torso_wxyz[0]);
            // let euler = new THREE.Euler();
            // euler.setFromQuaternion(quat);
            // torso_rpy[0] = euler.x;
            // torso_rpy[1] = euler.y;
            // torso_rpy[2] = euler.z;
            let rpy = quat_to_rpy(torso_wxyz[0], torso_wxyz[1], torso_wxyz[2], torso_wxyz[3]);
            torso_rpy[0] = rpy[0];
            torso_rpy[1] = rpy[1];
            torso_rpy[2] = rpy[2];
        }
        }
    }
    mean_xyz[0] /= n;
    mean_xyz[1] /= n;
    mean_xyz[2] /= n;
    mjd.DBG_div_element.innerHTML += "vx: " + torso_vel[0].toFixed(2) + "<br>vy: " + torso_vel[1].toFixed(2) + "<br>vz: " + torso_vel[2].toFixed(2);
    mjd.DBG_div_element.innerHTML += "<br>roll: " + torso_rpy[0].toFixed(2) + "<br>pitch: " + torso_rpy[1].toFixed(2) + "<br>yaw: " + torso_rpy[2].toFixed(2);
    let target_theta = Math.atan2(target_xyz[1] - mean_xyz[1], target_xyz[0] - mean_xyz[0]);
    let angle_to_target = target_theta - torso_rpy[2];
    let rot_minus_yaw = [
        [Math.cos(-torso_rpy[2]), -Math.sin(-torso_rpy[2]), 0],
        [Math.sin(-torso_rpy[2]),  Math.cos(-torso_rpy[2]), 0],
        [0, 0, 1]
    ];
    let vx = rot_minus_yaw[0][0] * torso_vel[0] + rot_minus_yaw[0][1] * torso_vel[1] + rot_minus_yaw[0][2] * torso_vel[2];
    let vy = rot_minus_yaw[1][0] * torso_vel[0] + rot_minus_yaw[1][1] * torso_vel[1] + rot_minus_yaw[1][2] * torso_vel[2];
    let vz = rot_minus_yaw[2][0] * torso_vel[0] + rot_minus_yaw[2][1] * torso_vel[1] + rot_minus_yaw[2][2] * torso_vel[2];
    // 34 Joint values (rel pos, rel vel)
    let jnt_dbg = [];
    let qpos = [];
    let qvel = [];
    let qpos_rel = [];
    for (let jidx = 1; jidx < mjd.model.njnt; jidx++) { // joint 0 is root
        let jnt_name = "?";
        if (mjd.model.name_jntadr[jidx] in mjd.model.DBG_name_index) {
        jnt_name = mjd.model.DBG_name_index[mjd.model.name_jntadr[jidx]];
        }
        if (jnt_name == "root") {
        continue;
        }
        let jnt_pos = mjd.simulation.qpos[mjd.model.jnt_qposadr[jidx]];
        let jnt_vel = mjd.simulation.qvel[mjd.model.jnt_dofadr[jidx]];
        let jnt_lower_lim = mjd.model.jnt_range[jidx*2];
        let jnt_upper_lim = mjd.model.jnt_range[jidx*2+1];
        // to joint observations (34,), assumes motor joints with 0 max vel
        let jnt_mid_lim = (jnt_upper_lim + jnt_lower_lim) / 2;
        let jnt_rel_pos = 2 * (jnt_pos - jnt_mid_lim) / (jnt_upper_lim - jnt_lower_lim);
        qpos.push(jnt_pos);
        qvel.push(jnt_vel);
        qpos_rel.push(jnt_rel_pos);
    }
    // 2 feet contact values
    // mjd.simulation.contact;
    let is_left_foot_contact = 0;
    let is_right_foot_contact = 0;
    for (let i = 0; i < mjd.simulation.contact.length; i++) {
        let contact = mjd.simulation.contact[i];
        let geom1 = mjd.model.DBG_name_index[mjd.model.name_geomadr[contact.geom1]];
        let geom2 = mjd.model.DBG_name_index[mjd.model.name_geomadr[contact.geom2]];
        if ((geom1 == lfoot_name || geom2 == lfoot_name) && (geom1 == floor_name || geom2 == floor_name)) {
        is_left_foot_contact = 1;
        }
        if ((geom1 == rfoot_name || geom2 == rfoot_name) && (geom1 == floor_name || geom2 == floor_name)) {
        is_right_foot_contact = 1;
        }
        // console.log("Contact", i, "Geom1", geom1, " ", contact.geom1, " Geom2", geom2, " ", contact.geom2);
    }
    mjd.DBG_div_element.innerHTML += "<br>lfoot_contact: " + is_left_foot_contact + "<br>rfoot_contact: " + is_right_foot_contact;

    let obs = [];
    if (ENV == "DEEP_MIMIC_HUMANOID") {
        let S = 0.1;
        for (let i = 0; i < qpos.length; i++) { obs.push(qpos[i]); }
        for (let i = 0; i < qvel.length; i++) { obs.push(qvel[i] * S); }
        // torso obs
        obs.push(torso_rpy[0] * S);
        obs.push(torso_rpy[1] * S);
        obs.push(vx * S);
        obs.push(vy * S);
        obs.push(vz * S);
        obs.push(torso_rotvel[0] * S);
        obs.push(torso_rotvel[1] * S);
        obs.push(torso_rotvel[2] * S);
        // foot contacts
        obs.push(is_right_foot_contact);
        obs.push(is_left_foot_contact);
    } else if (ENV == "DEEP_MIMIC_UNITREE_G1") {
        let S = 0.1;
        // qpos, qvel
        // let qpos_idx = [7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,   32, 33, 34, 35, 36   ] // exclude root and hand joints
        // let qvel_idx = [6, 7, 8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,   31, 32, 33, 34, 35   ]
        // for (let i = 0; i < qpos_idx.length; i++) { obs.push(mjd.simulation.qpos[qpos_idx[i]]); }
        // for (let i = 0; i < qvel_idx.length; i++) { obs.push(mjd.simulation.qvel[qvel_idx[i]] * S); }
        for (let i = 7; i < mjd.simulation.qpos.length; i++) { obs.push(mjd.simulation.qpos[i]); }
        for (let i = 6; i < mjd.simulation.qvel.length; i++) { obs.push(mjd.simulation.qvel[i] * S); }
        // torso obs
        obs.push(torso_rpy[0] * S);
        obs.push(torso_rpy[1] * S);
        obs.push(vx * S);
        obs.push(vy * S);
        obs.push(vz * S);
        obs.push(torso_rotvel[0] * S);
        obs.push(torso_rotvel[1] * S);
        obs.push(torso_rotvel[2] * S);
        // foot contacts
        obs.push(is_right_foot_contact);
        obs.push(is_left_foot_contact);
        // phase obs
        obs.push(((1.0 * (mjd.simulation.time / timestep) + mjd.mocap_start_frame) / mjd.mocap_len) % 1.0);
        // player action
        // obs.push(1.0); // hx
        // obs.push(0.0); // hy
        // obs.push(1.0); // walk
        // obs.push(0.0); // run
        // obs.push(0.0); // action
    }

    let a = policy.act(obs);

    if (ENV == "DEEP_MIMIC_HUMANOID") {
        for (let i = 0; i < a.length; i++) {
            a[i] = Math.min(0.5, Math.max(-0.5, a[i]));
        }
    } else if (ENV == "DEEP_MIMIC_UNITREE_G1") {
    }

    // zero the control signal for debugging sim
    if (ZEROACT) {
        a = a.map(x => 0.0)
    }
    if (ONEACT) {
        a = a.map(x => 1.0)
    }

    // log the mujoco action
    if (ACTOBS_LOG) {
        mjd.DBG_actobs_log += "Frame " + mjd.DBG_actobs_log_frames + "\n";
        mjd.DBG_actobs_log += "MJ time: " + mjd.mujoco_time.toFixed(3) + "\n";
        mjd.DBG_actobs_log += "Obs: \n";
        for (let i = 0; i < obs.length; i++) {
        mjd.DBG_actobs_log += i.toString().padStart(2, '0') + ": " + obs[i].toFixed(3) + "\n";
        }
        mjd.DBG_actobs_log += "Act: \n";
        for (let i = 0; i < a.length; i++) {
        mjd.DBG_actobs_log += i.toString().padStart(2, '0') + ": " + a[i].toFixed(3) + "\n";
        }
        mjd.DBG_actobs_log += "\n";

        mjd.DBG_actobs_qlog += mjd.simulation.time.toFixed(3) + ",";
        mjd.DBG_actobs_qlog += obs.length + ",";
        for (let i = 0; i < obs.length; i++) {
        mjd.DBG_actobs_qlog += obs[i].toFixed(5) + ",";
        }
        mjd.DBG_actobs_qlog += a.length + ",";
        for (let i = 0; i < a.length; i++) {
        mjd.DBG_actobs_qlog += a[i].toFixed(5) + ",";
        }
        mjd.DBG_actobs_qlog += "1" + ",";
        mjd.DBG_actobs_qlog += "0";
        mjd.DBG_actobs_qlog += "\n";
        mjd.DBG_actobs_log_frames += 1;
        if (mjd.DBG_actobs_log_frames == N_FRAMES) {
        console.log(mjd.DBG_actobs_log);
        console.log(mjd.DBG_actobs_qlog);
        mjd.params["paused"] = true;
        }
    }

    // pre-process actions to mujoco
    if (ENV == "DEEP_MIMIC_UNITREE_G1") {
        for (let i = 0; i < a.length; i++) {
            a[i] = a[i] * 20.;
        }
        for (let i = 0; i < 14; i++) {
            a.push(0.0);
        }
        for (let i = 0; i < a.length; i++) {
            a[i] = Math.min(mjd.model.actuator_ctrlrange[i*2+1], Math.max(mjd.model.actuator_ctrlrange[i*2], a[i]));
        }
    }

    if (false) {
        // if mean_z is below 0.4, add a div with red text that says "WASTED"
        if (mean_xyz[2] < 0.4) {
            if (mjd.WASTED_DIV) {
                // make background progressively darker
                let wastedDiv = mjd.WASTED_DIV;
                let wastedDivColor = wastedDiv.style.backgroundColor;
                let wastedDivColorParts = wastedDivColor.split(",");
                let wastedDivAlpha = parseFloat(wastedDivColorParts[3].split(")")[0]);
                wastedDivAlpha = Math.min(0.6, wastedDivAlpha + 0.01);
                wastedDiv.style.backgroundColor = "rgba(0, 0, 0, " + wastedDivAlpha + ")";
            } else {
                let wastedDiv = document.createElement("div");
                // create text element in the middle of div
                let text = document.createElement("p");
                text.innerHTML = "WASTED";
                text.style.color = "red";
                text.style.fontSize = "100px";
                wastedDiv.appendChild(text);
                wastedDiv.style.display = "flex";
                wastedDiv.style.justifyContent = "center";
                wastedDiv.style.alignItems = "center";
                wastedDiv.style.position = "absolute";
                wastedDiv.style.top = "50%";
                wastedDiv.style.left = "50%";
                wastedDiv.style.transform = "translate(-50%, -50%)";
                wastedDiv.style.color = "red";
                wastedDiv.style.backgroundColor = "rgba(255, 255, 255, 0.0)";
                wastedDiv.style.width = "100%";
                wastedDiv.style.height = "100%";
                document.body.appendChild(wastedDiv);
                mjd.WASTED_DIV = wastedDiv;
            }
        } else {
            if (mjd.WASTED_DIV) {
                document.body.removeChild(mjd.WASTED_DIV);
                mjd.WASTED_DIV = null;
            }
        }
    }

    // Apply the control signal to the simulation
    for (let i = 0; i < a.length; i++) {
        mjd.simulation.ctrl[i] = a[i] * mjd.params["rlactscale"];
    }
}