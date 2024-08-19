import { MuJoCoDemo } from './mujocoWebDemo.js';
import { downloadAnyScenesFolder } from './mujocoUtils.js';
import { ExtractedPolicy } from './extracted_policy_run_polar_breeze.js';
import { getDeepmimicObs } from './obs_deepmimic.js';

let scene_file = 'deepmimic_unitree_g1_nocolmesh.xml'; // something is wrong with obj collision meshes, disabled for running
let policy = new ExtractedPolicy();
policy.test();

/** @param {MuJoCoDemo} mjd*/
async function downloadUnitreeG1(mjd) {
    let allFiles = [
        scene_file,
        "assets/pelvis.obj",
        "assets/pelvis_contour_link.obj",
        "assets/left_hip_pitch_link.obj",
        "assets/left_hip_roll_link.obj",
        "assets/left_hip_yaw_link.obj",
        "assets/left_knee_link.obj",
        "assets/left_ankle_pitch_link.obj",
        "assets/left_ankle_roll_link.obj",
        "assets/right_hip_pitch_link.obj",
        "assets/right_hip_roll_link.obj",
        "assets/right_hip_yaw_link.obj",
        "assets/right_knee_link.obj",
        "assets/right_ankle_pitch_link.obj",
        "assets/right_ankle_roll_link.obj",
        "assets/torso_link.obj",
        "assets/head_link.obj",
        "assets/left_shoulder_pitch_link.obj",
        "assets/left_shoulder_roll_link.obj",
        "assets/left_shoulder_yaw_link.obj",
        "assets/left_elbow_pitch_link.obj",
        "assets/left_elbow_roll_link.obj",
        "assets/right_shoulder_pitch_link.obj",
        "assets/right_shoulder_roll_link.obj",
        "assets/right_shoulder_yaw_link.obj",
        "assets/right_elbow_pitch_link.obj",
        "assets/right_elbow_roll_link.obj",
        "assets/logo_link.obj",
        "assets/left_palm_link.obj",
        "assets/left_zero_link.obj",
        "assets/left_one_link.obj",
        "assets/left_two_link.obj",
        "assets/left_three_link.obj",
        "assets/left_four_link.obj",
        "assets/left_five_link.obj",
        "assets/left_six_link.obj",
        "assets/right_palm_link.obj",
        "assets/right_zero_link.obj",
        "assets/right_one_link.obj",
        "assets/right_two_link.obj",
        "assets/right_three_link.obj",
        "assets/right_four_link.obj",
        "assets/right_five_link.obj",
        "assets/right_six_link.obj",
        "assets/pelvis.mtl",
        "assets/pelvis_contour_link.mtl",
        "assets/left_hip_pitch_link.mtl",
        "assets/left_hip_roll_link.mtl",
        "assets/left_hip_yaw_link.mtl",
        "assets/left_knee_link.mtl",
        "assets/left_ankle_pitch_link.mtl",
        "assets/left_ankle_roll_link.mtl",
        "assets/right_hip_pitch_link.mtl",
        "assets/right_hip_roll_link.mtl",
        "assets/right_hip_yaw_link.mtl",
        "assets/right_knee_link.mtl",
        "assets/right_ankle_pitch_link.mtl",
        "assets/right_ankle_roll_link.mtl",
        "assets/torso_link.mtl",
        "assets/head_link.mtl",
        "assets/left_shoulder_pitch_link.mtl",
        "assets/left_shoulder_roll_link.mtl",
        "assets/left_shoulder_yaw_link.mtl",
        "assets/left_elbow_pitch_link.mtl",
        "assets/left_elbow_roll_link.mtl",
        "assets/right_shoulder_pitch_link.mtl",
        "assets/right_shoulder_roll_link.mtl",
        "assets/right_shoulder_yaw_link.mtl",
        "assets/right_elbow_pitch_link.mtl",
        "assets/right_elbow_roll_link.mtl",
        "assets/logo_link.mtl",
        "assets/left_palm_link.mtl",
        "assets/left_zero_link.mtl",
        "assets/left_one_link.mtl",
        "assets/left_two_link.mtl",
        "assets/left_three_link.mtl",
        "assets/left_four_link.mtl",
        "assets/left_five_link.mtl",
        "assets/left_six_link.mtl",
        "assets/right_palm_link.mtl",
        "assets/right_zero_link.mtl",
        "assets/right_one_link.mtl",
        "assets/right_two_link.mtl",
        "assets/right_three_link.mtl",
        "assets/right_four_link.mtl",
        "assets/right_five_link.mtl",
        "assets/right_six_link.mtl",
    ];
    await downloadAnyScenesFolder(allFiles, mjd.mujoco, mjd.loadingDiv);
}

/** @param {MuJoCoDemo} mjd*/
function initMocapFrame(mjd) {
        // run
        // let init_qpos = [ 0.00000, 0.00000, 0.66352, 0.97888, -0.06168, 0.19450, 0.01226, -0.15590, 0.16549, 0.13046, 1.41712, 0.08498, -0.31234, -0.94887, 0.21348, 0.13386, 0.81427, -0.38318, -0.00510, -0.05324, -0.14178, 0.57539, -0.34317, 0.19292, -0.00000, 0.00000, 0.00000, -0.00000, -0.00000, -0.00000, -0.00000, -0.00000, 0.18981, -0.53566, 0.32439, 0.21625, -0.00000, -0.00000, -0.00000, 0.00000, 0.00000, -0.00000, -0.00000, 0.00000 ];
        // let init_qvel = [ 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000 ];
        // mjd.mocap_start_frame =  0 ;
        // mjd.mocap_len =  48 ;
        // frame 20
        let init_qpos = [ 0.99530, 0.00000, 0.71758, 0.98436, -0.02146, 0.17400, 0.01718, -0.88649, 0.02867, 0.22147, 0.29923, -0.26927, -0.29058, 0.18363, -0.04680, 0.03644, 1.08014, 0.36891, 0.38303, 0.07367, 0.70000, 0.85305, -0.45213, 0.26929, -0.00000, 0.00000, 0.00000, -0.00000, -0.00000, -0.00000, -0.00000, -0.00000, -0.38660, -0.53363, 0.36657, 0.05325, -0.00000, -0.00000, -0.00000, 0.00000, 0.00000, -0.00000, -0.00000, 0.00000 ];
        let init_qvel = [ 3.35408, 0.00000, -1.10594, -0.31716, -1.37514, 0.44255, 3.20421, -0.85443, 0.73147, 1.66654, -0.72885, 0.76704, -0.17734, 0.21057, 1.30334, 9.92347, -0.36906, 4.67384, -0.21975, 0.00000, -3.10441, 2.77632, -0.71732, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 5.20781, 4.17500, 4.52369, 4.93767, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000 ];
        mjd.mocap_start_frame =  20 ;
        mjd.mocap_len =  48 ;
        for (let i = 0; i < init_qpos.length; i++) {
            mjd.simulation.qpos[i] = init_qpos[i];
        }
        for (let i = 0; i < init_qvel.length; i++) {
            mjd.simulation.qvel[i] = init_qvel[i];
        }
}


/** @param {MuJoCoDemo} mjd*/
function obsAndActCallback(mjd) {
    let ENV = "DEEP_MIMIC_UNITREE_G1";
    let ACTOBS_LOG = false;
    let ZEROACT = false;
    let ONEACT = false;

    getDeepmimicObs(mjd, policy, ENV, ACTOBS_LOG, ZEROACT, ONEACT);
}


let mujocoDemo = new MuJoCoDemo(scene_file);
mujocoDemo.sceneLoaderCallback = downloadUnitreeG1;
mujocoDemo.mocapInitCallback = initMocapFrame;
mujocoDemo.obsAndActCallback = obsAndActCallback;
// set default camera position
mujocoDemo.camera.position.set(-2.0, 1.7, 1.2);
mujocoDemo.controls.target.set(0, 1.2, 1.0);
mujocoDemo.controls.update(); 
await mujocoDemo.init();