import { MuJoCoDemo } from './mujocoWebDemo.js';
import { downloadAnyScenesFolder } from './mujocoUtils.js';
import { ExtractedPolicy } from './extracted_policy_walk_absurd_snow.js';
import { getDeepmimicObs } from './obs_deepmimic.js';

let scene_file = 'deepmimic_unitree_g1.xml';
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
        // walk (original f10)
        let init_qpos = [ 0.15561, -0.01475, 0.73105, 0.99816, -0.04001, 0.04384, 0.01260, -0.22452, 0.17087, 0.07579, 1.23601, 0.28667, -0.02498, -0.35576, 0.14424, -0.09457, 0.37628, -0.14600, -0.08560, -0.07676, -0.07923, 0.22337, -0.16256, 1.15320, -0.00000, 0.00000, -0.00000, -0.00000, -0.00000, -0.00000, 0.00000, -0.00000, 0.08324, -0.19456, 0.27860, 1.36150, 0.00000, -0.00000, -0.00000, 0.00000, 0.00000, -0.00000, 0.00000, 0.00000 ]
        let init_qvel = [ 0.83513, -0.04000, 0.13633, -0.04764, -0.27942, 0.28129, -2.92039, 0.43303, -0.90585, 3.94202, 5.62355, -1.45153, 1.73314, -0.01858, -0.40540, -0.51570, -0.83053, -0.11082, 0.17579, 0.87497, -0.49160, 0.27796, 1.67104, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, -1.38442, -0.23420, 0.10698, -0.86603, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000 ]
        mjd.mocap_start_frame = 10;
        mjd.mocap_len = 76;
        // walk
        // let init_qpos = [ 0.56171, 0.00401, 0.71960, 0.99861, 0.00379, 0.05209, -0.00746, -0.50462, -0.05848, 0.07225, 0.29777, 0.00871, -0.02539, 0.12109, -0.00985, -0.17633, 0.53079, -0.26619, -0.18116, 0.09461, 0.24487, 0.15916, -0.19646, 1.38850, -0.00000, 0.00000, -0.00000, -0.00000, -0.00000, -0.00000, 0.00000, -0.00000, -0.23167, -0.32635, 0.26062, 1.00274, 0.00000, -0.00000, -0.00000, 0.00000, 0.00000, -0.00000, 0.00000, 0.00000 ];
        // let init_qvel = [ 1.05969, 0.12046, -0.02416, 1.09678, 0.16607, -0.43348, 0.38869, -1.14429, -1.24592, 1.46023, 0.84774, -0.14521, -0.93050, -1.97828, 0.19240, 4.17944, 0.95156, 0.35890, -0.01893, 0.17034, 1.32266, -4.20755, -0.32815, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.88232, 0.79876, 0.45065, 0.42284, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000 ];
        // mjd.mocap_start_frame = 40;
        // mjd.mocap_len = 76;
        // walk
        // let init_qpos = [ 0.28408, -0.01396, 0.74302, 0.99913, -0.01970, 0.03066, -0.02028, -0.53630, 0.18551, 0.12158, 1.19927, -0.04583, 0.12245, -0.08004, 0.10560, -0.11426, 0.24016, -0.23609, -0.07988, -0.03555, 0.13871, 0.20251, 0.02051, 1.34351, -0.00000, 0.00000, -0.00000, -0.00000, -0.00000, -0.00000, 0.00000, -0.00000, -0.17532, -0.27961, 0.41606, 1.12480, 0.00000, -0.00000, -0.00000, 0.00000, 0.00000, -0.00000, 0.00000, 0.00000 ];
        // let init_qvel = [ 0.71355, 0.01913, 0.00648, 0.23818, 0.00822, -0.56044, -1.22888, 0.20541, 0.74198, -3.04208, -2.89046, -0.04244, 1.28438, -0.27957, -0.10014, -0.57341, -0.53587, 0.02762, 0.19883, 1.44443, -0.11714, 1.73141, 0.75405, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, -1.73330, -0.60896, 0.89941, -1.26903, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000 ];
        // mjd.mocap_start_frame =  20 - 1;
        // mjd.mocap_len = 76;
        // run
        // let init_qpos = [ 0.00000, 0.00000, 0.66352, 0.97888, -0.06168, 0.19450, 0.01226, -0.15590, 0.16549, 0.13046, 1.41712, 0.08498, -0.31234, -0.94887, 0.21348, 0.13386, 0.81427, -0.38318, -0.00510, -0.05324, -0.14178, 0.57539, -0.34317, 0.19292, -0.00000, 0.00000, 0.00000, -0.00000, -0.00000, -0.00000, -0.00000, -0.00000, 0.18981, -0.53566, 0.32439, 0.21625, -0.00000, -0.00000, -0.00000, 0.00000, 0.00000, -0.00000, -0.00000, 0.00000 ];
        // let init_qvel = [ 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000 ];
        // mjd.mocap_start_frame =  0 ;
        // mjd.mocap_len =  48 ;
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
await mujocoDemo.init();