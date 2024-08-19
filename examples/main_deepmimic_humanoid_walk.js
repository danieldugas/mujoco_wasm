import { MuJoCoDemo } from './mujocoWebDemo.js';
import { downloadAnyScenesFolder } from './mujocoUtils.js';
import { ExtractedPolicy } from './extracted_policy.js';
import { quat_to_rpy } from './transforms.js';
import { getDeepmimicObs } from './obs_deepmimic.js';

let scene_file = 'humanoid_deep_mimic.xml';
let policy = new ExtractedPolicy();
policy.test();

/** @param {MuJoCoDemo} mjd*/
async function downloadUnitreeG1(mjd) {
    let allFiles = [
        scene_file,
    ];
    await downloadAnyScenesFolder(allFiles, mjd.mujoco, mjd.loadingDiv);
}

/** @param {MuJoCoDemo} mjd*/
function initMocapFrame(mjd) {
      // walk frame 0
      // let init_qpos = [ 0.00000000e+00,  0.00000000e+00,  8.47532000e-01,  9.98678000e-01, 1.41040000e-02,  4.94230000e-02, -6.98000000e-04,  1.93749951e-02, 8.03725488e-03, -9.52390281e-02, -0.00000000e+00,  0.00000000e+00, -0.00000000e+00, -1.55535325e-01,  2.39194293e-01,  2.07396570e-01, 1.70571000e-01,  3.52963185e-01, -2.61068295e-01, -2.45605321e-01, 5.81348000e-01,  2.03520526e-02, -5.17574245e-01, -1.13763390e-01, -2.49116000e-01,  2.05562366e-02, -1.95344988e-02,  6.55269813e-02, -5.60635014e-02,  1.52095783e-01,  1.82742095e-01, -3.91532000e-01, 1.93116785e-01, -2.97891855e-01, -8.30571523e-02];
      // let init_qvel = [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.];
      // walk frame 14
      let init_qpos = [ 0.445316  , -0.011915  ,  0.867059  ,  0.997844  , -0.022831  , 0.046376  , -0.040443  ,  0.03361764, -0.00314016, -0.02899837, -0.        ,  0.        , -0.        , -0.32657944, -0.35447961, 0.42145908,  0.545359  ,  0.18011039,  0.27595953,  0.18995153, 0.146415  ,  0.09650251,  0.03675212, -0.10699356, -0.20607   , -0.06359675, -0.29446854,  0.13235988,  0.2107373 , -0.55985615, 0.24157671, -0.442624  ,  0.10157878,  0.02110246, -0.01248015];
      let init_qvel = [ 0.87330493,  0.02706108, -0.0849934 ,  0.30155512, -0.24171644, 0.23992617, -0.24136156,  0.0220603 , -0.01428208,  0.        , 0.        ,  0.        ,  0.58706723,  0.7541451 ,  0.03532035, 0.5524121 ,  0.03504714, -0.88143469, -1.2011517 , -0.26203048, -0.0599469 , -0.94703751, -0.01142519,  0.29089164, -0.25083826, 0.27046541, -0.2003035 , -0.35684099, -0.47253004, -1.18983206, 6.37165487,  0.1882424 , -0.26708774,  1.02664063];
      for (let i = 0; i < init_qpos.length; i++) {
        mjd.simulation.qpos[i] = init_qpos[i];
      }
      for (let i = 0; i < init_qvel.length; i++) {
        mjd.simulation.qvel[i] = init_qvel[i] * 0.6;
      }
}


/** @param {MuJoCoDemo} mjd*/
function obsAndActCallback(mjd) {
    let ENV = "DEEP_MIMIC_HUMANOID";
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