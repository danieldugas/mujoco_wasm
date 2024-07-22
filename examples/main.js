
import * as THREE           from 'three';
import { GUI              } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import { OrbitControls    } from '../node_modules/three/examples/jsm/controls/OrbitControls.js';
import { DragStateManager } from './utils/DragStateManager.js';
import { setupGUI, downloadExampleScenesFolder, loadSceneFromURL, getPosition, getQuaternion, toMujocoPos, standardNormal } from './mujocoUtils.js';
import   load_mujoco        from '../dist/mujoco_wasm.js';
import { SimpleReactivePolicy } from './policy.js';
import { ExtractedPolicy } from './extracted_policy.js';

// Load the MuJoCo Module
const mujoco = await load_mujoco();

var ENV = "DEEP_MIMIC_HUMANOID"
// Set up Emscripten's Virtual File System
var initialScene = "humanoid.xml";
var repeat_iter = 1;
if (ENV == "FLAGRUN") {
  initialScene = "humanoid_gym_roboschool_flagrun.xml";
  repeat_iter = 4;
} else if (ENV == "DEEP_MIMIC_HUMANOID") {
  initialScene = "humanoid_deep_mimic.xml";
  repeat_iter = 1; // Should be 6 but somehow 1 gives most accurate results
}
var ACTOBS_LOG = false;
var ZEROACT = false;
var ONEACT = false;
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
mujoco.FS.writeFile("/working/" + initialScene, await(await fetch("./examples/scenes/" + initialScene)).text());

export class MuJoCoDemo {
  constructor() {
    this.mujoco = mujoco;

    // Load in the state from XML
    this.model      = new mujoco.Model("/working/" + initialScene);
    this.state      = new mujoco.State(this.model);
    this.simulation = new mujoco.Simulation(this.model, this.state);

    // this.policy = new SimpleReactivePolicy();
    this.policy = new ExtractedPolicy();
    this.policy.test();

    this.model.DBG_name_index = {}; // added by Daniel to store joint names in an easier to access format than model.names
    this.DBG_div_element = document.createElement('div'); // added by Daniel to store html for debugging
    this.DBG_actobs_log = "";
    this.DBG_actobs_qlog = "";
    this.DBG_actobs_log_frames = 0;

    // Define Random State Variables
    this.params = { scene: initialScene, paused: false, help: false, ctrlnoiserate: 0.0, ctrlnoisestd: 0.0, rlactscale: 1.0, keyframeNumber: 0 };
    this.mujoco_time = 0.0;
    this.bodies  = {}, this.lights = {};
    this.tmpVec  = new THREE.Vector3();
    this.tmpQuat = new THREE.Quaternion();
    this.updateGUICallbacks = [];

    this.container = document.createElement( 'div' );
    document.body.appendChild( this.container );

    this.scene = new THREE.Scene();
    this.scene.name = 'scene';

    this.camera = new THREE.PerspectiveCamera( 45, window.innerWidth / window.innerHeight, 0.001, 100 );
    this.camera.name = 'PerspectiveCamera';
    this.camera.position.set(2.0, 1.7, 1.7);
    this.scene.add(this.camera);

    this.scene.background = new THREE.Color(0.15, 0.25, 0.35);
    this.scene.fog = new THREE.Fog(this.scene.background, 15, 25.5 );

    this.ambientLight = new THREE.AmbientLight( 0xffffff, 0.1 );
    this.ambientLight.name = 'AmbientLight';
    this.scene.add( this.ambientLight );

    this.renderer = new THREE.WebGLRenderer( { antialias: true } );
    this.renderer.setPixelRatio( window.devicePixelRatio );
    this.renderer.setSize( window.innerWidth, window.innerHeight );
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap; // default THREE.PCFShadowMap
    this.renderer.setAnimationLoop( this.render.bind(this) );

    this.container.appendChild( this.renderer.domElement );

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.7, 0);
    this.controls.panSpeed = 2;
    this.controls.zoomSpeed = 1;
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.10;
    this.controls.screenSpacePanning = true;
    this.controls.update();

    window.addEventListener('resize', this.onWindowResize.bind(this));

    // Initialize the Drag State Manager.
    this.dragStateManager = new DragStateManager(this.scene, this.renderer, this.camera, this.container.parentElement, this.controls);
  }

  set_mocap_keyframe() {
    // init qpos, qvel
    if (ENV == "DEEP_MIMIC_HUMANOID") {
      // walk frame 0
      // let init_qpos = [ 0.00000000e+00,  0.00000000e+00,  8.47532000e-01,  9.98678000e-01, 1.41040000e-02,  4.94230000e-02, -6.98000000e-04,  1.93749951e-02, 8.03725488e-03, -9.52390281e-02, -0.00000000e+00,  0.00000000e+00, -0.00000000e+00, -1.55535325e-01,  2.39194293e-01,  2.07396570e-01, 1.70571000e-01,  3.52963185e-01, -2.61068295e-01, -2.45605321e-01, 5.81348000e-01,  2.03520526e-02, -5.17574245e-01, -1.13763390e-01, -2.49116000e-01,  2.05562366e-02, -1.95344988e-02,  6.55269813e-02, -5.60635014e-02,  1.52095783e-01,  1.82742095e-01, -3.91532000e-01, 1.93116785e-01, -2.97891855e-01, -8.30571523e-02];
      // let init_qvel = [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.];
      // walk frame 14
      let init_qpos = [ 0.445316  , -0.011915  ,  0.867059  ,  0.997844  , -0.022831  , 0.046376  , -0.040443  ,  0.03361764, -0.00314016, -0.02899837, -0.        ,  0.        , -0.        , -0.32657944, -0.35447961, 0.42145908,  0.545359  ,  0.18011039,  0.27595953,  0.18995153, 0.146415  ,  0.09650251,  0.03675212, -0.10699356, -0.20607   , -0.06359675, -0.29446854,  0.13235988,  0.2107373 , -0.55985615, 0.24157671, -0.442624  ,  0.10157878,  0.02110246, -0.01248015];
      let init_qvel = [ 0.87330493,  0.02706108, -0.0849934 ,  0.30155512, -0.24171644, 0.23992617, -0.24136156,  0.0220603 , -0.01428208,  0.        , 0.        ,  0.        ,  0.58706723,  0.7541451 ,  0.03532035, 0.5524121 ,  0.03504714, -0.88143469, -1.2011517 , -0.26203048, -0.0599469 , -0.94703751, -0.01142519,  0.29089164, -0.25083826, 0.27046541, -0.2003035 , -0.35684099, -0.47253004, -1.18983206, 6.37165487,  0.1882424 , -0.26708774,  1.02664063];
      for (let i = 0; i < init_qpos.length; i++) {
        this.simulation.qpos[i] = init_qpos[i];
      }
      for (let i = 0; i < init_qvel.length; i++) {
        this.simulation.qvel[i] = init_qvel[i] * 0.6;
      }
    }
  }

  async init() {
    // Download the the examples to MuJoCo's virtual file system
    await downloadExampleScenesFolder(mujoco);

    // Initialize the three.js Scene using the .xml Model in initialScene
    [this.model, this.state, this.simulation, this.bodies, this.lights] =  
      await loadSceneFromURL(mujoco, initialScene, this);
    this.set_mocap_keyframe();
    this.simulation.forward(); // otherwise the first frame tends to be right before or after init position somehow

    this.gui = new GUI();
    setupGUI(this);
  }

  onWindowResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize( window.innerWidth, window.innerHeight );
  }

  render(timeMS) {
    this.controls.update();

    // check if simulation is initialized
    let is_initialized = true;
    for (let jidx = 0; jidx < this.model.njnt; jidx++) {
      if (this.model.name_jntadr[jidx] in this.model.DBG_name_index) {
      } else {
        is_initialized = false;
        break;
      }
    }

    let HUMANOID_RL = true; // indicates parts of the code that are specific to the humanoid flagrun env

    if (!this.params["paused"]) {
      let timestep = this.model.getOptions().timestep;
      if (timeMS - this.mujoco_time > 35.0) { this.mujoco_time = timeMS; }
      while (this.mujoco_time < timeMS) {

        // Jitter the control state with gaussian random noise
        if (this.params["ctrlnoisestd"] > 0.0) {
          let rate  = Math.exp(-timestep / Math.max(1e-10, this.params["ctrlnoiserate"]));
          let scale = this.params["ctrlnoisestd"] * Math.sqrt(1 - rate * rate);
          let currentCtrl = this.simulation.ctrl;
          for (let i = 0; i < currentCtrl.length; i++) {
            currentCtrl[i] = rate * currentCtrl[i] + scale * standardNormal();
            this.params["Actuator " + i] = currentCtrl[i];
          }
        }

        // Clear old perturbations, apply new ones.
        for (let i = 0; i < this.simulation.qfrc_applied.length; i++) { this.simulation.qfrc_applied[i] = 0.0; }
        let dragged = this.dragStateManager.physicsObject;
        if (dragged && dragged.bodyID) {
          for (let b = 0; b < this.model.nbody; b++) {
            if (this.bodies[b]) {
              getPosition  (this.simulation.xpos , b, this.bodies[b].position);
              getQuaternion(this.simulation.xquat, b, this.bodies[b].quaternion);
              this.bodies[b].updateWorldMatrix();
            }
          }
          let bodyID = dragged.bodyID;
          this.dragStateManager.update(); // Update the world-space force origin
          let force = toMujocoPos(this.dragStateManager.currentWorld.clone().sub(this.dragStateManager.worldHit).multiplyScalar(this.model.body_mass[bodyID] * 250));
          let point = toMujocoPos(this.dragStateManager.worldHit.clone());
          this.simulation.applyForce(force.x, force.y, force.z, 0, 0, 0, point.x, point.y, point.z, bodyID);

          // TODO: Apply pose perturbations (mocap bodies only).
        }

        // debug: print all body positions
        for (let b = 0; b < this.model.nbody; b++) {
          if (this.bodies[b]) {
            console.log("Body", b, this.bodies[b].name, "Position", this.bodies[b].position);
          }
        }
        // Print joint name, pos and vel for each joint
        for (let jidx = 0; jidx < this.model.njnt; jidx++) {
          let jnt_name = "?";
          if (this.model.name_jntadr[jidx] in this.model.DBG_name_index) {
            jnt_name = this.model.DBG_name_index[this.model.name_jntadr[jidx]];
          }
          let jnt_pos = this.simulation.qpos[this.model.jnt_qposadr[jidx]];
          let jnt_vel = this.simulation.qvel[this.model.jnt_dofadr[jidx]];
          let jnt_lower_lim = this.model.jnt_range[jidx*2];
          let jnt_upper_lim = this.model.jnt_range[jidx*2+1];
          console.log("Joint", jidx, jnt_name, "Position", jnt_pos, "Velocity", jnt_vel, "Limits", jnt_lower_lim, jnt_upper_lim);
        }
        // print actuator name and control value for each actuator
        for (let i = 0; i < this.simulation.ctrl.length; i++) {
          let actuator_name = "?";
          if (this.model.name_actuatoradr[i] in this.model.DBG_name_index) {
            actuator_name = this.model.DBG_name_index[this.model.name_actuatoradr[i]];
          }
          console.log("Actuator", i, actuator_name, "Control", this.simulation.ctrl[i]);
        }

        // RL policy state, action
        if (is_initialized && !this.params["paused"]) {
          let target_xyz = [14.0, 0.0, 1.3];
          let initial_z = 0.8; // from flagrun
          this.DBG_div_element.innerHTML = "";
          let torso_name = "torso";
          let lfoot_name = "left_foot";
          let rfoot_name = "right_foot";
          let floor_name = "floor";
          if (ENV == "DEEP_MIMIC_HUMANOID") {
            torso_name = "chest";
            lfoot_name = "left_ankle";
            rfoot_name = "right_ankle";
          }

          // Get observation vector (see gym_forward_walker.py calc_state())
          let obs = [];
          // 8 Body pose, target values
          let mean_xyz = [0.0, 0.0, 0.0];
          let n = 0;
          let torso_xyz = [0.0, 0.0, 0.0];
          let torso_wxyz = [0.0, 0.0, 0.0, 0.0];
          let torso_rpy = [0.0, 0.0, 0.0];
          let torso_vel = [0.0, 0.0, 0.0];
          let torso_rotvel = [0.0, 0.0, 0.0];
          for (let b = 0; b < this.model.nbody; b++) {
            if (this.bodies[b]) {
              n += 1;
              mean_xyz[0] += this.simulation.xpos[b*3+0];
              mean_xyz[1] += this.simulation.xpos[b*3+1];
              mean_xyz[2] += this.simulation.xpos[b*3+2];
              if (this.bodies[b].name == torso_name) {
                torso_xyz[0] = this.simulation.xpos[b*3+0];
                torso_xyz[1] = this.simulation.xpos[b*3+1];
                torso_xyz[2] = this.simulation.xpos[b*3+2];
                torso_vel[0] = this.simulation.cvel[b*6+3];
                torso_vel[1] = this.simulation.cvel[b*6+4];
                torso_vel[2] = this.simulation.cvel[b*6+5];
                torso_rotvel[0] = this.simulation.cvel[b*6+0];
                torso_rotvel[1] = this.simulation.cvel[b*6+1];
                torso_rotvel[2] = this.simulation.cvel[b*6+2];
                torso_wxyz[0] = this.simulation.xquat[b*4+0];
                torso_wxyz[1] = this.simulation.xquat[b*4+1];
                torso_wxyz[2] = this.simulation.xquat[b*4+2];
                torso_wxyz[3] = this.simulation.xquat[b*4+3];
                let quat = new THREE.Quaternion(torso_wxyz[1], torso_wxyz[2], torso_wxyz[3], torso_wxyz[0]);
                let euler = new THREE.Euler();
                euler.setFromQuaternion(quat);
                torso_rpy[0] = euler.x;
                torso_rpy[1] = euler.y;
                torso_rpy[2] = euler.z;
              }
            }
          }
          mean_xyz[0] /= n;
          mean_xyz[1] /= n;
          mean_xyz[2] /= n;
          this.DBG_div_element.innerHTML += "vx: " + torso_vel[0].toFixed(2) + "<br>vy: " + torso_vel[1].toFixed(2) + "<br>vz: " + torso_vel[2].toFixed(2);
          this.DBG_div_element.innerHTML += "<br>roll: " + torso_rpy[0].toFixed(2) + "<br>pitch: " + torso_rpy[1].toFixed(2) + "<br>yaw: " + torso_rpy[2].toFixed(2);
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
          for (let jidx = 1; jidx < this.model.njnt; jidx++) { // joint 0 is root
            let jnt_name = "?";
            if (this.model.name_jntadr[jidx] in this.model.DBG_name_index) {
              jnt_name = this.model.DBG_name_index[this.model.name_jntadr[jidx]];
            }
            if (jnt_name == "root") {
              continue;
            }
            let jnt_pos = this.simulation.qpos[this.model.jnt_qposadr[jidx]];
            let jnt_vel = this.simulation.qvel[this.model.jnt_dofadr[jidx]];
            let jnt_lower_lim = this.model.jnt_range[jidx*2];
            let jnt_upper_lim = this.model.jnt_range[jidx*2+1];
            // to joint observations (34,), assumes motor joints with 0 max vel
            let jnt_mid_lim = (jnt_upper_lim + jnt_lower_lim) / 2;
            let jnt_rel_pos = 2 * (jnt_pos - jnt_mid_lim) / (jnt_upper_lim - jnt_lower_lim);
            qpos.push(jnt_pos);
            qvel.push(jnt_vel);
            qpos_rel.push(jnt_rel_pos);
          }
          // 2 feet contact values
          // this.simulation.contact;
          let is_left_foot_contact = 0;
          let is_right_foot_contact = 0;
          for (let i = 0; i < this.simulation.contact.length; i++) {
            let contact = this.simulation.contact[i];
            let geom1 = this.model.DBG_name_index[this.model.name_geomadr[contact.geom1]];
            let geom2 = this.model.DBG_name_index[this.model.name_geomadr[contact.geom2]];
            if ((geom1 == lfoot_name || geom2 == lfoot_name) && (geom1 == floor_name || geom2 == floor_name)) {
              is_left_foot_contact = 1;
            }
            if ((geom1 == rfoot_name || geom2 == rfoot_name) && (geom1 == floor_name || geom2 == floor_name)) {
              is_right_foot_contact = 1;
            }
            //console.log("Contact", i, "Geom1", geom1, "Geom2", geom2);
          }
          this.DBG_div_element.innerHTML += "<br>lfoot_contact: " + is_left_foot_contact + "<br>rfoot_contact: " + is_right_foot_contact;

          if (ENV == "FLAGRUN") {
            obs.push(torso_xyz[2] - initial_z);
            obs.push(Math.sin(angle_to_target));
            obs.push(Math.cos(angle_to_target));
            obs.push(0.3 * vx);
            obs.push(0.3 * vy);
            obs.push(0.3 * vz);
            obs.push(torso_rpy[0]);
            obs.push(torso_rpy[1]);
            for (let i = 0; i < qpos_rel.length; i++) {
              obs.push(qpos_rel[i]);
              obs.push(qvel[i] * 0.1);
              jnt_dbg.push(jnt_name + " " + jnt_rel_pos.toFixed(2) + " " + jnt_rel_vel.toFixed(2));
            }
            obs.push(is_right_foot_contact);
            obs.push(is_left_foot_contact);
            // clip obs to -5, 5
            obs = obs.map(x => Math.min(5, Math.max(-5, x)));
          } else if (ENV == "DEEP_MIMIC_HUMANOID") {
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
          }

          let a = this.policy.act(obs);

          if (ENV == "FLAGRUN") {
            for (let i = 0; i < a.length; i++) {
              a[i] = Math.min(1.0, Math.max(-1.0, a[i] * 0.41));
            }
          } else if (ENV == "DEEP_MIMIC_HUMANOID") {
            for (let i = 0; i < a.length; i++) {
              a[i] = Math.min(0.5, Math.max(-0.5, a[i]));
            }
          }

          // zero the control signal for debugging sim
          if (ZEROACT) {
            a = a.map(x => 0.0)
          }
          if (ONEACT) {
            a = a.map(x => 1.0)
          }

          if (ACTOBS_LOG) {
            this.DBG_actobs_log += "Frame " + this.DBG_actobs_log_frames + "\n";
            this.DBG_actobs_log += "MJ time: " + this.mujoco_time.toFixed(3) + "\n";
            this.DBG_actobs_log += "Obs: \n";
            for (let i = 0; i < obs.length; i++) {
              this.DBG_actobs_log += i.toString().padStart(2, '0') + ": " + obs[i].toFixed(3) + "\n";
            }
            this.DBG_actobs_log += "Act: \n";
            for (let i = 0; i < a.length; i++) {
              this.DBG_actobs_log += i.toString().padStart(2, '0') + ": " + a[i].toFixed(3) + "\n";
            }
            this.DBG_actobs_log += "\n";

            this.DBG_actobs_qlog += this.simulation.time.toFixed(3) + ",";
            this.DBG_actobs_qlog += obs.length + ",";
            for (let i = 0; i < obs.length; i++) {
              this.DBG_actobs_qlog += obs[i].toFixed(5) + ",";
            }
            this.DBG_actobs_qlog += a.length + ",";
            for (let i = 0; i < a.length; i++) {
              this.DBG_actobs_qlog += a[i].toFixed(5) + ",";
            }
            this.DBG_actobs_qlog += "1" + ",";
            this.DBG_actobs_qlog += "0";
            this.DBG_actobs_qlog += "\n";
            this.DBG_actobs_log_frames += 1;
            if (this.DBG_actobs_log_frames == 100) {
              console.log(this.DBG_actobs_log);
              console.log(this.DBG_actobs_qlog);
              this.params["paused"] = true;
            }
          }

          // Apply the control signal to the simulation
          for (let i = 0; i < a.length; i++) {
            this.simulation.ctrl[i] = a[i] * this.params["rlactscale"];
          }
        }

        for (let i = 0; i < repeat_iter; i++) {
          this.simulation.step();
          this.mujoco_time += timestep * 1000.0;
        }

      }

    } else if (this.params["paused"]) {
      this.dragStateManager.update(); // Update the world-space force origin
      let dragged = this.dragStateManager.physicsObject;
      if (dragged && dragged.bodyID) {
        let b = dragged.bodyID;
        getPosition  (this.simulation.xpos , b, this.tmpVec , false); // Get raw coordinate from MuJoCo
        getQuaternion(this.simulation.xquat, b, this.tmpQuat, false); // Get raw coordinate from MuJoCo

        let offset = toMujocoPos(this.dragStateManager.currentWorld.clone()
          .sub(this.dragStateManager.worldHit).multiplyScalar(0.3));
        if (this.model.body_mocapid[b] >= 0) {
          // Set the root body's mocap position...
          console.log("Trying to move mocap body", b);
          let addr = this.model.body_mocapid[b] * 3;
          let pos  = this.simulation.mocap_pos;
          pos[addr+0] += offset.x;
          pos[addr+1] += offset.y;
          pos[addr+2] += offset.z;
        } else {
          // Set the root body's position directly...
          let root = this.model.body_rootid[b];
          let addr = this.model.jnt_qposadr[this.model.body_jntadr[root]];
          let pos  = this.simulation.qpos;
          pos[addr+0] += offset.x;
          pos[addr+1] += offset.y;
          pos[addr+2] += offset.z;

          //// Save the original root body position
          //let x  = pos[addr + 0], y  = pos[addr + 1], z  = pos[addr + 2];
          //let xq = pos[addr + 3], yq = pos[addr + 4], zq = pos[addr + 5], wq = pos[addr + 6];

          //// Clear old perturbations, apply new ones.
          //for (let i = 0; i < this.simulation.qfrc_applied().length; i++) { this.simulation.qfrc_applied()[i] = 0.0; }
          //for (let bi = 0; bi < this.model.nbody(); bi++) {
          //  if (this.bodies[b]) {
          //    getPosition  (this.simulation.xpos (), bi, this.bodies[bi].position);
          //    getQuaternion(this.simulation.xquat(), bi, this.bodies[bi].quaternion);
          //    this.bodies[bi].updateWorldMatrix();
          //  }
          //}
          ////dragStateManager.update(); // Update the world-space force origin
          //let force = toMujocoPos(this.dragStateManager.currentWorld.clone()
          //  .sub(this.dragStateManager.worldHit).multiplyScalar(this.model.body_mass()[b] * 0.01));
          //let point = toMujocoPos(this.dragStateManager.worldHit.clone());
          //// This force is dumped into xrfc_applied
          //this.simulation.applyForce(force.x, force.y, force.z, 0, 0, 0, point.x, point.y, point.z, b);
          //this.simulation.integratePos(this.simulation.qpos(), this.simulation.qfrc_applied(), 1);

          //// Add extra drag to the root body
          //pos[addr + 0] = x  + (pos[addr + 0] - x ) * 0.1;
          //pos[addr + 1] = y  + (pos[addr + 1] - y ) * 0.1;
          //pos[addr + 2] = z  + (pos[addr + 2] - z ) * 0.1;
          //pos[addr + 3] = xq + (pos[addr + 3] - xq) * 0.1;
          //pos[addr + 4] = yq + (pos[addr + 4] - yq) * 0.1;
          //pos[addr + 5] = zq + (pos[addr + 5] - zq) * 0.1;
          //pos[addr + 6] = wq + (pos[addr + 6] - wq) * 0.1;


        }
      }

      // this.simulation.forward();
    }

    // Update body transforms.
    for (let b = 0; b < this.model.nbody; b++) {
      if (this.bodies[b]) {
        getPosition  (this.simulation.xpos , b, this.bodies[b].position);
        getQuaternion(this.simulation.xquat, b, this.bodies[b].quaternion);
        this.bodies[b].updateWorldMatrix();
      }
    }

    // Update light transforms.
    for (let l = 0; l < this.model.nlight; l++) {
      if (this.lights[l]) {
        getPosition(this.simulation.light_xpos, l, this.lights[l].position);
        getPosition(this.simulation.light_xdir, l, this.tmpVec);
        this.lights[l].lookAt(this.tmpVec.add(this.lights[l].position));
      }
    }

    // Update tendon transforms.
    let numWraps = 0;
    if (this.mujocoRoot && this.mujocoRoot.cylinders) {
      let mat = new THREE.Matrix4();
      for (let t = 0; t < this.model.ntendon; t++) {
        let startW = this.simulation.ten_wrapadr[t];
        let r = this.model.tendon_width[t];
        for (let w = startW; w < startW + this.simulation.ten_wrapnum[t] -1 ; w++) {
          let tendonStart = getPosition(this.simulation.wrap_xpos, w    , new THREE.Vector3());
          let tendonEnd   = getPosition(this.simulation.wrap_xpos, w + 1, new THREE.Vector3());
          let tendonAvg   = new THREE.Vector3().addVectors(tendonStart, tendonEnd).multiplyScalar(0.5);

          let validStart = tendonStart.length() > 0.01;
          let validEnd   = tendonEnd  .length() > 0.01;

          if (validStart) { this.mujocoRoot.spheres.setMatrixAt(numWraps    , mat.compose(tendonStart, new THREE.Quaternion(), new THREE.Vector3(r, r, r))); }
          if (validEnd  ) { this.mujocoRoot.spheres.setMatrixAt(numWraps + 1, mat.compose(tendonEnd  , new THREE.Quaternion(), new THREE.Vector3(r, r, r))); }
          if (validStart && validEnd) {
            mat.compose(tendonAvg, new THREE.Quaternion().setFromUnitVectors(
              new THREE.Vector3(0, 1, 0), tendonEnd.clone().sub(tendonStart).normalize()),
              new THREE.Vector3(r, tendonStart.distanceTo(tendonEnd), r));
            this.mujocoRoot.cylinders.setMatrixAt(numWraps, mat);
            numWraps++;
          }
        }
      }
      this.mujocoRoot.cylinders.count = numWraps;
      this.mujocoRoot.spheres  .count = numWraps > 0 ? numWraps + 1: 0;
      this.mujocoRoot.cylinders.instanceMatrix.needsUpdate = true;
      this.mujocoRoot.spheres  .instanceMatrix.needsUpdate = true;
    }

    // Render!
    this.renderer.render( this.scene, this.camera );
  }
}

let demo = new MuJoCoDemo();
await demo.init();
