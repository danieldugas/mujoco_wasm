
import * as THREE           from 'three';
import { GUI              } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import { OrbitControls    } from '../node_modules/three/examples/jsm/controls/OrbitControls.js';
import { DragStateManager } from './utils/DragStateManager.js';
import { setupGUI, downloadExampleScenesFolder, loadSceneFromURL, getPosition, getQuaternion, toMujocoPos, standardNormal } from './mujocoUtils.js';
import   load_mujoco        from '../dist/mujoco_wasm.js';

// Load the MuJoCo Module
const mujoco = await load_mujoco();

export class MuJoCoDemo {
  constructor(initialScene="humanoid_deep_mimic.xml", repeat_iter=1) {
    this.initialScene = initialScene;
    this.repeat_iter = repeat_iter;
    this.mujoco = mujoco;
    this.init_complete = false;
    this.flag_step_once = false;

    this.sceneLoaderCallback = null;
    this.mocapInitCallback = null;
    this.obsAndActCallback = null;

    // Load in the state from XML
    this.model      = new mujoco.Model("/working/" + this.initialScene);
    this.state      = new mujoco.State(this.model);
    this.simulation = new mujoco.Simulation(this.model, this.state);


    this.model.DBG_name_index = {}; // added by Daniel to store joint names in an easier to access format than model.names
    this.DBG_div_element = document.createElement('div'); // added by Daniel to store html for debugging
    this.DBG_actobs_log = "";
    this.DBG_actobs_qlog = "";
    this.DBG_actobs_log_frames = 0;

    // Define Random State Variables
    this.params = { scene: this.initialScene, paused: false, help: false, ctrlnoiserate: 0.0, ctrlnoisestd: 0.0, rlactscale: 1.0,
      keyframeNumber: 0, gravity: true, policy_enabled: true };
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
    if (this.mocapInitCallback) {
      this.mocapInitCallback(this);
    }
  }

  async init() {
    // Create a floating abs div for showing loading progress
    let loadingDiv = document.createElement('div');
    this.loadingDiv = loadingDiv;
    loadingDiv.style.position = 'absolute';
    loadingDiv.style.top = '50%';
    loadingDiv.style.left = '50%';
    loadingDiv.style.transform = 'translate(-50%, -50%)';
    loadingDiv.style.zIndex = '1000';
    loadingDiv.style.backgroundColor = 'rgba(0, 0, 0, 0.8)';
    loadingDiv.style.color = 'white';
    loadingDiv.style.padding = '20px';
    loadingDiv.style.borderRadius = '10px';
    loadingDiv.style.fontFamily = 'Arial';
    loadingDiv.style.fontSize = '20px';
    loadingDiv.innerHTML = 'Loading...';
    document.body.appendChild(loadingDiv);

    // Download the the examples to MuJoCo's virtual file system
    mujoco.FS.mkdir('/working');
    mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
    mujoco.FS.writeFile("/working/" + this.initialScene, await(await fetch("./examples/scenes/" + this.initialScene)).text());
    // await downloadExampleScenesFolder(mujoco, loadingDiv);
    if (this.sceneLoaderCallback) {
      await this.sceneLoaderCallback(this);
    }

    loadingDiv.innerHTML = 'Initializing scene...';

    // Initialize the three.js Scene using the .xml Model in initialScene
    [this.model, this.state, this.simulation, this.bodies, this.lights] =  
      await loadSceneFromURL(mujoco, this.initialScene, this);
    this.set_mocap_keyframe();
    this.simulation.forward(); // otherwise the first frame tends to be right before or after init position somehow

    this.gui = new GUI();
    setupGUI(this);

    // Remove the loading div
    document.body.removeChild(loadingDiv);
    this.loadingDiv = null;

    this.init_complete = true;
  }

  onWindowResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize( window.innerWidth, window.innerHeight );
  }

  render(timeMS) {
    if (!this.init_complete) { return; }
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
        
        // call environment obs and act functions
        if (this.params["policy_enabled"]) {
          if (this.obsAndActCallback) {
            this.obsAndActCallback(this);
          }
        }

        for (let i = 0; i < this.repeat_iter; i++) {
          this.simulation.step();
          this.mujoco_time += timestep * 1000.0;
        }

        // Pause if the user has requested a single step.
        if (this.flag_step_once) {
          this.params["paused"] = true;
          this.flag_step_once = false;
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