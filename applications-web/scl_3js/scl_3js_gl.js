//These are global scope variables...
var SCL = SCL || {};

/** All the global data is collected here to simplify things */
function sclInitDataStructures()
{
  SCL.scene_ = null;
  SCL.renderer_ = null;
  SCL.camera_ = null;
  SCL.camera_controls_ = null;
  SCL.lights_ = [];
  SCL.meshes_ = [];
  SCL.robots_ = [];
  SCL.frames_rendered_ = 0;

  // UI interaction (mouse etc.)
  SCL.clock_=null;
  SCL.mouse_x_ = 0;
  SCL.mouse_y_ = 0;
  SCL.window_halfx_ = window.innerWidth/2;
  SCL.window_halfy_ = window.innerHeight/2;
  SCL.mouse_lclick_ = 0;
  SCL.mouse_rclick_ = 0;
  SCL.mouse_mclick_ = 0;

  // Redis server interaction
  SCL.redis_msg_ = null;
  SCL.redis_msg_dyn_ = null;
  SCL.redis_msg_parsed_ = null;

  // Graphics stack
  var m1 = new THREE.Matrix4();
  var m2 = new THREE.Matrix4();
  m1.makeRotationX(-Math.PI/2);
  m1.makeRotationZ(-Math.PI/2);

  var Tj = new THREE.Matrix4();
  Tj = m2 * m1;
  SCL.T_scl_3js_ = Tj;

  m1.makeRotationX(Math.PI/2);
  m1.makeRotationZ(Math.PI/2);

  var Ts = new THREE.Matrix4();
  Ts = m1 * m2;
  SCL.T_3js_scl_ = Ts;
}

/** Random initialization and error handling related to getting
 * a webgl canvas. Add browser-specific stuff here... */
function sclInit3jsScene() {
  var scene = new THREE.Scene();
  SCL.scene_ = scene;

  var renderer = new THREE.WebGLRenderer();
  renderer.setSize( window.innerWidth, window.innerHeight );
  document.body.appendChild(renderer.domElement );
  SCL.renderer_ = renderer;

  // Add global root... Rotate stuff to align with scl..
  var geometry = new THREE.SphereGeometry(0.05, 0.05, 0.05);
  var material = new THREE.MeshBasicMaterial( { color: 0xaaaaaa } );
  var sphere = new THREE.Mesh( geometry, material );
  SCL.scene_.add( sphere );
  sphere.rotation.x = -Math.PI/2;
  sphere.rotation.z = -Math.PI/2;
  SCL.global_root_ = sphere;

  var clock = new THREE.Clock();
  clock.autoStart  = true; //Will start at the first update.
  SCL.clock_ = clock;
}

function sclInitCamerasAndLights(){
  // NOTE TODO : Uses the window DOM (replace with canvas)
  // (fov, aspect, near plane, far plane).
  var camera = new THREE.PerspectiveCamera( 75,
        window.innerWidth/window.innerHeight, 0.1, 20 );
  camera.position.set(0.0, 0.4, 1.3);
  SCL.camera_ = camera;

  var controls = new THREE.TrackballControls( camera );
  controls.target.set( 0, -0.4, 0 );
  controls.rotateSpeed = 1.0;
  controls.zoomSpeed = 1.2;
  controls.panSpeed = 0.8;
  controls.noZoom = false;
  controls.noPan = false;
  controls.staticMoving = true;
  controls.dynamicDampingFactor = 0.3;
  SCL.camera_controls_ = controls;

  // add subtle blue ambient lighting
  var ambientLight = new THREE.AmbientLight(0x000000);
  SCL.scene_.add(ambientLight);
  SCL.lights_[SCL.lights_.length] = ambientLight;

  // directional lighting
  var directionalLight = new THREE.DirectionalLight(0xffffff);
  directionalLight.position.set(1, 1, 1).normalize();
  SCL.scene_.add(directionalLight);
  SCL.lights_[SCL.lights_.length] = directionalLight;

  directionalLight = new THREE.DirectionalLight(0xffffff);
  directionalLight.position.set(-8, 4, 1).normalize();
  SCL.scene_.add(directionalLight);
  SCL.lights_[SCL.lights_.length] = directionalLight;
}


function sclAddOriginFrame(base_node, frame_size)
{
  //Default arg...
  frame_size = typeof frame_size !== 'undefined' ? frame_size : .03;
  if(frame_size > .3){ frameSz=.3;}
  if(frame_size < 0.01){ frameSz=0.01;}

  // Add rgb for xyz axes...
  var geometry = new THREE.BoxGeometry(frame_size*5, frame_size, frame_size);
  var material = new THREE.MeshBasicMaterial( { color: 0xff0000 } );
  var cube = new THREE.Mesh( geometry, material );
  cube.position.x = frame_size*3.5;
  cube.updateMatrix(); //Only needed once
  cube.matrixAutoUpdate = false;
  base_node.add( cube );

  // Add rgb for xyz axes...
  geometry = new THREE.BoxGeometry(frame_size, frame_size*5, frame_size);
  material = new THREE.MeshBasicMaterial( { color: 0x00ff00 } );
  cube = new THREE.Mesh( geometry, material );
  cube.position.y = frame_size*3.5;
  cube.updateMatrix(); //Only needed once
  cube.matrixAutoUpdate = false;
  base_node.add( cube );

  // Add rgb for xyz axes...
  geometry = new THREE.BoxGeometry(frame_size, frame_size, frame_size*5);
  material = new THREE.MeshBasicMaterial( { color: 0x0000ff } );
  cube = new THREE.Mesh( geometry, material );
  cube.position.z = frame_size*3.5;
  cube.updateMatrix(); //Only needed once
  cube.matrixAutoUpdate = false;
  base_node.add( cube );
}


function sclAddStaticMeshes()
{
  // Nice example of how 3js' right handed axis has x=right, y=up.
  var geometry = new THREE.BoxGeometry( .4, .4, .4 );
  var material = new THREE.MeshBasicMaterial( { color: 0x00ff00 } );
  var cube = new THREE.Mesh( geometry, material );
  cube.position.y = 1.5;
  cube.matrixAutoUpdate = true;
  SCL.global_root_.add( cube );
  SCL.meshes_[SCL.meshes_.length] = cube;
}


function sclTestAddPuma()
{
  var newroot = new THREE.Object3D;
  var robot = new Object();
  robot.name_ = "PumaBot";
  robot.meshes_ = [];

  // prepare loader and load the model
  var oLoader = new THREE.OBJLoader();
  oLoader.load('Puma/base_bl.obj', function(object, materials) {
    var material2 = new THREE.MeshLambertMaterial({ color: 0x888888 });
    object.traverse( function(child) {
      if (child instanceof THREE.Mesh) {
        child.material = material2;
        child.castShadow = true;
        child.receiveShadow = true;
      }
    });
    object.userData = {name:"root"};
    object.position.set(-0.300,  0.000,  -0.900);
    object.updateMatrix();
    object.matrixAutoUpdate = false;
    newroot.add(object); newroot = object;
    robot.meshes_[robot.meshes_.length] = new THREE.Object3D(object);
  });

  oLoader = new THREE.OBJLoader();
  oLoader.load('Puma/shoulder_bl.obj', function(object, materials) {
    var material2 = new THREE.MeshLambertMaterial({ color: 0x888888 });
    object.traverse( function(child) {
      if (child instanceof THREE.Mesh) {
        child.material = material2;
        child.castShadow = true;
        child.receiveShadow = true;
      }
    });
    object.userData = {name:"link0"};
    object.position.set(0.00,  0.000,  0.6600);
    object.updateMatrix();
    object.matrixAutoUpdate = true;
    newroot.add(object); newroot = object;
    robot.meshes_[robot.meshes_.length] = new THREE.Object3D(object);
  });

  oLoader = new THREE.OBJLoader();
  oLoader.load('Puma/upper_arm_bl.obj', function(object, materials) {
    var material2 = new THREE.MeshLambertMaterial({ color: 0x888888 });
    object.traverse( function(child) {
      if (child instanceof THREE.Mesh) {
        child.material = material2;
        child.castShadow = true;
        child.receiveShadow = true;
      }
    });
    object.userData = {name:"link1"};
    object.position.set(0.0,  0.24350,  0.0);
    object.rotation.x = 3*Math.PI/2;
    object.updateMatrix();
    object.matrixAutoUpdate = true;
    newroot.add(object); newroot = object;
    robot.meshes_[robot.meshes_.length] = new THREE.Object3D(object);
  });

  oLoader = new THREE.OBJLoader();
  oLoader.load('Puma/lower_arm_bl.obj', function(object, materials) {
    var material2 = new THREE.MeshLambertMaterial({ color: 0x888888 });
    object.traverse( function(child) {
      if (child instanceof THREE.Mesh) {
        child.material = material2;
        child.castShadow = true;
        child.receiveShadow = true;
      }
    });
    object.userData = {name:"link2"};
    object.position.set(0.4318, 0.0, -0.093400);
    object.updateMatrix();
    object.matrixAutoUpdate = true;
    newroot.add(object); newroot = object;
    robot.meshes_[robot.meshes_.length] = new THREE.Object3D(object);
  });

  oLoader = new THREE.OBJLoader();
  oLoader.load('Puma/hand_bl.obj', function(object, materials) {
    var material2 = new THREE.MeshLambertMaterial({ color: 0x888888 });
    object.traverse( function(child) {
      if (child instanceof THREE.Mesh) {
        child.material = material2;
        child.castShadow = true;
        child.receiveShadow = true;
      }
    });
    object.userData = {name:"link3"};
    object.position.set(0.00, -0.434, 0.011);
    object.rotation.x = Math.PI/2;
    object.updateMatrix();
    object.matrixAutoUpdate = true;
    newroot.add(object); newroot = object;
    robot.meshes_[robot.meshes_.length] = new THREE.Object3D(object);
  });

  oLoader = new THREE.OBJLoader();
  oLoader.load('Puma/finger_bl.obj', function(object, materials) {
    var material2 = new THREE.MeshLambertMaterial({ color: 0x888888 });
    object.traverse( function(child) {
      if (child instanceof THREE.Mesh) {
        child.material = material2;
        child.castShadow = true;
        child.receiveShadow = true;
      }
    });
    object.userData = {name:"link4"};
    object.position.set(0.00, 0.0, 0.0);
    object.rotation.y = Math.PI/2;
    object.updateMatrix();
    object.matrixAutoUpdate = true;
    newroot.add(object); newroot = object;
    robot.meshes_[robot.meshes_.length] = new THREE.Object3D(object);
  });

  oLoader = new THREE.OBJLoader();
  oLoader.load('Puma/shaft2_bl.obj', function(object, materials) {
    var material2 = new THREE.MeshLambertMaterial({ color: 0x660022 });
    object.traverse( function(child) {
      if (child instanceof THREE.Mesh) {
        child.material = material2;
        child.castShadow = true;
        child.receiveShadow = true;
      }
    });
    object.userData = {name:"link5-mesh"};
    object.position.set(-0.07, 0.0, 0.0);
    object.rotation.y = -Math.PI/2;
    object.updateMatrix();
    object.matrixAutoUpdate = true;
    newroot.add(object); newroot = object;
    robot.meshes_[robot.meshes_.length] = new THREE.Object3D(object);
    sclAddOriginFrame(object)
  });


  SCL.robots_[SCL.robots_.length] = robot;
  SCL.global_root_.add(newroot);
}

/** The main update loop. This will be called forever. Note that
 * you should add all your communication/state-update/render
 * operations here... */
function sclMainLoop () {
  requestAnimationFrame( sclMainLoop );

  // Render the scene...
  SCL.camera_controls_.update();
  SCL.renderer_.render(SCL.scene_, SCL.camera_);

  // Get the REDIS JSON and dump it into an object..
  jQuery.get('http://localhost:7379/GET/bobo', function(data) {
    SCL.redis_msg_ = data;
  });

  // Parse the string message into an object..
  if(SCL.redis_msg_){
    try{
      SCL.redis_msg_parsed_ = JSON.parse(SCL.redis_msg_.GET);
      keys = Object.keys(SCL.redis_msg_parsed_.muscles_);
      // Print something to make sure this works..
      document.getElementById("scl_json_box").innerHTML =
        "Extracted mappedlist data for key : " + keys[0] + "<br />" +
        JSON.stringify(SCL.redis_msg_parsed_.muscles_[keys[0]]);
    }
    catch(err) {//If it doesn't find some scl format, just dump the whole message..
      document.getElementById("scl_json_box").innerHTML = err;
    }
  }

  // Do other stuff
  SCL.frames_rendered_++;
  sclHtmlUpdateInRenderLoop("Time : " + SCL.clock_.getElapsedTime() +
                            "<br />Frames rendered : "+ SCL.frames_rendered_ +
                            "<br />Mouse pos : " + SCL.mouse_x_ + " " +SCL.mouse_y_
                            + " (Works properly in full-screen. Zero at center of screen.).");
};

// ******************************************
// THE CODE ENTRY POINT IS HERE!!!
// The HTML code loads the js at this spot..
// ******************************************
function sclJsEntryPoint(){
  // Set up data structures..
  sclInitDataStructures();
  sclInit3jsScene();
  sclInitCamerasAndLights();

  // Test code
  sclAddOriginFrame(SCL.global_root_);
  //sclAddStaticMeshes();
  sclTestAddPuma();

  // ******************************************
  // Add any html related event handlers here.
  // ******************************************
  /* Get JSON from wedis/redis using AJAX!
  var enableRedisComm = true;
  if (enableRedisComm){
    setInterval(function() {
      var client = new XMLHttpRequest();
      client.open('GET', 'http://localhost:7379/GET/bobo');
      client.onreadystatechange = function() {
        SCL.redis_msg_parsed_ = client.responseText;
      };
      client.send();
    },50);
  }*/

  window.addEventListener("resize",function (){
    // For mouse pointer tracking..
    SCL.window_halfx_ = window.innerWidth/2;
    SCL.window_halfy_ = window.innerHeight/2;

    // For actual rendering..
    SCL.renderer_.setSize( window.innerWidth, window.innerHeight);
    SCL.camera_.aspect = window.innerWidth/window.innerHeight;
    SCL.camera_.updateProjectionMatrix();
  });

  document.addEventListener( 'mousemove', function(){
    SCL.mouse_x_ = ( event.clientX - SCL.window_halfx_) / 2;
    SCL.mouse_y_ = ( event.clientY - SCL.window_halfy_) / 2;
  }, false );

  // ******************************************
  // THE MAIN LOOP ENTRY POINT IS HERE!!!
  // ******************************************
  sclMainLoop();
}

