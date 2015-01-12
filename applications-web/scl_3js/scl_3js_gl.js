/** Global variable scl obj */
var scl_obj = scl_obj || null;
var tmp_ctr = tmp_ctr || 0;
var scl_data_parsed = scl_data_parsed || null;
var scl_data_graphics = scl_data_graphics || null;
var scl_data_io = scl_data_io || null;
var SCL_DEBUG = tmp_ctr || false;

/** Convert scl to 3js (coord frame fix):
  * scl.x,y,z == 3js.z,x,y */
function vec3jsFromScl(sx,sy,sz){
  var tmp_vec_3js = new THREE.Vector3();
  tmp_vec_3js.x = sy;
  tmp_vec_3js.y = sz;
  tmp_vec_3js.z = sx;
  return tmp_vec_3js;
}

/** Convert scl to 3js (coord frame fix):
  * scl.x,y,z == 3js.z,x,y */
function vec3jsFromSclArr(arr){
  return vec3jsFromScl(arr[0],arr[1],arr[2]);
}

/** Convert 3js to scl (coord frame fix):
  * scl.x,y,z == 3js.z,x,y */
function vecSclFrom3js(v3x,v3y,v3z){
  var tmp_vec_scl = new THREE.Vector3();
  tmp_vec_scl.x = v3z;
  tmp_vec_scl.y = v3x;
  tmp_vec_scl.z = v3y;
  return tmp_vec_scl;
}
/** Convert 3js to scl (coord frame fix):
  * scl.x,y,z == 3js.z,x,y */
function vecSclFrom3jsArr(arr){
  return vecSclFrom3js(arr[0],arr[1],arr[2]);
}

/** Runs a sync AJAX call to get data from redis (via webdis) */
function getRedisKey(key_str){
  return JSON.parse(JSON.parse($.ajax({
    type: "GET",
    url: "http://localhost:7379/GET/"+key_str,//SCL:PumaBot:Graphics",
    async: false
  }).responseText).GET);
}

/** Define class. Get an object like this:
  * var scl_obj = new SCL(); */
var SCL = function(){
  this.scene_ = null;
  this.renderer_ = null;
  this.camera_ = null;
  this.camera_controls_ = null;
  this.lights_ = [];
  this.meshes_ = [];
  this.robots_ = [];
  this.frames_rendered_ = 0;

  // UI interaction (mouse etc.)
  this.clock_=null;
  this.mouse_x_ = 0;
  this.mouse_y_ = 0;
  this.window_halfx_ = window.innerWidth/2;
  this.window_halfy_ = window.innerHeight/2;
  this.mouse_lclick_ = 0;
  this.mouse_rclick_ = 0;
  this.mouse_mclick_ = 0;

  // Redis server interaction
  this.redis_msg_ = null;
  this.redis_msg_p_ = null;
  this.redis_msg_dyn_ = null;

  this.data_io_ = null;
  this.data_parsed_ = null;

  // Tmp data structures
  this.tmp_vec_3js_ = new THREE.Vector3();
  this.tmp_vec_scl_ = new THREE.Vector3();
}

/** Random initialization and error handling related to getting
 * a webgl canvas. Add browser-specific stuff here...
 *
 * Member function of class SCL.*/
SCL.prototype.initWebGlScene = function(){
  var scene = new THREE.Scene();
  this.scene_ = scene;

  // NOTE TODO : Attach renderer to a canvas instead..
  var renderer = new THREE.WebGLRenderer();
  renderer.setSize( window.innerWidth, window.innerHeight );
  document.body.appendChild(renderer.domElement );
  this.renderer_ = renderer;

  // Add global root... Rotate stuff to align with scl..
  var geometry = new THREE.SphereGeometry(0.05, 0.05, 0.05);
  var material = new THREE.MeshBasicMaterial( { color: 0xaaaaaa } );
  var sphere = new THREE.Mesh( geometry, material );
  this.scene_.add( sphere );
  sphere.rotation.x = -Math.PI/2;
  sphere.rotation.z = -Math.PI/2;
  this.global_root_ = sphere;

  var clock = new THREE.Clock();
  clock.autoStart  = true; //Will start at the first update.
  this.clock_ = clock;

  return true;
}


SCL.prototype.initCamAndLights = function(){
  // NOTE TODO : Uses the window DOM (replace with canvas)
  if(scl_data_graphics){
    if(SCL_DEBUG){
      alert("Initializing SCL 3js graphics: Adding camera and lights..");
    }
  }
  else{
    alert("Initializing SCL 3js graphics: Could not find parsed graphics info.. Setting to default");
    // NOTE TODO : Fix me...
    //scl_data_graphics = JSON.parse();
    return false;
  }

  // (fov, aspect, near plane, far plane).
  var camera = new THREE.PerspectiveCamera( 75,
        window.innerWidth/window.innerHeight, 0.1, 20 );
  var cpos = vec3jsFromSclArr(scl_data_graphics.cam_pos_);
  camera.position.set(cpos.x,cpos.y,cpos.z);
  //camera.position.set(0.0,0.1,1);
  this.camera_ = camera;

  // Set up camera controls...
  var controls = new THREE.TrackballControls( camera );
  controls.target.set( 0, -0.4, 0 );
  controls.rotateSpeed = 1.0;
  controls.zoomSpeed = 1.2;
  controls.panSpeed = 0.8;
  controls.noZoom = false;
  controls.noPan = false;
  controls.staticMoving = true;
  controls.dynamicDampingFactor = 0.3;
  this.camera_controls_ = controls;

  // NOTE TODO : Get lighting information from SCL
  // add subtle blue ambient lighting
  var ambientLight = new THREE.AmbientLight(0x000000);
  this.scene_.add(ambientLight);
  this.lights_[this.lights_.length] = ambientLight;

  // directional lighting
  var directionalLight = new THREE.DirectionalLight(0xffffff);
  directionalLight.position.set(1, 1, 1).normalize();
  this.scene_.add(directionalLight);
  this.lights_[this.lights_.length] = directionalLight;

  directionalLight = new THREE.DirectionalLight(0xffffff);
  directionalLight.position.set(-8, 4, 1).normalize();
  this.scene_.add(directionalLight);
  this.lights_[this.lights_.length] = directionalLight;
}


SCL.prototype.graphicsAddFrame = function (base_node, frame_size)
{
  //Default arg...
  frame_size = typeof frame_size !== 'undefined' ? frame_size : .02;
  if(frame_size > .3){ frameSz=.3;}
  if(frame_size < 0.01){ frameSz=0.01;}

  var frame_mult = 5;

  // Add rgb for xyz axes...
  var geometry = new THREE.BoxGeometry(frame_size*5, frame_size, frame_size);
  var material = new THREE.MeshBasicMaterial( { color: 0xff0000 } );
  var cube = new THREE.Mesh( geometry, material );
  cube.position.x = frame_size*frame_mult;
  cube.updateMatrix(); //Only needed once
  cube.matrixAutoUpdate = false;
  base_node.add( cube );

  // Add rgb for xyz axes...
  geometry = new THREE.BoxGeometry(frame_size, frame_size*5, frame_size);
  material = new THREE.MeshBasicMaterial( { color: 0x00ff00 } );
  cube = new THREE.Mesh( geometry, material );
  cube.position.y = frame_size*frame_mult;
  cube.updateMatrix(); //Only needed once
  cube.matrixAutoUpdate = false;
  base_node.add( cube );

  // Add rgb for xyz axes...
  geometry = new THREE.BoxGeometry(frame_size, frame_size, frame_size*5);
  material = new THREE.MeshBasicMaterial( { color: 0x0000ff } );
  cube = new THREE.Mesh( geometry, material );
  cube.position.z = frame_size*frame_mult;
  cube.updateMatrix(); //Only needed once
  cube.matrixAutoUpdate = false;
  base_node.add( cube );
}



SCL.prototype.testAddPuma = function()
{
  var robot = new Object();
  robot.name_ = "PumaBot";
  robot.meshes_ = [];
  robot.links_ = [];

  /** Need this weird function factory to make sure that the inner
    * function can actually access the parent_mesh and mesh_names
    * variables without screwing up.
    * mesh_props is optional (pos, ori etc.)*/
  function makeLoaderClosure(parent_mesh, mesh_name, pos, ori){
    return function(object, materials) {
      var material2 = new THREE.MeshLambertMaterial({ color: 0x888888 });
      object.traverse( function(child) {
        if (child instanceof THREE.Mesh) {
          child.material = material2;
          child.castShadow = true;
          child.receiveShadow = true;
        }
      });
      object.userData = {name: mesh_name};
      if(pos){
        object.position.set(pos[0],pos[1],pos[2]);
        object.updateMatrix();
      }
      parent_mesh.add(object);
    }
  }

  //NOTE : For now, this is hard wired.. Do a parse later...
  // **************** Load a link **************************
  robot.links_[0] = new THREE.Object3D;
  var p = scl_data_parsed.rb_tree_["ground"].pos_in_parent_;
  robot.links_[0].position.set(p[0],  p[1],  p[2]);

  //Add the graphics as child nodes of the links...
  var oLoader = new THREE.OBJLoader();
  // NOTE TODO : SCL base path ../../ doesn't work with a webserv..
  // scl_data_parsed.rb_tree_["base"].graphics_obj_vec_[0].file_name_
  oLoader.load('Puma/base_bl.obj',
               makeLoaderClosure(robot.links_[0],"ground"));
  scl_obj.graphicsAddFrame(robot.links_[0]);

  // **************** Load a link **************************
  robot.links_[1] = new THREE.Object3D;
  p = scl_data_parsed.rb_tree_["base"].pos_in_parent_;
  robot.links_[1].position.set(p[0],  p[1],  p[2]);
  oLoader = new THREE.OBJLoader();
  oLoader.load('Puma/shoulder_bl.obj',
               makeLoaderClosure(robot.links_[1],"base"));
  robot.links_[0].add(robot.links_[1]);
  scl_obj.graphicsAddFrame(robot.links_[1]);

  // **************** Load a link **************************
  robot.links_[2] = new THREE.Object3D;
  p = scl_data_parsed.rb_tree_["upper_arm"].pos_in_parent_;
  robot.links_[2].position.set(p[0],  p[1],  p[2]);
  robot.links_[2].rotation.x = 3*Math.PI/2;
  oLoader = new THREE.OBJLoader();
  oLoader.load('Puma/upper_arm_bl.obj',
               makeLoaderClosure(robot.links_[2],"upper_arm"));
  robot.links_[1].add(robot.links_[2]);
  scl_obj.graphicsAddFrame(robot.links_[2]);

  // **************** Load a link **************************
  robot.links_[3] = new THREE.Object3D;
  p = scl_data_parsed.rb_tree_["lower_arm"].pos_in_parent_;
  robot.links_[3].position.set(p[0],  p[1],  p[2]);
  oLoader = new THREE.OBJLoader();
  oLoader.load('Puma/lower_arm_bl.obj',
               makeLoaderClosure(robot.links_[3],"lower_arm"));
  robot.links_[2].add(robot.links_[3]);
  scl_obj.graphicsAddFrame(robot.links_[3]);

  // **************** Load a link **************************
  robot.links_[4] = new THREE.Object3D;
  p = scl_data_parsed.rb_tree_["wrist-hand"].pos_in_parent_;
  robot.links_[4].position.set(p[0],  p[1],  p[2]);
  robot.links_[4].rotation.x = Math.PI/2;
  oLoader = new THREE.OBJLoader();
  oLoader.load('Puma/hand_bl.obj',
               makeLoaderClosure(robot.links_[4],"wrist-hand"));
  robot.links_[3].add(robot.links_[4]);
  scl_obj.graphicsAddFrame(robot.links_[4]);

  // **************** Load a link **************************
  robot.links_[5] = new THREE.Object3D;
  p = scl_data_parsed.rb_tree_["wrist-finger"].pos_in_parent_;
  robot.links_[5].position.set(p[0],  p[1],  p[2]);
  robot.links_[5].rotation.y = Math.PI/2;
  oLoader = new THREE.OBJLoader();
  oLoader.load('Puma/finger_bl.obj',
               makeLoaderClosure(robot.links_[5],"wrist-finger"));
  robot.links_[4].add(robot.links_[5]);
  scl_obj.graphicsAddFrame(robot.links_[5]);

  // **************** Load a link **************************
  robot.links_[6] = new THREE.Object3D;
  p = scl_data_parsed.rb_tree_["end-effector"].pos_in_parent_;
  robot.links_[6].position.set(p[0],  p[1],  p[2]);
  oLoader = new THREE.OBJLoader();
  oLoader.load('Puma/shaft2_bl.obj',
               makeLoaderClosure(robot.links_[6],"end-effector"));
  robot.links_[5].add(robot.links_[6]);
  scl_obj.graphicsAddFrame(robot.links_[6]);

  // ************** Init Graphics & Exit *******************
  //Set data structures
  this.robots_[this.robots_.length] = robot;
  this.global_root_.add(robot.links_[0]);
}

/** The main update loop. This will be called forever. Note that
 * you should add all your communication/state-update/render
 * operations here... */
function sclMainLoop () {
  requestAnimationFrame( sclMainLoop );

  // Run I/O stuff & update state
  scl_data_io = getRedisKey("SCL:PumaBot:IO");

  // Track mouse etc.
  scl_obj.camera_controls_.update();

  // Render the scene...
  scl_obj.renderer_.render(scl_obj.scene_, scl_obj.camera_);
  scl_obj.frames_rendered_++;

  // Tests etc..
  sclHtmlUpdateInRenderLoop("Time : " + scl_obj.clock_.getElapsedTime() +
    "<br />Frames rendered : "+ scl_obj.frames_rendered_ +
    "<br />Mouse pos : " + scl_obj.mouse_x_ + " " +scl_obj.mouse_y_+
    "<br />q_ : " + scl_data_io.sensors_.q_);
};

// ******************************************
// THE CODE ENTRY POINT IS HERE!!!
// The HTML code loads the js at this spot..
// ******************************************
function sclJsEntryPoint(){
  scl_data_parsed = getRedisKey("SCL:PumaBot:Parsed");
  scl_data_graphics = getRedisKey("SCL:PumaBot:Graphics");
  scl_data_io = getRedisKey("SCL:PumaBot:IO");

  scl_obj = new SCL();
  scl_obj.initWebGlScene();
  scl_obj.initCamAndLights();
  scl_obj.graphicsAddFrame(scl_obj.global_root_);
  scl_obj.testAddPuma();

  /*
  // Read in the parsed data

  // Test CODE
  sclAddOriginFrame(scl_obj.global_root_);
  //sclAddStaticMeshes();
  sclTestAddPuma();*/

  // ******************************************
  // Add any html related event handlers here.
  // ******************************************
  window.addEventListener("resize",function (){
    // For mouse pointer tracking..
    scl_obj.window_halfx_ = window.innerWidth/2;
    scl_obj.window_halfy_ = window.innerHeight/2;

    // For actual rendering..
    scl_obj.renderer_.setSize( window.innerWidth, window.innerHeight);
    scl_obj.camera_.aspect = window.innerWidth/window.innerHeight;
    scl_obj.camera_.updateProjectionMatrix();
  });

  document.addEventListener( 'mousemove', function(){
    scl_obj.mouse_x_ = ( event.clientX - scl_obj.window_halfx_) / 2;
    scl_obj.mouse_y_ = ( event.clientY - scl_obj.window_halfy_) / 2;
  }, false );

  // ******************************************
  // THE MAIN LOOP ENTRY POINT IS HERE!!!
  // ******************************************
  sclMainLoop();
}

