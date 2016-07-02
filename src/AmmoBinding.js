// Defines the global varibale, Ammo
require('ammo.js');

/**
 * Bindings for the GaemWorld to Ammo.js
 */
function AmmoBinding () {

  var collisionConfiguration = new Ammo.btDefaultCollisionConfiguration();
  var dispatcher = new Ammo.btCollisionDispatcher(collisionConfiguration);
  var overlappingPairCache = new Ammo.btDbvtBroadphase();
  var solver = new Ammo.btSequentialImpulseConstraintSolver();

  this.dynamicsWorld = new Ammo.btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
  this.dynamicsWorld.setGravity(new Ammo.btVector3(0, -5, 0));

  // Generator callbacks for different shapes and materials
  this.geometries = {};

  this.geometries.box = function (properties, scale) {
    return new Ammo.btBoxShape(new Ammo.btVector3(
      properties.size[0] * scale / 2,
      properties.size[1] * scale / 2,
      properties.size[2] * scale / 2
    ));
  };

  this.geometries.sphere = function (properties, scale) {
    return new Ammo.btSphereShape(properties.radius * scale);
  };
}

AmmoBinding.prototype.bindTo = function (gameWorld) {
  this.gameWorld = gameWorld;
}

/**
 * Returns true if the given object should be included in the physics sim
 * Objects must have geometry in order to be included, and the isGhost property
 * shouldn't be set.
 */
AmmoBinding.prototype.objectHasPhysics = function (gameObject) {
  return (gameObject.properties.geometry && !gameObject.properties.isGhost);
}

/**
 * Add a Ammo.btRigidBody to the bound gameworld, based on the properties payload
 * Links the mesh to the given gameObject
 */
AmmoBinding.prototype.addToGameObject = function (gameObject, properties) {
  if(!this.objectHasPhysics(gameObject)) return;

//  properties = Object.assign({ physicalMaterial: 'default' }, properties);

  if (!gameObject.properties.scale) gameObject.properties.scale = 1;

  this.updateGameObject(gameObject, properties);
};

AmmoBinding.prototype.updateGameObject = function (gameObject, properties) {
  if(!this.objectHasPhysics(gameObject)) return;

  if((properties.scale && properties.scale != gameObject.ammoLastScale) || properties.geometry) {
    gameObject.ammoLastScale = gameObject.properties.scale;

    // Clear previous shape
    if (gameObject.body) {
      this.dynamicsWorld.removeRigidBody(gameObject.body);
    }

    var shape = this.renderGeometry(gameObject.properties.geometry, gameObject.properties.scale);

    // Calculate mass and inertia
    var mass = gameObject.properties.mass;
    var localInertia = new Ammo.btVector3(0, 0, 0);
    if(mass) {
      shape.calculateLocalInertia(mass, localInertia);
    }

    var startTransform = new Ammo.btTransform();
    startTransform.setIdentity();
    var myMotionState = new Ammo.btDefaultMotionState(startTransform);

    var rbInfo = new Ammo.btRigidBodyConstructionInfo(mass, myMotionState, shape, localInertia);
    var body = new Ammo.btRigidBody(rbInfo);
    gameObject.body = body;

    this.dynamicsWorld.addRigidBody(body);
  }

  if(properties.position || properties.quaternion) {
    var transform = gameObject.body.getWorldTransform();

    if(properties.position) {
      var origin = transform.getOrigin();
      origin.setX(properties.position[0]);
      origin.setY(properties.position[1]);
      origin.setZ(properties.position[2]);
    }
    if(properties.quaternion) {
      transform.setRotation(new Ammo.btQuaternion(properties.quaternion[0], properties.quaternion[1], properties.quaternion[2], properties.quaternion[3]));
    }

    gameObject.body.getMotionState().setWorldTransform(transform);
  }

  if(properties.velocity) {
    var velocity = gameObject.body.getLinearVelocity();
    velocity.setX(properties.velocity[0] * gameObject.properties.scale);
    velocity.setY(properties.velocity[1] * gameObject.properties.scale);
    velocity.setZ(properties.velocity[2] * gameObject.properties.scale);
  }

  if(properties.angularVelocity) {
    var angularVelocity = gameObject.body.getAngularVelocity();
    angularVelocity.setX(properties.angularVelocity[0]);
    angularVelocity.setY(properties.angularVelocity[1]);
    angularVelocity.setZ(properties.angularVelocity[2]);
  }

  /*
  if(properties.physicalMaterial) {
    gameObject.body.material = this.getMaterial(properties.physicalMaterial);
  }
  */
}

/**
 * Processes the removal of a given game object
 * @param properties the altered properties
 */
AmmoBinding.prototype.removeGameObject = function (gameObject) {
  if(!this.objectHasPhysics(gameObject)) return;

  if (gameObject.body) {
    this.dynamicsWorld.removeRigidBody(gameObject.body);
    delete gameObject.body;
  }
}


/**
 * Render the given geometry payload to a CannonJS shape
 */
AmmoBinding.prototype.renderGeometry = function (properties, scale) {
  return this.geometries[properties.type](properties, scale || 1);
};

// Global to reduce memory leaking :-/
var bodyReadTransform = new Ammo.btTransform();

AmmoBinding.prototype.step = function(time, options) {
  // Set defaults
  options = Object.assign(
    { syncToWorld: true, flushQueue: true },
    options
  );

  // Run the sim
  this.dynamicsWorld.stepSimulation(time, 2);

  // Send world changes
  if (options.syncToWorld) {
    var i, gameObject, origin, rotation;
    var doFlush = 0;
    for(i in this.gameWorld.objects) {
      doFlush++;
      // To do - only send if masss > 0
      gameObject = this.gameWorld.objects[i];
      if(gameObject.body) {
        gameObject.body.getMotionState().getWorldTransform(bodyReadTransform);
        origin = bodyReadTransform.getOrigin();
        rotation = bodyReadTransform.getRotation();
        velocity = gameObject.body.getLinearVelocity();
        angularVelocity = gameObject.body.getAngularVelocity();

        this.gameWorld.queueUpdate(i, {
          position: [origin.x(), origin.y(), origin.z()],
          quaternion: [rotation.x(), rotation.y(), rotation.z(), rotation.w()],
          velocity: [velocity.x(), velocity.y(), velocity.z()],
          angularVelocity: [angularVelocity.x(), angularVelocity.y(), angularVelocity.z()],
        },
        {
            skipBindings: [ this.identifier ]
        });
      }
    }

    if (doFlush > 0 && options.flushQueue) {
      this.gameWorld.flushQueue();
    }
  }
};


module.exports = AmmoBinding;
