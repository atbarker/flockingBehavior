

var flock;



function setup() {
  var canvas = createCanvas(1000,720);
  canvas.parent('sketch-holder');
  
  flock = new Flock();
  //Add an initial set of boids into the system
  for (var i = 0; i < 100; i++) {
    var rand = random([0, 1, 2, 3]);
    var b = new ind(width/2,height/2, rand);
    flock.addInd(b);
  }
}

function draw() {
  background(51);
  flock.run();
}

// Add a new boid into the System
function mouseDragged() {
  var rand = random([0, 1, 2, 3]);
  flock.addInd(new ind(mouseX,mouseY, rand));
}

function preload() {
  twitter = loadImage("twitter-color-iso.png");
  tumblr = loadImage("tumblr-color-iso.png");
  facebook = loadImage("facebook-color-iso.png");
  reddit = loadImage("reddit-color-iso.png");
}


//For each boid you will have a social media site it is associated with.
/* id 0 = twitter
 * 1 = facebook
 * 2 = reddit
 * 3 = tumblr
 */

// Flock object
// Does very little, simply manages the array of all the boids

function Flock() {
  // An array for all the boids
  this.internet = []; // Initialize the array
}

Flock.prototype.run = function() {
  for (var i = 0; i < this.internet.length; i++) {
    this.internet[i].run(this.internet);  // Passing the entire list of boids to each boid individually
  }
}

Flock.prototype.addInd = function(b) {
  this.internet.push(b);
}


// Boid class
// Methods for Separation, Cohesion, Alignment added

function ind(x,y, media) {
  this.acceleration = createVector(0,0);
  this.velocity = createVector(random(-1,1),random(-1,1));
  this.position = createVector(x,y);
  this.r = 3.0;
  this.maxspeed = 3;    // Maximum speed
  this.maxforce = 0.05; // Maximum steering force
  this.id = media;
}

ind.prototype.run = function(internet) {
  this.flock(internet);
  this.update();
  this.borders();
  this.render();
}

ind.prototype.applyForce = function(force) {
  // We could add mass here if we want A = F / M
  this.acceleration.add(force);
}

// We accumulate a new acceleration each time based on three rules
ind.prototype.flock = function(internet) {
  var sep = this.separate(internet);   // Separation
  var ali = this.align(internet);      // Alignment
  var coh = this.cohesion(internet);   // Cohesion
  // Arbitrarily weight these forces
  sep.mult(1.5);
  ali.mult(1.0);
  coh.mult(1.0);
  // Add the force vectors to acceleration
  this.applyForce(sep);
  this.applyForce(ali);
  this.applyForce(coh);
}

// Method to update location
ind.prototype.update = function() {
  // Update velocity
  this.velocity.add(this.acceleration);
  // Limit speed
  this.velocity.limit(this.maxspeed);
  this.position.add(this.velocity);
  // Reset accelertion to 0 each cycle
  this.acceleration.mult(0);
}

// A method that calculates and applies a steering force towards a target
// STEER = DESIRED MINUS VELOCITY
ind.prototype.seek = function(target) {
  var desired = p5.Vector.sub(target,this.position);  // A vector pointing from the location to the target
  // Normalize desired and scale to maximum speed
  desired.normalize();
  desired.mult(this.maxspeed);
  // Steering = Desired minus Velocity
  var steer = p5.Vector.sub(desired,this.velocity);
  steer.limit(this.maxforce);  // Limit to maximum steering force
  return steer;
}

ind.prototype.render = function() {
  // Draw a triangle rotated in the direction of velocity
  var theta = this.velocity.heading() + radians(90);
  if (this.id === 0){
    image(twitter, this.position.x, this.position.y, 30, 30);
  }else if (this.id === 1){
    image(facebook, this.position.x, this.position.y, 30, 30);
  }else if (this.id === 2){
    image(reddit, this.position.x, this.position.y, 30, 30);
  }else if (this.id === 3){
    image(tumblr, this.position.x, this.position.y, 30, 30);
  }
  pop();
}

// Wraparound
ind.prototype.borders = function() {
  if (this.position.x < -this.r)  this.position.x = width +this.r;
  if (this.position.y < -this.r)  this.position.y = height+this.r;
  if (this.position.x > width +this.r) this.position.x = -this.r;
  if (this.position.y > height+this.r) this.position.y = -this.r;
}


// Separation
// Method checks for nearby boids and steers away
ind.prototype.separate = function(internet) {
  var desiredseparation = 25.0;
  var steer = createVector(0,0);
  var count = 0;
  // For every boid in the system, check if it's too close
  for (var i = 0; i < internet.length; i++) {
    if (this.id != internet[i].id) desiredseperation = 40.0;
    if (this.id === 2 && internet[i].id === 3) desiredseperation = 200.0;
    if (this.id === 1 && internet[i].id === 1) desiredseperation = 100.0;
    var d = p5.Vector.dist(this.position,internet[i].position);
    // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
    if ((d > 0) && (d < desiredseparation)) {
      // Calculate vector pointing away from neighbor
      var diff = p5.Vector.sub(this.position,internet[i].position);
      diff.normalize();
      diff.div(d);        // Weight by distance
      steer.add(diff);
      count++;            // Keep track of how many
    }
  }
  // Average -- divide by how many
  if (count > 0) {
    steer.div(count);
  }

  // As long as the vector is greater than 0
  if (steer.mag() > 0) {
    // Implement Reynolds: Steering = Desired - Velocity
    steer.normalize();
    steer.mult(this.maxspeed);
    steer.sub(this.velocity);
    steer.limit(this.maxforce);
  }
  return steer;
}

// Alignment
// For every nearby boid in the system, calculate the average velocity
ind.prototype.align = function(internet) {
  var neighbordist = 50;
  var sum = createVector(0,0);
  var count = 0;
  for (var i = 0; i < internet.length; i++) {
    var d = p5.Vector.dist(this.position,internet[i].position);
    if ((d > 0) && (d < neighbordist)) {
      sum.add(internet[i].velocity);
      count++;
    }
  }
  if (count > 0) {
    sum.div(count);
    sum.normalize();
    sum.mult(this.maxspeed);
    var steer = p5.Vector.sub(sum,this.velocity);
    steer.limit(this.maxforce);
    return steer;
  } else {
    return createVector(0,0);
  }
}

// Cohesion
// For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
// Only applies to units with the same id (idealy)
ind.prototype.cohesion = function(internet) {
  var neighbordist = 50;
  var sum = createVector(0,0);   // Start with empty vector to accumulate all locations
  var count = 0;
  for (var i = 0; i < internet.length; i++) {
    if (this.id === internet[i].id){
      var d = p5.Vector.dist(this.position,internet[i].position);
      if ((d > 0) && (d < neighbordist)) {
        sum.add(internet[i].position); // Add location
        count++;
      }
    }
    if (this.id === 2 && internet[i].id === 3){
      var d = p5.Vector.dist(this.position,internet[i].position);
      if ((d > 0) && (d < neighbordist)) {
        sum.add(internet[i].position); // Add location
        count--;
      }
    }
  }
  if (count > 0) {
    sum.div(count);
    return this.seek(sum);  // Steer towards the location
  } else {
    return createVector(0,0);
  }
}