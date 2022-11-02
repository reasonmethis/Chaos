// Physics constants
const simStepsPerFrame = 1000;
const frameDelayMillis = 20;
var speedupFactor = 0.5;
const frictionHalfLifeSeconds = 0.5;
const inverseFrictionHalfLifeSeconds = 0;
const ballMass = 10;
const springRestLength = 0.5; // won't use
const springConst = 10.0;
const springConstMax = 1000.0;
const nBalls = 2;
var springRestLs = [0.5, 0.27];
const yAnchor = 0.5;
var springInitAngles = [-3.1416 * 0.4, 3.1416 * 0.01];
var masses = [5, 0.6];
var springConsts = [500.0, 500.0];
//var masses = [5, 0.1];
//var springConsts = [500.0, 90.0];
const gconstant = 9.8;

//ball types
const ANCHOR = 1;
const BUTTERFLY = 2;

const TINY = 1.0e-19;
// Rendering constants
const pixelsPerMeter = 200.0; // rendering zoom factor
const radiusPerMass = 0.05;
let iOrigin; // hor location of world origin on canvas [pixels]
let jOrigin; // ver location of world origin on canvas [pixels]
let xmin;
let xmax;
let ymin, ymax, xrng, yrng;
const ballRadiusMeters = 0.04;
const grabDistanceLimit = 0.0;

var sim, sims;
var kinE = 0.0;
var potE = 0.0;
var dispDiff = 0;
var dif1, dif2;

var context, canvas;

function* enumerate(it, start = 0) {
  let i = start;
  for (const x of it) yield [i++, x];
}
function removeElems(elements, indexesToBeRemoved) {
  //assumes indices are in ascending order
  while (indexesToBeRemoved.length) {
    elements.splice(indexesToBeRemoved.pop(), 1);
  }
}
class valHolder {
  constructor() {
    this.val = 0.0;
    this.cumu = 0.0;
    this.n = 0;
  }
  add(newval) {
    this.n += 1;
    this.cumu += newval;
    this.val = newval;
  }
  avg() {
    if (this.n) return (this.cumu * 1.0) / this.n;
    return 0.0;
  }
}
function clone(obj) {
  if (obj === null || typeof obj !== "object" || "isActiveClone" in obj)
    return obj;

  if (obj instanceof Date) var temp = new obj.constructor();
  else var temp = new obj.constructor();

  for (var key in obj) {
    if (Object.prototype.hasOwnProperty.call(obj, key)) {
      obj["isActiveClone"] = null;
      temp[key] = clone(obj[key]);
      delete obj["isActiveClone"];
    }
  }
  return temp;
}
class Ball {
  constructor(mass, anchor, x, y, color, radius) {
    this.mass = mass;
    this.color = color;
    this.anchor = anchor; // 0=mobile, 1 = ANCHOR, 2 = BUTTERFLY

    // position vector
    this.x = x;
    this.y = y;

    // velocity vector
    this.vx = 0.0;
    this.vy = 0.0;

    // force vector
    this.fx = 0.0;
    this.fy = 0.0;

    if (radius) {
      this.radius = radius;
    } else this.radius = radiusPerMass * this.mass ** 0.333333333;
  }

  distance(x, y) {
    const dx = this.x - x;
    const dy = this.y - y;
    return Math.sqrt(dx * dx + dy * dy);
  }
  potEnergy() {
    return this.mass * (this.y * gconstant);
  }
  kinEnergy() {
    return this.mass * 0.5 * (this.vx ** 2 + this.vy ** 2);
  }
}

class Spring {
  constructor(ball1, ball2, restLength, springConst) {
    this.ball1 = ball1;
    this.ball2 = ball2;
    this.restLength = restLength; // meters
    this.springConst = springConst; // newtons/meter
  }
  energy() {
    // Calculate the length of the spring.
    const dx = this.ball2.x - this.ball1.x;
    const dy = this.ball2.y - this.ball1.y;
    const len = Math.sqrt(dx * dx + dy * dy);

    // The difference between the spring's rest length and its current length
    // tells how much it is stretched or compressed.
    // Multiply by the spring constant to get the magnitude of the force.
    const displacement = len - this.restLength;
    return (this.springConst * displacement * displacement) / 2;
  }
  addForce() {
    // Calculate the length of the spring.
    const dx = this.ball2.x - this.ball1.x;
    const dy = this.ball2.y - this.ball1.y;
    const len = Math.sqrt(dx * dx + dy * dy);

    // The difference between the spring's rest length and its current length
    // tells how much it is stretched or compressed.
    // Multiply by the spring constant to get the magnitude of the force.
    const displacement = len - this.restLength;
    const force = this.springConst * displacement;

    // Safety valve: if two balls are at the same location, we avoid division by zero.
    if (Math.abs(len) >= TINY) {
      //was e-6
      // Calculate force vector = force magnitude * directional unit vector
      const fx = force * (dx / len);
      const fy = force * (dy / len);
      /// Rigid case
      if (this.springConst > springConstMax) {
      }

      // Add equal and opposite forces to the two connected balls.
      this.ball1.fx += fx;
      this.ball1.fy += fy;
      this.ball2.fx -= fx;
      this.ball2.fy -= fy;
    }
  }
}

class Electric {
  constructor(ball1, ball2, kfactor, pow = -2, unary = false) {
    this.ball1 = ball1;
    this.ball2 = ball2;
    this.kfactor = kfactor;
    this.pow = pow;
    this.unary = unary;
  }
  energy() {
    // Calculate the length of the spring.
    const dx = this.ball2.x - this.ball1.x;
    const dy = this.ball2.y - this.ball1.y;
    const len = Math.sqrt(dx * dx + dy * dy);
    const p = this.pow + 1;
    return p ? (this.kfactor * len ** p) / p : this.kfactor * Math.ln(len);
  }
  addForce() {
    // Calculate the length of the spring.
    const dx = this.ball2.x - this.ball1.x;
    const dy = this.ball2.y - this.ball1.y;
    const len = Math.sqrt(dx * dx + dy * dy);

    const force = this.kfactor * len ** this.pow;
    //alert("kkkk"+ force + "len"+len+"dx"+dx+"mass"+this.ball2.mass+"***" +this.ball2.x);

    // Safety valve: if two balls are at the same location, we avoid division by zero.
    if (len >= 1.0e-19) {
      //was e-6
      // Calculate force vector = force magnitude * directional unit vector
      const fx = force * (dx / len);
      const fy = force * (dy / len);

      // Add equal and opposite forces to the two connected balls.
      this.ball1.fx += fx;
      this.ball1.fy += fy;
      if (this.unary) return;
      this.ball2.fx -= fx;
      this.ball2.fy -= fy;
    }
  }
}
class Collision {
  constructor(ball1, ball2) {
    this.ball1 = ball1;
    this.ball2 = ball2;
  }
  iscollision() {
    if (dist2(this.ball1, this.ball2) < this.ball1.radius + this.ball2.radius) {
      return true;
    } else return false;
  }
}
class UnaryForce {
  constructor(ball, kfriction, powfriction = 1) {
    this.ball = ball;
    this.kfriction = kfriction;
    this.powfriction = powfriction;
  }
  addForce() {
    let v = (this.ball.vx ** 2 + this.ball.vy ** 2) ** 0.5;
    if (v < TINY) return;
    let f = (this.kfriction * v ** this.powfriction) / v;
    this.ball.fx -= f ** this.ball.vx;
    this.ball.fy -= f ** this.ball.vy;
  }
}
class Simulation {
  constructor() {
    this.ballList = [];
    this.springList = [];
    this.electricList = [];
    this.collisionList = [];
    this.interactions = [
      this.springList,
      this.electricList,
      this.collisionList,
    ];
    this.energy0 = 0.0;

    // Initialize gravity vector: 9.8 m/s^2, pointing straight down.
    this.gravity = -gconstant;

    this.grabbedBall = null;
  }

  copy() {
    let sim2 = clone(this);
    for (let i = 0; i < this.springList.length; ++i) {
      let s = this.springList[i];
      let i1 = this.ballList.indexOf(s.ball1);
      let i2 = this.ballList.indexOf(s.ball2);
      sim2.springList[i].ball1 = sim2.ballList[i1];
      sim2.springList[i].ball2 = sim2.ballList[i2];
    }
    return sim2;
  }

  perturb(frac) {
    for (let b of this.ballList) {
      b.x = b.x * (1 + (Math.random() - 0.5) * 2 * frac);
      b.y = b.y * (1 + (Math.random() - 0.5) * 2 * frac);
    }
  }

  addBall(ball) {
    this.ballList.push(ball);
    return ball;
  }

  addSpring(spring) {
    this.springList.push(spring);
    return spring;
  }
  addElectric(electric) {
    this.electricList.push(electric);
    return electric;
  }
  removeball(i) {
    let b = this.ballList[i];
    let badinds = [];
    for (let inter of this.interactions) {
      for (const [i, s] of enumerate(inter)) {
        if (s.ball1 === b || s.ball2 === b) {
          badinds.push(i);
        }
      }
      removeElems(inter, badinds);
    }
    this.ballList.splice(i, 1);
  }
  kinEnergy() {
    var e = 0;
    for (var b of this.ballList) {
      e += b.kinEnergy();
    }
    return e;
  }
  potEnergy() {
    var e = 0;
    var b;
    for (b of this.ballList) {
      e += b.potEnergy();
    }
    for (b of this.springList) {
      e += b.energy();
    }
    for (b of this.electricList) {
      e += b.energy();
    }

    return e;
  }

  setEnergy() {
    this.energy0 = this.kinEnergy() + this.potEnergy();
  }

  update(dt) {
    var b, s;

    // Calculate the force vectors acting on all the balls:
    // both from springs and from gravity.

    // Start out with just the gravitational force on each ball.
    for (b of this.ballList) {
      b.fx = 0.0;
      b.fy = b.anchor ? 0.0 : b.mass * this.gravity;
    }

    // Go through all the springs and calculate
    // the forces on the balls connected to their endpoints.
    // There will be equal and opposite forces on each pair.
    for (s of this.springList) {
      s.addForce();
    }
    for (s of this.electricList) {
      s.addForce();
    }
    for (s of this.collisionList) {
      if (s.iscollision()) {
        if (s.ball1.anchor == BUTTERFLY) {
        }
      }
    }

    // Now all the forces are correct.
    // Use the forces to update the position and speed of each ball.
    //const friction = Math.pow(0.5, dt / FrictionHalfLifeSeconds);
    const friction = Math.pow(0.5, dt * inverseFrictionHalfLifeSeconds);
    for (b of this.ballList) {
      if (b.anchor != ANCHOR) {
        // skip anchors, because they don't move
        // F = ma, therefore a = dv/dt = F/m.
        // dv = dt * F/m
        let dvx = (dt * b.fx) / b.mass;
        let dvy = (dt * b.fy) / b.mass;

        // Update the position using the mean speed in this increment.
        b.x += dt * (b.vx + dvx / 2.0);
        b.y += dt * (b.vy + dvy / 2.0);

        // Update the ball's speed. Apply friction to gradually reduce energy.
        b.vx = friction * b.vx + dvx;
        b.vy = friction * b.vy + dvy;
      }
    }
    const kineticE = this.kinEnergy();
    potE = this.potEnergy();
    const mismatchEnergy = -potE - kineticE + this.energy0;
    var factor = (mismatchEnergy + kineticE) / kineticE;
    if (factor < 0) {
      factor = 0;
    } else {
      factor = factor ** 0.5;
    }
    kinE = factor * kineticE;

    for (b of this.ballList) {
      if (b.anchor != ANCHOR) {
        // skip anchors, because they don't move
        b.vx = factor * b.vx;
        b.vy = factor * b.vy;
      }
    }
  }

  grab(x, y) {
    // Not allowed to grab more than one ball at a time (safety valve).
    if (this.grabbedBall) return;

    // Find the ball closest to the mouse coordinates.
    let closest;
    let bestDistance;
    if (this.ballList.length > 0) {
      closest = this.ballList[0];
      bestDistance = closest.distance(x, y);
      for (let i = 0; i < this.ballList.length; ++i) {
        const b = this.ballList[i];
        const distance = b.distance(x, y);
        if (distance < bestDistance) {
          closest = b;
          bestDistance = distance;
        }
      }

      // If it is close enough to be grabbed, grab it.
      if (bestDistance <= grabDistanceLimit) {
        ++closest.anchor;
        this.grabbedBall = closest;
        this.pull(x, y);
      }
    }
  }

  pull(x, y) {
    if (this.grabbedBall) {
      this.grabbedBall.x = x;
      this.grabbedBall.y = y;
      this.grabbedBall.vx = 0;
      this.grabbedBall.vy = 0;
    }
  }

  release() {
    if (this.grabbedBall) {
      --this.grabbedBall.anchor;
      this.grabbedBall = null;
    }
  }
}

function makeString(sim, x, y, shadow) {
  if (shadow) {
    var col1 = "#888";
    var col2 = col1;
  } else {
    var col1 = "#000";
    var col2 = "#e08";
  }
  let anchor1 = sim.addBall(
    new Ball(ballMass, ANCHOR, x, y, col1, ballRadiusMeters)
  );
  //let anchor1 = sim.AddBall(new Ball(BallMass, ANCHOR, x, y, col1, BallRadiusMeters * PixelsPerMeter));
  let xcur = x;
  let ycur = y;
  let prevBall = anchor1;
  for (let i = 0; i < nBalls; ++i) {
    //springInitAngles
    //let ball = sim.AddBall(new Ball(BallMass, 0, x + (0.027 * i), -0.05 * i));
    xcur += springRestLs[i] * Math.cos(springInitAngles[i]);
    ycur += springRestLs[i] * Math.sin(springInitAngles[i]);
    let ball = sim.addBall(new Ball(masses[i], 0, xcur, ycur, col2));
    sim.addSpring(new Spring(ball, prevBall, springRestLs[i], springConsts[i]));
    prevBall = ball;
  }
  //Set up electric forces
  /*		for (let i = 0; i < nBalls; ++i){
                  
                  for (let j = i + 1; j < nBalls; ++j) {
                      var bi = sim.ballList[i];
                      var bj = sim.ballList[j];
                      sim.AddElectric(new Electric(bi, bj, bi.mass * bj.mass * Gconst));
                }
            }
    */
}

function initWorld() {
  let sim = new Simulation();
  makeString(sim, 0.0, yAnchor, 0);
  sim.setEnergy();
  sims = [];
  sims.push(sim);
  /*sim = new Simulation();
            MakeString(sim, 0.2, y_anchor, 0);
              sim.energy0 = sim.kinEnergy() + sim.potEnergy();
              sims.push(sim);*/

  return sim;
}
function shiftsim(sim, dx, dy) {
  for (let b of sim.ballList) {
    b.x += dx;
    b.y += dy;
  }
}
function triple() {
  if (sims.length > 1) {
    //sims = [sims[0]];
    sims[1].ballList[0].anchor = 0;
    sims[2].ballList[0].anchor = 0;
    sims[1].ballList[0].vx = -12;
    sims[2].ballList[0].vx = 12;
    sims[1].setEnergy();
    sims[2].setEnergy();
    dispDiff = false;
    return;
  }
  let sim2 = sim.copy();
  shiftsim(sim2, xmin + 0.25 * xrng, 0);
  sim2.perturb(0.001);
  sims.push(sim2);
  let sim3 = sim.copy();
  shiftsim(sim3, xmin + 0.75 * xrng, 0);
  sim3.perturb(0.000001);
  sims.push(sim3);
  dispDiff = true;
  dif1 = new valHolder();
  dif2 = new valHolder();

  /*let massbutterf1 = 0.1
              let massbutterf2 = 0.01
              let b = sims[1].AddBall(new Ball(massbutterf1, BUTTERFLY, xmin, ymin, "#03f"))
              sims[1].AddElectric(new Electric(b, sims[1].ballList[1], 0.1, 0.5, true))
              b = sims[2].AddBall(new Ball(massbutterf2, BUTTERFLY, xmax, ymin, "#03f"))
              sims[2].AddElectric(new Electric(b, sims[2].ballList[1], 0.01, -0.5, true))
          
              sims.forEach((sim) => {disp(sim); sim.setEnergy()} )*/
  //sims.forEach((sim) => ( disp(555); return sim.setEnergy();))
  //disp(simDiff(sims[0], sim2))
  //disp(simDiff(sims[0], sim3))
}
function screenHor(x) {
  return iOrigin + pixelsPerMeter * x;
}

function screenVer(y) {
  return jOrigin - pixelsPerMeter * y;
}

function worldX(hor) {
  return (hor - iOrigin) / pixelsPerMeter;
}

function worldY(ver) {
  return (jOrigin - ver) / pixelsPerMeter;
}

function render(sim) {
  context.strokeStyle = "#ddd";
  context.lineWidth = 1;
  for (let s of sim.springList) {
    const d = 0.025;
    const n = 10;
    const segx = (s.ball2.x - s.ball1.x) / 4.0 / n;
    const segy = (s.ball2.y - s.ball1.y) / 4.0 / n;
    const segl = (segx ** 2 + segy ** 2) ** 0.5;
    const devx = (segy / segl) * d;
    const devy = (-segx / segl) * d;
    context.beginPath();
    context.moveTo(screenHor(s.ball1.x), screenVer(s.ball1.y));
    var xcur = s.ball1.x;
    var ycur = s.ball1.y;
    for (let i = 1; i <= n; i++) {
      context.lineTo(
        screenHor(xcur + segx + devx),
        screenVer(ycur + segy + devy)
      );
      context.lineTo(screenHor(xcur + segx * 2), screenVer(ycur + segy * 2));
      context.lineTo(
        screenHor(xcur + segx * 3 - devx),
        screenVer(ycur + segy * 3 - devy)
      );
      context.lineTo(screenHor(xcur + segx * 4), screenVer(ycur + segy * 4));
      xcur += segx * 4;
      ycur += segy * 4;
    }
    ///context.lineTo(ScreenHor(s.ball2.x), ScreenVer(s.ball2.y));
    context.stroke();
  }

  context.lineWidth = 1;

  for (let b of sim.ballList) {
    if (0 === 0) {
      // Draw each mobile ball as a filled-in circle.
      context.strokeStyle = b.color;
      context.fillStyle = context.strokeStyle;
      context.beginPath();
      context.arc(
        screenHor(b.x),
        screenVer(b.y),
        b.radius * pixelsPerMeter,
        0,
        2 * Math.PI,
        true
      );
      context.fill();
      context.stroke();
    } else {
      // Draw each anchored ball as a filled-in, red square.
      context.fillStyle = "#f80";
      const x1 = screenHor(b.x) - pixelRadius;
      const y1 = screenVer(b.y) - pixelRadius;
      context.strokeRect(x1, y1, 2 * pixelRadius, 2 * pixelRadius);
      context.fillRect(x1, y1, 2 * pixelRadius, 2 * pixelRadius);
    }
  }
  /* */
}
function disp(st) {
  console.log(st);
}
function rnd(x, ndec) {
  if (ndec === undefined) {
    disp(ndec);
    ndec = 2;
  }
  let k = 10 ** ndec;
  return (Math.round((x * k * 10) / 10) / k).toFixed(ndec);
}
function smartrnd(x, sigfigs, ndecmax, xref) {
  var thresh = 1.0;
  var dx = Math.abs(x - xref);
  for (var ndec = sigfigs; ndec <= ndecmax; ++ndec) {
    if (dx >= thresh) break;
    thresh /= 10.0;
  }
  return rnd(x, ndec);
}

function dist2(a, b) {
  return (a.x - b.x) ** 2 + (a.y - b.y) ** 2;
}

function drawLine(x1, y1, x2, y2) {
  context.beginPath();
  context.moveTo(x1, y1);
  context.lineTo(x2, y2);
  context.stroke();
}

function simDiff(sim1, sim2) {
  var dev = 0;
  var devv = 0;
  var dxref = 0.0000001;
  var dvref = 0.0000001;
  var dx = sim2.ballList[0].x - sim1.ballList[0].x;
  var nballs = Math.min(sim1.ballList.length, sim2.ballList.length);
  //var dx = sim2.ballList[0].x - sim1.ballList[0].x;
  for (let i = 1; i < nballs; ++i) {
    dxref += dist2(sim1.ballList[0], sim1.ballList[i]);
    dev += (sim2.ballList[i].x - sim1.ballList[i].x - dx) ** 2;
    dev += (sim2.ballList[i].y - sim1.ballList[i].y) ** 2;
    dvref += sim1.ballList[i].vx ** 2 + sim1.ballList[i].vy ** 2;
    dvref += sim2.ballList[i].vx ** 2 + sim2.ballList[i].vy ** 2;
    devv += (sim2.ballList[i].vx - sim1.ballList[i].vx) ** 2;
    devv += (sim2.ballList[i].vy - sim1.ballList[i].vy) ** 2;
  }
  //return (devv / dvref) ** 0.5 * 500;
  //return (dev / dxref * 2 + devv / dvref) ** 0.5 * 500;
  var diff = ((dev / dxref) * 2 + devv / dvref) ** 0.5 * 500;
  var linsync = Math.max(0.0, 1 - diff / 500);
  return linsync ** 4 * 100;
}
function manageAnimation(timestamp) {
  window.setTimeout(animationFrame, frameDelayMillis / 1000);
}
function animationFrame() {
  //check if thrown sims left the visible area, then delete them
  let still = false;
  for (let i = 1; i < sims.length; ++i) {
    for (let b of sims[i].ballList) {
      if (
        b.x < xmax + b.radius &&
        b.x > xmin - b.radius &&
        b.y < ymax + b.radius &&
        b.y > ymin - b.radius
      ) {
        still = true;
        break;
      }
    }
    if (still) break;
  }
  if (!still) {
    sims = [sims[0]];
  }
  //perform updates and render
  const dt = ((0.001 * frameDelayMillis) / simStepsPerFrame) * speedupFactor;
  for (let i = 0; i < simStepsPerFrame; ++i) {
    for (let s of sims) {
      s.update(dt);
    }
  }
  context.clearRect(0, 0, canvas.clientWidth, canvas.clientHeight);
  for (let s of sims) {
    render(s);
  }
  if (dispDiff) {
    context.strokeStyle = "#000";
    dif1.add(simDiff(sims[0], sims[1]));
    dif2.add(simDiff(sims[0], sims[2]));
    let y1 = 5;
    let y2 = y1 + 10;
    let x1 = iOrigin - dif1.val;
    let x2 = iOrigin + dif1.val;
    //drawLine(x1, y1, x1, y2); drawLine(x2, y1, x2, y2);
    context.fillStyle = context.strokeStyle;
    context.font = "14px Poppins";
    let ysync = 14;
    context.fillText("Sync:", screenHor(xmin + xrng * 0.001), ysync);
    context.fillText(
      "" + smartrnd(dif1.val, 0, 3, 100) + "%",
      screenHor(xmin + xrng * 0.235),
      ysync
    );
    context.fillText(
      "" + smartrnd(dif2.val, 0, 3, 100) + "%",
      screenHor(xmin + xrng * 0.735),
      ysync
    );
  }
  requestAnimationFrame(manageAnimation);
}

function onMouseDown(evt) {
  const canvas = document.getElementById("SimCanvas");
  const hor = evt.pageX - canvas.offsetLeft;
  const ver = evt.pageY - canvas.offsetTop;
  const x = worldX(hor);
  const y = worldY(ver);
  sim.grab(x, y);
  console.log("ms dn");
}

function onMouseUp(evt) {
  sim.release();
  triple();
}

function onMouseMove(evt) {
  const canvas = document.getElementById("SimCanvas");
  const hor = evt.pageX - canvas.offsetLeft;
  const ver = evt.pageY - canvas.offsetTop;
  const x = worldX(hor);
  const y = worldY(ver);
  sim.pull(x, y);
}

function onMouseLeave() {
  sim.Release();
}

function process_touchstart(e) {
  console.log("tstart");
}

function process_touchend(e) {
  console.log("tstart");
}

function process_touchmove(e) {
  console.log("tstart");
}

function process_touchcancel(e) {
  console.log("tstart");
}
function widthNoPadd(element) {
  var cs = getComputedStyle(element);

  var paddingX = parseFloat(cs.paddingLeft) + parseFloat(cs.paddingRight);
  var paddingY = parseFloat(cs.paddingTop) + parseFloat(cs.paddingBottom);

  var borderX =
    parseFloat(cs.borderLeftWidth) + parseFloat(cs.borderRightWidth);
  var borderY =
    parseFloat(cs.borderTopWidth) + parseFloat(cs.borderBottomWidth);

  // Element width and height minus padding and border
  var elementWidth = element.offsetWidth - paddingX - borderX;
  var elementHeight = element.offsetHeight - paddingY - borderY;
  return elementWidth;
}

window.onload = function () {
  canvas = document.getElementById("SimCanvas");
  const canvframe = document.getElementById("canvframe");
  const w = widthNoPadd(canvframe);
  canvas.width = w;
  context = canvas.getContext("2d");
  //canvas.clientWidth = w;
  iOrigin = canvas.width / 2;
  jOrigin = canvas.height / 2;
  xmin = worldX(0);
  ymax = worldY(0);
  xmax = worldX(w);
  ymin = worldY(canvas.height);
  xrng = xmax - xmin;
  yrng = ymax - ymin;
  sim = initWorld();
  //console.log(canvas.clientWidth);
  //console.log(canvas.innerWidth);
  //console.log(canvas.width);
  canvas.addEventListener("mousedown", onMouseDown);
  canvas.addEventListener("mouseup", onMouseUp);
  canvas.addEventListener("mousemove", onMouseMove);
  canvas.addEventListener("mouseleave", onMouseLeave);

  // Register touch event handlers
  canvas.addEventListener("touchstart", process_touchstart, false);
  canvas.addEventListener("touchmove", process_touchmove, false);
  canvas.addEventListener("touchcancel", process_touchcancel, false);
  canvas.addEventListener("touchend", process_touchend, false);
  animationFrame();
};
