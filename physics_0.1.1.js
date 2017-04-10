// physics.js is an add-on ZIM module to help with Box2D
// (17K) currently only providing a non-minified file - minify to save 10K
// built for and requires Box2dWeb-2.1.a.3.min.js (220K)

// physics.js abstracts the world creation
// makes borders around the world
// makes rectangles, circles and triangles and adds x, y and rotation
// maps these to ZIM assets like shapes and bitmaps
// wraps mouse control code
// abstracts the debug mode
// abstracts the update function and provides Ticker access before and after stepping

// ZIM js Interactive Media modules http://zimjs.com by Dan Zen http://danzen.com (c) 2016
// free to use - donations welcome of course! http://zimjs.com/donate

// PHYSICS CLASS DOC

/*--
zim.Physics = function(frame, borders, gravity)

create a new zim.Physics(parameters) object
which makes a world and provides access to all ZIM physics properties and methods
to build and map shapes, run mouse countrol and debugging, etc.
see ZIM Bits examples on Physics for usage

PARAMETERS:
frame - the zim.Frame object
borders - an object literal with x, y, width and height properties
	default is the frame bounds - pass in "none" for no borders
	you can remove borders afterwards with physics.remove(physics.borderTop)
	and then borderLeft, borderRight and borderBottom
gravity - default 10, sets your gravity

METHODS
makeRectangle() - makes a Box2D body in the shape of a rectangle - see parameters
makeCircle() - makes a Box2D body in the shape of a rectangle - see parameters
makeTriangle() - makes a Box2D body in the shape of a rectangle - see parameters
	all three add x, y and rotation properties to body (use at start)
	all three support ZIM DUO single parameter as an object using param names as keys
remove(body) - lets you remove a body from the world including borders
debug() - activates the debugging
updateDebug() - updates the debug canvas if the frame has been scaled (put in frame.resize())
removeDebug() - removes the debug canvas - you can add it again later (or toggle, etc.)
drag([body, body]) - drag all dynamic bodies or filter with optional body array as parameter
noDrag() - stops dragging all bodies
addMap(body, asset) - sets x, y, rotation of ZIM asset to Box2D body
removeMap(body) - removes mapping (then probably will want to remove body and removeChild)
dispose() - stops the update, removes debug if there - you still need to remove ZIM assets
*** also see Ticker below for methods to add and remove functions from update function

PROPERTIES
world - the Box2D world that is made
scale (read only) scale used in world (constant 30)
step (read only) step used in world (constant 20)
gravity (read only) gravity used in world (whatever parameter was passed)
Ticker - gives access to update function to add your own functions:
	physics.Ticker.add(function, after)
		after defaults to true for after world step and force clear
		set after to false to run function before world step
	physics.Ticker.remove(function)

GLOBAL VARIABLES
physics.js gives global shortcut access to the following Box2D classes
*/

var b2Vec2 = Box2D.Common.Math.b2Vec2;
var b2BodyDef = Box2D.Dynamics.b2BodyDef;
var b2Body = Box2D.Dynamics.b2Body;
var b2FixtureDef = Box2D.Dynamics.b2FixtureDef;
var b2Fixture = Box2D.Dynamics.b2Fixture;
var b2World = Box2D.Dynamics.b2World;
var b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape;
var b2CircleShape = Box2D.Collision.Shapes.b2CircleShape;
var b2MouseJointDef = Box2D.Dynamics.Joints.b2MouseJointDef;
var b2DistanceJointDef = Box2D.Dynamics.Joints.b2DistanceJointDef;
var b2AABB = Box2D.Collision.b2AABB;
var b2DebugDraw = Box2D.Dynamics.b2DebugDraw;
var b2BuoyancyController = Box2D.Dynamics.Controllers.b2BuoyancyController;
var b2ContactListener = Box2D.Dynamics.b2ContactListener;

var zim = function(zim) {

	zim.Physics = function(frame, borders, gravity) {

		if (zon) zog("ZIM PHYSICS");

		if (zot(frame) || zot(frame.asset)) {console.log("zim.Physics() - please provide a zim Frame object"); return;}
		if (zot(borders)) borders = {x:0, y:0, width:frame.width, height:frame.height};
		if (zot(gravity)) gravity = 10;

		var that = this;
		this.gravity = gravity;
		var scale = this.scale = 30;
		var step = this.step = 20;
		var timeStep = 1/step;

		var world = this.world = new b2World(new b2Vec2(0, gravity), true); // gravity, allow sleep

		// each of these return a b2Body with x, y, and rotation properties added
		this.makeRectangle = function(width, height, dynamic, friction, angular, density, restitution, maskBits, categoryBits, linear) {
			var duo; if (duo = zob(that.makeRectangle, arguments)) return duo;
			return makeShape(["rectangle", width, height], dynamic, friction, angular, density, restitution, maskBits, categoryBits, linear);
		}
		this.makeCircle = function(radius, dynamic, friction, angular, density, restitution, maskBits, categoryBits, linear) {
			var duo; if (duo = zob(that.makeCircle, arguments)) return duo;
			return makeShape(["circle", radius], dynamic, friction, angular, density, restitution, maskBits, categoryBits, linear);
		}
		this.makeTriangle = function(a, b, c, dynamic, friction, angular, density, restitution, maskBits, categoryBits, linear) {
			var duo; if (duo = zob(that.makeTriangle, arguments)) return duo;
			return makeShape(["triangle", a, b, c], dynamic, friction, angular, density, restitution, maskBits, categoryBits, linear);
		}

		function makeShape(shape, dynamic, friction, angular, density, restitution, maskBits, categoryBits, linear) {
			var type = shape[0];
			if (zot(dynamic)) dynamic = true;
			if (zot(friction)) friction = 1;
			if (zot(angular)) angular = .5; // rotational damping
			if (zot(linear)) linear = .5; // linear damping
			if (zot(density)) density = 1;
			if (zot(restitution)) restitution = 0;

			var definition = new b2BodyDef();

			if (dynamic) {
				definition.type = b2Body.b2_dynamicBody;
			} else {
				definition.type = b2Body.b2_staticBody;
			}
			definition.angularDamping = angular;
			definition.linearDamping = linear;
			var body = world.CreateBody(definition);
			var s;
			if (type=="rectangle") {
				s = new b2PolygonShape();
				if (zot(shape[1])) shape[1] = 100;
				if (zot(shape[2])) shape[2] = 100;
				s.width = shape[1];
				s.height = shape[2];
				s.SetAsBox(s.width/scale/2, s.height/scale/2);

			} else if (type=="triangle") {
				if (!zim) return;
				s = new b2PolygonShape();
				if (zot(shape[1])) shape[1] = 100;
				if (zot(shape[2])) shape[2] = 100;
				if (zot(shape[3])) shape[3] = 100;
				// uses zim Triangle to match Box2D shape
				var tri = new zim.Triangle(shape[1], shape[2], shape[3]);
				s.width = tri.width;
				s.height = tri.height;
				var points = [];
				// outside is right of line - so needed to reverse order
				points[2] = new b2Vec2((tri.one.x-tri.regX)/scale, (tri.one.y+tri.regY)/scale);
				points[1] = new b2Vec2((tri.two.x-tri.regX)/scale, (tri.two.y+tri.regY)/scale);
				points[0] = new b2Vec2((tri.three.x-tri.regX)/scale, (tri.three.y+tri.regY)/scale);
				s.SetAsArray(points, points.length);
			} else { // circle
				if (zot(shape[1])) shape[1] = 50;
				s = new b2CircleShape(shape[1]/scale);
				s.width = s.height = shape[1]*2;
			}
			var fixture = new b2FixtureDef();
			if (!zot(categoryBits)) fixture.filter.categoryBits = categoryBits;
			fixture.shape = s;
			fixture.density = density;
			fixture.friction = friction;
			fixture.restitution = restitution;
			body.CreateFixture(fixture);
			body.width = s.width;
			body.height = s.height;

			// these hold x, y and rotation local values
			body.zimX = 0;
			body.zimY = 0;
			body.zimR = 0;
			setBasicProperties(body);
			return body;
		}

		function setBasicProperties(body) {
			Object.defineProperty(body, 'x', {
				get: function() {
					return body.GetWorldCenter().x*that.scale;
				},
				set: function(x) {
					body.zimX = x;
					body.SetPosition(new b2Vec2(body.zimX/scale, body.zimY/scale));
				}
			});
			Object.defineProperty(body, 'y', {
				get: function() {
					return body.GetWorldCenter().y*that.scale;
				},
				set: function(y) {
					body.zimY = y;
					body.SetPosition(new b2Vec2(body.zimX/scale, body.zimY/scale));
				}
			});
			Object.defineProperty(body, 'rotation', {
				get: function() {
					return body.GetAngle()*180/Math.PI;
				},
				set: function(rotation) {
					body.zimR = rotation;
					body.SetAngle(rotation*Math.PI/180);
				}
			});
		}

		var debug;
		this.debug = function() {
			if (debug) {
				debug.debugCanvas.style.display = "block";
			} else {
				// make the Debug object with its canvas only once
				debug = new this.Debug();
			}
			debug.active = true;
			that.updateDebug();
		}
		this.Debug = function() {
			var debugCanvas = this.debugCanvas = document.createElement("canvas");
			debugCanvas.setAttribute("id", "debugCanvas");
			if (frame.scale != 1) {
				debugCanvas.setAttribute("width", frame.width);
				debugCanvas.setAttribute("height", frame.height);
			} else {
				var largest = Math.max(window.innerWidth, screen.width, window.innerHeight, screen.height);
				debugCanvas.setAttribute("width", largest);
				debugCanvas.setAttribute("height", largest);
			}
			frame.canvas.parentElement.appendChild(debugCanvas);

			var debugDraw = new b2DebugDraw();
			debugDraw.SetSprite(debugCanvas.getContext('2d'));
			debugDraw.SetDrawScale(scale);
			debugDraw.SetFillAlpha(0.7);
			debugDraw.SetLineThickness(1.0);
			debugDraw.SetFlags(b2DebugDraw.e_shapeBit | b2DebugDraw.e_jointBit);
			world.SetDebugDraw(debugDraw);
			this.update = function() {
				world.m_debugDraw.m_sprite.graphics.clear();
				world.DrawDebugData();
			}
		}
		this.updateDebug = function() {
			if (zot(debug)) return;
			var canvasPosition = getElementPosition(frame.canvas);
			var c = debug.debugCanvas;
			c.style.position = "absolute";
			c.style.zIndex = 2;
			c.style.left = frame.canvas.style.left;
			c.style.top = frame.canvas.style.top;
			c.style.width = frame.canvas.style.width;
			c.style.height = frame.canvas.style.height;
		}
		this.removeDebug = function() {
			if (!debug) return;
			debug.active = false;
			debug.debugCanvas.style.display = "none";
		}

		// keep a remove list and remove in update function
		// at the correct time so world does not get confused
		var removeList = [];
		this.remove = function(body) {
			removeList.push(body);
		}
		function doRemove() {
			var len = removeList.length;
			if (len==0) return;
			var body;
			for (var i=len-1; i>=0; i--) {
				body = removeList[i];
				mappings.remove(body);
				world.DestroyBody(body);
				body = null;
				removeList.pop();
			}
		}

		// Drag wraps the demo example mouse code
		var drag;
		this.drag = function(list) {
			drag = new that.Drag(list);
		}
		this.noDrag = function() {
			drag.removeListeners();
			drag = null;
		}
		this.Drag = function(list) {
			if (zot(frame)) frame = {scale:1};
			if (zot(list)) list = [];
			this.list = list;
			var that = this;

			// modified demo.html code at https://code.google.com/p/box2dweb/
			var canvasPosition, mouseX, mouseY, mousePVec, isMouseDown, selectedBody, mouseJoint;

			document.addEventListener("mousedown", dragEvent, true);
			function dragEvent(e) {
				canvasPosition = getElementPosition(frame.canvas);
				isMouseDown = true;
				handleMouseMove(e);
				document.addEventListener("mousemove", handleMouseMove, true);
			};

			document.addEventListener("mouseup", function() {
				document.removeEventListener("mousemove", handleMouseMove, true);
				isMouseDown = false;
				mouseX = undefined;
				mouseY = undefined;
			}, true)

			function handleMouseMove(e) {
				mouseX = (e.clientX-canvasPosition.x)/frame.scale/scale;
				mouseY = (e.clientY-canvasPosition.y)/frame.scale/scale;
			};

			function getBodyAtMouse() {
				mousePVec = new b2Vec2(mouseX, mouseY);
				var aabb = new b2AABB();
				aabb.lowerBound.Set(mouseX - 0.001, mouseY - 0.001);
				aabb.upperBound.Set(mouseX + 0.001, mouseY + 0.001);

				// Query the world for overlapping shapes.
				selectedBody = null;
				world.QueryAABB(getBodyCB, aabb);
				return selectedBody;
			}

			function getBodyCB(fixture) {
				if(fixture.GetBody().GetType() != b2Body.b2_staticBody) {
					if(fixture.GetShape().TestPoint(fixture.GetBody().GetTransform(), mousePVec)) {
					  selectedBody = fixture.GetBody();
					  return false;
					}
				}
				return true;
			}
			this.update = function() {

				if(isMouseDown && (!mouseJoint)) {
					var body = getBodyAtMouse();
					if(body) {
						if (that.list.length > 0 && that.list.indexOf(body) < 0) return;
						var md = new b2MouseJointDef();
						md.bodyA = world.GetGroundBody();
						md.bodyB = body;
						md.target.Set(mouseX, mouseY);
						md.collideConnected = true;
						md.maxForce = 300.0 * body.GetMass();
						mouseJoint = world.CreateJoint(md);
						body.SetAwake(true);
					}
				}

				if(mouseJoint) {
					if(isMouseDown) {
						mouseJoint.SetTarget(new b2Vec2(mouseX, mouseY));
					} else {
						world.DestroyJoint(mouseJoint);
						mouseJoint = null;
					}
				}
			}
			this.removeListeners = function() {
				document.removeEventListener("mousedown", dragEvent);
			};
		}
		//http://js-tut.aardon.de/js-tut/tutorial/position.html
		function getElementPosition(element) {
			var elem=element, tagname="", x=0, y=0;
			while((typeof(elem) == "object") && (typeof(elem.tagName) != "undefined")) {
				y += elem.offsetTop;
				x += elem.offsetLeft;
				tagname = elem.tagName.toUpperCase();
				if(tagname == "BODY") elem=0;
				if(typeof(elem) == "object") {
					if(typeof(elem.offsetParent) == "object") elem = elem.offsetParent;
				}
			}
			return {x:x-zim.scrollX(), y:y-zim.scrollY()};
		}

		// mappings put zim assets to the x, y and rotation of Box2D bodies
		// a dictionary is used for easy adding and removing
		if (zim) {
			var mappings = new zim.Dictionary();
			function updateMap() {
				for (var i=0; i<mappings.length; i++) {
					var zimObj = mappings.values[i];
					var box2DBody = mappings.objects[i];
					var p = box2DBody.GetWorldPoint(new b2Vec2(0, 0));
					zimObj.x = p.x * scale;
					zimObj.y = p.y * scale;
					zimObj.rotation = box2DBody.GetAngle() * (180 / Math.PI);
				}
				frame.stage.update();
			}
			this.addMap = function(box2DBody, zimObj) {
				if (zot(box2DBody) || zot(zimObj)) {console.log("physics.Map() - please provide a box2DBody and zimObj"); return;}
				mappings.add(box2DBody, zimObj);
			}
			this.removeMap = function(box2DBody) {
				mappings.remove(box2DBody);
			}
		}

		function makeBorders(rect, restitution) {
			if (rect.constructor !== {}.constructor) return;
			var w = 1;	// width of wall
			// Create border of boxes
			var wall = new b2PolygonShape();
			var wallBd = new b2BodyDef();
			var wallB;
			// Left
			wallBd.position.Set((rect.x-w/2)/scale, rect.height/2/scale);
			wall.SetAsBox(w/scale, rect.height/2/scale);
			wallB = that.borderLeft = world.CreateBody(wallBd);
			wallB.CreateFixture2(wall);
			// Right
			wallBd.position.Set((rect.x+rect.width+w/2)/scale, rect.height/2/scale);
			wallB = that.borderRight = world.CreateBody(wallBd);
			wallB.CreateFixture2(wall);
			// Top
			wallBd.position.Set((rect.x + rect.width/2)/scale, (rect.y-w/2)/scale);
			wall.SetAsBox(rect.width/2/scale, w/scale);
			wallB = that.borderTop = world.CreateBody(wallBd);
			wallB.CreateFixture2(wall);
			// Bottom
			wallBd.position.Set((rect.x + rect.width/2)/scale, (rect.y+rect.height+w/2)/scale);
			wallB = that.borderBottom =  world.CreateBody(wallBd);
			wallB.CreateFixture2(wall);
		}
		makeBorders(borders);

		// the Ticker keeps add and remove methods
		// to add and remove functions to the update function
		// either before the world step or after the word step
		// these can be used for adding forces
		var beforeList = new zim.Dictionary();
		var afterList = new zim.Dictionary();
		this.Ticker = {
			add:function(f, after) {
				if (zot(after)) after = true;
				if (after) afterList.add(f, 1);
				else beforeList.add(f, 1);
				return f;
			},
			remove:function(f) {
				afterList.remove(f);
				beforeList.remove(f);
			}
		};

		// update world
		var request;
		function update() {
			request = requestAnimationFrame(update);
			if (drag) drag.update();
			for (var i=0; i<beforeList.objects.length; i++) beforeList.objects[i]();
			world.Step(timeStep, 10, 10); // last two are velocity iterations, position iterations
			doRemove();
			world.ClearForces();
			for (i=0; i<afterList.objects.length; i++) afterList.objects[i]();
			if (debug && debug.active) debug.update();
			updateMap();
		}
		update();

		this.dispose = function() {
			that.removeDebug();
			cancelAnimationFrame(request);
			if (drag) drag.removeListeners();
		}

	};
	return zim;
}(zim || {});
