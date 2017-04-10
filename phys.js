
var phys = function(phys) {
	
	function not(v) {
		if (v === null) return true;
		return typeof v === "undefined";
	}
	
	phys.makeBox = function(width, height, world, scale, dynamic, friction, density, angular) {	
	
		if (not(width)) width = 100;
		if (not(height)) height = 100;
		if (not(world) || not(scale)) {console.log("phys.Mouse() - please provide a world and scale"); return;}
		if (not(dynamic)) dynamic = true;
		if (not(friction)) friction = 1;
		if (not(density)) density = 1;
		if (not(angular)) angular = .5; // rotational damping
		var definition = new b2BodyDef();
		if (dynamic) {
			definition.type = b2Body.b2_dynamicBody;
		} else {
			definition.type = b2Body.b2_staticBody;
		}		
		definition.angularDamping = angular;
		var body = world.CreateBody(definition);	
		var shape = new b2PolygonShape();
		shape.SetAsBox(width/2, height/2);
		var fixture = new b2FixtureDef();
		fixture.shape = shape;
		fixture.density = density;
		fixture.friction = friction;
		body.CreateFixture(fixture);
		
		// ******************
		// Not making a class that extends a Box2D body 
		// because Box2D uses functions like world.CreateBody() so can't extend - sigh
		// but this function does add x,y,rotation properties to the body it returns
		// it can remember these because of JavaScript closure (or lack of closure)

		var _x = 0;
		var _y = 0;
		var _rotation = 0;
		
		Object.defineProperty(body, 'x', {
			get: function() {			
				return _x;
			},
			set: function(x) {
				_x = x;
				body.SetPosition(new b2Vec2(_x, _y));
			}
		});
		Object.defineProperty(body, 'y', {
			get: function() {			
				return _y;
			},
			set: function(y) {
				_y = y;
				body.SetPosition(new b2Vec2(_x, _y));
			}
		});
		Object.defineProperty(body, 'rotation', {
			get: function() {			
				return _rotation;
			},
			set: function(rotation) {
				_rotation = rotation;
				body.SetAngle(rotation*Math.PI/180);				
			}
		});
			
		return body; 
	}
		
	
	// Drag wraps the demo example mouse code
	phys.Drag = function(canvas, world, scale, list) {
		
		if (not(canvas) || not(world) || not(scale)) {console.log("phys.Mouse() - please provide a canvas, world and scale"); return;}
		if (not(list)) list = [];
		this.list = list;
		var that = this;
		
		// modified demo.html code at https://code.google.com/p/box2dweb/
		var mouseX, mouseY, mousePVec, isMouseDown, selectedBody, mouseJoint;
		var canvasPosition = getElementPosition(canvas);
		
		document.addEventListener("mousedown", function(e) {
			isMouseDown = true;
			handleMouseMove(e);
			document.addEventListener("mousemove", handleMouseMove, true);
		}, true)
		
		document.addEventListener("mouseup", function() {
			document.removeEventListener("mousemove", handleMouseMove, true);
			isMouseDown = false;
			mouseX = undefined;
			mouseY = undefined;
		}, true)
		
		function handleMouseMove(e) {
			mouseX = (e.clientX - canvasPosition.x) / scale;
			mouseY = (e.clientY - canvasPosition.y) / scale;
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
			
			return {x: x, y: y};
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
	}
	
	
	phys.Map = function(box2DBody, createjsObj, name, scale) {
		
		if (not(box2DBody) || not(createjsObj) || not(name) || not(scale)) {console.log("phys.Map() - please provide a box2DBody, createjsObj, name and scale"); return;}
	
		this.name = name;		
		this.update = function() {			
			createjsObj.x = box2DBody.GetWorldCenter().x * scale;
			createjsObj.y = box2DBody.GetWorldCenter().y * scale;
			createjsObj.rotation = box2DBody.GetAngle() * (180 / Math.PI);
		}		
	}
	
	phys.MapManager = function() {
		var maps = [];
		var names = [];
		this.add = function(map) {
			maps.push(map);
			names.push(map.name);
		}
		this.remove = function(map) {
			var index = names.indexOf(map.name);
			if (index >= 0) {
				maps.splice(index,1);
				names.splice(index,1);
			}
		}
		this.update = function() {
			var map;
			for (var i=0; i<maps.length; i++) {
				map = maps[i];
				map.update();
			}
		}		
	}
	
	phys.Debug = function(canvas, world, scale) {
		
		if (not(canvas) || not(world) || not(scale)) {console.log("phys.Debug() - please provide a canvas, world and scale"); return;}
			
		var debugDraw = new b2DebugDraw();
		debugDraw.SetSprite(canvas.getContext('2d'));
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
	
	phys.borders = function(rect, world, scale) {
		
		if (not(rect) || not(world) || not(scale)) {console.log("phys.borders() - please provide a rect, world and scale"); return;}
						
		var w = 1;	// width of wall		
		
		// Create border of boxes
		var wall = new b2PolygonShape();
		var wallBd = new b2BodyDef();
		var wallB;
		
		// Left
		wallBd.position.Set((rect.x-w/2)/scale, rect.height/2/scale);
		wall.SetAsBox(w/scale, rect.height/scale);
		wallB = world.CreateBody(wallBd);
		wallB.CreateFixture2(wall);
		
		// Right
		wallBd.position.Set((rect.x+rect.width+w/2)/scale, rect.height/2/scale);
		wallB = world.CreateBody(wallBd);
		wallB.CreateFixture2(wall);
		
		// Top
		wallBd.position.Set((rect.x + rect.width/2)/scale, (rect.y-w/2)/scale);
		wall.SetAsBox(rect.width/scale, w/scale);
		wallB = world.CreateBody(wallBd);
		wallB.CreateFixture2(wall);
		
		// Bottom
		wallBd.position.Set((rect.x + rect.width/2)/scale, (rect.y+rect.height+w/2)/scale);
		wallB = world.CreateBody(wallBd);
		wallB.CreateFixture2(wall);
		
		
	}
	
	return phys;	
}(phys || {});