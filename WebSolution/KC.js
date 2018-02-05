var minRadius;

function buildPathKinematicCar() {
	drawMap();
	ctx.save();
	
	pathInfo = getPathKC_RRT();
	path = pathInfo[0];
	
	firstPoint = translatePoint(map.pos_start);
	ctx.lineWidth = 2;
	ctx.strokeStyle = "#228B22";
	ctx.moveTo(firstPoint[0], firstPoint[1]);
	prev_point = map.pos_start;
	
	for (var i = 0; i < path.length; i++) {
		v = translatePoint(path[i]);
		ctx.lineTo(v[0], v[1]);
		prev_point = path[i];		
	}
	ctx.stroke();
	ctx.font = "15px Arial";
	ctx.restore();
	str1 = "Time: ";
	$("#time_results").text(str1.concat((pathInfo[1]).toFixed(4)));
	
	ctx.restore();
}

function getPathKC_RRT() {
	
	var startTime = Date.now();
	minRadius = map.vehicle_L/Math.tan(map.vehicle_phi_max);
	var NEIGHBORHOOD_RADIUS = 10;
	
	start_node = { state: { p_x: map.pos_start[0], p_y: map.pos_start[1], v_x: map.vel_start[0], v_y: map.vel_start[1] },
		parent_node: null, path_cost: 0, part_path: null };
	var tree = [ start_node ];
	
	final_node = { state: { p_x: map.pos_goal[0], p_y: map.pos_goal[1], v_x: map.vel_goal[0], v_y: map.vel_goal[1] },
		parent_node: null, path_cost: Number.MAX_VALUE, part_path: null };
	
	dt = map.vehicle_dt;
	vehicleL = map.vehicle_L;
	vehiclePhi = map.vehicle_phi_max;
	
	for (var iter = 0; iter < 100000; iter++) {
		
		if (final_node.parent_node != null && (Date.now() - startTime) > max_comp_time_sec*1000) {
			break;
		}
		
		if (iter % 1000 == 0) {
			console.log(iter);
			//console.log(threshold);
		}
		
		var q_rand = {p_x: 0, p_y: 0, v_x: 0, v_y: 0};
		
		// Sample a random free point
		var p;
		freePoint = false;
		while (!freePoint) {
			
			p = [(Math.random()*(max_x-min_x)+min_x), (Math.random()*(max_y-min_y)+min_y)];
			
			if (!checkInside(p, map.bounding_polygon)) {
				continue;
			}
			
			freePoint = true;
			for (var i = 0; i < obstacles.length; i++) {
				if (checkInside(p, obstacles[i])) {
					freePoint = false;
					break;
				}
			}
		}
		
		var vel;
		while (true) {
			vel = [ Math.random()*2*map.vehicle_v_max-(map.vehicle_v_max), Math.random()*2*map.vehicle_v_max-(map.vehicle_v_max) ];
			if (Math.sqrt(vel[0]*vel[0] + vel[1]*vel[1]) <= map.vehicle_v_max) {
				break;
			}
		}
		
		q_rand.p_x = p[0];
		q_rand.p_y = p[1];
		q_rand.v_x = vel[0];
		q_rand.v_y = vel[1];
		
		neighbors = [];
		best_neighbor = null;
		best_part_path = null;
		best_cost = Number.MAX_VALUE;
		for (var i = 0; i < tree.length; i++) {
			node = tree[i];
			
			partPath = getPartPath(node.state, q_rand, minRadius);
			
			// Check neighborhood
			if (partPath != null && partPath.cost < NEIGHBORHOOD_RADIUS) {
				
				if (!checkFisibilityCar(partPath.path)) {
					continue;
				}
				
				console.log("feasible neighbor");
				
				neighbors.push(node);
				if ((partPath.cost) < best_cost) {
					best_cost = partPath.cost;
					best_neighbor = node;
					best_part_path = partPath;
				}
			}
		}
		
		if (best_neighbor == null) {
			continue;
		}
		
		new_node = { state: q_rand, parent_node: best_neighbor, path_cost: best_cost+best_neighbor.path_cost, part_path: best_part_path };
		
		tree.push(new_node);
		
		// final_node
		partPath = getPartPath(q_rand, final_node.state, minRadius);
		if (partPath != null) {
			if (best_cost + partPath.cost < final_node.path_cost) {
				if (checkFisibilityCar(partPath.path)) {
					final_node.parent_node = new_node;
					final_node.path_cost = best_cost + partPath.cost;
					final_node.part_path = partPath;
					console.log("reached finish");
					break;
				}
			}
		}
	}
	
	path = undefined;
	if (final_node.parent_node != null) {
		path = [];
		end_node = final_node;
		while (end_node.parent_node != null) {
			for (var i = end_node.part_path.path.length-1; i >= 0; i--) {
				path.unshift(end_node.part_path.path[i]);
			}
			end_node = end_node.parent_node;
		}
	}
	
	return [path, final_node.path_cost];
}

function getPartPath(startState, finalState, radius) {
	
	dt = map.vehicle_dt;
	p_x0 = startState.p_x;
	p_y0 = startState.p_y;
	v_x0 = startState.v_x;
	v_y0 = startState.v_y;
	p_x1 = finalState.p_x;
	p_y1 = finalState.p_y;
	v_x1 = finalState.v_x;
	v_y1 = finalState.v_y;
	
	a_y = v_y0;
	a_x = v_x0;
	b_y = v_y1;
	b_x = v_x1;
	
	a = v_y0/v_x0;
	b = v_y1/v_x1;
	c = (v_x0*p_y0-v_y0*p_x0)/v_x0;
	d = (v_x1*p_y1-v_y1*p_x1)/v_x1;
	
	p = [(d-c)/(a-b), (a*d-b*c)/(a-b)];
	
	dirCorr0 = (Math.sign(v_x0) == Math.sign(p[0]-p_x0));
	dirCorr1 = (Math.sign(v_x1) == Math.sign(p[0]-p_x1));
	
	if (dirCorr0 && dirCorr1 || !dirCorr0 && !dirCorr1) {
		// Cannot reach with one turn
		return null;
	}
	
	travelTime = 0;
	partPath = [];
	partPath.push([p_x0, p_y0]);
	
	if (dirCorr0) {
		//signedAngle = Math.atan2(b_y, b_x) - Math.atan2(a_y, a_x);
		
		var angle1 = Math.atan2(b_y, b_x);
		while (angle1 < Math.atan2(a_y, a_x)) {
			angle1 += (2*Math.PI);
		}
		var angle2 = Math.atan2(a_y,a_x);
		while (angle2 < Math.atan2(b_y,b_x)) {
			angle2 += (2*Math.PI);
		}
		
		diff1 = (angle1-Math.atan2(a_y, a_x));
		diff2 = (angle2-Math.atan2(b_y,b_x));
		if (diff2 < diff1) {
			// Turn right
			
			angle = diff2;
			if (angle < Math.PI/2) {
				// Slight turn
				alpha = (Math.PI-angle)/2;
				//segmentLength = radius*Math.cos(alpha);
				segmentLength = radius/Math.tan(alpha);
				
				vec1Angle = Math.atan2(a_y, a_x);
				
				startTurnPoint = [p[0]-(segmentLength*Math.cos(vec1Angle)), p[1]-segmentLength*Math.sin(vec1Angle)];
				
				// Check that the turn is ahead of the start point
				if (Math.sign(v_x0) == Math.sign(startTurnPoint[0]-p_x0)) {
					
					vec2Angle = Math.atan2(b_y,b_x);
					endTurnPoint = [p[0]+(segmentLength*Math.cos(vec2Angle)), p[1]+segmentLength*Math.sin(vec2Angle)];
					
					// Check that the turn is ahead of the destination
					if (Math.sign(v_x1) == Math.sign(p_x1-endTurnPoint[0])) {
						//partPath.push(startTurnPoint);
						travelTime += Math.sqrt((startTurnPoint[0]-p_x0)*(startTurnPoint[0]-p_x0) +
							(startTurnPoint[1]-p_y0)*(startTurnPoint[1]-p_y0))/map.vehicle_v_max;
						
						// add the tangent
						rCenter = [startTurnPoint[0] + radius*Math.sin(vec1Angle), startTurnPoint[1] - radius*Math.cos(vec1Angle)];
						tangLength = ((Math.PI*radius*((Math.PI/2) - alpha)*2)/Math.PI);
						phi_curr = Math.atan2(startTurnPoint[1]-rCenter[1], startTurnPoint[0]-rCenter[0]);
						dl = map.vehicle_v_max*dt;
						for (var l = 0; l < tangLength; l += dl) {
							partPath.push([ rCenter[0]+(radius*Math.cos(phi_curr)), rCenter[1] + (radius*Math.sin(phi_curr)) ]);
							phi_curr -= (dl/radius);
						}
						
						partPath.push(endTurnPoint);
						travelTime += tangLength/map.vehicle_v_max; // the tangent traveling time
						partPath.push([p_x1, p_y1]);
						travelTime += Math.sqrt((endTurnPoint[0]-p_x1)*(endTurnPoint[0]-p_x1) +
							(endTurnPoint[1]-p_y1)*(endTurnPoint[1]-p_y1))/map.vehicle_v_max;
						
						return { path: partPath, cost: travelTime};
					}
					
				}
			} else {
				// Sharp turn
				alpha = (Math.PI-angle)/2;
				segmentLength = radius/Math.tan(alpha);
				vec1Angle = Math.atan2(a_y, a_x);
				startTurnPoint = [p[0]-(segmentLength*Math.cos(vec1Angle)), p[1]-segmentLength*Math.sin(vec1Angle)];
				if (Math.sign(v_x0) == Math.sign(startTurnPoint[0]-p_x0)) {
					vec2Angle = Math.atan2(b_y,b_x);
					endTurnPoint = [p[0]+(segmentLength*Math.cos(vec2Angle)), p[1]+segmentLength*Math.sin(vec2Angle)];
					if (Math.sign(v_x1) == Math.sign(p_x1-endTurnPoint[0])) {
						//partPath.push(startTurnPoint);
						travelTime += Math.sqrt((startTurnPoint[0]-p_x0)*(startTurnPoint[0]-p_x0) +
							(startTurnPoint[1]-p_y0)*(startTurnPoint[1]-p_y0))/map.vehicle_v_max;
						
						// add the tangent
						rCenter = [startTurnPoint[0] + radius*Math.sin(vec1Angle), startTurnPoint[1] - radius*Math.cos(vec1Angle)];
						tangLength = ((Math.PI*radius*((Math.PI/2) - alpha)*2)/Math.PI);
						phi_curr = Math.atan2(startTurnPoint[1]-rCenter[1], startTurnPoint[0]-rCenter[0]);
						dl = map.vehicle_v_max*dt;
						for (var l = 0; l < tangLength; l += dl) {
							partPath.push([ rCenter[0]+(radius*Math.cos(phi_curr)), rCenter[1] + (radius*Math.sin(phi_curr)) ]);
							phi_curr -= (dl/radius);
						}
						
						partPath.push(endTurnPoint);
						travelTime += ((Math.PI*radius*((Math.PI/2) - alpha)*2)/Math.PI)/map.vehicle_v_max; // the tangent traveling time
						partPath.push([p_x1, p_y1]);
						travelTime += Math.sqrt((endTurnPoint[0]-p_x1)*(endTurnPoint[0]-p_x1) +
							(endTurnPoint[1]-p_y1)*(endTurnPoint[1]-p_y1))/map.vehicle_v_max;
						
						return { path: partPath, cost: travelTime};
					}
				}
				
			}
		} else {
			
			// Turn left
			
			angle = diff1;
			if (angle < Math.PI/2) {
				// Slight turn
				alpha = (Math.PI-angle)/2;
				//segmentLength = radius*Math.cos(alpha);
				segmentLength = radius/Math.tan(alpha);
				
				vec1Angle = Math.atan2(a_y, a_x);
				
				startTurnPoint = [p[0]-(segmentLength*Math.cos(vec1Angle)), p[1]-segmentLength*Math.sin(vec1Angle)];
				
				// Check that the turn is ahead of the start point
				if (Math.sign(v_x0) == Math.sign(startTurnPoint[0]-p_x0)) {
					
					vec2Angle = Math.atan2(b_y,b_x);
					endTurnPoint = [p[0]+(segmentLength*Math.cos(vec2Angle)), p[1]+segmentLength*Math.sin(vec2Angle)];
					
					// Check that the turn is ahead of the destination
					if (Math.sign(v_x1) == Math.sign(p_x1-endTurnPoint[0])) {
						//partPath.push(startTurnPoint);
						travelTime += Math.sqrt((startTurnPoint[0]-p_x0)*(startTurnPoint[0]-p_x0) +
							(startTurnPoint[1]-p_y0)*(startTurnPoint[1]-p_y0))/map.vehicle_v_max;
						
						// add the tangent
						rCenter = [startTurnPoint[0] - radius*Math.sin(vec1Angle), startTurnPoint[1] + radius*Math.cos(vec1Angle)];
						tangLength = ((Math.PI*radius*((Math.PI/2) - alpha)*2)/Math.PI);
						phi_curr = Math.atan2(startTurnPoint[1]-rCenter[1], startTurnPoint[0]-rCenter[0]);
						dl = map.vehicle_v_max*dt;
						for (var l = 0; l < tangLength; l += dl) {
							partPath.push([ rCenter[0]+(radius*Math.cos(phi_curr)), rCenter[1] + (radius*Math.sin(phi_curr)) ]);
							phi_curr += (dl/radius);
						}
						
						partPath.push(endTurnPoint);
						travelTime += ((Math.PI*radius*((Math.PI/2) - alpha)*2)/Math.PI)/map.vehicle_v_max; // the tangent traveling time
						partPath.push([p_x1, p_y1]);
						travelTime += Math.sqrt((endTurnPoint[0]-p_x1)*(endTurnPoint[0]-p_x1) +
							(endTurnPoint[1]-p_y1)*(endTurnPoint[1]-p_y1))/map.vehicle_v_max;
						return { path: partPath, cost: travelTime};
					}
					
				}
			} else {
				alpha = (Math.PI-angle)/2;
				segmentLength = radius/Math.tan(alpha);
				vec1Angle = Math.atan2(a_y, a_x);
				startTurnPoint = [p[0]-(segmentLength*Math.cos(vec1Angle)), p[1]-segmentLength*Math.sin(vec1Angle)];
				if (Math.sign(v_x0) == Math.sign(startTurnPoint[0]-p_x0)) {
					vec2Angle = Math.atan2(b_y,b_x);
					endTurnPoint = [p[0]+(segmentLength*Math.cos(vec2Angle)), p[1]+segmentLength*Math.sin(vec2Angle)];
					if (Math.sign(v_x1) == Math.sign(p_x1-endTurnPoint[0])) {
						//partPath.push(startTurnPoint);
						travelTime += Math.sqrt((startTurnPoint[0]-p_x0)*(startTurnPoint[0]-p_x0) +
							(startTurnPoint[1]-p_y0)*(startTurnPoint[1]-p_y0))/map.vehicle_v_max;
						
						// add the tangent
						rCenter = [startTurnPoint[0] - radius*Math.sin(vec1Angle), startTurnPoint[1] + radius*Math.cos(vec1Angle)];
						tangLength = ((Math.PI*radius*((Math.PI/2) - alpha)*2)/Math.PI);
						phi_curr = Math.atan2(startTurnPoint[1]-rCenter[1], startTurnPoint[0]-rCenter[0]);
						dl = map.vehicle_v_max*dt;
						for (var l = 0; l < tangLength; l += dl) {
							partPath.push([ rCenter[0]+(radius*Math.cos(phi_curr)), rCenter[1] + (radius*Math.sin(phi_curr)) ]);
							phi_curr += (dl/radius);
						}
						
						partPath.push(endTurnPoint);
						travelTime += ((Math.PI*radius*((Math.PI/2) - alpha)*2)/Math.PI)/map.vehicle_v_max; // the tangent traveling time
						partPath.push([p_x1, p_y1]);
						travelTime += Math.sqrt((endTurnPoint[0]-p_x1)*(endTurnPoint[0]-p_x1) +
							(endTurnPoint[1]-p_y1)*(endTurnPoint[1]-p_y1))/map.vehicle_v_max;
						
						return { path: partPath, cost: travelTime};
					}
				}
			}
		}
	}
	return null;
}

function checkFisibilityCar(path) {
	p_prev_x = path[0][0];
	p_prev_y = path[0][1];
	
	for (var i = 1; i < path.length; i++) {
		p_x_curr = path[i][0];
		p_y_curr = path[i][1];
		
		for (var k = 0; k < obst_edges.length; k++) {
			if (segmentsIntersect(p_x_curr, p_y_curr, p_prev_x, p_prev_y, obst_edges[k][0][0], obst_edges[k][0][1], obst_edges[k][1][0], obst_edges[k][1][1])) {
				return false;
			}
		}
		
		p_prev_x = p_x_curr;
		p_prev_y = p_y_curr;
	}
	
	return true;
}