var r = 1;
var N = 1000;

function buildPathDynamicRRT() {
	drawMap();
	ctx.save();
	
	// map.pos_start, map.pos_goal, map.vel_start, map.vel_goal, map.vehicle_v_max, map.vehicle_a_max, map.vehicle_dt
	path = getPathDynamicRRT();
	//path = pathInfo;
	
	prev_state = { p_x: map.pos_start[0], p_y: map.pos_start[1], v_x: map.vel_start[0], v_y: map.vel_start[1] };
	for (var i = 0; i < path.length; i++) {
		tau = getOptimalTau(path[i].state, prev_state);
		drawPartPath(path[i].state, prev_state, tau);
		prev_state = path[i].state;
	}
	ctx.stroke();
	ctx.font = "15px Arial";
	ctx.restore();
	str1 = "Time: ";
	$("#time_results").text(str1.concat((path[path.length-1].path_cost).toFixed(4)));
}

function getPathDynamicRRT(N = 3577) {
	
	var NEIGHBORHOOD_RADIUS = 200;
	
	mu_x_free = (max_x-min_x)*(max_y-min_y)*(map.vehicle_v_max*map.vehicle_v_max*0.5);
	gamma = NEIGHBORHOOD_RADIUS*Math.pow(2,4)*(1 + 1/4)*mu_x_free;
	unit_sphere = 0.5*Math.PI*Math.PI;
	
	start_node = { state: { p_x: map.pos_start[0], p_y: map.pos_start[1], v_x: map.vel_start[0], v_y: map.vel_start[1] },
		parent_node: null, path_cost: 0 };
	var tree = [ start_node ];
	
	final_node = { state: { p_x: map.pos_goal[0], p_y: map.pos_goal[1], v_x: map.vel_goal[0], v_y: map.vel_goal[1] },
		parent_node: null, path_cost: Number.MAX_VALUE };
	
	dt = map.vehicle_dt;
	
	for (var iter = 0; iter < N; iter++) {
		
		threshold = Math.pow((gamma/unit_sphere)*Math.log(tree.length+1)/(tree.length+1), 1/4);
		
		if (iter % 100 == 0) {
			console.log(iter);
			console.log(threshold);
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
			vel = [ Math.random()*map.vehicle_v_max, Math.random()*map.vehicle_v_max ];
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
		best_cost = Number.MAX_VALUE;
		for (var i = 0; i < tree.length; i++) {
			node = tree[i];
			cost = getOptimalTau(q_rand, node.state, 1.5*threshold);
			if (isNaN(cost) || cost <= 0) {
				continue;
			}
			
			// Check neighborhood
			if (cost < threshold) {
				tau = cost;
				if (!checkFisibility(q_rand, node.state, tau)) {
					continue;
				}
				
				//console.log("feasible neighbor");
				
				neighbors.push(node);
				if ((cost + node.path_cost) < best_cost) {
					best_cost = cost + node.path_cost;
					best_neighbor = node;
				}
			}
		}
		
		if (best_neighbor == null) {
			continue;
		}
		
		new_node = { state: q_rand, parent_node: best_neighbor, path_cost: best_cost };
		tree.push(new_node);
		
		for (var i = 0; i < neighbors.length; i++) {
			
			cost = getOptimalTau(neighbors[i].state, q_rand, 1.5*threshold);
			if (!isNaN(cost) && cost > 0) {
				if (best_cost + cost < neighbors[i].path_cost) {
					tau = cost;
					if (checkFisibility(neighbors[i].state, q_rand, tau)) {
						neighbors[i].parent_node = new_node;
						neighbors[i].path_cost = best_cost + cost;
					}
				}
			}
		}
		
		// final_node
		cost = getOptimalTau(final_node.state, q_rand, 1.5*threshold);
		if (!isNaN(cost) && cost > 0) {
			if (best_cost + cost < final_node.path_cost) {
				tau = cost;
				if (checkFisibility(final_node.state, q_rand, tau)) {
					final_node.parent_node = new_node;
					final_node.path_cost = best_cost + cost;
					console.log("reached finish");
				}
			}
		}
	}
	
	path = undefined;
	if (final_node.parent_node != null) {
		path = [];
		end_node = final_node;
		while (end_node.parent_node != null) {
			path.unshift(end_node);
			end_node = end_node.parent_node;
		}
	}
	
	return path;
}

function getOptimalTau(finalState, startState, t0 = 30) {
	p_x0 = startState.p_x;
	p_y0 = startState.p_y;
	v_x0 = startState.v_x;
	v_y0 = startState.v_y;
	
	p_x1 = finalState.p_x;
	p_y1 = finalState.p_y;
	v_x1 = finalState.v_x;
	v_y1 = finalState.v_y;
	
	a = ((24*r*v_x1*v_x0)+(12*r*v_x1*(v_x1-v_x0))-(4*r*Math.pow(2*(v_x1-v_x0)-3*v_x0,2))-(4*r*Math.pow(2*(v_y1-v_y0)-3*v_y0,2)));
	b = (-24*r*v_x1*(p_x1-p_x0) + 24*r*(2*(v_x1-v_x0) - 3*v_x0)*(p_x1-p_x0) + 24*r*(2*(v_y1-v_y0) - 3*v_y0)*(p_y1-p_y0));
	c = -36*r*((p_x1-p_x0)*(p_x1-p_x0) + (p_y1-p_y0)*(p_y1-p_y0));
	
	return Newton([c, b, a, 0, 1], eval, t0);
}

function eval(a, t) {
    // f(x) = a0+ a1x + ... + anxn
    var n = a.length - 1;
    var b = [];
    var c = [];
    var i, k;
    for (i = 0; i <= n; i++)
        b.push(0), c.push(0);

    b[n] = a[n];
    c[n] = b[n];
    for (k = n-1; k >= 1; k--) {
        b[k] = a[k] + t*b[k+1];
        c[k] = b[k] + t*c[k+1];
    }
    b[0] = a[0] + t*b[1];

    return [b[0],c[1]];
}

// Simple Newton
function Newton(coeff, eval, x0) {
    var imax = 10;
    for (var i = 0; i < imax; i++) {
        var fdf = eval(coeff, x0);
        x1 = x0 - fdf[0]/fdf[1];
        x0 = x1;
    }
    return x1;
}

function checkFisibility(finalState, startState, tau) {
	dt = map.vehicle_dt;
	p_x0 = startState.p_x;
	p_y0 = startState.p_y;
	v_x0 = startState.v_x;
	v_y0 = startState.v_y;
	p_x1 = finalState.p_x;
	p_y1 = finalState.p_y;
	v_x1 = finalState.v_x;
	v_y1 = finalState.v_y;
	
	d1Tau = 2*r*(6*(p_x1-p_x0-v_x0*tau)/(tau*tau*tau) - 3*(v_x1-v_x0)/(tau*tau));
	d2Tau = 2*r*(6*(p_y1-p_y0-v_y0*tau)/(tau*tau*tau) - 3*(v_y1-v_y0)/(tau*tau));
	d3Tau = 2*r*((2/tau)*(v_x1-v_x0) - 3*(p_x1-p_x0-v_x0*tau)/(tau*tau));
	d4Tau = 2*r*((2/tau)*(v_y1-v_y0) - 3*(p_y1-p_y0-v_y0*tau)/(tau*tau));
	
	a_x_curr = (d3Tau-(1e-7-tau)*d1Tau)/r;
	a_y_curr = (d4Tau-(1e-7-tau)*d2Tau)/r;
	
	if (Math.sqrt(a_x_curr*a_x_curr + a_y_curr*a_y_curr) > map.vehicle_a_max) {
		return false;
	}
	
	p_prev_x = p_x0;
	p_prev_y = p_y0;
	
	v_prev_x = v_x0;
	v_prev_y = v_y0;
	
	for (var t = dt; t < tau; t+=dt) {
		p_x_curr = p_x1 + v_x1*(t-tau) + Math.pow(t-tau,2)*d3Tau/(2*r) - t*d1Tau*(t*t/3 - t*tau + tau*tau)/(2*r) + Math.pow(tau,3)*d1Tau/(6*r);
		p_y_curr = p_y1 + v_y1*(t-tau) + Math.pow(t-tau,2)*d4Tau/(2*r) - t*d2Tau*(t*t/3 - t*tau + tau*tau)/(2*r) + Math.pow(tau,3)*d2Tau/(6*r);
		
		for (var k = 0; k < obst_edges.length; k++) {
			if (segmentsIntersect(p_x_curr, p_y_curr,p_prev_x, p_prev_y, obst_edges[k][0][0], obst_edges[k][0][1], obst_edges[k][1][0], obst_edges[k][1][1])) {
				return false;
			}
		}
		
		v_x_curr = v_x1 - Math.pow(t-tau,2)*d1Tau/(2*r) + (t-tau)*(d3Tau)/r;
		v_y_curr = v_y1 - Math.pow(t-tau,2)*d2Tau/(2*r) + (t-tau)*(d4Tau)/r;
		if (Math.sqrt(v_x_curr*v_x_curr + v_y_curr*v_y_curr) > map.vehicle_v_max) {
			return false;
		}
		
		if (Math.sqrt((v_x_curr-v_prev_x)*(v_x_curr-v_prev_x) + (v_y_curr-v_prev_y)*(v_y_curr-v_prev_y))/dt > map.vehicle_a_max) {
			return false;
		}
		
		a_x_curr = (d3Tau-(t-tau)*d1Tau)/r;
		a_y_curr = (d4Tau-(t-tau)*d2Tau)/r;
		
		if (Math.sqrt(a_x_curr*a_x_curr + a_y_curr*a_y_curr) > map.vehicle_a_max) {
			return false;
		}
		
		p_prev_x = p_x_curr;
		p_prev_y = p_y_curr;
		v_prev_x = v_x_curr;
		v_prev_y = v_y_curr;
	}
	
	return true;
}

function drawPartPath(finalState, startState, tau) {
	dt = map.vehicle_dt;
	p_x0 = startState.p_x;
	p_y0 = startState.p_y;
	v_x0 = startState.v_x;
	v_y0 = startState.v_y;
	p_x1 = finalState.p_x;
	p_y1 = finalState.p_y;
	v_x1 = finalState.v_x;
	v_y1 = finalState.v_y;
	
	d1Tau = 2*r*(6*(p_x1-p_x0-v_x0*tau)/(tau*tau*tau) - 3*(v_x1-v_x0)/(tau*tau));
	d2Tau = 2*r*(6*(p_y1-p_y0-v_y0*tau)/(tau*tau*tau) - 3*(v_y1-v_y0)/(tau*tau));
	d3Tau = 2*r*((2/tau)*(v_x1-v_x0) - 3*(p_x1-p_x0-v_x0*tau)/(tau*tau));
	d4Tau = 2*r*((2/tau)*(v_y1-v_y0) - 3*(p_y1-p_y0-v_y0*tau)/(tau*tau));
	
	a_x_curr = (d3Tau-(1e-7-tau)*d1Tau)/r;
	a_y_curr = (d4Tau-(1e-7-tau)*d2Tau)/r;
	
	p_prev_x = p_x0;
	p_prev_y = p_y0;
	
	v_prev_x = v_x0;
	v_prev_y = v_y0;
	
	ctx.beginPath();
	vertNorm = translatePoint([p_prev_x, p_prev_y])
	ctx.moveTo(vertNorm[0], vertNorm[1]);
	
	for (var t = dt; t < tau; t+=dt) {
		p_x_curr = p_x1 + v_x1*(t-tau) + Math.pow(t-tau,2)*d3Tau/(2*r) - t*d1Tau*(t*t/3 - t*tau + tau*tau)/(2*r) + Math.pow(tau,3)*d1Tau/(6*r);
		p_y_curr = p_y1 + v_y1*(t-tau) + Math.pow(t-tau,2)*d4Tau/(2*r) - t*d2Tau*(t*t/3 - t*tau + tau*tau)/(2*r) + Math.pow(tau,3)*d2Tau/(6*r);
		
		v_x_curr = v_x1 - Math.pow(t-tau,2)*d1Tau/(2*r) + (t-tau)*(d3Tau)/r;
		v_y_curr = v_y1 - Math.pow(t-tau,2)*d2Tau/(2*r) + (t-tau)*(d4Tau)/r;
		
		a_x_curr = (d3Tau-(t-tau)*d1Tau)/r;
		a_y_curr = (d4Tau-(t-tau)*d2Tau)/r;
		
		p = translatePoint([p_x_curr, p_y_curr]);
		ctx.lineTo(p[0], p[1]);
		
		p_prev_x = p_x_curr;
		p_prev_y = p_y_curr;
		v_prev_x = v_x_curr;
		v_prev_y = v_y_curr;
	}
	
	ctx.stroke();
}