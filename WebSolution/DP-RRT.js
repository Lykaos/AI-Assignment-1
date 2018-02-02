var r = 1;
var N = 3000;
var sigma1 = 1;
var sigma2 = 0.5;

async function buildPathDynamicRRT() {
	drawMap();
	ctx.save();
	
	// map.pos_start, map.pos_goal, map.vel_start, map.vel_goal, map.vehicle_v_max, map.vehicle_a_max, map.vehicle_dt
	path = getPathDynamicRRT();
	//path = pathInfo;
	
	prev_state = { p_x: map.pos_start[0], p_y: map.pos_start[1], v_x: map.vel_start[0], v_y: map.vel_start[1] };
	for (var i = 0; i < path.length; i++) {
		//tau = getOptimalTau(path[i].state, prev_state,);
		tau = path[i].tau;
		await drawPartPath(path[i].state, prev_state, tau);
		prev_state = path[i].state;
	}
	ctx.stroke();
	ctx.font = "15px Arial";
	ctx.restore();
	str1 = "Time: ";
	$("#time_results").text(str1.concat((path[path.length-1].path_cost).toFixed(4)));
}

function getPathDynamicRRT() {
	
	var NEIGHBORHOOD_RADIUS = 200;
	
	mu_x_free = (max_x-min_x)*(max_y-min_y)*(map.vehicle_v_max*map.vehicle_v_max*0.5);
	gamma = NEIGHBORHOOD_RADIUS*Math.pow(2,4)*(1 + 1/4)*mu_x_free;
	unit_sphere = 0.5*Math.PI*Math.PI;
	
	start_node = { state: { p_x: map.pos_start[0], p_y: map.pos_start[1], v_x: map.vel_start[0], v_y: map.vel_start[1] },
		parent_node: null, path_cost: 0, depth: 0, tau: 0 };
	var tree = [ start_node ];
	
	final_node = { state: { p_x: map.pos_goal[0], p_y: map.pos_goal[1], v_x: map.vel_goal[0], v_y: map.vel_goal[1] },
		parent_node: null, path_cost: Number.MAX_VALUE, depth: 0, tau: 0 };
	
	dt = map.vehicle_dt;
	
	for (var iter = 0; iter < N; iter++) {
		
		threshold = Math.pow((gamma/unit_sphere)*Math.log(tree.length+1)/(tree.length+1), 1/4);
		
		if (iter % 100 == 0) {
			console.log(iter);
			//console.log(threshold);
		}
		
		var q_rand = {p_x: 0, p_y: 0, v_x: 0, v_y: 0};
		
		// Sample a random free point
		var p;
		freePoint = false;
		if (final_node.parent_node == null) {
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
		} else {
			// final_node.depth
			index = Math.round(Math.random()*final_node.depth);
			node = final_node;
			for (var i = 0; i < index; i++) {
				if (node.parent_node == null) {
					break;
				}
				node = node.parent_node;
			}
			
			while (!freePoint) {
				
				p = [ node.state.p_x+sampleGaussian(0,sigma1), node.state.p_y+sampleGaussian(0,sigma1) ];
				
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
				vel = [ node.state.v_x+sampleGaussian(0,sigma2), node.state.v_y+sampleGaussian(0,sigma2) ];
				if (Math.sqrt(vel[0]*vel[0] + vel[1]*vel[1]) <= map.vehicle_v_max) {
					break;
				}
			}
		}
		
		q_rand.p_x = p[0];
		q_rand.p_y = p[1];
		q_rand.v_x = vel[0];
		q_rand.v_y = vel[1];
		
		neighbors = [];
		best_neighbor = null;
		best_cost = Number.MAX_VALUE;
		best_tau = 0;
		for (var i = 0; i < tree.length; i++) {
			node = tree[i];
			cost = getOptimalTau(q_rand, node.state, 1.5*threshold);
			if (isNaN(cost) || cost <= dt) {
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
					best_tau = cost;
				}
			}
		}
		
		if (best_neighbor == null) {
			continue;
		}
		
		new_node = { state: q_rand, parent_node: best_neighbor, path_cost: best_cost, depth: (best_neighbor.depth+1),
			tau: best_tau };
		tree.push(new_node);
		
		for (var i = 0; i < neighbors.length; i++) {
			
			cost = getOptimalTau(neighbors[i].state, q_rand, 1.5*threshold);
			if (!isNaN(cost) && cost > dt) {
				if (best_cost + cost < neighbors[i].path_cost) {
					tau = cost;
					if (checkFisibility(neighbors[i].state, q_rand, tau)) {
						neighbors[i].parent_node = new_node;
						neighbors[i].path_cost = best_cost + cost;
						oldDepth = neighbors[i].depth;
						neighbors[i].depth = new_node.depth+1;
						neighbors[i].tau = cost;
					}
				}
			}
		}
		
		// final_node
		cost = getOptimalTau(final_node.state, q_rand, 1.5*threshold);
		if (!isNaN(cost) && cost > dt) {
			if (best_cost + cost < final_node.path_cost) {
				tau = cost;
				if (checkFisibility(final_node.state, q_rand, tau)) {
					final_node.parent_node = new_node;
					final_node.path_cost = best_cost + cost;
					final_node.depth = new_node.depth+1;
					final_node.tau = cost;
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

function getOptimalTau(finalState, startState, t0 = 30, max_n = 10) {
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
	
	return Newton([c, b, a, 0, 1], eval, t0, max_n);
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
function Newton(coeff, eval, x0, max_n) {
	
    for (var i = 0; i < max_n; i++) {
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
		
		//if (Math.sqrt((v_x_curr-v_prev_x)*(v_x_curr-v_prev_x) + (v_y_curr-v_prev_y)*(v_y_curr-v_prev_y))/dt > map.vehicle_a_max) {
		//	return false;
		//}
		
		//a_x_curr = (d3Tau-(t-tau)*d1Tau)/r;
		//a_y_curr = (d4Tau-(t-tau)*d2Tau)/r;
		
		//if (Math.sqrt(a_x_curr*a_x_curr + a_y_curr*a_y_curr) > map.vehicle_a_max) {
		//	return false;
		//}
		
		p_prev_x = p_x_curr;
		p_prev_y = p_y_curr;
		//v_prev_x = v_x_curr;
		//v_prev_y = v_y_curr;
	}
	
	t_crit = (d3Tau+d1Tau*tau+d4Tau+d2Tau*tau)/(d1Tau+d2Tau);
	if (t_crit > 0 && t_crit < tau) {
		a_x_crit = (d3Tau-(t_crit-tau)*d1Tau)/r;
		a_y_crit = (d4Tau-(t_crit-tau)*d2Tau)/r;
		if (Math.sqrt(a_x_crit*a_x_crit + a_y_crit*a_y_crit) > map.vehicle_a_max) {
			return false;
		}
	}
	
	a_x_curr = (d3Tau-(-1e-7)*d1Tau)/r;
	a_y_curr = (d4Tau-(-1e-7)*d2Tau)/r;
	
	if (Math.sqrt(a_x_curr*a_x_curr + a_y_curr*a_y_curr) > map.vehicle_a_max) {
		return false;
	}
	
	return true;
}

async function drawPartPath(finalState, startState, tau) {
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
	
	vertNorm = translatePoint([p_prev_x, p_prev_y]);
	//ctx.moveTo(vertNorm[0], vertNorm[1]);	
	//ctx.stroke();
	
	ctx.fillRect(vertNorm[0], vertNorm[1], 1, 1);
	await sleep(10);
	
	//await sleep(10);
	//prevPoint = translatePoint([p_prev_x, p_prev_y]);
	//ctx.moveTo(prevPoint[0], prevPoint[1]);
	
	for (var t = dt; t < tau; t+=dt) {
		p_x_curr = p_x1 + v_x1*(t-tau) + Math.pow(t-tau,2)*d3Tau/(2*r) - t*d1Tau*(t*t/3 - t*tau + tau*tau)/(2*r) + Math.pow(tau,3)*d1Tau/(6*r);
		p_y_curr = p_y1 + v_y1*(t-tau) + Math.pow(t-tau,2)*d4Tau/(2*r) - t*d2Tau*(t*t/3 - t*tau + tau*tau)/(2*r) + Math.pow(tau,3)*d2Tau/(6*r);
		
		v_x_curr = v_x1 - Math.pow(t-tau,2)*d1Tau/(2*r) + (t-tau)*(d3Tau)/r;
		v_y_curr = v_y1 - Math.pow(t-tau,2)*d2Tau/(2*r) + (t-tau)*(d4Tau)/r;
		
		a_x_curr = (d3Tau-(t-tau)*d1Tau)/r;
		a_y_curr = (d4Tau-(t-tau)*d2Tau)/r;
		
		p = translatePoint([p_x_curr, p_y_curr]);
		//prevPoint = translatePoint([p_prev_x, p_prev_y]);
		//ctx.beginPath();
		//ctx.moveTo(prevPoint[0], prevPoint[1]);
		//ctx.lineTo(p[0], p[1]);
		//ctx.stroke();
		
		ctx.fillRect(p[0], p[1], 1, 1);
		
		await sleep(7);
		
		p_prev_x = p_x_curr;
		p_prev_y = p_y_curr;
		v_prev_x = v_x_curr;
		v_prev_y = v_y_curr;
	}
	
}

function sleep(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}