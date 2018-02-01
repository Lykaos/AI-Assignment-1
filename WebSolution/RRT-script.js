function buildPathKinematicRRT() {
	drawMap();
	ctx.save();
	
	path = getPathRRT(map.pos_start, map.pos_goal, obstacles, map.bounding_polygon);
	
	firstPoint = translatePoint(map.pos_start);
	ctx.lineWidth = 2;
	ctx.strokeStyle = "#228B22";
	ctx.moveTo(firstPoint[0], firstPoint[1]);
	prev_point = map.pos_start;
	t = 0;
	total_dist = 0;
	for (var i = 0; i < path.length; i++) {
		v = translatePoint(path[i]);
		ctx.lineTo(v[0], v[1]);
		total_dist += dist(prev_point, path[i]);
		prev_point = path[i];		
	}
	ctx.stroke();
	ctx.font = "15px Arial";
	//ctx.fillText("Traveling time: " + t, 10, 20);
	ctx.restore();
	str1 = "Time: ";
	$("#time_results").text(str1.concat((total_dist/map.vehicle_v_max).toFixed(4)));
	
	ctx.restore();
}

function getPathRRT(pos_start, pos_goal, obstacles, bound_polygon, N = 1000) {
	
	var GOAL_SAMPLE_PROBABILITY = 0.1;
	//var MAX_STEP_LENGTH = 10;
	var NEIGHBORHOOD_RADIUS = 30;
	var tree = [ { point: pos_start, parent_node: null, path_length: 0 } ];
	
	final_node = null;
	
	for (var iter = 0; iter < N; iter++) {
		
		var q_rand = [0, 0];
		if (Math.random() < GOAL_SAMPLE_PROBABILITY) {
			// Sample the goal
			q_rand = pos_goal;
		} else {
			// Sample a random free point
			freePoint = false;
			while (!freePoint) {
				
				q_rand[0] = (Math.random()*(max_x-min_x)+min_x);
				q_rand[1] = (Math.random()*(max_y-min_y)+min_y);
				
				if (!checkInside(q_rand, bound_polygon)) {
					continue;
				}
				
				freePoint = true;
				for (var i = 0; i < obstacles.length; i++) {
					if (checkInside(q_rand, obstacles[i])) {
						freePoint = false;
						break;
					}
				}
			}
		}
		
		neighbors = [];
		best_neighbor = null;
		best_length = Number.MAX_VALUE;
		for (var i = 0; i < tree.length; i++) {
			node = tree[i];
			distance = dist(q_rand, node.point);
			if (distance < NEIGHBORHOOD_RADIUS) {
				obst_free = true;
				for (var k = 0; k < obst_edges.length; k++) {
					if (segmentsIntersect(q_rand[0], q_rand[1], node.point[0], node.point[1], obst_edges[k][0][0],
								obst_edges[k][0][1], obst_edges[k][1][0], obst_edges[k][1][1])) {
						obst_free = false;
						break;
					}
				}
				if (obst_free) {
					neighbors.push(node);
					if ((distance + node.path_length) < best_length) {
						best_length = distance + node.path_length;
						best_neighbor = node;
					}
				}
			}
		}
		
		if (best_neighbor == null) {
			continue;
		}
		
		new_node = { point: q_rand, parent_node: best_neighbor, path_length: best_length };
		tree.push(new_node);
		
		if (q_rand == pos_goal) {
			final_node = new_node;
		}
		
		for (var i = 0; i < neighbors.length; i++) {
			distance = dist(q_rand, neighbors[i].point);
			if (best_length + distance < neighbors[i].path_length) {
				obst_free = true;
				for (var k = 0; k < obst_edges.length; k++) {
					if (segmentsIntersect(q_rand[0], q_rand[1], neighbors[i].point[0], neighbors[i].point[1], obst_edges[k][0][0],
								obst_edges[k][0][1], obst_edges[k][1][0], obst_edges[k][1][1])) {
						obst_free = false;
						break;
					}
				}
				if (obst_free) {
					neighbors[i].parent_node = new_node;
					neighbors[i].path_length = best_length + distance;
				}
			}
		}
	}
	
	path = undefined;
	if (final_node != null) {
		path = [];
		end_node = final_node;
		while (end_node != tree[0]) {
			path.unshift(end_node.point);
			end_node = end_node.parent_node;
		}
	}
	
	return path;
}

function sampleGaussian(mean, sd) {
    u1 = Math.random();
	u2 = Math.random();
	z0 = Math.sqrt(-2*Math.log(u1))*Math.cos(2*Math.PI*u2);
	return mean + z0*sd;
}

function checkInside(point, vs) {
    // ray-casting algorithm based on
    // http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html

    var x = point[0], y = point[1];

    var inside = false;
    for (var i = 0, j = vs.length - 1; i < vs.length; j = i++) {
        var xi = vs[i][0], yi = vs[i][1];
        var xj = vs[j][0], yj = vs[j][1];

        var intersect = ((yi > y) != (yj > y))
            && (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
        if (intersect) inside = !inside;
    }

    return inside;
}