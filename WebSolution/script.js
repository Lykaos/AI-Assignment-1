var canvas;
var ctx;
var obstacles;
var vertices;
var adj_matrix;
var start_vertex_ind;
var goal_vertex_ind;

// Values for outlining the area of drawing
var min_x = Number.MAX_VALUE;
var max_x = Number.MIN_VALUE;
var min_y = Number.MAX_VALUE;
var max_y = Number.MIN_VALUE;
var gap_perc = 5;
var gap_x;
var gap_y;
var unit_x;
var unit_y;
var marker_width = 9;
var marker_height = 9;

var	path = []; // Optimal path
var acceleration;
var vel;

function init() {
	canvas = document.getElementById('canvas');
	ctx = canvas.getContext('2d');
	gap_x = canvas.width*gap_perc / 100;
	gap_y = canvas.height*gap_perc / 100;
	initMap(map1);
}

function initMap(newMap) {
	map = newMap;
	obstacles = [];
	for (var i = 1; i < 100; i++) {
		val = map["obstacle_"+i];
		if (val == undefined) {
			break;
		}
		obstacles.push(val);
	}
	vertices = [];
	obst_edges = [];
	for (var i = 0; i < obstacles.length; i++) {
		
		prev_obst_vert = obstacles[i][obstacles[i].length - 1];
		for (var j = 0; j < obstacles[i].length; j++) {
			
			x = obstacles[i][j][0];
			y = obstacles[i][j][1];
			if (x < min_x) {
				min_x = x;
			}
			if (x > max_x) {
				max_x = x;
			}
			if (y < min_y) {
				min_y = y;
			}
			if (y > max_y) {
				max_y = y;
			}
			
			v = [obstacles[i][j][0], obstacles[i][j][1]];
			//nextInd = j % obstacles[i].length;
			//next_obs_vert = obstacles[i][nextInd];
			// Adding some margin to avoid collisions
			//if (prev_obst_vert[0] > v[0]) {
			//	v[0] -= eps;
			//} else {
			//	v[0] += eps;
			//}
			//if (prev_obst_vert[1] > v[1]) {
			//	v[1] -= eps;
			//} else {
			//	v[1] += eps;
			//}
			
			//if (next_obs_vert[0] > v[0]) {
			//	v[0] -= eps;
			//} else {
			//	v[0] += eps;
			//}
			//if (next_obs_vert[1] > v[1]) {
			//	v[1] -= eps;
			//} else {
			//	v[1] += eps;
			//}

			// ???
			
			/* for (var k = 0; k < 7; k++) {
				vertices.push([v[0]+sampleGaussian(0, 1e-3), v[1]+sampleGaussian(0, 1e-3)]);
			} */

			vertices.push([v[0], v[1]]);
			
			obst_edges.push([prev_obst_vert, obstacles[i][j]]);
			prev_obst_vert = obstacles[i][j];
		}
	}
	
	prev_obst_vert = map.bounding_polygon[map.bounding_polygon.length - 1];
	for (var i = 0; i < map.bounding_polygon.length; i++) {
		
		x = map.bounding_polygon[i][0];
		y = map.bounding_polygon[i][1];
		if (x < min_x) {
			min_x = x;
		}
		if (x > max_x) {
			max_x = x;
		}
		if (y < min_y) {
			min_y = y;
		}
		if (y > max_y) {
			max_y = y;
		}
		//vertices.push(map.bounding_polygon[i]);
		obst_edges.push([prev_obst_vert, map.bounding_polygon[i]]);
		prev_obst_vert = map.bounding_polygon[i];
	}
	
	start_vertex_ind = vertices.length;
	vertices.push(map.pos_start);
	goal_vertex_ind = vertices.length;
	vertices.push(map.pos_goal);
	
	unit_x = (canvas.width-2*gap_x)/(max_x-min_x);
	unit_y = (canvas.height-2*gap_y)/(max_y-min_y);
	if (unit_x > unit_y) {
		unit_x = unit_y;
		gap_x = (canvas.width - unit_x*(max_x-min_x))/2;
	} else {
		unit_y = unit_x;
		gap_y = (canvas.height - unit_y*(max_y-min_y))/2;
	}
	
	adj_matrix = new Array(vertices.length);
	for (var i = 0; i < vertices.length; i++) {		
		v1 = vertices[i];
		adj_matrix[i] = new Array(vertices.length);
		for (var j = 0; j < vertices.length; j++) {		
			if (i == j) {
				adj_matrix[i][i] = 0;
				continue;
			}
			v2 = vertices[j];
			for (var k = 0; k < obst_edges.length; k++) {
				if (segmentsIntersect(v1[0], v1[1], v2[0], v2[1], obst_edges[k][0][0], obst_edges[k][0][1], obst_edges[k][1][0], obst_edges[k][1][1])) {
					adj_matrix[i][j] = Number.MAX_VALUE;
					break;
				}
				else {
					adj_matrix[i][j] = dist(v1, v2);
				}
			}
		}
	}
	document.getElementById("info").innerHTML = 'v_max: '+map.vehicle_v_max+';  phi_max: '+map.vehicle_phi_max+';  omega_max: '+map.vehicle_omega_max+';  a_max: '+map.vehicle_a_max+';  L: '+map.vehicle_L;
	
	drawMap();
}

function drawMap() {
	
	ctx.save();
	ctx.clearRect(0, 0, canvas.width, canvas.height);
	
	// Draw the bounding polygon
	ctx.beginPath();
	last_vertex = map.bounding_polygon[map.bounding_polygon.length-1];
	last_vertex_norm = translatePoint(last_vertex);
	ctx.moveTo(last_vertex_norm[0], last_vertex_norm[1]);
	for (var i = 0; i < map.bounding_polygon.length; i++) {
		v = map.bounding_polygon[i];
		v_norm = translatePoint(v);
		ctx.lineTo(v_norm[0], v_norm[1]);
	}
	ctx.closePath();
	ctx.stroke();
	
	// Draw the obstacles
	for (var i = 0; i < obstacles.length; i++) {
		ctx.beginPath();
		last_vertex = obstacles[i][obstacles[i].length-1];
		last_vertex_norm = translatePoint(last_vertex);
		ctx.moveTo(last_vertex_norm[0], last_vertex_norm[1]);
		for (var j = 0; j < obstacles[i].length; j++) {
			v = obstacles[i][j];
			v_norm = translatePoint(v);
			ctx.lineTo(v_norm[0], v_norm[1]);
		}
		ctx.closePath();
		ctx.fill();
	}
	
	// Draw the markers
	
	start_point = translatePoint(map.pos_start);
	ctx.translate(start_point[0], start_point[1]);
	rotateAngle = Math.atan2(map.vel_start[0], map.vel_start[1]);
	ctx.rotate(rotateAngle);
	ctx.fillStyle = "#FF0000";
	ctx.fillRect(-(marker_width/2), -(marker_height/2), marker_width, marker_height);
	// Draw vector
	ctx.strokeStyle = "#000000";
	ctx.beginPath();
	ctx.moveTo(0, 0);
	ctx.lineTo(0, -Math.sqrt(map.vel_start[0]*map.vel_start[0] +
		map.vel_start[1]*map.vel_start[1])*unit_y);
	ctx.stroke();
	ctx.rotate(-rotateAngle);
	ctx.translate(-start_point[0], -start_point[1]);
	
	goal_point = translatePoint(map.pos_goal);
	ctx.translate(goal_point[0], goal_point[1]);
	rotateAngle = Math.atan2(map.vel_goal[0], map.vel_goal[1]);
	ctx.rotate(rotateAngle);
	ctx.fillStyle = "#0000CD";
	ctx.fillRect(-(marker_width/2), -(marker_height/2), marker_width, marker_height);
	// Draw vector
	ctx.strokeStyle = "#000000";
	ctx.beginPath();
	ctx.moveTo(0, 0);
	ctx.lineTo(0, -Math.sqrt(map.vel_goal[0]*map.vel_goal[0] +
		map.vel_goal[1]*map.vel_goal[1])*unit_y);
	ctx.stroke();
	ctx.rotate(-rotateAngle);
	ctx.translate(-goal_point[0], -goal_point[1]);
	
	ctx.restore();
}

function buildPathKinematic() {
	drawMap();
	ctx.save();
	
	var shortestPathInfo = shortestPath(adj_matrix, vertices.length, start_vertex_ind);
	path = constructPath(shortestPathInfo, goal_vertex_ind);

	// What is this for?
	for (var i = 0; i < 10 && path == undefined; i++) {
		initMap(map);
		shortestPathInfo = shortestPath(adj_matrix, vertices.length, start_vertex_ind);
		path = constructPath(shortestPathInfo, goal_vertex_ind);
	} 
	
	// Won't happen
	if (path == undefined) {
		alert("No path found. Try again?")
		return;
	}
	
	//TODO: check for Nan
	firstPoint = translatePoint(vertices[start_vertex_ind]);
	ctx.lineWidth = 2;
	ctx.strokeStyle = "#228B22";
	ctx.moveTo(firstPoint[0], firstPoint[1]);
	prev_point = vertices[start_vertex_ind];
	t = 0;
	total = 0;
	for (var i = 0; i < path.length; i++) {
		v = translatePoint(vertices[path[i]]);
		ctx.lineTo(v[0], v[1]);
		t += dist(prev_point, vertices[path[i]])/map.vehicle_v_max;
		total += dist(prev_point, vertices[path[i]]);
		prev_point = vertices[path[i]];		
	}
	ctx.stroke();
	ctx.font = "15px Arial";
	//ctx.fillText("Traveling time: " + t, 10, 20);
	ctx.restore();
	str1 = "Time: ";
	$("#time_results").text(str1.concat((t).toFixed(4)));
}

function buildPathDynamic() {
	prev_point = vertices[start_vertex_ind];
	drawMap();
	ctx.save();	
	var shortestPathInfo = shortestPath(adj_matrix, vertices.length, start_vertex_ind);
	path = constructPath(shortestPathInfo, goal_vertex_ind);
	firstPoint = translatePoint(vertices[start_vertex_ind]);
	ctx.strokeStyle = "#228B22";
	ctx.lineWidth = 2;
	ctx.moveTo(firstPoint[0], firstPoint[1]);

	// TODO: Implement this better when there is more than one node in the optimal path
	if (path.length == 1) { // First week case
		v = translatePoint(vertices[path[0]]);
		ctx.lineTo(v[0], v[1]);
		acceleration = map1.vehicle_a_max;
		vel_max = map1.vehicle_v_max
		time_max_a = vel_max / acceleration; // Time spent accelerating until max speed.
		distance_while_accelerating = acceleration * Math.pow(time_max_a, 2) / 2; // Distance travelled while accelerating.		
		total_distance = dist(vertices[vertices.length -1], prev_point);
		remaining_distance = total_distance - distance_while_accelerating; // Distance travelled with max speed
		if (remaining_distance > 0) {
			acceleration = 0;
		}
		remaining_time = remaining_distance / vel_max;
		total_time = time_max_a + remaining_time;
		ctx.stroke();
		ctx.font = "15px Arial";
		//ctx.fillText("Traveling time: " + t, 10, 20);
		str1 = "Path completed in ";
		$("#time_results").text(str1.concat((time_max_a + total_time).toFixed(4)));
		str2 = "Arriving with velocity ";
		$("#velocity_results").text(str2.concat(vel_max.toFixed(2)));
		str3 = "Arriving with acceleration ";
		$("#acceleration_results").text(str3.concat(acceleration.toFixed(2)));
	}
	ctx.restore();
}

/* function buildPathDD() {
	prev_point = vertices[start_vertex_ind];
	drawMap();
	ctx.save();	
	var shortestPathInfo = shortestPath(adj_matrix, vertices.length, start_vertex_ind);
	path = constructPath(shortestPathInfo, goal_vertex_ind);
	firstPoint = translatePoint(vertices[start_vertex_ind]);
	ctx.strokeStyle = "#228B22";
	ctx.moveTo(firstPoint[0], firstPoint[1]);
	// TODO: Implement this better when there is more than one node in the optimal path
	if (path.length == 1) { // First week case
		v = translatePoint(vertices[path[0]]);
		ctx.lineTo(v[0], v[1]);
		vel_max = map1.vehicle_v_max;
		acceleration = 0;
		total_distance = dist(vertices[vertices.length -1], prev_point);
		v1 = vertices[0];
		v2 = vertices[vertices.length - 1];
		radians_to_rotate = Math.atan(v2[1]-v1[1]/v2[0]/v1[0]);
		rotation_time = radians_to_rotate / map1.vehicle_phi_max;
		remaining_time = total_distance / vel_max;
		total_time = rotation_time + remaining_time;
		ctx.stroke();
		ctx.font = "15px Arial";
		//ctx.fillText("Traveling time: " + t, 10, 20);
		str1 = "Path completed in ";
		$("#time_results").text(str1.concat((total_time).toFixed(4)).concat(" s"));
		str2 = "Arriving with velocity ";
		$("#velocity_results").text(str2.concat(vel_max.toFixed(2)));
		str3 = "Arriving with acceleration ";
		$("#acceleration_results").text(str3.concat(acceleration.toFixed(2)));
	}
	ctx.restore();
} */

function buildPathDD() {
	drawMap();
	ctx.save();	
	var shortestPathInfo = shortestPath(adj_matrix, vertices.length, start_vertex_ind);
	path = constructPath(shortestPathInfo, goal_vertex_ind);

	//TODO: check for Nan
	firstPoint = translatePoint(vertices[start_vertex_ind]);
	ctx.lineWidth = 2;
	ctx.strokeStyle = "#228B22";
	ctx.moveTo(firstPoint[0], firstPoint[1]);
	prev_point = vertices[start_vertex_ind];
	t = 0;

	for (var i = 0; i < path.length; i++) {
		v = translatePoint(vertices[path[i]]);
		ctx.lineTo(v[0], v[1]);
		t += (dist(prev_point, vertices[path[i]]))/map.vehicle_v_max;
		prev_point = vertices[i];
	}

	ctx.stroke();
	ctx.font = "15px Arial";
	//ctx.fillText("Traveling time: " + t, 10, 20);
	ctx.restore();
	str1 = "Path completed in ";
	$("#time_results").text(str1.concat((t).toFixed(4)));
	str2 = "Arriving with velocity ";
	$("#velocity_results").text(str2.concat(map1.vehicle_v_max.toFixed(2)));
	str3 = "Arriving with acceleration ";
	$("#acceleration_results").text(str3.concat((0).toFixed(2)));
}

function buildPathKC() {
	alert('Build path under kinematic car model');
}

// https://stackoverflow.com/questions/9043805/test-if-two-lines-intersect-javascript-function
// Returns true iff the line from (a,b)->(c,d) intersects with (p,q)->(r,s)
function segmentsIntersect(a,b,c,d,p,q,r,s) {
  var det, gamma, lambda;
  det = (c - a) * (s - q) - (r - p) * (d - b);
  if (det === 0) {
    return false;
  } else {
    lambda = ((s - q) * (r - a) + (p - r) * (s - b)) / det;
    gamma = ((b - d) * (r - a) + (c - a) * (s - b)) / det;
    return (0 < lambda && lambda < 1) && (0 < gamma && gamma < 1);
  }
};

// Euclidian distance between two points
function dist(p1, p2) {
	return Math.sqrt((p2[0]-p1[0])*(p2[0]-p1[0]) + (p2[1]-p1[1])*(p2[1]-p1[1]));
}

function translatePoint(p) {
	x = gap_x + (p[0]-min_x) * unit_x;
	y = canvas.height-(gap_y + (p[1]-min_y) * unit_y);
	return [x,y];
}

// Taken from: http://mcc.id.au/2004/10/dijkstra.js
function shortestPath(edges, numVertices, startVertex) {
	var done = new Array(numVertices);
	done[startVertex] = true;
	var pathLengths = new Array(numVertices);
	var predecessors = new Array(numVertices);
	for (var i = 0; i < numVertices; i++) {
		pathLengths[i] = edges[startVertex][i];
		if (edges[startVertex][i] != Infinity) {
			predecessors[i] = startVertex;
		}
	}
	pathLengths[startVertex] = 0;
	for (var i = 0; i < numVertices - 1; i++) {
		var closest = -1;
		var closestDistance = Infinity;
		for (var j = 0; j < numVertices; j++) {
			if (!done[j] && pathLengths[j] < closestDistance) {
				closestDistance = pathLengths[j];
				closest = j;
			}
		}
		done[closest] = true;
		for (var j = 0; j < numVertices; j++) {
			if (!done[j]) {
				var possiblyCloserDistance = pathLengths[closest] + edges[closest][j];
				if (possiblyCloserDistance < pathLengths[j]) {
					pathLengths[j] = possiblyCloserDistance;
					predecessors[j] = closest;
				}
			}
		}
	}
	return { "startVertex": startVertex, "pathLengths": pathLengths, "predecessors": predecessors };
}

function constructPath(shortestPathInfo, endVertex) {
	var path = [];
	while (endVertex != shortestPathInfo.startVertex) {
		path.unshift(endVertex);
		if (shortestPathInfo.pathLengths[endVertex] == Number.MAX_VALUE) {
			return undefined;
		}
		endVertex = shortestPathInfo.predecessors[endVertex];
	}
	return path;
}