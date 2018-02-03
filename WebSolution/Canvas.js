// Euclidean distance between two points
function dist(p1, p2) {
	return Math.sqrt((p2[0]-p1[0])*(p2[0]-p1[0]) + (p2[1]-p1[1])*(p2[1]-p1[1]));
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
}

function translatePoint(p) {
	x = gap_x + (p[0]-min_x) * unit_x;
	y = canvas.height-(gap_y + (p[1]-min_y) * unit_y);
	return [x,y];
}