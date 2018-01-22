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

var	path = []; // Optimal path
var acceleration;
var vel;

function init() {
	canvas = document.getElementById('canvas');
	ctx = canvas.getContext('2d');
	gap_x = canvas.width*gap_perc / 100;
	gap_y = canvas.height*gap_perc / 100;
	obstacles = [ map1.obstacle_1, map1.obstacle_2, map1.obstacle_3, map1.obstacle_4, map1.obstacle_5 ];
	vertices = [];
	obst_edges = [];
	
	start_vertex_ind = vertices.length;
	vertices.push(map1.pos_start);

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
			vertices.push(obstacles[i][j]);
			obst_edges.push([prev_obst_vert, obstacles[i][j]]);
			prev_obst_vert = obstacles[i][j];
		}
	}
	
	prev_obst_vert = map1.bounding_polygon[map1.bounding_polygon.length - 1];
	for (var i = 0; i < map1.bounding_polygon.length; i++) {
		
		x = map1.bounding_polygon[i][0];
		y = map1.bounding_polygon[i][1];
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
		vertices.push(map1.bounding_polygon[i]);
		obst_edges.push([prev_obst_vert, map1.bounding_polygon[i]]);
		prev_obst_vert = map1.bounding_polygon[i];
	}
	

	goal_vertex_ind = vertices.length;
	vertices.push(map1.pos_goal);
	
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
			visible = true;
			for (var k = 0; k < obst_edges.length; k++) {
				if (segmentsIntersect(v1, v2, obst_edges[k][0], obst_edges[k][1])) {
					visible = false;
					break;
				}
			}
			if (visible) {
				adj_matrix[i][j] = dist(v1, v2);
			} else {
				adj_matrix[i][j] = Number.MAX_VALUE;
			}
		}
	}
	
	// Test
	//ctx.font = "30px Arial";
	//ctx.fillText("Drawing test", 10, 50);
	
	drawMap();
	dijkstra(adj_matrix);
}

function drawMap() {
	
	// Draw the bounding polygon
	ctx.beginPath();
	last_vertex = map1.bounding_polygon[map1.bounding_polygon.length-1];
	last_vertex_norm = translatePoint(last_vertex);
	ctx.moveTo(last_vertex_norm[0], last_vertex_norm[1]);
	for (var i = 0; i < map1.bounding_polygon.length; i++) {
		v = map1.bounding_polygon[i];
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
	
	/*
	ctx.moveTo(0, 0);
	ctx.lineTo(100,50);
	ctx.lineTo(50, 100);
	ctx.lineTo(0, 90);
	*/
}

function buildPathKinematic() {
	alert('Build path under kinematic point model');
}

function buildPathDynamic() {
// TODO: Implement this better when there is more than one node in the optimal path
	if (path.length == 1) { // First week case
		acceleration = map1.vehicle_a_max;
		vel_max = map1.vehicle_v_max
		time_max_a = vel_max / acceleration; // Time spent accelerating until max speed.
		distance_while_accelerating = acceleration * Math.pow(time_max_a, 2) / 2; // Distance travelled while accelerating.		
		total_distance = dist(vertices[vertices.length -1], vertices[0]);
		remaining_distance = total_distance - distance_while_accelerating; // Distance travelled with max speed
		if (remaining_distance > 0) {
			acceleration = 0;
		}
		remaining_time = remaining_distance / vel_max;
		total_time = time_max_a + remaining_time;
		str1 = "Path completed in ";
		$("#time_results").text(str1.concat((time_max_a + total_time).toFixed(4)).concat(" s"));
		str2 = "Arriving with velocity ";
		$("#velocity_results").text(str2.concat(vel_max.toFixed(2)).concat(" units/s"));
		str3 = "Arriving with acceleration ";
		$("#acceleration_results").text(str3.concat(acceleration.toFixed(2)).concat(" units/s2"));
	}
}

function buildPathDD() {
	alert('Build path under differential drive model');
}

function buildPathKC() {
	alert('Build path under kinematic car model');
}


function segmentsIntersect(p0, p1, p2, p3) {
	p0_x=p0[0]; p0_y=p0[1]; p1_x=p1[0]; p1_y=p1[1]; p2_x=p2[0]; p2_y=p2[1]; p3_x=p3[0]; p3_y=p3[1];
    s1_x = p1_x - p0_x;
    s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;
    s2_y = p3_y - p2_y;
    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);
    return s > 0 && s < 1 && t > 0 && t < 1;
}

// Euclidian distance between two points
function dist(p1, p2) {
	return Math.sqrt((p2[0]-p1[0])*(p2[0]-p1[0]) + (p2[1]-p1[1])*(p2[1]-p1[1]));
}

function translatePoint(p) {
	x = gap_x + (p[0]-min_x) * unit_x;
	y = canvas.height-(gap_y + (p[1]-min_y) * unit_y);
	return [x,y];
}

function dijkstra(adj_matrix) {
	/* TODO: Fix this algorithm. We can use Dmitry's.
	var unvisited_nodes = [];
	var node_weights = []; // First element is the starting node, last element is the final node.
	var node_paths = [];
	var best_path = [];

	for (i = 1; i < vertices.length; i++) {
		unvisited_nodes.push(i);
		node_weights.push(Number.MAX_VALUE);
		node_paths.push([0]);
	}

	node_weights[0] = 0; // The starting node has weight 0.

	for (i = 0; i < vertices.length; i++) {
		for (j = 0; j < unvisited_nodes.length; j++) {
			if (node_weights[unvisited_nodes[j]] > node_weights[i] + adj_matrix[i][unvisited_nodes[j]]) {
				node_weights[unvisited_nodes[j]] = node_weights[i] + adj_matrix[i][unvisited_nodes[j]];
				node_paths[unvisited_nodes[j]].push(i);
			}
		}
		unvisited_nodes.shift();

	} 
	return node_paths; */

	// No obstacles scenario
	path.push(vertices.length);
	return path;
}



