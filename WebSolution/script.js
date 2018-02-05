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

// Methods here moved to DDAux.js
function buildPathKinematic() {
	setInitialVariables(); 
	for (var i = 0; i < path.length; i++) {
		calculatePathToNodeKP(i);
	}
	drawResults(t);
}

function buildPathDD() {

	setInitialVariables();

	for (var i = 0; i < path.length; i++) {
		radius = vel / map.vehicle_omega_max;
		goal = vertices[path[i]];

		// If it's the goal, find the best tangent to the circunference centered on it and make it a temporary goal
		if (i == path.length - 1) {
			findTangentPointInGoal(false);
		}

		// We need to change our orientation for the next node, so we find a tangent point in the turning circunference
		faceNewNode();
		// When we are facing the next node, we just set maximum velocity and go in a straight line
		calculatePathToNodeDD();
	    // We set the new variables (orientation and velocity) for going to the next node
	    setVariablesForNextNode();

	    // If it's the goal, just go from the tangent point to it
	    if (i == path.length - 1) {
	    	turnToGoal();
	    }
	    prev_point = goal;
	}
	drawResults(t);
}

function buildPathKC() {

	setInitialVariables();

	for (var i = 0; i < path.length; i++) {
		radius = map.vehicle_L / Math.tan(map.vehicle_phi_max);
		goal = vertices[path[i]];

		if (i < path.length - 1) {
			if (dist(prev_point, goal) < 2*radius) {
				continue;
			}
			drawNextPath()
			setVariablesForNextNode();
		}


		// If it's the goal, find the best tangent to the circunference centered on it and make it a temporary goal
		else {
			findTangentPointInGoalKC();
		}
	}
	time_acceleration = (map.vehicle_v_max - getModule(map.vel_start)) / map.vehicle_a_max;
	time_decceleration = (map.vehicle_v_max - getModule(map.vel_goal)) / map.vehicle_a_max;
	distance_acceleration = getModule(map.vel_start) * time_acceleration + 0.5 * map.vehicle_a_max * Math.pow(time_acceleration, 2);
	distance_decceleration = map.vehicle_v_max * time_decceleration - 0.5 * map.vehicle_a_max * Math.pow(time_decceleration, 2);
	if (distance_decceleration + distance_acceleration < total_dist) {
		timeKC = ((total_dist - distance_acceleration - distance_decceleration) / map.vehicle_v_max) + time_acceleration + time_decceleration;
	}
	else {
		time_acceleration = Math.abs(getModule(map.vel_goal) - getModule(map.vel_start)) / map.vehicle_a_max;
		distance_acceleration = getModule(map.vel_start) * time_acceleration + 0.5 * map.vehicle_a_max * Math.pow(time_acceleration, 2);
		timeKC = ((total_dist - distance_acceleration) / getModule(map.vel_goal)) + time_acceleration; 	
	}
	drawResults(timeKC);
}