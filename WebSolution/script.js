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
	drawResults();
}

function buildPathDynamic() {
	alert('In progress');
}

function buildPathDD() {

	setInitialVariables();

	for (var i = 0; i < path.length; i++) {
		radius = vel / map.vehicle_omega_max;
		goal = vertices[path[i]];

		// If it's the goal, find the best tangent to the circunference centered on it and make it the new goal
		if (i == path.length - 1) {
			findTangentPointInGoal();
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
	drawResults();
}

function buildPathKC() {
	alert('Build path under kinematic car model');
}