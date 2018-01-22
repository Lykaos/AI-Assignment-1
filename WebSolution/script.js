var canvas;
var ctx;

function init() {
	canvas = document.getElementById('canvas');
	ctx = canvas.getContext('2d');
	
	// Test
	ctx.font = "30px Arial";
	ctx.fillText("Drawing test", 10, 50);
	
	drawMap();
}

function drawMap() {
	// TODO: draw the map
}

function buildPathKinematic() {
	alert('Build path under kinematic point model');
}

function buildPathDynamic() {
	alert('Build path under dynamic point model');
}

function buildPathDD() {
	alert('Build path under differential drive model');
}

function buildPathKC() {
	alert('Build path under kinematic car model');
}