function calculatePathToNodeKP(i) { 
    v = translatePoint(vertices[path[i]]);
    ctx.lineTo(v[0], v[1]);
    t += dist(prev_point, vertices[path[i]])/map.vehicle_v_max;
    prev_point = vertices[path[i]];     
}

function calculatePathToNodeDD() {
    v = translatePoint(goal);
    ctx.lineTo(v[0], v[1]);
    t += dist(tangent_point, goal) / map.vehicle_v_max;
}

function convertAngle(angle) {
    if (angle < 0) {
        return angle * -1;
    }
    else {
        return 2 * Math.PI - angle; 
    }
}

function drawArc(center_circle, turn, tangent_points, prev_point, radius, inverse) {
	startAngle = convertAngle(Math.atan2(prev_point[1] - center_circle[1], prev_point[0] - center_circle[0]));
    endAngle1 = convertAngle(Math.atan2(tangent_points[0][1] - center_circle[1], tangent_points[0][0] - center_circle[0]));
    endAngle2 = convertAngle(Math.atan2(tangent_points[1][1] - center_circle[1], tangent_points[1][0] - center_circle[0]));
    // Turn left
    if (turn == 0) {
    	if ((startAngle - endAngle1 < 0 && startAngle - endAngle2 < 0) || (startAngle - endAngle1 > 0 && startAngle - endAngle2 > 0)) {
    		endAngle = Math.max(endAngle1, endAngle2);
    		tangent_point = tangent_points[0];
    	}
    	else {
    		endAngle = Math.min(endAngle1, endAngle2);
    		tangent_point = tangent_points[1];
    	}
    }

    // Turn right
    else {
    	if ((startAngle - endAngle1 < 0 && startAngle - endAngle2 < 0) || (startAngle - endAngle1 > 0 && startAngle - endAngle2 > 0)) {
    		endAngle = Math.min(endAngle1, endAngle2);
            if (inverse) {
                endAngle = Math.max(endAngle1, endAngle2);
            }
    		tangent_point = tangent_points[1];
    	}
    	else {
    		endAngle = Math.max(endAngle1, endAngle2);
            if (inverse) {
                endAngle = Math.min(endAngle1, endAngle2);
            }
    		tangent_point = tangent_points[0];
    	}
    }

    v = translatePoint(center_circle);
    canvas_factor = translatePoint([2, 0])[0] - translatePoint([1, 0])[0];

    if (turn == 0) {
        if (!inverse) {
            ctx.arc(v[0], v[1], radius*canvas_factor, startAngle, endAngle, true);
        }
        else {
            ctx.arc(v[0], v[1], radius*canvas_factor, endAngle, startAngle, true);
        }
    }
    else {
        if (!inverse) {
            ctx.arc(v[0], v[1], radius*canvas_factor, startAngle, endAngle);
        }
        else {
            ctx.arc(v[0], v[1], radius*canvas_factor, endAngle, startAngle);
        }
    }
    dist_arc = radius * (Math.abs(endAngle - startAngle));
    t += dist_arc / vel;
}

function drawResults() {
    ctx.stroke();
    ctx.font = "15px Arial";
    //ctx.fillText("Traveling time: " + t, 10, 20);
    ctx.restore();
    printResults(t, 0, 0);
}

function faceNewNode() {
    [center_circle, turn] = findNearestCenter(prev_point, dir, goal, radius);
    tangent_points = findTangentPoints(center_circle, [goal[0] - center_circle[0], goal[1] - center_circle[1]], goal, radius);
    drawArc(center_circle, turn, tangent_points, prev_point, radius, false);
}

function findNearestCenter(point, vel, goal, radius) {
    vel_start = scaleVector(vel, radius);
    vel_perp = [[-vel_start[1], vel_start[0]], [vel_start[1], -vel_start[0]]]; // [Turn left, turn right]
    center_1 = [point[0] + vel_perp[0][0], point[1] + vel_perp[0][1]];
    center_2 = [point[0] + vel_perp[1][0], point[1] + vel_perp[1][1]];
    if (dist(center_1, goal) < dist(center_2, goal)) {
        return [center_1, 0]; // Turn left
    }
    else {
        return [center_2, 1]; // Turn right
    }
}

function findTangentPoints(point, vel, goal, radius) {
    vel_start = [radius * vel[0] / Math.sqrt(Math.pow(vel[0], 2) + Math.pow(vel[1], 2)), radius * vel[1] / Math.sqrt(Math.pow(vel[0], 2) + Math.pow(vel[1], 2))]
    vel_perp = [[-vel_start[1], vel_start[0]], [vel_start[1], -vel_start[0]]];
    center_1 = [point[0] + vel_perp[0][0], point[1] + vel_perp[0][1]];
    center_2 = [point[0] + vel_perp[1][0], point[1] + vel_perp[1][1]];
    return [center_1, center_2];
}

function findTangentPointInGoal() {
    vel = getModule(map.vel_goal);
    radius = vel / map.vehicle_omega_max;
    [center_circle_goal, turn_goal] = findNearestCenter(vertices[path[path.length - 1]], map.vel_goal, prev_point, radius);
    tangent_points_goal = findTangentPoints(center_circle_goal, [prev_point[0] - center_circle_goal[0], prev_point[1] - center_circle_goal[1]], prev_point, radius);
    if (turn_goal == 0) {
        goal = tangent_points_goal[0];
    }
    else {
        goal = tangent_points_goal[1];
    }
}

function getModule(vector) {
    return Math.sqrt(Math.pow(vector[0], 2) + Math.pow(vector[1], 2));
}

function printResults(time, vel, acceleration) {
    str1 = "Path completed in ";
    $("#time_results").text(str1.concat((time).toFixed(4)));
    str2 = "Arriving with velocity ";
    $("#velocity_results").text(str2.concat(map1.vehicle_v_max.toFixed(2)));
    str3 = "Arriving with acceleration ";
    $("#acceleration_results").text(str3.concat((0).toFixed(2)));
}

function scaleVector(vector, factor) {
	return [factor * vector[0] / Math.sqrt(Math.pow(vector[0], 2) + Math.pow(vector[1], 2)), factor * vector[1] / Math.sqrt(Math.pow(vector[0], 2) + Math.pow(vector[1], 2))];
}

function setInitialVariables() {
    path = setPath();
    prev_point = vertices[start_vertex_ind];
    t = 0; // Time to complete the path
    vel = getModule(map.vel_start); // Current velocity module
    dir = map.vel_start // Current direction vector
}

function setPath() {
    drawMap();
    ctx.save();
    var shortestPathInfo = shortestPath(adj_matrix, vertices.length, start_vertex_ind);
    path = constructPath(shortestPathInfo, goal_vertex_ind);

    firstPoint = translatePoint(vertices[start_vertex_ind]);
    ctx.lineWidth = 2;
    ctx.strokeStyle = "#228B22";
    ctx.moveTo(firstPoint[0], firstPoint[1]);

    return path;
}

function setVariablesForNextNode() {
    dir = [goal[0] - tangent_point[0], goal[1] - tangent_point[1]];
    vel = map.vehicle_v_max / 2;
}

function turnToGoal() {
    goal = vertices[path[path.length - 1]];
    drawArc(center_circle_goal, turn_goal, tangent_points_goal, goal, radius, true);
}