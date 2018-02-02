function drawArc(center_circle, turn, tangent_points, prev_point, radius, inverse) {
	startAngle = ConvertAngle(Math.atan2(prev_point[1] - center_circle[1], prev_point[0] - center_circle[0]));
    endAngle1 = ConvertAngle(Math.atan2(tangent_points[0][1] - center_circle[1], tangent_points[0][0] - center_circle[0]));
    endAngle2 = ConvertAngle(Math.atan2(tangent_points[1][1] - center_circle[1], tangent_points[1][0] - center_circle[0]));
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

function getModule(vector) {
    return Math.sqrt(Math.pow(vector[0], 2) + Math.pow(vector[1], 2));
}

function scaleVector(vector, factor) {
	return [factor * vector[0] / Math.sqrt(Math.pow(vector[0], 2) + Math.pow(vector[1], 2)), factor * vector[1] / Math.sqrt(Math.pow(vector[0], 2) + Math.pow(vector[1], 2))];
}

function ConvertAngle(angle) {
	if (angle < 0) {
		return angle * -1;
	}
	else {
		return 2 * Math.PI - angle; 
	}
}
function AngleToPoint(start, goal) {
	x = goal[0] - start[0];
	y = goal[1] - start[1];
	return Math.atan(y/x);
}

function FindNearestCenter(point, vel, goal, radius) {
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

function FindTangentPoints(point, vel, goal, radius) {
	vel_start = [radius * vel[0] / Math.sqrt(Math.pow(vel[0], 2) + Math.pow(vel[1], 2)), radius * vel[1] / Math.sqrt(Math.pow(vel[0], 2) + Math.pow(vel[1], 2))]
	vel_perp = [[-vel_start[1], vel_start[0]], [vel_start[1], -vel_start[0]]];
	center_1 = [point[0] + vel_perp[0][0], point[1] + vel_perp[0][1]];
	center_2 = [point[0] + vel_perp[1][0], point[1] + vel_perp[1][1]];
	return [center_1, center_2];
}

function printResults(time, vel, acceleration) {
    str1 = "Path completed in ";
    $("#time_results").text(str1.concat((time).toFixed(4)));
    str2 = "Arriving with velocity ";
    $("#velocity_results").text(str2.concat(map1.vehicle_v_max.toFixed(2)));
    str3 = "Arriving with acceleration ";
    $("#acceleration_results").text(str3.concat((0).toFixed(2)));
}