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