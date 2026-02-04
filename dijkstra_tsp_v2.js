// ============================================
// ALGORITHME DIJKSTRA + TSP OFFLINE V2
// Avec géométries précises des segments de route
// ============================================

class OfflineRouterV2 {
    constructor(graph, roadsGeoJSON) {
        this.graph = graph;
        this.roadsGeoJSON = roadsGeoJSON;
        this.nodeIndex = {};
        this.allSegments = []; // Tous les segments de route
        this.edgeGeometries = {}; // Cache des géométries entre nœuds
        
        this.buildIndex();
        this.extractAllSegments();
        this.buildEdgeGeometries();
    }
    
    buildIndex() {
        // Créer un index spatial pour trouver rapidement le noeud le plus proche
        for (const nodeId in this.graph) {
            const [lat, lng] = this.graph[nodeId];
            const key = `${Math.floor(lat * 100)},${Math.floor(lng * 100)}`;
            if (!this.nodeIndex[key]) this.nodeIndex[key] = [];
            this.nodeIndex[key].push(nodeId);
        }
    }
    
    extractAllSegments() {
        // Extraire tous les segments du GeoJSON
        if (!this.roadsGeoJSON || !this.roadsGeoJSON.features) return;
        
        for (const feature of this.roadsGeoJSON.features) {
            if (feature.geometry.type === 'MultiLineString') {
                for (const line of feature.geometry.coordinates) {
                    this.allSegments.push(line.map(c => [c[1], c[0]])); // [lat, lng]
                }
            } else if (feature.geometry.type === 'LineString') {
                this.allSegments.push(feature.geometry.coordinates.map(c => [c[1], c[0]]));
            }
        }
        console.log(`📍 ${this.allSegments.length} segments de route extraits`);
    }
    
    buildEdgeGeometries() {
        // Pour chaque arête du graphe, trouver la géométrie correspondante
        for (const nodeId in this.graph) {
            const node = this.graph[nodeId];
            const [lat1, lng1] = node;
            const edges = node[2];
            
            for (const [neighborId, dist, time] of edges) {
                const neighbor = this.graph[neighborId];
                if (!neighbor) continue;
                
                const [lat2, lng2] = neighbor;
                const edgeKey = `${nodeId}|${neighborId}`;
                
                // Chercher la géométrie qui connecte ces deux points
                const geom = this.findGeometryBetweenPoints(lat1, lng1, lat2, lng2, dist);
                
                if (geom) {
                    this.edgeGeometries[edgeKey] = geom;
                }
            }
        }
        console.log(`📐 ${Object.keys(this.edgeGeometries).length} géométries d'arêtes trouvées`);
    }
    
    findGeometryBetweenPoints(lat1, lng1, lat2, lng2, expectedDist) {
        // Chercher un segment qui connecte ces deux points
        const tolerance = 0.001; // ~100m de tolérance pour les extrémités
        
        for (const segment of this.allSegments) {
            if (segment.length < 2) continue;
            
            // Vérifier les extrémités
            const startDist1 = this.quickDist(segment[0][0], segment[0][1], lat1, lng1);
            const endDist1 = this.quickDist(segment[segment.length-1][0], segment[segment.length-1][1], lat2, lng2);
            
            if (startDist1 < tolerance && endDist1 < tolerance) {
                return [...segment]; // Copie dans le bon sens
            }
            
            // Vérifier dans l'autre sens
            const startDist2 = this.quickDist(segment[0][0], segment[0][1], lat2, lng2);
            const endDist2 = this.quickDist(segment[segment.length-1][0], segment[segment.length-1][1], lat1, lng1);
            
            if (startDist2 < tolerance && endDist2 < tolerance) {
                return [...segment].reverse(); // Copie inversée
            }
            
            // Chercher aussi dans les sous-parties du segment
            const subGeom = this.findSubGeometry(segment, lat1, lng1, lat2, lng2, tolerance);
            if (subGeom) return subGeom;
        }
        
        return null; // Pas de géométrie trouvée, on utilisera une ligne droite
    }
    
    findSubGeometry(segment, lat1, lng1, lat2, lng2, tolerance) {
        // Chercher si les points sont à l'intérieur du segment
        let startIdx = -1;
        let endIdx = -1;
        
        for (let i = 0; i < segment.length; i++) {
            const dist1 = this.quickDist(segment[i][0], segment[i][1], lat1, lng1);
            const dist2 = this.quickDist(segment[i][0], segment[i][1], lat2, lng2);
            
            if (dist1 < tolerance && startIdx === -1) startIdx = i;
            if (dist2 < tolerance) endIdx = i;
        }
        
        if (startIdx !== -1 && endIdx !== -1 && startIdx !== endIdx) {
            if (startIdx < endIdx) {
                return segment.slice(startIdx, endIdx + 1);
            } else {
                return segment.slice(endIdx, startIdx + 1).reverse();
            }
        }
        
        return null;
    }
    
    quickDist(lat1, lng1, lat2, lng2) {
        // Distance rapide en degrés (suffisant pour comparaison)
        const dlat = lat2 - lat1;
        const dlng = lng2 - lng1;
        return Math.sqrt(dlat * dlat + dlng * dlng);
    }
    
    // Trouver le noeud du graphe le plus proche d'un point
    findNearestNode(lat, lng) {
        const key = `${Math.floor(lat * 100)},${Math.floor(lng * 100)}`;
        
        let bestNode = null;
        let bestDist = Infinity;
        
        // Chercher dans les cellules voisines (élargi à 3 cellules)
        for (let dlat = -3; dlat <= 3; dlat++) {
            for (let dlng = -3; dlng <= 3; dlng++) {
                const searchKey = `${Math.floor(lat * 100) + dlat},${Math.floor(lng * 100) + dlng}`;
                const nodes = this.nodeIndex[searchKey] || [];
                
                for (const nodeId of nodes) {
                    const [nodeLat, nodeLng] = this.graph[nodeId];
                    const dist = this.haversine(lat, lng, nodeLat, nodeLng);
                    if (dist < bestDist) {
                        bestDist = dist;
                        bestNode = nodeId;
                    }
                }
            }
        }
        
        // Si pas trouvé localement, chercher dans tout le graphe
        if (!bestNode) {
            for (const nodeId in this.graph) {
                const [nodeLat, nodeLng] = this.graph[nodeId];
                const dist = this.haversine(lat, lng, nodeLat, nodeLng);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestNode = nodeId;
                }
            }
        }
        
        return { nodeId: bestNode, distance: bestDist };
    }
    
    haversine(lat1, lng1, lat2, lng2) {
        const R = 6371000;
        const dLat = (lat2 - lat1) * Math.PI / 180;
        const dLng = (lng2 - lng1) * Math.PI / 180;
        const a = Math.sin(dLat/2) * Math.sin(dLat/2) +
                  Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
                  Math.sin(dLng/2) * Math.sin(dLng/2);
        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
        return R * c;
    }
    
    // Dijkstra pour trouver le plus court chemin (retourne les nœuds)
    dijkstra(startNode, endNode) {
        const distances = {};
        const previous = {};
        const visited = new Set();
        const pq = new MinHeapV2();
        
        distances[startNode] = 0;
        pq.insert(startNode, 0);
        
        while (!pq.isEmpty()) {
            const current = pq.extractMin();
            
            if (current === endNode) break;
            if (visited.has(current)) continue;
            visited.add(current);
            
            const node = this.graph[current];
            if (!node) continue;
            
            const edges = node[2]; // [[to, dist, time], ...]
            
            for (const [neighbor, dist, time] of edges) {
                if (visited.has(neighbor)) continue;
                
                const newDist = distances[current] + dist;
                
                if (newDist < (distances[neighbor] || Infinity)) {
                    distances[neighbor] = newDist;
                    previous[neighbor] = current;
                    pq.insert(neighbor, newDist);
                }
            }
        }
        
        // Reconstruire le chemin avec les géométries détaillées
        if (!previous[endNode] && startNode !== endNode) {
            return null;
        }
        
        // Collecter les nœuds du chemin
        const nodeList = [];
        let current = endNode;
        while (current) {
            nodeList.unshift(current);
            current = previous[current];
        }
        
        // Construire le chemin avec les géométries détaillées
        const path = [];
        
        for (let i = 0; i < nodeList.length - 1; i++) {
            const fromNode = nodeList[i];
            const toNode = nodeList[i + 1];
            const edgeKey = `${fromNode}|${toNode}`;
            
            if (this.edgeGeometries[edgeKey] && this.edgeGeometries[edgeKey].length > 0) {
                // Utiliser la géométrie détaillée
                const geom = this.edgeGeometries[edgeKey];
                // Éviter les doublons aux jonctions
                if (path.length > 0) {
                    // Vérifier si le premier point de geom est proche du dernier point de path
                    const lastPoint = path[path.length - 1];
                    if (this.quickDist(lastPoint[0], lastPoint[1], geom[0][0], geom[0][1]) < 0.0001) {
                        path.push(...geom.slice(1));
                    } else {
                        path.push(...geom);
                    }
                } else {
                    path.push(...geom);
                }
            } else {
                // Fallback: ligne droite entre les nœuds
                const fromCoords = this.graph[fromNode];
                const toCoords = this.graph[toNode];
                if (path.length > 0) {
                    path.push([toCoords[0], toCoords[1]]);
                } else {
                    path.push([fromCoords[0], fromCoords[1]]);
                    path.push([toCoords[0], toCoords[1]]);
                }
            }
        }
        
        return {
            path: path,
            distance: distances[endNode] || 0,
            nodes: nodeList
        };
    }
    
    // Calculer la matrice des distances entre plusieurs points
    computeDistanceMatrix(points) {
        const n = points.length;
        const matrix = [];
        
        for (let i = 0; i < n; i++) {
            matrix[i] = [];
            for (let j = 0; j < n; j++) {
                if (i === j) {
                    matrix[i][j] = { distance: 0, path: [] };
                } else {
                    const result = this.dijkstra(points[i].nodeId, points[j].nodeId);
                    matrix[i][j] = result || { distance: Infinity, path: [] };
                }
            }
        }
        
        return matrix;
    }
    
    // Résoudre le TSP avec l'algorithme du plus proche voisin
    solveTSP(points, startIndex = 0) {
        const n = points.length;
        const matrix = this.computeDistanceMatrix(points);
        
        const visited = new Set([startIndex]);
        const order = [startIndex];
        let current = startIndex;
        
        while (order.length < n) {
            let nearest = -1;
            let nearestDist = Infinity;
            
            for (let i = 0; i < n; i++) {
                if (!visited.has(i) && matrix[current][i].distance < nearestDist) {
                    nearestDist = matrix[current][i].distance;
                    nearest = i;
                }
            }
            
            if (nearest === -1) break;
            
            visited.add(nearest);
            order.push(nearest);
            current = nearest;
        }
        
        // Calculer la distance totale et le chemin complet
        let totalDistance = 0;
        const fullPath = [];
        
        for (let i = 0; i < order.length - 1; i++) {
            const segment = matrix[order[i]][order[i + 1]];
            totalDistance += segment.distance;
            
            // Éviter les doublons aux jonctions
            if (fullPath.length > 0 && segment.path.length > 0) {
                const lastPoint = fullPath[fullPath.length - 1];
                const firstPoint = segment.path[0];
                if (this.quickDist(lastPoint[0], lastPoint[1], firstPoint[0], firstPoint[1]) < 0.0001) {
                    fullPath.push(...segment.path.slice(1));
                } else {
                    fullPath.push(...segment.path);
                }
            } else {
                fullPath.push(...segment.path);
            }
        }
        
        return {
            order: order,
            totalDistance: totalDistance,
            path: fullPath,
            matrix: matrix
        };
    }
    
    // Route entre deux points avec géométrie précise
    route(startLat, startLng, endLat, endLng) {
        const start = this.findNearestNode(startLat, startLng);
        const end = this.findNearestNode(endLat, endLng);
        
        if (!start.nodeId || !end.nodeId) {
            return null;
        }
        
        const result = this.dijkstra(start.nodeId, end.nodeId);
        
        if (result) {
            // Ajouter le point de départ réel au début si éloigné
            if (result.path.length > 0) {
                const firstPoint = result.path[0];
                if (this.haversine(startLat, startLng, firstPoint[0], firstPoint[1]) > 20) {
                    result.path.unshift([startLat, startLng]);
                }
            } else {
                result.path = [[startLat, startLng]];
            }
            
            // Ajouter le point d'arrivée réel à la fin si éloigné
            const lastPoint = result.path[result.path.length - 1];
            if (this.haversine(endLat, endLng, lastPoint[0], lastPoint[1]) > 20) {
                result.path.push([endLat, endLng]);
            }
            
            result.distance += start.distance + end.distance;
        }
        
        return result;
    }
    
    // Route multi-points optimisée (TSP)
    routeMulti(points) {
        // points = [{lat, lng, id}, ...]
        
        // Trouver les noeuds les plus proches pour chaque point
        const snappedPoints = points.map(p => ({
            ...p,
            ...this.findNearestNode(p.lat, p.lng)
        }));
        
        // Résoudre le TSP
        const tspResult = this.solveTSP(snappedPoints);
        
        // Reconstruire l'ordre des points originaux
        const orderedPoints = tspResult.order.map(i => points[i]);
        
        return {
            orderedPoints: orderedPoints,
            totalDistance: tspResult.totalDistance,
            path: tspResult.path
        };
    }
}

// Min-Heap pour Dijkstra
class MinHeapV2 {
    constructor() {
        this.heap = [];
    }
    
    insert(node, priority) {
        this.heap.push({ node, priority });
        this.bubbleUp(this.heap.length - 1);
    }
    
    extractMin() {
        if (this.heap.length === 0) return null;
        const min = this.heap[0].node;
        const last = this.heap.pop();
        if (this.heap.length > 0) {
            this.heap[0] = last;
            this.bubbleDown(0);
        }
        return min;
    }
    
    isEmpty() {
        return this.heap.length === 0;
    }
    
    bubbleUp(i) {
        while (i > 0) {
            const parent = Math.floor((i - 1) / 2);
            if (this.heap[parent].priority <= this.heap[i].priority) break;
            [this.heap[parent], this.heap[i]] = [this.heap[i], this.heap[parent]];
            i = parent;
        }
    }
    
    bubbleDown(i) {
        const n = this.heap.length;
        while (true) {
            const left = 2 * i + 1;
            const right = 2 * i + 2;
            let smallest = i;
            
            if (left < n && this.heap[left].priority < this.heap[smallest].priority) {
                smallest = left;
            }
            if (right < n && this.heap[right].priority < this.heap[smallest].priority) {
                smallest = right;
            }
            
            if (smallest === i) break;
            [this.heap[i], this.heap[smallest]] = [this.heap[smallest], this.heap[i]];
            i = smallest;
        }
    }
}
