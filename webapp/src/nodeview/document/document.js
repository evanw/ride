function Document() {
	this.rawDoc = new RawDocument();
	this.undoStack = new UndoStack();
}

Document.prototype.getNodes = function() {
	return this.rawDoc.nodes;
};

Document.prototype.getSelection = function() {
	return this.rawDoc.sel;
};

Document.prototype.fromJSON = function(json) {
	var nodes = json.nodes.map(function(n) {
		return new Node().fromJSON(n);
	});

	var connections = {};
	nodes.map(function(node) {
		node.createElement();
		node.inputs.map(function(input) {
			connections[input.id] = input;
		});
		node.outputs.map(function(output) {
			connections[output.id] = output;
		});
	});
	nodes.map(function(node) {
		node.inputs.map(function(input) {
			input._json_ids.map(function(id) {
				input.connectTo(connections[id]);
			});
		});
		node.outputs.map(function(output) {
			output._json_ids.map(function(id) {
				output.connectTo(connections[id]);
			});
		});
	});

	this.rawDoc.nodes = nodes;
	this.rawDoc.sel = [];
	this.undoStack = new UndoStack();
};

Document.prototype.getNodesInRect = function(rect) {
	var nodes = [];
	for (var i = 0; i < this.rawDoc.nodes.length; i++) {
		var node = this.rawDoc.nodes[i];
		if (node.rect.intersects(rect)) {
			nodes.push(node);
		}
	}
	return nodes;
};

Document.prototype.addNode = function(node) {
	this.undoStack.push(new AddNodeCommand(this.rawDoc, node));
};

Document.prototype.removeNode = function(node) {
	this.undoStack.beginBatch();

	// disconnect all inputs
	for (var i = 0; i < node.inputs.length; i++) {
		var input = node.inputs[i];
		while (input.connections.length) {
			this.removeConnection(input, input.connections[0]);
		}
	}

	// disconnect all outputs
	for (var i = 0; i < node.outputs.length; i++) {
		var output = node.outputs[i];
		while (output.connections.length) {
			this.removeConnection(output.connections[0], output);
		}
	}

	this.undoStack.push(new RemoveNodeCommand(this.rawDoc, node));
	this.undoStack.endBatch();
};

Document.prototype.updateNode = function(node, name, value) {
	if (node[name] !== value) {
		this.undoStack.push(new UpdateNodeCommand(this.rawDoc, node, name, value));
	}
};

Document.prototype.setSelection = function(sel) {
	// only change the selection if it's different
	var different = false;
	if (sel.length != this.rawDoc.sel.length) {
		different = true;
	} else {
		function compareNodes(a, b) { return a.id - b.id; }
		sel.sort(compareNodes);
		this.rawDoc.sel.sort(compareNodes);
		for (var i = 0; i < sel.length; i++) {
			if (sel[i] != this.rawDoc.sel[i]) {
				different = true;
				break;
			}
		}
	}
	
	if (different) this.undoStack.push(new SetSelectionCommand(this.rawDoc, sel));
};

Document.prototype.deleteSelection = function() {
	while (this.rawDoc.sel.length > 0) {
		this.removeNode(this.rawDoc.sel[0]);
	}
};

Document.prototype.addConnection = function(input, output) {
	// only add the connection if needed
	if (!input.connections.contains(output)) {
		this.undoStack.push(new AddConnectionCommand(this.rawDoc, input, output));
	}
};

Document.prototype.removeConnection = function(input, output) {
	// only remove the connection if needed
	if (input.connections.contains(output)) {
		this.undoStack.push(new RemoveConnectionCommand(this.rawDoc, input, output));
	}
};
