// Document handles undo using a RawDocument as backing
function RawDocument(updateCallback) {
	this.nodes = [];
	this.sel = [];
	this.updateCallback = updateCallback;
}

RawDocument.prototype.fromJSON = function(json) {
	this.nodes = json.nodes.map(function(n) {
		return new Node().fromJSON(n);
	});
	this.sel = [];
	for (var i = 0; i < this.nodes.length; i++) {
		this.nodes[i].createElement();
	}
	this.updateCallback();
	return this;
};

RawDocument.prototype.toJSON = function() {
	return {
		nodes: this.nodes.map(function(n) {
			return node.toJSON();
		})
	};
};

RawDocument.prototype.addNode = function(node) {
	this.nodes.push(node);
	node.createElement();
	this.updateCallback();
};

RawDocument.prototype.removeNode = function(node) {
	// remove links
	for (var i = 0; i < node.inputs.length; i++) {
		var input = node.inputs[i];
		for (var j = 0; j < input.connections.length; j++) {
			input.outputs[j].disconnectFrom(input);
		}
	}
	for (var i = 0; i < node.outputs.length; i++) {
		var output = node.outputs[i];
		for (var j = 0; j < output.connections.length; j++) {
			output.disconnectFrom(output.connections[j]);
		}
	}

	this.sel.removeOnce(node);
	this.nodes.removeOnce(node);
	node.deleteElement();
	this.updateCallback();
};

RawDocument.prototype.updateNode = function(node, name, value) {
	node.update(name, value);
	this.updateCallback();
};

RawDocument.prototype.setSelection = function(sel) {
	for (var i = 0; i < this.nodes.length; i++) {
		this.nodes[i].element.className = 'node';
	}
	for (var i = 0; i < sel.length; i++) {
		sel[i].element.className = 'selected node';
	}
	this.sel = sel;
	this.updateCallback();
};

RawDocument.prototype.addConnection = function(input, output) {
	input.connectTo(output);
	this.updateCallback();
};

RawDocument.prototype.removeConnection = function(input, output) {
	input.disconnectFrom(output);
	this.updateCallback();
};
