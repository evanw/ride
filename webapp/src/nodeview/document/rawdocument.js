// Document handles undo using a RawDocument as backing
function RawDocument() {
	this.nodes = [];
	this.sel = [];
}

RawDocument.prototype.fromJSON = function(json) {
	this.nodes = json.nodes.map(function(n) {
		return new Node().fromJSON(n);
	});
	this.sel = [];
	for (var i = 0; i < this.nodes.length; i++) {
		this.nodes[i].createElement();
	}
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
};

RawDocument.prototype.removeNode = function(node) {
	// remove links
	for (var i = 0; i < node.inputs.length; i++) {
		var input = node.inputs[i];
		for (var j = 0; j < input.connections.length; j++) {
			input.connections[j].disconnectFrom(input);
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

	channel('project', projectName, 'node', 'remove').publish({
		id: node.id
	});
};

RawDocument.prototype.updateNode = function(node, name, value) {
	node.update(name, value);

	// Only send the property that changed, not the whole node
	var json = { id: node.id };
	json[name] = value;
	channel('project', projectName, 'node', 'update').publish(json);
};

RawDocument.prototype.setSelection = function(sel) {
	for (var i = 0; i < this.nodes.length; i++) {
		this.nodes[i].element.className = 'node';
	}
	for (var i = 0; i < sel.length; i++) {
		sel[i].element.className = 'selected node';
	}
	this.sel = sel;
};

RawDocument.prototype.addConnection = function(input, output) {
	input.connectTo(output);
	channel('project', projectName, 'node', 'disconnect').publish({
		input: input.id,
		output: output.id
	});
};

RawDocument.prototype.removeConnection = function(input, output) {
	input.disconnectFrom(output);
	channel('project', projectName, 'node', 'disconnect').publish({
		input: input.id,
		output: output.id
	});
};
