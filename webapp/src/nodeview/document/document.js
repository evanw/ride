function Document() {
	this.nodes = [];
	this.sel = [];
}

Document.prototype.getNodesInRect = function(rect) {
	var nodes = [];
	for (var i = 0; i < this.nodes.length; i++) {
		var node = this.nodes[i];
		if (node.rect.intersects(rect)) {
			nodes.push(node);
		}
	}
	return nodes;
};

Document.prototype.setSelection = function(sel) {
	for (var i = 0; i < this.nodes.length; i++) {
		this.nodes[i].element.className = 'node';
	}
	for (var i = 0; i < sel.length; i++) {
		sel[i].element.className = 'selected node';
	}
	this.sel = sel;
};

Document.prototype.deleteNode = function(node) {
	// remove links
	for (var i = 0; i < node.inputs.length; i++) {
		var input = node.inputs[i];
		for (var j = 0; j < input.outputs.length; j++) {
			input.outputs[j].disconnectFrom(input);
		}
	}
	for (var i = 0; i < node.outputs.length; i++) {
		var output = node.outputs[i];
		for (var j = 0; j < output.inputs.length; j++) {
			output.disconnectFrom(output.inputs[j]);
		}
	}

	// deselect node
	for (var i = 0; i < this.sel.length; i++) {
		if (this.sel[i] == node) {
			this.sel.splice(i, 1);
			break;
		}
	}

	// remove node
	for (var i = 0; i < this.nodes.length; i++) {
		if (this.nodes[i] == node) {
			this.nodes.splice(i, 1);
			break;
		}
	}

	// remove <div> from screen
	node.deleteElement();
};

Document.prototype.deleteSelection = function() {
	while (this.sel.length > 0) {
		this.deleteNode(this.sel[0]);
	}
};
