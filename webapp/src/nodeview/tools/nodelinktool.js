function NodeLinkTool(doc) {
	this.doc = doc;
	this.output = null;
	this.element = document.createElement('canvas');
	this.element.className = 'nodelink';
	this.c = this.element.getContext('2d');
	document.body.appendChild(this.element);
}

NodeLinkTool.prototype.updateElement = function(x, y) {
	var input = this.getInputFromPoint(x, y);
	var startX = this.output.rect.centerX;
	var startY = this.output.rect.centerY;
	var endX = input != null ? input.rect.left - 8 : x;
	var endY = input != null ? input.rect.centerY : y;

	var padding = 30;
	var left = Math.min(startX, endX) - padding;
	var top = Math.min(startY, endY) - padding;
	var right = Math.max(startX, endX) + padding;
	var bottom = Math.max(startY, endY) + padding;

	this.element.style.left = left + 'px';
	this.element.style.top = top + 'px';
	this.element.width = right - left;
	this.element.height = bottom - top;

	var ax = startX - left, ay = startY - top;
	var bx = endX - left, by = endY - top;
	var offset = 100;
	var c = this.c;

	drawLink(c, ax, ay, bx, by);
};

NodeLinkTool.prototype.getInputFromPoint = function(x, y) {
	var nodes = this.doc.getNodes();
	for (var i = 0; i < nodes.length; i++) {
		var node = nodes[i];
		for (var j = 0; j < node.inputs.length; j++) {
			var input = node.inputs[j];
			if (input.rect.contains(x, y)) {
				return input;
			}
		}
	}
	return null;
};

NodeLinkTool.prototype.getOutputFromPoint = function(x, y) {
	var nodes = this.doc.getNodes();
	for (var i = 0; i < nodes.length; i++) {
		var node = nodes[i];
		for (var j = 0; j < node.outputs.length; j++) {
			var output = node.outputs[j];
			if (output.rect.contains(x, y)) {
				return output;
			}
		}
	}
	return null;
};

NodeLinkTool.prototype.mousePressed = function(x, y) {
	// See if we are starting to drag a new link
	this.output = this.getOutputFromPoint(x, y);

	// Otherwise see if we are disconnecting an existing link
	if (this.output == null) {
		var input = this.getInputFromPoint(x, y);
		if (input != null && input.connections.length > 0) {
			this.output = input.connections[0];
			this.doc.removeConnection(input, this.output);
		}
	}

	// Manipulate the link if we have one
	if (this.output != null) {
		this.doc.setSelection([]);
		this.updateElement(x, y);
		this.element.style.display = 'block';
		return true;
	}

	return false;
};

NodeLinkTool.prototype.mouseDragged = function(x, y) {
	this.updateElement(x, y);
};

NodeLinkTool.prototype.mouseReleased = function(x, y) {
	this.element.style.display = 'none';

	var input = this.getInputFromPoint(x, y);
	if (input != null) {
		this.doc.addConnection(input, this.output);
	}
};
