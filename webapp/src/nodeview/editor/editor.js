function Editor() {
	this.doc = new Document();
	this.tools = [
		// Listed in order of precedence
		new PopupTool(this.doc),
		new NodeLinkTool(this.doc),
		new DraggingTool(this.doc),
		new SelectionTool(this.doc)
	];
}

Editor.prototype.insertNode = function(title, inputs, outputs) {
	var node = new Node(title, inputs, outputs);
	this.doc.nodes.push(node);
	node.generateHTML();

	var rect = Rect.getFromElement(node.element);
	node.element.style.left = Math.floor(($(window).width() - rect.width) / 2) + 'px';
	node.element.style.top = Math.floor(($(window).height() - rect.height) / 2) + 'px';
};

Editor.prototype.drawLinks = function(c) {
	for (var i = 0; i < this.doc.nodes.length; i++) {
		var node = this.doc.nodes[i];
		for (var j = 0; j < node.outputs.length; j++) {
			var output = node.outputs[j];
			for (var k = 0; k < output.inputs.length; k++) {
				var input = output.inputs[k];
				var ax = output.rect.centerX, ay = output.rect.centerY;
				var bx = input.rect.left - 8, by = input.rect.centerY;
				drawLink(c, ax, ay, bx, by);
			}
		}
	}
};

Editor.prototype.draw = function(c) {
	c.save();
	c.clearRect(0, 0, c.canvas.width, c.canvas.height);
	this.drawLinks(c);
	c.restore();
};

Editor.prototype.getMinSize = function() {
	var minSize = { width: 0, height: 0 };
	for (var i = 0; i < this.doc.nodes.length; i++) {
		var rect = this.doc.nodes[i].rect;
		minSize.width = Math.max(minSize.width, rect.right + 50);
		minSize.height = Math.max(minSize.height, rect.bottom + 50);
	}
	return minSize;
};

Editor.prototype.mousePressed = function(x, y) {
	this.tool = null;
	for (var i = 0; i < this.tools.length; i++) {
		var tool = this.tools[i];
		if (tool.mousePressed(x, y)) {
			this.tool = tool;
			this.updateRects();
			break;
		}
	}
};

Editor.prototype.mouseMoved = function(x, y) {
	if (this.tool != null) {
		this.tool.mouseDragged(x, y);
		this.updateRects();
	}
};

Editor.prototype.mouseReleased = function(x, y) {
	if (this.tool != null) {
		this.tool.mouseReleased(x, y);
		this.tool = null;
		this.updateRects();
	}
};

Editor.prototype.updateRects = function() {
	for (var i = 0; i < this.doc.nodes.length; i++) {
		this.doc.nodes[i].updateRects();
	}
};

Editor.prototype.selectAll = function() {
	this.doc.setSelection(this.doc.nodes);
};

Editor.prototype.deleteSelection = function() {
	this.doc.deleteSelection();
};
