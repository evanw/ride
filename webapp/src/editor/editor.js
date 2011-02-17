function Editor() {
	this.doc = new Document();
	this.tools = [
		new DraggingTool(this.doc),
		new SelectionTool(this.doc)
	];
}

Editor.prototype.drawLinks = function(c) {
	c.strokeStyle = 'yellow';
	c.lineWidth = 2;
	c.shadowBlur = 3;
	c.shadowColor = 'black';
	c.shadowOffsetY = 1;

	c.beginPath();
	for (var i = 0; i < this.doc.nodes.length; i++) {
		var node = this.doc.nodes[i];
		for (var j = 0; j < node.outputs.length; j++) {
			var output = node.outputs[j];
			for (var k = 0; k < output.nodes.length; k++) {
				var input = output.nodes[k].getInputFromNode(node);
				var rectIn = input.rect;
				var rectOut = output.rect;
				var ax = rectOut.right - 3, ay = rectOut.top + rectOut.height / 2;
				var bx = rectIn.left + 3, by = rectIn.top + rectIn.height / 2;
				c.moveTo(ax, ay);
				c.bezierCurveTo(ax + 100, ay, bx - 100, by, bx, by);
			}
		}
	}
	c.stroke();
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
