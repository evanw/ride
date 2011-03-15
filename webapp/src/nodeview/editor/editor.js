function Editor() {
	this.doc = new Document();
	this.tools = [
		// Listed in order of precedence
		new PopupTool(this.doc),
		// new NodeLinkTool(this.doc),
		new DraggingTool(this.doc),
		new SelectionTool(this.doc)
	];

	this.doc.rawDoc.fromJSON({'nodes': [
		{'inputs': [], 'x': 20, 'title': 'gscam', 'y': 20, 'outputs': [{'name': 'image'}], 'id': 0},
		{'inputs': [{'name': 'image'}], 'x': 20, 'title': 'cmvision', 'y': 20, 'outputs': [{'name': 'blobs'}], 'id': 1},
		{'inputs': [{'name': 'image'}], 'x': 20, 'title': 'ar_recog', 'y': 20, 'outputs': [{'name': 'tags'}], 'id': 2},
		{'inputs': [{'name': 'blob_list'}], 'x': 20, 'title': 'merge_blobs', 'y': 20, 'outputs': [{'name': 'blob_list'}], 'id': 3},
		{'inputs': [{'name': 'blob_list'}], 'x': 20, 'title': 'filter_blobs', 'y': 20, 'outputs': [{'name': 'orange_over_green'}, {'name': 'green_over_orange'}, {'name': 'orange'}, {'name': 'green'}, {'name': 'yellow'}, {'name': 'pink'}], 'id': 4},
		{'inputs': [{'name': 'blob_list'}], 'x': 20, 'title': 'seek_objects', 'y': 20, 'outputs': [{'name': 'twist'}], 'id': 5},
		{'inputs': [{'name': 'blob_list'}], 'x': 20, 'title': 'avoid_objects', 'y': 20, 'outputs': [{'name': 'twist'}], 'id': 6},
		{'inputs': [{'name': 'twist'}], 'x': 20, 'title': 'irobot_create_2_1', 'y': 20, 'outputs': [], 'id': 7}
	]});
}

Editor.prototype.drawLinks = function(c) {
	var nodes = this.doc.getNodes();
	for (var i = 0; i < nodes.length; i++) {
		var node = nodes[i];
		for (var j = 0; j < node.outputs.length; j++) {
			var output = node.outputs[j];
			for (var k = 0; k < output.connections.length; k++) {
				var input = output.connections[k];
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
	var nodes = this.doc.getNodes();
	for (var i = 0; i < nodes.length; i++) {
		var rect = nodes[i].rect;
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
			break;
		}
	}
};

Editor.prototype.mouseMoved = function(x, y) {
	if (this.tool != null) {
		this.tool.mouseDragged(x, y);
	}
};

Editor.prototype.mouseReleased = function(x, y) {
	if (this.tool != null) {
		this.tool.mouseReleased(x, y);
		this.tool = null;
	}
};

Editor.prototype.selectAll = function() {
	this.doc.setSelection(this.doc.getNodes());
};

Editor.prototype.undo = function() {
	this.doc.undoStack.undo();
};

Editor.prototype.redo = function() {
	this.doc.undoStack.redo();
};

Editor.prototype.deleteSelection = function() {
	this.doc.deleteSelection();
};

Editor.prototype.insertNode = function(json) {
	this.doc.addNode(new Node().fromJSON(json));
};
