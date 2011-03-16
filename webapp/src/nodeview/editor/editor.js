function Editor(context) {
	var this_ = this;
	this.context = context;
	this.doc = new Document();
	this.tools = [
		// Listed in order of precedence
		new PopupTool(this.doc),
		new NodeLinkTool(this.doc),
		new DraggingTool(this.doc),
		new SelectionTool(this.doc)
	];
}

Editor.prototype.drawLinks = function() {
	var nodes = this.doc.getNodes();
	for (var i = 0; i < nodes.length; i++) {
		var node = nodes[i];
		for (var j = 0; j < node.outputs.length; j++) {
			var output = node.outputs[j];
			for (var k = 0; k < output.connections.length; k++) {
				var input = output.connections[k];
				var ax = output.rect.centerX, ay = output.rect.centerY;
				var bx = input.rect.left - 8, by = input.rect.centerY;
				drawLink(this.context, ax, ay, bx, by);
			}
		}
	}
};

Editor.prototype.draw = function() {
	var canvas = this.context.canvas;
	var minSize = this.getMinSize();
	canvas.width = minSize.width;
	canvas.height = minSize.height;
	this.context.save();
	this.context.clearRect(0, 0, canvas.width, canvas.height);
	this.drawLinks();
	this.context.restore();
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

Editor.prototype.setProjectName = function(projectName) {
	window.projectName = projectName;

	// subscribe to node updates
	var this_ = this;
	channel('project', projectName, 'node', 'update').subscribe(function(json) {
		var nodes = this_.doc.getNodes();
		for (var i = 0; i < nodes.length; i++) {
			var node = nodes[i];
			if (node.id !== json.id) continue;
			for (var name in json) {
				// TODO: what to do about connections?
				if (name != 'id' && name != 'inputs' && name != 'outputs') {
					this_.doc.updateNode(node, name, json[name]);
				}
			}
		}
	});

	// poll until we get the node list
	var this_ = this;
	this.gotNodes = false;
	channel('project', projectName, 'nodes', 'response').subscribe(function(data) {
		if (!this_.gotNodes) {
			this_.doc.fromJSON(data);
			this_.gotNodes = true;
			clearInterval(interval);
		}
	});
	var interval = setInterval(function() {
		channel('project', projectName, 'nodes', 'request').publish({});
	}, 100);
};
