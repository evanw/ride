function DraggingTool(doc) {
	this.doc = doc;
	this.sel = [];
	this.positions = [];
	this.startX = 0;
	this.startY = 0;
}

DraggingTool.prototype.mousePressed = function(x, y) {
	this.sel = this.doc.getSelection();

	// Did we click on the existing selection?
	var draggingExistingSelection = false;
	for (var i = 0; i < this.sel.length; i++) {
		if (this.sel[i].rect.contains(x, y)) {
			draggingExistingSelection = true;
			break;
		}
	}

	// Drag the existing selection or a single unselected node
	if (!draggingExistingSelection) {
		this.sel = this.doc.getNodesInRect(new Rect(x, y, 0, 0));
		if (this.sel.length == 0) {
			// We didn't click on a node, let another tool handle this click
			return false;
		} else if (this.sel.length > 1) {
			// If we've clicked on more than one node, just pick one so we can drag overlapping nodes apart
			this.sel = [this.sel[this.sel.length - 1]];
		}
		this.doc.setSelection(this.sel);
	}

	// Calculate the minimum allowed mouse coordinate so the nodes don't get dragged offscreen
	this.minX = Number.MAX_VALUE;
	this.minY = Number.MAX_VALUE;
	for (var i = 0; i < this.sel.length; i++) {
		var rect = this.sel[i].rect;
		this.minX = Math.min(this.minX, rect.left);
		this.minY = Math.min(this.minY, rect.top);
	}
	this.minX = 30 + x - this.minX;
	this.minY = 30 + y - this.minY;
	
	this.startX = x;
	this.startY = y;
	this.positions = [];
	for (var i = 0; i < this.sel.length; i++) {
		this.positions.push({
			x: this.sel[i].x,
			y: this.sel[i].y
		});
	}

	// We might not have gotten a mouseup, so end any previous operation now
	this.doc.undoStack.endAllBatches();
	return true;
};

DraggingTool.prototype.mouseDragged = function(x, y) {
	x = Math.max(x, this.minX);
	y = Math.max(y, this.minY);
	for (var i = 0; i < this.sel.length; i++) {
		var node = this.sel[i];
		var pos = this.positions[i];
		node.x = pos.x + x - this.startX;
		node.y = pos.y + y - this.startY;
		node.updateRects();
	}
	editor.draw();
};

DraggingTool.prototype.mouseReleased = function(x, y) {
	x = Math.max(x, this.minX);
	y = Math.max(y, this.minY);
	this.doc.undoStack.beginBatch();
	for (var i = 0; i < this.sel.length; i++) {
		var node = this.sel[i];
		var pos = this.positions[i];
		node.x = pos.x;
		node.y = pos.y;
		this.doc.updateNode(node, 'x', pos.x + x - this.startX);
		this.doc.updateNode(node, 'y', pos.y + y - this.startY);
	}
	this.doc.undoStack.endBatch();
};
