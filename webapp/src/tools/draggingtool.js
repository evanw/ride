function DraggingTool(doc) {
	this.doc = doc;
	this.dragging = false;
	this.oldX = 0;
	this.oldY = 0;
}

DraggingTool.prototype.mousePressed = function(x, y) {
	// Did we click on the existing selection?
	var draggingExistingSelection = false;
	for (var i = 0; i < this.doc.sel.length; i++) {
		if (this.doc.sel[i].rect.contains(x, y)) {
			draggingExistingSelection = true;
			break;
		}
	}

	// Drag the existing selection or a single unselected node
	if (draggingExistingSelection) {
		var sel = this.doc.sel;
	} else {
		var sel = this.doc.getNodesInRect(new Rect(x, y, 0, 0));
		if (sel.length > 1) {
			sel = [sel[sel.length - 1]];
		}
		this.doc.setSelection(sel);
	}

	// Calculate the minimum allowed mouse coordinate so the nodes don't get dragged offscreen
	this.minX = Number.MAX_VALUE;
	this.minY = Number.MAX_VALUE;
	for (var i = 0; i < sel.length; i++) {
		var rect = sel[i].rect;
		this.minX = Math.min(this.minX, rect.left);
		this.minY = Math.min(this.minY, rect.top);
	}
	this.minX = 10 + x - this.minX;
	this.minY = 10 + y - this.minY;

	// Variables for mouseMoved()
	this.dragging = true;
	this.oldX = x;
	this.oldY = y;
};

DraggingTool.prototype.mouseMoved = function(x, y) {
	if (this.dragging) {
		x = Math.max(x, this.minX);
		y = Math.max(y, this.minY);
		var dx = x - this.oldX;
		var dy = y - this.oldY;
		var sel = this.doc.sel;
		for (var i = 0; i < sel.length; i++) {
			var node = sel[i];
			node.element.style.left = node.rect.left + dx + 'px';
			node.element.style.top = node.rect.top + dy + 'px';
		}
		this.oldX = x;
		this.oldY = y;
	}
};

DraggingTool.prototype.mouseReleased = function(x, y) {
	this.dragging = false;
};
