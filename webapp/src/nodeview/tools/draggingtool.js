function DraggingTool(doc) {
	this.doc = doc;
	this.oldX = 0;
	this.oldY = 0;
}

DraggingTool.prototype.mousePressed = function(x, y) {
	var sel = this.doc.getSelection();
	
	this.oldX = x;
	this.oldY = y;

	// Did we click on the existing selection?
	var draggingExistingSelection = false;
	for (var i = 0; i < sel.length; i++) {
		if (sel[i].rect.contains(x, y)) {
			draggingExistingSelection = true;
			break;
		}
	}

	// Drag the existing selection or a single unselected node
	if (!draggingExistingSelection) {
		sel = this.doc.getNodesInRect(new Rect(x, y, 0, 0));
		if (sel.length == 0) {
			return false;
		} else if (sel.length > 1) {
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
	this.minX = 30 + x - this.minX;
	this.minY = 30 + y - this.minY;

	this.doc.undoStack.endAllBatches();
	this.doc.undoStack.beginBatch();
	return true;
};

DraggingTool.prototype.mouseDragged = function(x, y) {
	x = Math.max(x, this.minX);
	y = Math.max(y, this.minY);
	this.doc.moveSelection(x - this.oldX, y - this.oldY);
	this.oldX = x;
	this.oldY = y;
};

DraggingTool.prototype.mouseReleased = function(x, y) {
	this.doc.undoStack.endBatch();
};
