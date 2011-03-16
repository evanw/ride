function SelectionTool(doc) {
	this.doc = doc;
	this.startX = 0;
	this.startY = 0;
	this.element = document.createElement('div');
	this.element.className = 'selectionbox';
	document.body.appendChild(this.element);
}

SelectionTool.prototype.updateElement = function(endX, endY) {
	var left = Math.min(this.startX, endX);
	var top = Math.min(this.startY, endY);
	var right = Math.max(this.startX, endX);
	var bottom = Math.max(this.startY, endY);
	this.element.style.left = left + 'px';
	this.element.style.top = top + 'px';
	this.element.style.width = (right - left) + 'px';
	this.element.style.height = (bottom - top) + 'px';
	this.doc.setSelection(this.doc.getNodesInRect(new Rect(left, top, right - left, bottom - top)));
};

SelectionTool.prototype.mousePressed = function(x, y) {
	this.startX = x;
	this.startY = y;
	this.element.style.display = 'block';
	this.updateElement(x, y);
	this.doc.undoStack.endAllBatches();
	this.doc.undoStack.beginBatch();
	return true;
};

SelectionTool.prototype.mouseDragged = function(x, y) {
	this.updateElement(x, y);
};

SelectionTool.prototype.mouseReleased = function(x, y) {
	this.element.style.display = 'none';
	this.doc.undoStack.endBatch();
};
