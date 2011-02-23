function PopupTool(doc) {
	this.doc = doc;
}

PopupTool.prototype.mousePressed = function(x, y) {
	// don't lose focus from visible popups
	for (var i = 0; i < this.doc.nodes.length; i++) {
		var node = this.doc.nodes[i];
		if (node.popup.isVisible && Rect.getFromElement(node.popup.element, false).contains(x, y)) {
			return true;
		}
	}

	// show at most one popup, and stop other tools if a popup is shown
	var hitEditRect = false;
	for (var i = 0; i < this.doc.nodes.length; i++) {
		var node = this.doc.nodes[i];
		if (!hitEditRect && node.editRect.contains(x, y)) {
			hitEditRect = true;
			node.popup.show();
		} else {
			node.popup.hide();
		}
	}

	return hitEditRect;
};

PopupTool.prototype.mouseDragged = function(x, y) {
};

PopupTool.prototype.mouseReleased = function(x, y) {
};
