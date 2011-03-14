function PopupTool(doc) {
	this.doc = doc;
}

PopupTool.prototype.mousePressed = function(x, y) {
	var nodes = this.doc.rawDoc.nodes;
	
	// don't lose focus from visible popups
	for (var i = 0; i < nodes.length; i++) {
		var node = nodes[i];
		if (node.popup.isVisible && Rect.getFromElement(node.popup.element, false).contains(x, y)) {
			return true;
		}
	}

	// show at most one popup, and stop other tools if a popup is shown
	var hitEditRect = false;
	for (var i = 0; i < nodes.length; i++) {
		var node = nodes[i];
		if (!hitEditRect && node.editRect.contains(x, y)) {
			hitEditRect = true;
			node.showPopup();
		} else {
			node.hidePopup();
		}
	}

	return hitEditRect;
};

PopupTool.prototype.mouseDragged = function(x, y) {
};

PopupTool.prototype.mouseReleased = function(x, y) {
};
