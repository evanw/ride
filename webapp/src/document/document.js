function Document() {
	this.nodes = [];
	this.sel = [];
}

Document.prototype.getNodesInRect = function(rect) {
	var nodes = [];
	for (var i = 0; i < this.nodes.length; i++) {
		var node = this.nodes[i];
		if (node.rect.intersects(rect)) {
			nodes.push(node);
		}
	}
	return nodes;
};

Document.prototype.setSelection = function(sel) {
	for (var i = 0; i < this.nodes.length; i++) {
		this.nodes[i].element.className = 'node';
	}
	for (var i = 0; i < sel.length; i++) {
		sel[i].element.className = 'selected node';
	}
	this.sel = sel;
};
